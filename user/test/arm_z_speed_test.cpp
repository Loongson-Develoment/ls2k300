#include "ArmMotorPosition.h"
#include "ArmKinematics.h"

#include "LS2K0300_UART.h"
#include "X42_V2.h"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <unistd.h>

#define ARM_MOTOR_UART UART4
#define ARM_MOTOR_BAUD B921600

#define ARM_MOTOR_ADDR_2 2U
#define ARM_MOTOR_ADDR_3 3U

#define ARM_GEAR_RATIO_1 (32.0 / 9.0)
#define ARM_GEAR_RATIO_2 (32.0 / 9.0)
#define ARM_GEAR_RATIO_3 (32.0 / 9.0)

#define ARM_INITIAL_THETA1_RAD 0.0
#define ARM_INITIAL_THETA2_RAD (0.0 * M_PI / 180.0)
#define ARM_INITIAL_THETA3_RAD (140.0 * M_PI / 180.0)

#define ARM_STARTUP_TARGET_THETA1_RAD 0.0
#define ARM_STARTUP_TARGET_THETA2_RAD 0.0
#define ARM_STARTUP_TARGET_THETA3_RAD (140.0 * M_PI / 180.0)

#define ARM_JOINT_DIRECTION_2 -1.0 /* 2号电机实时角换算为 theta2 时按 0626 方向取反 */
#define ARM_JOINT_DIRECTION_3 -1.0

#define ARM_MAX_MOTOR_RPM 30.0f
#define ARM_VELOCITY_RAMP_RPMPS 20U
#define ARM_READ_PERIOD_MS 30L
#define ARM_PRINT_PERIOD_MS 300LL
#define ARM_POSITION_READ_TIMEOUT_MS 500
#define ARM_POSITION_READ_RETRY_GAP_MS 2L
#define ARM_POSITION_READ_FAIL_STOP_COUNT 3

#define ARM_ORIGIN_MODE_NEAREST_ZERO 0x00U
#define ARM_ORIGIN_COMMAND_CODE 0x9AU
#define ARM_ORIGIN_STATUS_CODE 0x3BU
#define ARM_MOTOR_STATUS_CODE 0x3AU
#define ARM_POSITION_COMMAND_CODE 0xFBU
#define ARM_X42_CHECK_BYTE 0x6BU
#define ARM_COMMAND_STATUS_ALREADY_DONE 0x12U
#define ARM_COMMAND_STATUS_ACTION_DONE 0x9FU
#define ARM_COMMAND_STATUS_PARAM_ERROR 0xE2U
#define ARM_COMMAND_STATUS_FORMAT_ERROR 0xEEU
#define ARM_ORIGIN_READY_MASK 0x03U
#define ARM_ORIGIN_READY_VALUE 0x03U
#define ARM_ORIGIN_STATE_MASK 0x0CU
#define ARM_ORIGIN_STATE_BUSY 0x04U
#define ARM_ORIGIN_STATE_FAILED 0x08U
#define ARM_ORIGIN_STATE_DONE 0x00U
#define ARM_ORIGIN_NEAREST_DONE_STATUS 0x0BU
#define ARM_ORIGIN_TRIGGER_RESPONSE_TIMEOUT_MS 200
#define ARM_ORIGIN_STATUS_RESPONSE_TIMEOUT_MS 200
#define ARM_ORIGIN_FIRST_POLL_DELAY_MS 1L
#define ARM_ORIGIN_STATUS_POLL_MS 20L
#define ARM_ORIGIN_WAIT_TIMEOUT_MS 15000LL
#define ARM_ORIGIN_STABLE_DONE_POLLS 3U

#define ARM_POSITION_MODE_REL_CURRENT 0x02U
#define ARM_POSITION_RESPONSE_TIMEOUT_MS 200
#define ARM_POSITION_STATUS_RESPONSE_TIMEOUT_MS 200
#define ARM_POSITION_STATUS_POLL_MS 20L
#define ARM_POSITION_WAIT_TIMEOUT_MS 15000LL
#define ARM_POSITION_STABLE_DONE_POLLS 3U
#define ARM_POSITION_ARRIVED_MASK 0x02U
#define ARM_POSITION_MOVE_SPEED_RPM 10.0f
#define ARM_POSITION_SKIP_EPS_DEG 0.2

#define ARM_SHUTDOWN_STAGE_X_LIMIT_MM 170.0
#define ARM_SHUTDOWN_STAGE_X_EPS_MM 3.0

static volatile sig_atomic_t running = 1;
static volatile sig_atomic_t shutdown_requested = 0;

typedef struct {
    double motor2_zero_degrees;
    double motor3_zero_degrees;
} ArmMotorZero;

static void signal_handler(int signum)
{
    (void)signum;
    if (shutdown_requested) {
        running = 0;
    }
    shutdown_requested = 1;
}

static int install_signal_handlers(void)
{
    struct sigaction action;

    std::memset(&action, 0, sizeof(action));
    action.sa_handler = signal_handler;
    sigemptyset(&action.sa_mask);

    if (sigaction(SIGINT, &action, NULL) != 0 ||
        sigaction(SIGTERM, &action, NULL) != 0) {
        perror("sigaction");
        return -1;
    }

    return 0;
}

static int64_t monotonic_time_ms(void)
{
    struct timespec now;

    if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) {
        return 0;
    }

    return (int64_t)now.tv_sec * 1000LL +
           (int64_t)now.tv_nsec / 1000000LL;
}

static void sleep_ms(long milliseconds)
{
    struct timespec delay;

    delay.tv_sec = milliseconds / 1000L;
    delay.tv_nsec = (milliseconds % 1000L) * 1000000L;
    while (running && nanosleep(&delay, &delay) != 0) {
    }
}

static void command_gap_ms(long milliseconds)
{
    struct timespec delay;

    delay.tv_sec = milliseconds / 1000L;
    delay.tv_nsec = (milliseconds % 1000L) * 1000000L;
    while (nanosleep(&delay, &delay) != 0 && errno == EINTR) {
    }
}

static double clamp_double(double value, double min_value, double max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static ssize_t read_uart_byte_timeout(ls2k0300_uart_t *uart,
                                      uint8_t *byte,
                                      int64_t deadline_ms)
{
    if (uart == NULL || byte == NULL) {
        return -1;
    }

    while (running && monotonic_time_ms() < deadline_ms) {
        ssize_t n = ls2k0300_uart_read(uart, byte, 1);
        if (n == 1) {
            return 1;
        }
        if (n < 0 && errno != EINTR && errno != EAGAIN &&
            errno != EWOULDBLOCK) {
            return -1;
        }

        usleep(1000);
    }

    return 0;
}

static ssize_t read_x42_4byte_response_timeout(ls2k0300_uart_t *uart,
                                               uint8_t addr,
                                               uint8_t code,
                                               uint8_t response[4],
                                               int timeout_ms)
{
    size_t frame_pos = 0U;
    int64_t deadline_ms;

    if (uart == NULL || response == NULL || timeout_ms <= 0) {
        return -1;
    }

    deadline_ms = monotonic_time_ms() + (int64_t)timeout_ms;
    while (running && monotonic_time_ms() < deadline_ms) {
        uint8_t byte = 0U;
        ssize_t n = read_uart_byte_timeout(uart, &byte, deadline_ms);

        if (n < 0) {
            return -1;
        }
        if (n == 0) {
            break;
        }

        if (frame_pos == 0U) {
            if (byte != addr) {
                continue;
            }
            response[frame_pos++] = byte;
            continue;
        }

        if (frame_pos == 1U) {
            if (byte != code) {
                frame_pos = (byte == addr) ? 1U : 0U;
                if (frame_pos == 1U) {
                    response[0] = byte;
                }
                continue;
            }
            response[frame_pos++] = byte;
            continue;
        }

        response[frame_pos++] = byte;
        if (frame_pos == 4U) {
            if (response[3] == ARM_X42_CHECK_BYTE) {
                return 4;
            }
            frame_pos = 0U;
        }
    }

    return 0;
}

static bool arm_origin_status_done(uint8_t status)
{
    return ((status & ARM_ORIGIN_STATE_MASK) == ARM_ORIGIN_STATE_DONE) &&
           ((status & ARM_ORIGIN_READY_MASK) == ARM_ORIGIN_READY_VALUE);
}

static bool read_arm_origin_status(ls2k0300_uart_t *uart,
                                   uint8_t addr,
                                   uint8_t *status)
{
    uint8_t response[4] = {0};
    ssize_t response_size;

    if (uart == NULL || status == NULL) {
        return false;
    }

    if (ls2k0300_uart_flush(uart) != 0) {
        return false;
    }

    ZDT_X42_V2_Read_Sys_Params(uart, addr, S_OFLAG);
    response_size = read_x42_4byte_response_timeout(
        uart,
        addr,
        ARM_ORIGIN_STATUS_CODE,
        response,
        ARM_ORIGIN_STATUS_RESPONSE_TIMEOUT_MS);
    if (response_size != 4 ||
        response[0] != addr ||
        response[1] != ARM_ORIGIN_STATUS_CODE ||
        response[3] != ARM_X42_CHECK_BYTE) {
        return false;
    }

    *status = response[2];
    return true;
}

static bool home_arm_motor_nearest(ls2k0300_uart_t *uart, uint8_t addr)
{
    uint8_t response[4] = {0};
    ssize_t response_size;
    uint8_t stable_done_count = 0U;
    int64_t start_ms;

    if (uart == NULL || !running) {
        return false;
    }

    if (ls2k0300_uart_flush(uart) != 0) {
        return false;
    }

    ZDT_X42_V2_Origin_Trigger_Return(uart,
                                     addr,
                                     ARM_ORIGIN_MODE_NEAREST_ZERO,
                                     false);

    response_size = read_x42_4byte_response_timeout(
        uart,
        addr,
        ARM_ORIGIN_COMMAND_CODE,
        response,
        ARM_ORIGIN_TRIGGER_RESPONSE_TIMEOUT_MS);
    if (response_size == 4 &&
        response[0] == addr &&
        response[1] == ARM_ORIGIN_COMMAND_CODE &&
        response[3] == ARM_X42_CHECK_BYTE) {
        if (response[2] == ARM_COMMAND_STATUS_ALREADY_DONE ||
            response[2] == ARM_COMMAND_STATUS_ACTION_DONE) {
            return true;
        }
        if (response[2] == ARM_COMMAND_STATUS_PARAM_ERROR ||
            response[2] == ARM_COMMAND_STATUS_FORMAT_ERROR) {
            fprintf(stderr, "ERR motor%u home command status=0x%02X\n",
                    (unsigned int)addr, response[2]);
            return false;
        }
    }

    sleep_ms(ARM_ORIGIN_FIRST_POLL_DELAY_MS);
    start_ms = monotonic_time_ms();

    while (running && !shutdown_requested) {
        uint8_t status;
        uint8_t origin_state;

        if (monotonic_time_ms() - start_ms > ARM_ORIGIN_WAIT_TIMEOUT_MS) {
            fprintf(stderr, "ERR motor%u home timeout\n",
                    (unsigned int)addr);
            return false;
        }

        if (!read_arm_origin_status(uart, addr, &status)) {
            sleep_ms(ARM_ORIGIN_STATUS_POLL_MS);
            continue;
        }

        if (status == ARM_ORIGIN_NEAREST_DONE_STATUS) {
            return true;
        }

        origin_state = (uint8_t)(status & ARM_ORIGIN_STATE_MASK);
        if (origin_state == ARM_ORIGIN_STATE_BUSY) {
            stable_done_count = 0U;
        } else if (origin_state == ARM_ORIGIN_STATE_FAILED) {
            fprintf(stderr, "ERR motor%u home failed status=0x%02X\n",
                    (unsigned int)addr, status);
            return false;
        } else if (arm_origin_status_done(status)) {
            stable_done_count++;
            if (stable_done_count >= ARM_ORIGIN_STABLE_DONE_POLLS) {
                return true;
            }
        } else {
            stable_done_count = 0U;
        }

        sleep_ms(ARM_ORIGIN_STATUS_POLL_MS);
    }

    return false;
}

static bool home_arm_motors_nearest(ls2k0300_uart_t *uart)
{
    const uint8_t addrs[] = {
        ARM_MOTOR_ADDR_2,
        ARM_MOTOR_ADDR_3,
    };

    for (size_t i = 0U; i < sizeof(addrs) / sizeof(addrs[0]); ++i) {
        if (!home_arm_motor_nearest(uart, addrs[i])) {
            return false;
        }
        command_gap_ms(1);
    }

    return true;
}

static bool read_arm_motor_status(ls2k0300_uart_t *uart,
                                  uint8_t addr,
                                  uint8_t *status)
{
    uint8_t response[4] = {0};
    ssize_t response_size;

    if (uart == NULL || status == NULL) {
        return false;
    }

    if (ls2k0300_uart_flush(uart) != 0) {
        return false;
    }

    ZDT_X42_V2_Read_Sys_Params(uart, addr, S_SFLAG);
    response_size = read_x42_4byte_response_timeout(
        uart,
        addr,
        ARM_MOTOR_STATUS_CODE,
        response,
        ARM_POSITION_STATUS_RESPONSE_TIMEOUT_MS);
    if (response_size != 4 ||
        response[0] != addr ||
        response[1] != ARM_MOTOR_STATUS_CODE ||
        response[3] != ARM_X42_CHECK_BYTE) {
        return false;
    }

    *status = response[2];
    return true;
}

static bool wait_arm_position_arrived(ls2k0300_uart_t *uart,
                                      uint8_t addr)
{
    uint8_t stable_arrived_count = 0U;
    int64_t start_ms = monotonic_time_ms();

    while (running) {
        uint8_t status;

        if (monotonic_time_ms() - start_ms > ARM_POSITION_WAIT_TIMEOUT_MS) {
            fprintf(stderr, "ERR motor%u position timeout\n",
                    (unsigned int)addr);
            return false;
        }

        if (!read_arm_motor_status(uart, addr, &status)) {
            sleep_ms(ARM_POSITION_STATUS_POLL_MS);
            continue;
        }

        if ((status & ARM_POSITION_ARRIVED_MASK) != 0U) {
            stable_arrived_count++;
            if (stable_arrived_count >= ARM_POSITION_STABLE_DONE_POLLS) {
                return true;
            }
        } else {
            stable_arrived_count = 0U;
        }

        sleep_ms(ARM_POSITION_STATUS_POLL_MS);
    }

    return false;
}

static bool read_motor23_degrees(ls2k0300_uart_t *uart,
                                 double *motor2_degrees,
                                 double *motor3_degrees)
{
    if (!arm_read_motor_position_degrees(uart, ARM_MOTOR_ADDR_2,
                                         motor2_degrees,
                                         ARM_POSITION_READ_TIMEOUT_MS)) {
        fprintf(stderr, "ERR read motor2 0x36 failed\n");
        return false;
    }
    command_gap_ms(ARM_POSITION_READ_RETRY_GAP_MS);

    if (!arm_read_motor_position_degrees(uart, ARM_MOTOR_ADDR_3,
                                         motor3_degrees,
                                         ARM_POSITION_READ_TIMEOUT_MS)) {
        fprintf(stderr, "ERR read motor3 0x36 failed\n");
        return false;
    }
    command_gap_ms(ARM_POSITION_READ_RETRY_GAP_MS);

    return true;
}

static bool read_theta23_with_motor(ls2k0300_uart_t *uart,
                                    const ArmMotorZero *zero,
                                    double *theta2,
                                    double *theta3,
                                    double *motor2_degrees_out,
                                    double *motor3_degrees_out)
{
    double motor2_degrees;
    double motor3_degrees;

    if (zero == NULL || theta2 == NULL || theta3 == NULL) {
        return false;
    }

    if (!read_motor23_degrees(uart, &motor2_degrees, &motor3_degrees)) {
        return false;
    }

    double motor2_delta = motor2_degrees - zero->motor2_zero_degrees;
    double motor3_delta = motor3_degrees - zero->motor3_zero_degrees;

    *theta2 = ARM_INITIAL_THETA2_RAD +
              ARM_JOINT_DIRECTION_2 *
              (motor2_delta * M_PI / 180.0) / ARM_GEAR_RATIO_2;
    *theta3 = ARM_INITIAL_THETA3_RAD +
              ARM_JOINT_DIRECTION_3 *
              (motor3_delta * M_PI / 180.0) / ARM_GEAR_RATIO_3;

    if (motor2_degrees_out != NULL) {
        *motor2_degrees_out = motor2_degrees;
    }
    if (motor3_degrees_out != NULL) {
        *motor3_degrees_out = motor3_degrees;
    }

    return true;
}

static bool read_theta23(ls2k0300_uart_t *uart,
                         const ArmMotorZero *zero,
                         double *theta2,
                         double *theta3)
{
    return read_theta23_with_motor(uart,
                                   zero,
                                   theta2,
                                   theta3,
                                   NULL,
                                   NULL);
}

static double joint_delta_to_motor_degrees(double joint_delta_rad,
                                           double gear_ratio,
                                           double joint_direction)
{
    if (std::fabs(joint_direction) < 1e-9) {
        return 0.0;
    }

    return joint_delta_rad * 180.0 / M_PI * gear_ratio / joint_direction;
}

static uint8_t arm_position_dir_from_delta(double motor_delta_degrees)
{
    return (motor_delta_degrees >= 0.0) ? 0U : 1U;
}

static bool start_relative_position_degrees(ls2k0300_uart_t *uart,
                                            uint8_t addr,
                                            double motor_delta_degrees,
                                            bool *needs_wait)
{
    uint8_t response[4] = {0};
    ssize_t response_size;
    float abs_degrees;
    uint8_t dir;

    if (uart == NULL || needs_wait == NULL) {
        return false;
    }

    *needs_wait = false;
    if (std::fabs(motor_delta_degrees) <= ARM_POSITION_SKIP_EPS_DEG) {
        return true;
    }

    dir = arm_position_dir_from_delta(motor_delta_degrees);
    abs_degrees = (float)std::fabs(motor_delta_degrees);

    if (ls2k0300_uart_flush(uart) != 0) {
        return false;
    }

    ZDT_X42_V2_Bypass_Position_LV_Control(uart,
                                          addr,
                                          dir,
                                          ARM_POSITION_MOVE_SPEED_RPM,
                                          abs_degrees,
                                          ARM_POSITION_MODE_REL_CURRENT,
                                          0U);

    response_size = read_x42_4byte_response_timeout(
        uart,
        addr,
        ARM_POSITION_COMMAND_CODE,
        response,
        ARM_POSITION_RESPONSE_TIMEOUT_MS);
    if (response_size != 4) {
        *needs_wait = true;
        return true;
    }

    if (response[0] != addr ||
        response[1] != ARM_POSITION_COMMAND_CODE ||
        response[3] != ARM_X42_CHECK_BYTE) {
        *needs_wait = true;
        return true;
    }

    if (response[2] == ARM_COMMAND_STATUS_PARAM_ERROR ||
        response[2] == ARM_COMMAND_STATUS_FORMAT_ERROR) {
        fprintf(stderr, "ERR motor%u position command status=0x%02X\n",
                (unsigned int)addr, response[2]);
        return false;
    }

    if (response[2] == ARM_COMMAND_STATUS_ACTION_DONE ||
        response[2] == ARM_COMMAND_STATUS_ALREADY_DONE) {
        return true;
    }

    *needs_wait = true;
    return true;
}

static double normalize_angle_delta(double angle_rad)
{
    while (angle_rad > M_PI) {
        angle_rad -= 2.0 * M_PI;
    }
    while (angle_rad < -M_PI) {
        angle_rad += 2.0 * M_PI;
    }

    return angle_rad;
}

static double joint_solution_distance(double theta2,
                                      double theta3,
                                      double ref_theta2,
                                      double ref_theta3)
{
    return std::fabs(normalize_angle_delta(theta2 - ref_theta2)) +
           std::fabs(normalize_angle_delta(theta3 - ref_theta3));
}

static bool solve_xz_to_joint_angles(double x,
                                     double z,
                                     double ref_theta2,
                                     double ref_theta3,
                                     double *target_theta2,
                                     double *target_theta3)
{
    double rho;
    double delta_z;
    double reach;
    double gamma;
    double half_angle;
    double theta2_a;
    double theta3_a;
    double theta2_b;
    double theta3_b;
    double distance_a;
    double distance_b;

    if (target_theta2 == NULL || target_theta3 == NULL) {
        return false;
    }

    rho = std::fabs(x);
    delta_z = z - ARM_LINK_HEIGHT_MM;
    reach = std::sqrt(rho * rho + delta_z * delta_z);
    if (reach > 2.0 * ARM_LINK_LENGTH_MM) {
        fprintf(stderr, "ERR xz target unreachable: x=%.2f z=%.2f\n", x, z);
        return false;
    }

    gamma = std::atan2(rho, delta_z);
    half_angle = std::acos(clamp_double(reach / (2.0 * ARM_LINK_LENGTH_MM),
                                        -1.0,
                                        1.0));

    theta2_a = gamma - half_angle;
    theta3_a = 2.0 * half_angle;
    theta2_b = gamma + half_angle;
    theta3_b = -2.0 * half_angle;

    distance_a = joint_solution_distance(theta2_a, theta3_a,
                                         ref_theta2, ref_theta3);
    distance_b = joint_solution_distance(theta2_b, theta3_b,
                                         ref_theta2, ref_theta3);
    if (distance_a <= distance_b) {
        *target_theta2 = theta2_a;
        *target_theta3 = theta3_a;
    } else {
        *target_theta2 = theta2_b;
        *target_theta3 = theta3_b;
    }

    return true;
}

static bool move_joints_to_target(ls2k0300_uart_t *uart,
                                  const ArmMotorZero *zero,
                                  double *theta2,
                                  double *theta3,
                                  double target_theta2,
                                  double target_theta3)
{
    bool wait_motor2 = false;
    bool wait_motor3 = false;
    double motor2_delta_degrees;
    double motor3_delta_degrees;

    if (uart == NULL || zero == NULL || theta2 == NULL || theta3 == NULL) {
        return false;
    }

    motor2_delta_degrees = joint_delta_to_motor_degrees(
        target_theta2 - *theta2,
        ARM_GEAR_RATIO_2,
        ARM_JOINT_DIRECTION_2);
    motor3_delta_degrees = joint_delta_to_motor_degrees(
        target_theta3 - *theta3,
        ARM_GEAR_RATIO_3,
        ARM_JOINT_DIRECTION_3);

    if (!start_relative_position_degrees(uart,
                                         ARM_MOTOR_ADDR_2,
                                         motor2_delta_degrees,
                                         &wait_motor2)) {
        return false;
    }
    command_gap_ms(1);

    if (!start_relative_position_degrees(uart,
                                         ARM_MOTOR_ADDR_3,
                                         motor3_delta_degrees,
                                         &wait_motor3)) {
        return false;
    }

    if (wait_motor2 && !wait_arm_position_arrived(uart, ARM_MOTOR_ADDR_2)) {
        return false;
    }
    if (wait_motor3 && !wait_arm_position_arrived(uart, ARM_MOTOR_ADDR_3)) {
        return false;
    }

    return read_theta23(uart, zero, theta2, theta3);
}

static bool move_to_startup_pose(ls2k0300_uart_t *uart,
                                 const ArmMotorZero *zero,
                                 double *theta2,
                                 double *theta3)
{
    point3d_t current_position;
    point3d_t target_position;
    double stage_z_theta2;
    double stage_z_theta3;

    if (!read_theta23(uart, zero, theta2, theta3)) {
        return false;
    }

    if (!arm_forward_kinematics(ARM_INITIAL_THETA1_RAD,
                                *theta2,
                                *theta3,
                                &current_position) ||
        !arm_forward_kinematics(ARM_STARTUP_TARGET_THETA1_RAD,
                                ARM_STARTUP_TARGET_THETA2_RAD,
                                ARM_STARTUP_TARGET_THETA3_RAD,
                                &target_position)) {
        return false;
    }

    if (!solve_xz_to_joint_angles(current_position.x,
                                  target_position.z,
                                  *theta2,
                                  *theta3,
                                  &stage_z_theta2,
                                  &stage_z_theta3)) {
        return false;
    }

    if (!move_joints_to_target(uart,
                               zero,
                               theta2,
                               theta3,
                               stage_z_theta2,
                               stage_z_theta3)) {
        return false;
    }

    return move_joints_to_target(uart,
                                 zero,
                                 theta2,
                                 theta3,
                                 ARM_STARTUP_TARGET_THETA2_RAD,
                                 ARM_STARTUP_TARGET_THETA3_RAD);
}

static bool return_to_zero_pose(ls2k0300_uart_t *uart,
                                const ArmMotorZero *zero,
                                double *theta2,
                                double *theta3)
{
    point3d_t current_position;
    double stage_x_theta2;
    double stage_x_theta3;

    if (!read_theta23(uart, zero, theta2, theta3)) {
        return false;
    }

    if (!arm_forward_kinematics(ARM_INITIAL_THETA1_RAD,
                                *theta2,
                                *theta3,
                                &current_position)) {
        return false;
    }

    if (current_position.x < ARM_SHUTDOWN_STAGE_X_LIMIT_MM -
                             ARM_SHUTDOWN_STAGE_X_EPS_MM) {
        if (!solve_xz_to_joint_angles(ARM_SHUTDOWN_STAGE_X_LIMIT_MM,
                                      current_position.z,
                                      *theta2,
                                      *theta3,
                                      &stage_x_theta2,
                                      &stage_x_theta3)) {
            return false;
        }

        if (!move_joints_to_target(uart,
                                   zero,
                                   theta2,
                                   theta3,
                                   stage_x_theta2,
                                   stage_x_theta3)) {
            return false;
        }
    }

    return move_joints_to_target(uart,
                                 zero,
                                 theta2,
                                 theta3,
                                 ARM_INITIAL_THETA2_RAD,
                                 ARM_INITIAL_THETA3_RAD);
}

static void apply_motor_limit(JointMotorSpeeds *speeds)
{
    float max_rpm = std::max(speeds->velocity_2.rpm_abs,
                             speeds->velocity_3.rpm_abs);

    if (max_rpm <= ARM_MAX_MOTOR_RPM || max_rpm <= 0.0f) {
        return;
    }

    float scale = ARM_MAX_MOTOR_RPM / max_rpm;
    speeds->velocity_2.rpm *= scale;
    speeds->velocity_3.rpm *= scale;
    speeds->velocity_2.rpm_abs = std::fabs(speeds->velocity_2.rpm);
    speeds->velocity_3.rpm_abs = std::fabs(speeds->velocity_3.rpm);
}

static void send_motor23_speeds(ls2k0300_uart_t *uart,
                                const JointMotorSpeeds *speeds)
{
    ZDT_X42_V2_Velocity_Control(uart, ARM_MOTOR_ADDR_2,
                                speeds->velocity_2.dir,
                                ARM_VELOCITY_RAMP_RPMPS,
                                speeds->velocity_2.rpm_abs, 0);
    command_gap_ms(1);
    ZDT_X42_V2_Velocity_Control(uart, ARM_MOTOR_ADDR_3,
                                speeds->velocity_3.dir,
                                ARM_VELOCITY_RAMP_RPMPS,
                                speeds->velocity_3.rpm_abs, 0);
    command_gap_ms(1);
}

static void stop_motor23(ls2k0300_uart_t *uart)
{
    JointMotorSpeeds zero;

    std::memset(&zero, 0, sizeof(zero));
    zero.valid = 1;
    send_motor23_speeds(uart, &zero);
}

static bool read_stdin_line_nonblocking(char *line, size_t line_size)
{
    static size_t pos = 0U;
    char ch;

    if (line == NULL || line_size == 0U) {
        return false;
    }

    while (true) {
        ssize_t n = read(STDIN_FILENO, &ch, 1);
        if (n == 1) {
            if (ch == '\r') {
                continue;
            }
            if (ch == '\n') {
                line[pos] = '\0';
                bool has_line = pos > 0U;
                pos = 0U;
                return has_line;
            }
            if (pos < line_size - 1U) {
                line[pos++] = ch;
            } else {
                pos = 0U;
            }
            continue;
        }
        if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK &&
            errno != EINTR) {
            pos = 0U;
        }
        return false;
    }
}

static bool parse_speed_line(const char *line, double *vx, double *vz)
{
    double parsed_vx;
    double parsed_vz;

    if (line == NULL || vx == NULL || vz == NULL) {
        return false;
    }

    if (std::sscanf(line, " %lf , %lf", &parsed_vx, &parsed_vz) != 2) {
        return false;
    }

    *vx = parsed_vx;
    *vz = parsed_vz;
    return true;
}

static void set_stdin_nonblocking(void)
{
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (flags >= 0) {
        (void)fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    }
}

int main(void)
{
    ls2k0300_uart_t motor_uart;
    ArmMotorZero motor_zero;
    double theta2;
    double theta3;
    double target_vx = 0.0;
    double target_vz = 0.0;
    double motor2_degrees = 0.0;
    double motor3_degrees = 0.0;
    int64_t last_print_ms = 0;
    int read_fail_count = 0;
    bool stopped_by_read_fail = false;
    char line[64];

    setvbuf(stdout, NULL, _IOLBF, 0);

    if (install_signal_handlers() != 0) {
        return 1;
    }
    set_stdin_nonblocking();

    std::memset(&motor_uart, 0, sizeof(motor_uart));
    if (ls2k0300_uart_init(&motor_uart, ARM_MOTOR_UART, ARM_MOTOR_BAUD,
                           LS_UART_STOP1, LS_UART_DATA8,
                           LS_UART_PARITY_NONE,
                           LS_UART_MODE_NON_BLOCKING) != 0) {
        fprintf(stderr, "ERR init motor uart failed: %s\n", ARM_MOTOR_UART);
        return 1;
    }

    stop_motor23(&motor_uart);
    sleep_ms(100);

    if (!home_arm_motors_nearest(&motor_uart)) {
        fprintf(stderr, "ERR startup home failed\n");
        stop_motor23(&motor_uart);
        ls2k0300_uart_deinit(&motor_uart);
        return 1;
    }

    if (!read_motor23_degrees(&motor_uart,
                              &motor_zero.motor2_zero_degrees,
                              &motor_zero.motor3_zero_degrees)) {
        fprintf(stderr, "ERR startup zero read failed\n");
        stop_motor23(&motor_uart);
        ls2k0300_uart_deinit(&motor_uart);
        return 1;
    }

    theta2 = ARM_INITIAL_THETA2_RAD;
    theta3 = ARM_INITIAL_THETA3_RAD;
    if (!move_to_startup_pose(&motor_uart, &motor_zero, &theta2, &theta3)) {
        fprintf(stderr, "ERR move to startup pose failed\n");
        stop_motor23(&motor_uart);
        ls2k0300_uart_deinit(&motor_uart);
        return 1;
    }

    while (running && !shutdown_requested) {
        if (read_stdin_line_nonblocking(line, sizeof(line))) {
            double input_vx;
            double input_vz;

            if (parse_speed_line(line, &input_vx, &input_vz)) {
                target_vx = input_vx;
                target_vz = input_vz;
            } else {
                fprintf(stderr, "ERR input format, use vx,vz such as 10,20\n");
            }
        }

        if (!read_theta23_with_motor(&motor_uart,
                                     &motor_zero,
                                     &theta2,
                                     &theta3,
                                     &motor2_degrees,
                                     &motor3_degrees)) {
            read_fail_count++;
            fprintf(stderr, "ERR read theta failed (%d/%d)\n",
                    read_fail_count,
                    ARM_POSITION_READ_FAIL_STOP_COUNT);
            if (read_fail_count >= ARM_POSITION_READ_FAIL_STOP_COUNT &&
                !stopped_by_read_fail) {
                stop_motor23(&motor_uart);
                stopped_by_read_fail = true;
            }
            sleep_ms(ARM_READ_PERIOD_MS);
            continue;
        }
        read_fail_count = 0;
        stopped_by_read_fail = false;

        JointMotorSpeeds speeds;
        std::memset(&speeds, 0, sizeof(speeds));
        compute_motor_speeds(target_vx,
                             0.0,
                             target_vz,
                             ARM_INITIAL_THETA1_RAD,
                             theta2,
                             theta3,
                             ARM_GEAR_RATIO_1,
                             ARM_GEAR_RATIO_2,
                             ARM_GEAR_RATIO_3,
                             &speeds);
        if (!speeds.valid) {
            fprintf(stderr, "ERR velocity solve failed\n");
            stop_motor23(&motor_uart);
            sleep_ms(ARM_READ_PERIOD_MS);
            continue;
        }

        speeds.velocity_1.rpm = 0.0f;
        speeds.velocity_1.rpm_abs = 0.0f;
        speeds.velocity_1.dir = 0U;
        apply_motor_limit(&speeds);
        send_motor23_speeds(&motor_uart, &speeds);

        int64_t now_ms = monotonic_time_ms();
        if (now_ms - last_print_ms >= ARM_PRINT_PERIOD_MS) {
            point3d_t position;

            if (arm_forward_kinematics(ARM_INITIAL_THETA1_RAD,
                                       theta2,
                                       theta3,
                                       &position)) {
                printf("vx=%.2f vz=%.2f rpm2=%.2f rpm3=%.2f "
                       "model_x=%.2f model_y=%.2f model_z=%.2f "
                       "m2=%.2fdeg m3=%.2fdeg "
                       "theta2=%.2fdeg theta3=%.2fdeg\n",
                       target_vx,
                       target_vz,
                       speeds.velocity_2.rpm,
                       speeds.velocity_3.rpm,
                       (double)position.x,
                       (double)position.y,
                       (double)position.z,
                       motor2_degrees,
                       motor3_degrees,
                       theta2 * 180.0 / M_PI,
                       theta3 * 180.0 / M_PI);
            }
            last_print_ms = now_ms;
        }

        sleep_ms(ARM_READ_PERIOD_MS);
    }

    stop_motor23(&motor_uart);
    if (shutdown_requested && running) {
        if (!return_to_zero_pose(&motor_uart, &motor_zero, &theta2, &theta3)) {
            fprintf(stderr, "ERR return zero failed\n");
        }
    }
    stop_motor23(&motor_uart);
    ls2k0300_uart_deinit(&motor_uart);

    return 0;
}
