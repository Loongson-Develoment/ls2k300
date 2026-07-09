#include "ArmMotorPosition.h"
#include "ArmKinematics.h"

#include "LS2K0300_UART.h"
#include "X42_V2.h"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <ctime>

#define ARM_MOTOR_UART UART4
#define ARM_MOTOR_BAUD B921600

#define ARM_TEST_UPDATE_THETA_EACH_PERIOD 1

#define ARM_MOTOR_ADDR_2 2U
#define ARM_MOTOR_ADDR_3 3U

#define ARM_GEAR_RATIO_1 (32.0 / 9.0)
#define ARM_GEAR_RATIO_2 (32.0 / 9.0)
#define ARM_GEAR_RATIO_3 (32.0 / 9.0)

#define ARM_THETA1_RAD 0.0
#define ARM_THETA2_ZERO_RAD 0.0
#define ARM_THETA3_ZERO_RAD (140.0 * M_PI / 180.0)

#define ARM_JOINT_DIRECTION_2 -1.0
#define ARM_JOINT_DIRECTION_3 -1.0

#define ARM_TEST_VX_MMPS -10.0
#define ARM_TEST_VY_MMPS 0.0
#define ARM_TEST_VZ_MMPS 0.0
#define ARM_ENDPOINT_X_DIRECTION -1.0

#define ARM_TEST_MAX_MOTOR_RPM 30.0f
#define ARM_TEST_VELOCITY_RAMP_RPMPS 5U
#define ARM_TEST_SPEED_PERIOD_MS 300L
#define ARM_TEST_CORRECTION_PERIOD_MS 30L
#define ARM_TEST_JOINT_ERROR_GAIN 1.0
#define ARM_TEST_MAX_CORRECTION_RADPS 3.0

static volatile sig_atomic_t running = 1;

typedef struct {
    double motor2_zero_degrees;
    double motor3_zero_degrees;
} ArmMotorZero;

typedef struct {
    double theta2;
    double theta3;
    bool valid;
} ArmJointTarget;

static void signal_handler(int signum)
{
    (void)signum;
    running = 0;
}

static int install_signal_handlers(void)
{
    struct sigaction action;

    std::memset(&action, 0, sizeof(action));
    action.sa_handler = signal_handler;
    sigemptyset(&action.sa_mask);
    action.sa_flags = 0;

    if (sigaction(SIGINT, &action, NULL) != 0 ||
        sigaction(SIGTERM, &action, NULL) != 0) {
        perror("sigaction");
        return -1;
    }

    return 0;
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

static void zero_speed(MotorSpeed *speed)
{
    speed->rpm = 0.0f;
    speed->rpm_abs = 0.0f;
    speed->dir = 0U;
}

static int64_t monotonic_time_us(void)
{
    struct timespec now;

    if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) {
        return 0;
    }

    return (int64_t)now.tv_sec * 1000000LL +
           (int64_t)now.tv_nsec / 1000LL;
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

static bool read_motor23_degrees(ls2k0300_uart_t *uart,
                                 double *motor2_degrees,
                                 double *motor3_degrees)
{
    if (!arm_read_motor_position_degrees(uart, ARM_MOTOR_ADDR_2,
                                         motor2_degrees,
                                         ARM_MOTOR_POSITION_TIMEOUT_MS)) {
        printf("read motor2 0x36 failed\n");
        return false;
    }

    if (!arm_read_motor_position_degrees(uart, ARM_MOTOR_ADDR_3,
                                         motor3_degrees,
                                         ARM_MOTOR_POSITION_TIMEOUT_MS)) {
        printf("read motor3 0x36 failed\n");
        return false;
    }

    return true;
}

static bool read_theta23(ls2k0300_uart_t *uart, const ArmMotorZero *zero,
                         double *theta2, double *theta3, bool verbose)
{
    double motor2_degrees;
    double motor3_degrees;

    if (!read_motor23_degrees(uart, &motor2_degrees, &motor3_degrees)) {
        return false;
    }

    double motor2_delta = motor2_degrees - zero->motor2_zero_degrees;
    double motor3_delta = motor3_degrees - zero->motor3_zero_degrees;

    *theta2 = ARM_THETA2_ZERO_RAD +
              ARM_JOINT_DIRECTION_2 *
              (motor2_delta * M_PI / 180.0) / ARM_GEAR_RATIO_2;
    *theta3 = ARM_THETA3_ZERO_RAD +
              ARM_JOINT_DIRECTION_3 *
              (motor3_delta * M_PI / 180.0) / ARM_GEAR_RATIO_3;

    if (verbose) {
        printf("motor position: M2=%.2fdeg(delta=%.2f), M3=%.2fdeg(delta=%.2f)\n",
               motor2_degrees, motor2_delta, motor3_degrees, motor3_delta);
        printf("joint theta: theta1=%.2fdeg, theta2=%.2fdeg, theta3=%.2fdeg\n",
               ARM_THETA1_RAD * 180.0 / M_PI,
               *theta2 * 180.0 / M_PI,
               *theta3 * 180.0 / M_PI);
    }

    return true;
}

static void apply_motor_limit(JointMotorSpeeds *speeds)
{
    float max_rpm = std::max(speeds->velocity_2.rpm_abs,
                             speeds->velocity_3.rpm_abs);

    if (max_rpm <= ARM_TEST_MAX_MOTOR_RPM || max_rpm <= 0.0f) {
        return;
    }

    float scale = ARM_TEST_MAX_MOTOR_RPM / max_rpm;
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
                                ARM_TEST_VELOCITY_RAMP_RPMPS,
                                speeds->velocity_2.rpm_abs, 0);
    command_gap_ms(1);
    ZDT_X42_V2_Velocity_Control(uart, ARM_MOTOR_ADDR_3,
                                speeds->velocity_3.dir,
                                ARM_TEST_VELOCITY_RAMP_RPMPS,
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

static void print_motor_speeds(const JointMotorSpeeds *speeds)
{
    printf("endpoint speed: vx=%.2f, vy=%.2f, vz=%.2f mm/s\n",
           ARM_TEST_VX_MMPS, ARM_TEST_VY_MMPS, ARM_TEST_VZ_MMPS);
    printf("solved motor speed: "
           "M1(signed=%.2f, not sent), "
           "M2(dir=%u,rpm=%.2f,signed=%.2f), "
           "M3(dir=%u,rpm=%.2f,signed=%.2f)\n",
           speeds->velocity_1.rpm,
           (unsigned int)speeds->velocity_2.dir,
           speeds->velocity_2.rpm_abs,
           speeds->velocity_2.rpm,
           (unsigned int)speeds->velocity_3.dir,
           speeds->velocity_3.rpm_abs,
           speeds->velocity_3.rpm);
}

int main(void)
{
    ls2k0300_uart_t motor_uart;

    setvbuf(stdout, NULL, _IOLBF, 0);

    if (install_signal_handlers() != 0) {
        return 1;
    }

    std::memset(&motor_uart, 0, sizeof(motor_uart));
    if (ls2k0300_uart_init(&motor_uart, ARM_MOTOR_UART, ARM_MOTOR_BAUD,
                           LS_UART_STOP1, LS_UART_DATA8,
                           LS_UART_PARITY_NONE,
                           LS_UART_MODE_NON_BLOCKING) != 0) {
        printf("failed to init motor uart: %s\n", ARM_MOTOR_UART);
        return 1;
    }

    printf("arm x speed test: uart=%s, vx=%.2f, vy=%.2f, vz=%.2f mm/s, read/send addr=2/3\n",
           ARM_MOTOR_UART, ARM_TEST_VX_MMPS, ARM_TEST_VY_MMPS,
           ARM_TEST_VZ_MMPS);
    printf("press Ctrl+C to stop\n");

    stop_motor23(&motor_uart);
    sleep_ms(100);

    double theta2;
    double theta3;
    ArmMotorZero motor_zero;

    if (!read_motor23_degrees(&motor_uart,
                              &motor_zero.motor2_zero_degrees,
                              &motor_zero.motor3_zero_degrees)) {
        printf("startup motor zero read failed, stop motors\n");
        stop_motor23(&motor_uart);
        ls2k0300_uart_deinit(&motor_uart);
        return 1;
    }

    printf("startup motor zero: M2=%.2fdeg -> theta2=0.00deg, "
           "M3=%.2fdeg -> theta3=140.00deg\n",
           motor_zero.motor2_zero_degrees,
           motor_zero.motor3_zero_degrees);
    if (!read_theta23(&motor_uart, &motor_zero, &theta2, &theta3, true)) {
        printf("initial joint theta read failed, stop motors\n");
        stop_motor23(&motor_uart);
        ls2k0300_uart_deinit(&motor_uart);
        return 1;
    }

    ArmJointTarget joint_target;
    joint_target.theta2 = theta2;
    joint_target.theta3 = theta3;
    joint_target.valid = true;
    int64_t last_correction_us = monotonic_time_us();

#if ARM_TEST_UPDATE_THETA_EACH_PERIOD
    printf("theta update mode: acquire endpoint speed every %ld ms, "
           "correct motor speed every %ld ms\n",
           ARM_TEST_SPEED_PERIOD_MS, ARM_TEST_CORRECTION_PERIOD_MS);
#else
    printf("theta update mode: use startup theta for this speed command\n");
#endif

    while (running) {
        double target_vx = ARM_TEST_VX_MMPS;
        double target_vy = ARM_TEST_VY_MMPS;
        double target_vz = ARM_TEST_VZ_MMPS;
        int64_t speed_period_start_us = monotonic_time_us();
        bool verbose_this_speed = true;

        printf("target endpoint speed update: vx=%.2f, vy=%.2f, vz=%.2f mm/s\n",
               target_vx, target_vy, target_vz);

        while (running) {
            int64_t now_us = monotonic_time_us();
            if (now_us - speed_period_start_us >=
                ARM_TEST_SPEED_PERIOD_MS * 1000LL) {
                break;
            }

            double dt = (double)(now_us - last_correction_us) / 1000000.0;
            if (dt < 0.0) {
                dt = 0.0;
            }
            if (dt > 0.2) {
                dt = 0.2;
            }
            last_correction_us = now_us;

#if ARM_TEST_UPDATE_THETA_EACH_PERIOD
            if (!read_theta23(&motor_uart, &motor_zero, &theta2, &theta3,
                              verbose_this_speed)) {
                stop_motor23(&motor_uart);
                sleep_ms(ARM_TEST_CORRECTION_PERIOD_MS);
                continue;
            }
#endif

            JointMotorSpeeds target_speeds;
            std::memset(&target_speeds, 0, sizeof(target_speeds));
            compute_motor_speeds(ARM_ENDPOINT_X_DIRECTION * target_vx,
                                 target_vy,
                                 target_vz,
                                 ARM_THETA1_RAD,
                                 theta2,
                                 theta3,
                                 ARM_GEAR_RATIO_1,
                                 ARM_GEAR_RATIO_2,
                                 ARM_GEAR_RATIO_3,
                                 &target_speeds);

            if (!target_speeds.valid) {
                printf("arm velocity solve failed, stop motors\n");
                stop_motor23(&motor_uart);
                sleep_ms(ARM_TEST_CORRECTION_PERIOD_MS);
                continue;
            }

            double target_dtheta2 =
                arm_motor_rpm_to_joint_radps(target_speeds.velocity_2.rpm,
                                             ARM_GEAR_RATIO_2);
            double target_dtheta3 =
                arm_motor_rpm_to_joint_radps(target_speeds.velocity_3.rpm,
                                             ARM_GEAR_RATIO_3);

            if (!joint_target.valid) {
                joint_target.theta2 = theta2;
                joint_target.theta3 = theta3;
                joint_target.valid = true;
            }

            joint_target.theta2 += target_dtheta2 * dt;
            joint_target.theta3 += target_dtheta3 * dt;

            double error2 = theta2 - joint_target.theta2;
            double error3 = theta3 - joint_target.theta3;
            double correction2 = clamp_double(error2 * ARM_TEST_JOINT_ERROR_GAIN,
                                              -ARM_TEST_MAX_CORRECTION_RADPS,
                                              ARM_TEST_MAX_CORRECTION_RADPS);
            double correction3 = clamp_double(error3 * ARM_TEST_JOINT_ERROR_GAIN,
                                              -ARM_TEST_MAX_CORRECTION_RADPS,
                                              ARM_TEST_MAX_CORRECTION_RADPS);

            JointMotorSpeeds speeds = target_speeds;
            arm_motor_speed_from_joint_radps(&speeds.velocity_2,
                                             target_dtheta2 - correction2,
                                             ARM_GEAR_RATIO_2,
                                             1U);
            arm_motor_speed_from_joint_radps(&speeds.velocity_3,
                                             target_dtheta3 - correction3,
                                             ARM_GEAR_RATIO_3,
                                             1U);
            zero_speed(&speeds.velocity_1);
            apply_motor_limit(&speeds);

            send_motor23_speeds(&motor_uart, &speeds);
            if (verbose_this_speed) {
                print_motor_speeds(&speeds);
                printf("joint target: theta2=%.2fdeg, theta3=%.2fdeg, "
                       "ahead_error2=%.2fdeg, ahead_error3=%.2fdeg, dt=%.3fs\n",
                       joint_target.theta2 * 180.0 / M_PI,
                       joint_target.theta3 * 180.0 / M_PI,
                       error2 * 180.0 / M_PI,
                       error3 * 180.0 / M_PI,
                       dt);
                verbose_this_speed = false;
            }
#if ARM_TEST_UPDATE_THETA_EACH_PERIOD
            sleep_ms(ARM_TEST_CORRECTION_PERIOD_MS);
#else
            printf("speed command sent once, press Ctrl+C to stop\n");
            while (running) {
                sleep_ms(100);
            }
#endif
        }
    }

    printf("stopping motor2/3\n");
    stop_motor23(&motor_uart);
    ls2k0300_uart_deinit(&motor_uart);

    return 0;
}
