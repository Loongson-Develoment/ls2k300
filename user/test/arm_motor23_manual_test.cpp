#include "ArmMotorPosition.h"

#include "LS2K0300_UART.h"
#include "X42_V2.h"

#include <cerrno>
#include <csignal>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define ARM_MOTOR_UART UART4
#define ARM_MOTOR_BAUD B921600

#define ARM_MOTOR_ADDR_2 2U
#define ARM_MOTOR_ADDR_3 3U

/* robotArm geometry parameters.
 * robotArm defines the two planar arm angles as absolute link angles from +Z:
 *   low  = lower link angle from vertical up
 *   high = upper link angle from vertical up
 * A vertical-up link is 0deg, horizontal-out is 90deg, vertical-down is 180deg.
 * Set the initial angles to the real pose when this test starts.
 */
#define ARM_LINK_LENGTH_MM 120.0
#define ARM_GEAR_RATIO_2 (32.0 / 9.0)
#define ARM_GEAR_RATIO_3 (32.0 / 9.0)
#define ARM_RADPS_TO_RPM (60.0 / (2.0 * M_PI))

#define ARM_INITIAL_THETA2_RAD (90.0 * M_PI / 180.0)
#define ARM_INITIAL_THETA3_RAD (180.0 * M_PI / 180.0)
#define ARM_JOINT_DIRECTION_2 -1.0
#define ARM_JOINT_DIRECTION_3 1.0
#define ARM_ENDPOINT_X_DIRECTION 1.0
#define ARM_ENDPOINT_Z_DIRECTION 1.0

#define ARM_MAX_TEST_RPM 20.0f
#define ARM_VELOCITY_RAMP_RPMPS 20U
#define ARM_COMMAND_GAP_US 1000
#define ARM_PRINT_PERIOD_MS 300LL
#define ARM_POSITION_READ_TIMEOUT_MS 500
#define ARM_LOOP_SLEEP_US 30000
#define ARM_ROBOTARM_SINGULAR_EPS 1e-6
#define ARM_ROBOTARM_DLS_SIN_LINK_DELTA 0.20
#define ARM_ROBOTARM_DLS_LAMBDA_MM 30.0
#define ARM_AXIS_HOLD_INPUT_EPS_MMPS 0.01
#define ARM_AXIS_HOLD_KP 2.0
#define ARM_AXIS_HOLD_MAX_CORRECTION_MMPS 10.0

static volatile sig_atomic_t running = 1;
static struct termios saved_terminal;
static bool terminal_saved = false;

typedef struct {
    double motor2_zero_degrees;
    double motor3_zero_degrees;
} ArmMotorZero;

typedef struct {
    bool hold_x_valid;
    bool hold_z_valid;
    double hold_x_mm;
    double hold_z_mm;
} ArmAxisHold;

static void signal_handler(int signum)
{
    (void)signum;
    running = 0;
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

static float clamp_rpm(float rpm)
{
    if (rpm > ARM_MAX_TEST_RPM) {
        return ARM_MAX_TEST_RPM;
    }
    if (rpm < -ARM_MAX_TEST_RPM) {
        return -ARM_MAX_TEST_RPM;
    }
    return rpm;
}

static double clamp_double(double value, double min_value, double max_value)
{
    if (value > max_value) {
        return max_value;
    }
    if (value < min_value) {
        return min_value;
    }
    return value;
}

static void reset_axis_hold(ArmAxisHold *hold)
{
    if (hold == NULL) {
        return;
    }

    hold->hold_x_valid = false;
    hold->hold_z_valid = false;
    hold->hold_x_mm = 0.0;
    hold->hold_z_mm = 0.0;
}

static uint8_t dir_from_signed_rpm(float rpm)
{
    return (rpm >= 0.0f) ? 0U : 1U;
}

static bool read_motor23_degrees(ls2k0300_uart_t *uart,
                                 double *motor2_degrees,
                                 double *motor3_degrees)
{
    if (!arm_read_motor_position_degrees(uart,
                                         ARM_MOTOR_ADDR_2,
                                         motor2_degrees,
                                         ARM_POSITION_READ_TIMEOUT_MS)) {
        return false;
    }
    usleep(ARM_COMMAND_GAP_US);

    if (!arm_read_motor_position_degrees(uart,
                                         ARM_MOTOR_ADDR_3,
                                         motor3_degrees,
                                         ARM_POSITION_READ_TIMEOUT_MS)) {
        return false;
    }
    usleep(ARM_COMMAND_GAP_US);
    return true;
}

static bool read_theta23(ls2k0300_uart_t *uart,
                         const ArmMotorZero *zero,
                         double *theta2,
                         double *theta3,
                         double *motor2_degrees,
                         double *motor3_degrees)
{
    double m2;
    double m3;

    if (uart == NULL || zero == NULL || theta2 == NULL || theta3 == NULL) {
        return false;
    }

    if (!read_motor23_degrees(uart, &m2, &m3)) {
        return false;
    }

    *theta2 = ARM_INITIAL_THETA2_RAD +
              ARM_JOINT_DIRECTION_2 *
              ((m2 - zero->motor2_zero_degrees) * M_PI / 180.0) /
              ARM_GEAR_RATIO_2;
    *theta3 = ARM_INITIAL_THETA3_RAD +
              ARM_JOINT_DIRECTION_3 *
              ((m3 - zero->motor3_zero_degrees) * M_PI / 180.0) /
              ARM_GEAR_RATIO_3;

    if (motor2_degrees != NULL) {
        *motor2_degrees = m2;
    }
    if (motor3_degrees != NULL) {
        *motor3_degrees = m3;
    }

    return true;
}

static void robotarm_forward(double low_rad,
                             double high_rad,
                             double *x,
                             double *z)
{
    if (x != NULL) {
        *x = ARM_LINK_LENGTH_MM * std::sin(low_rad) +
             ARM_LINK_LENGTH_MM * std::sin(high_rad);
    }
    if (z != NULL) {
        *z = ARM_LINK_LENGTH_MM * std::cos(low_rad) +
             ARM_LINK_LENGTH_MM * std::cos(high_rad);
    }
}

static void apply_axis_hold(double input_vx,
                            double input_vz,
                            double x,
                            double z,
                            ArmAxisHold *hold,
                            double *solve_vx,
                            double *solve_vz)
{
    double corrected_vx = input_vx;
    double corrected_vz = input_vz;

    if (hold == NULL || solve_vx == NULL || solve_vz == NULL) {
        return;
    }

    if (std::fabs(input_vx) <= ARM_AXIS_HOLD_INPUT_EPS_MMPS &&
        std::fabs(input_vz) > ARM_AXIS_HOLD_INPUT_EPS_MMPS) {
        if (!hold->hold_x_valid) {
            hold->hold_x_mm = x;
            hold->hold_x_valid = true;
        }
        corrected_vx += clamp_double((hold->hold_x_mm - x) * ARM_AXIS_HOLD_KP,
                                     -ARM_AXIS_HOLD_MAX_CORRECTION_MMPS,
                                     ARM_AXIS_HOLD_MAX_CORRECTION_MMPS);
    } else {
        hold->hold_x_valid = false;
        hold->hold_x_mm = x;
    }

    if (std::fabs(input_vz) <= ARM_AXIS_HOLD_INPUT_EPS_MMPS &&
        std::fabs(input_vx) > ARM_AXIS_HOLD_INPUT_EPS_MMPS) {
        if (!hold->hold_z_valid) {
            hold->hold_z_mm = z;
            hold->hold_z_valid = true;
        }
        corrected_vz += clamp_double((hold->hold_z_mm - z) * ARM_AXIS_HOLD_KP,
                                     -ARM_AXIS_HOLD_MAX_CORRECTION_MMPS,
                                     ARM_AXIS_HOLD_MAX_CORRECTION_MMPS);
    } else {
        hold->hold_z_valid = false;
        hold->hold_z_mm = z;
    }

    if (std::fabs(input_vx) <= ARM_AXIS_HOLD_INPUT_EPS_MMPS &&
        std::fabs(input_vz) <= ARM_AXIS_HOLD_INPUT_EPS_MMPS) {
        reset_axis_hold(hold);
        corrected_vx = 0.0;
        corrected_vz = 0.0;
    }

    *solve_vx = corrected_vx;
    *solve_vz = corrected_vz;
}

static bool robotarm_velocity_to_link_radps(double vx,
                                            double vz,
                                            double low_rad,
                                            double high_rad,
                                            double *low_radps,
                                            double *high_radps)
{
    double j11 = ARM_LINK_LENGTH_MM * std::cos(low_rad);
    double j12 = ARM_LINK_LENGTH_MM * std::cos(high_rad);
    double j21 = -ARM_LINK_LENGTH_MM * std::sin(low_rad);
    double j22 = -ARM_LINK_LENGTH_MM * std::sin(high_rad);
    double det = j11 * j22 - j12 * j21;
    double link_delta_sin = std::sin(high_rad - low_rad);

    if (low_radps == NULL || high_radps == NULL) {
        return false;
    }

    if (std::fabs(link_delta_sin) >= ARM_ROBOTARM_DLS_SIN_LINK_DELTA &&
        std::fabs(det) >= ARM_ROBOTARM_SINGULAR_EPS) {
        *low_radps = (vx * j22 - j12 * vz) / det;
        *high_radps = (j11 * vz - vx * j21) / det;
        return true;
    }

    double lambda2 = ARM_ROBOTARM_DLS_LAMBDA_MM * ARM_ROBOTARM_DLS_LAMBDA_MM;
    double a = j11 * j11 + j12 * j12 + lambda2;
    double b = j11 * j21 + j12 * j22;
    double d = j21 * j21 + j22 * j22 + lambda2;
    double denom = a * d - b * b;

    if (std::fabs(denom) < ARM_ROBOTARM_SINGULAR_EPS) {
        return false;
    }

    double w1 = (d * vx - b * vz) / denom;
    double w2 = (-b * vx + a * vz) / denom;

    *low_radps = j11 * w1 + j21 * w2;
    *high_radps = j12 * w1 + j22 * w2;
    return true;
}

static bool robotarm_velocity_to_motor_rpm(double vx,
                                           double vz,
                                           double low_rad,
                                           double high_rad,
                                           float *rpm2,
                                           float *rpm3)
{
    double low_radps;
    double high_radps;
    double motor2_rpm;
    double motor3_rpm;

    if (rpm2 == NULL || rpm3 == NULL) {
        return false;
    }

    vx *= ARM_ENDPOINT_X_DIRECTION;
    vz *= ARM_ENDPOINT_Z_DIRECTION;

    if (!robotarm_velocity_to_link_radps(vx,
                                         vz,
                                         low_rad,
                                         high_rad,
                                         &low_radps,
                                         &high_radps)) {
        return false;
    }

    motor2_rpm = low_radps * ARM_GEAR_RATIO_2 * ARM_RADPS_TO_RPM /
                 ARM_JOINT_DIRECTION_2;
    motor3_rpm = high_radps * ARM_GEAR_RATIO_3 * ARM_RADPS_TO_RPM /
                 ARM_JOINT_DIRECTION_3;

    *rpm2 = clamp_rpm((float)motor2_rpm);
    *rpm3 = clamp_rpm((float)motor3_rpm);
    return true;
}

static void send_motor_speed(ls2k0300_uart_t *uart, uint8_t addr, float rpm)
{
    float limited_rpm = clamp_rpm(rpm);

    ZDT_X42_V2_Velocity_Control(uart,
                                addr,
                                dir_from_signed_rpm(limited_rpm),
                                ARM_VELOCITY_RAMP_RPMPS,
                                std::fabs(limited_rpm),
                                0);
    usleep(ARM_COMMAND_GAP_US);
}

static void send_motor23_speed(ls2k0300_uart_t *uart, float rpm2, float rpm3)
{
    send_motor_speed(uart, ARM_MOTOR_ADDR_2, rpm2);
    send_motor_speed(uart, ARM_MOTOR_ADDR_3, rpm3);
}

static void stop_motor23(ls2k0300_uart_t *uart)
{
    send_motor23_speed(uart, 0.0f, 0.0f);
}

static void set_stdin_nonblocking(void)
{
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (flags >= 0) {
        (void)fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    }
}

static void set_stdin_noecho(void)
{
    struct termios terminal;

    if (tcgetattr(STDIN_FILENO, &saved_terminal) != 0) {
        return;
    }

    terminal_saved = true;
    terminal = saved_terminal;
    terminal.c_lflag &= (tcflag_t)~ECHO;
    (void)tcsetattr(STDIN_FILENO, TCSANOW, &terminal);
}

static void restore_terminal(void)
{
    if (terminal_saved) {
        (void)tcsetattr(STDIN_FILENO, TCSANOW, &saved_terminal);
        terminal_saved = false;
    }
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

static bool parse_speed_pair(const char *line, double *vx, double *vz)
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

static bool parse_raw_rpm_command(const char *line, float *rpm2, float *rpm3)
{
    float parsed_rpm2;
    float parsed_rpm3;

    if (line == NULL || rpm2 == NULL || rpm3 == NULL) {
        return false;
    }

    if (std::sscanf(line, " m %f , %f", &parsed_rpm2, &parsed_rpm3) != 2 &&
        std::sscanf(line, " M %f , %f", &parsed_rpm2, &parsed_rpm3) != 2) {
        return false;
    }

    *rpm2 = clamp_rpm(parsed_rpm2);
    *rpm3 = clamp_rpm(parsed_rpm3);
    return true;
}

static void print_position(ls2k0300_uart_t *uart,
                           const ArmMotorZero *zero,
                           double input_vx,
                           double input_vz,
                           double solve_vx,
                           double solve_vz,
                           float rpm2,
                           float rpm3,
                           bool raw_motor_mode)
{
    double motor2_degrees;
    double motor3_degrees;
    double low_rad;
    double high_rad;
    double x;
    double z;
    double link_delta_deg;

    if (!read_theta23(uart,
                      zero,
                      &low_rad,
                      &high_rad,
                      &motor2_degrees,
                      &motor3_degrees)) {
        fprintf(stderr, "ERR read position failed\n");
        return;
    }

    robotarm_forward(low_rad, high_rad, &x, &z);
    link_delta_deg = (high_rad - low_rad) * 180.0 / M_PI;

    printf("%s input_vx=%.2f input_vz=%.2f solve_vx=%.2f solve_vz=%.2f "
           "rpm2=%.2f rpm3=%.2f x=%.2f z=%.2f "
           "low=%.2fdeg high=%.2fdeg delta=%.2fdeg "
           "motor2=%.2fdeg d2=%.2fdeg motor3=%.2fdeg d3=%.2fdeg\n",
           raw_motor_mode ? "raw" : "robot",
           input_vx,
           input_vz,
           solve_vx * ARM_ENDPOINT_X_DIRECTION,
           solve_vz * ARM_ENDPOINT_Z_DIRECTION,
           rpm2,
           rpm3,
           x,
           z,
           low_rad * 180.0 / M_PI,
           high_rad * 180.0 / M_PI,
           link_delta_deg,
           motor2_degrees,
           motor2_degrees - zero->motor2_zero_degrees,
           motor3_degrees,
           motor3_degrees - zero->motor3_zero_degrees);
}

int main(void)
{
    ls2k0300_uart_t uart;
    ArmMotorZero zero;
    ArmAxisHold axis_hold;
    double target_vx = 0.0;
    double target_vz = 0.0;
    double solve_vx = 0.0;
    double solve_vz = 0.0;
    float rpm2 = 0.0f;
    float rpm3 = 0.0f;
    int64_t last_print_ms = 0;
    bool raw_motor_mode = false;
    char line[64];

    setvbuf(stdout, NULL, _IOLBF, 0);

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    set_stdin_noecho();
    set_stdin_nonblocking();
    reset_axis_hold(&axis_hold);

    std::memset(&uart, 0, sizeof(uart));
    if (ls2k0300_uart_init(&uart,
                           ARM_MOTOR_UART,
                           ARM_MOTOR_BAUD,
                           LS_UART_STOP1,
                           LS_UART_DATA8,
                           LS_UART_PARITY_NONE,
                           LS_UART_MODE_NON_BLOCKING) != 0) {
        fprintf(stderr, "ERR init uart failed: %s\n", ARM_MOTOR_UART);
        restore_terminal();
        return 1;
    }

    printf("motor23 robotArm absolute-link velocity test: uart=%s, max rpm=%.1f\n",
           ARM_MOTOR_UART,
           ARM_MAX_TEST_RPM);
    printf("model: x=L*sin(low)+L*sin(high), z=L*cos(low)+L*cos(high)\n");
    printf("input: vx,vz  example: 10,0 move +x; 0,10 move +z; 0,0 stop\n");
    printf("raw motor input: m rpm2,rpm3  example: m -5,0 only motor2\n");
    printf("input: s stop, q quit\n");

    stop_motor23(&uart);
    usleep(100000);

    if (!read_motor23_degrees(&uart,
                              &zero.motor2_zero_degrees,
                              &zero.motor3_zero_degrees)) {
        fprintf(stderr, "ERR read startup motor position failed\n");
        stop_motor23(&uart);
        ls2k0300_uart_deinit(&uart);
        restore_terminal();
        return 1;
    }

    while (running) {
        if (read_stdin_line_nonblocking(line, sizeof(line))) {
            float raw_rpm2;
            float raw_rpm3;
            double input_vx;
            double input_vz;

            if (std::strcmp(line, "q") == 0 || std::strcmp(line, "Q") == 0) {
                break;
            }
            if (std::strcmp(line, "s") == 0 || std::strcmp(line, "S") == 0) {
                target_vx = 0.0;
                target_vz = 0.0;
                solve_vx = 0.0;
                solve_vz = 0.0;
                rpm2 = 0.0f;
                rpm3 = 0.0f;
                raw_motor_mode = false;
                reset_axis_hold(&axis_hold);
                send_motor23_speed(&uart, rpm2, rpm3);
            } else if (parse_raw_rpm_command(line, &raw_rpm2, &raw_rpm3)) {
                rpm2 = raw_rpm2;
                rpm3 = raw_rpm3;
                target_vx = 0.0;
                target_vz = 0.0;
                solve_vx = 0.0;
                solve_vz = 0.0;
                raw_motor_mode = true;
                reset_axis_hold(&axis_hold);
                send_motor23_speed(&uart, rpm2, rpm3);
            } else if (parse_speed_pair(line, &input_vx, &input_vz)) {
                target_vx = input_vx;
                target_vz = input_vz;
                solve_vx = input_vx;
                solve_vz = input_vz;
                raw_motor_mode = false;
                reset_axis_hold(&axis_hold);
            } else {
                fprintf(stderr, "ERR input format, use vx,vz or m rpm2,rpm3\n");
            }
        }

        if (!raw_motor_mode) {
            double low_rad;
            double high_rad;
            double x;
            double z;

            if (!read_theta23(&uart, &zero, &low_rad, &high_rad, NULL, NULL)) {
                fprintf(stderr, "ERR read low/high angle failed\n");
                stop_motor23(&uart);
                usleep(ARM_LOOP_SLEEP_US);
                continue;
            }

            robotarm_forward(low_rad, high_rad, &x, &z);
            apply_axis_hold(target_vx,
                            target_vz,
                            x,
                            z,
                            &axis_hold,
                            &solve_vx,
                            &solve_vz);

            if (!robotarm_velocity_to_motor_rpm(solve_vx,
                                                solve_vz,
                                                low_rad,
                                                high_rad,
                                                &rpm2,
                                                &rpm3)) {
                fprintf(stderr, "ERR robotArm velocity solve failed near singular pose\n");
                rpm2 = 0.0f;
                rpm3 = 0.0f;
            }
            send_motor23_speed(&uart, rpm2, rpm3);
        }

        int64_t now_ms = monotonic_time_ms();
        if (now_ms - last_print_ms >= ARM_PRINT_PERIOD_MS) {
            print_position(&uart,
                           &zero,
                           target_vx,
                           target_vz,
                           solve_vx,
                           solve_vz,
                           rpm2,
                           rpm3,
                           raw_motor_mode);
            last_print_ms = now_ms;
        }

        usleep(ARM_LOOP_SLEEP_US);
    }

    stop_motor23(&uart);
    ls2k0300_uart_deinit(&uart);
    restore_terminal();
    return 0;
}
