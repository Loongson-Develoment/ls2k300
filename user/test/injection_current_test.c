#include "All_control.h"

#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <signal.h>

#define DEFAULT_SPEED_RPM  INJECT_DEFAULT_SPEED_RPM
#define DEFAULT_DIRECTION  0U

static volatile sig_atomic_t running = 1;

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

static int parse_speed(const char *text, float *speed_rpm)
{
    char *end = NULL;
    float value = std::strtof(text, &end);

    if (end == text || *end != '\0' || value < 0.0f || value > 4000.0f) {
        return -1;
    }

    *speed_rpm = value;
    return 0;
}

static int parse_direction(const char *text, uint8_t *direction)
{
    char *end = NULL;
    unsigned long value = std::strtoul(text, &end, 10);

    if (end == text || *end != '\0' || value > 1UL) {
        return -1;
    }

    *direction = (uint8_t)value;
    return 0;
}

static int parse_relative_units(const char *text, int32_t *relative_units)
{
    char *end = NULL;
    long value;

    errno = 0;
    value = std::strtol(text, &end, 10);
    while (*end == ' ' || *end == '\t' || *end == '\r' || *end == '\n') {
        ++end;
    }

    if (end == text || *end != '\0' || errno != 0 ||
        value < (long)std::numeric_limits<int32_t>::min() ||
        value > (long)std::numeric_limits<int32_t>::max()) {
        return -1;
    }

    *relative_units = (int32_t)value;
    return 0;
}

static int run_terminal_relative_position_loop(All_control *control,
                                               uint8_t default_direction,
                                               float speed_rpm)
{
    char line[64];

    printf("enter relative position raw units, q to quit\n");
    while (running) {
        int32_t relative_units;

        printf("> ");
        fflush(stdout);
        if (fgets(line, sizeof(line), stdin) == NULL) {
            if (!running) {
                break;
            }
            clearerr(stdin);
            continue;
        }

        if (line[0] == 'q' || line[0] == 'Q') {
            break;
        }

        if (parse_relative_units(line, &relative_units) != 0) {
            printf("invalid input, enter integer raw units or q\n");
            continue;
        }

        if (!control->Inject_move_relative(relative_units, default_direction,
                                           speed_rpm)) {
            return running ? -1 : 0;
        }
        printf("inject_position_arrived=%u\n",
               control->Is_inject_position_arrived() ? 1U : 0U);
    }

    return 0;
}

int main(int argc, char *argv[])
{
    float speed_rpm = DEFAULT_SPEED_RPM;
    uint8_t direction = DEFAULT_DIRECTION;
    int result = 0;

    setvbuf(stdout, NULL, _IOLBF, 0);

    if (argc > 3 || (argc >= 2 && parse_speed(argv[1], &speed_rpm) != 0) ||
        (argc == 3 && parse_direction(argv[2], &direction) != 0)) {
        fprintf(stderr, "usage: %s [speed_rpm 0..4000] [direction 0|1]\n",
                argv[0]);
        return 1;
    }

    if (install_signal_handlers() != 0) {
        return 1;
    }

    printf("uart=%s, inject_addr=%u, rotate_addr=%u, speed=%.1f RPM, direction=%u\n",
           UART_INJECTION_PIN, (unsigned int)INJECTION_MOTOR_ADD,
           (unsigned int)ROTATION_MOTOR_ADD,
           speed_rpm, (unsigned int)direction);

    All_control control;
    control.Inject_set_running_flag(&running);

    if (!control.Rotate_control()) {
        result = running ? 1 : 0;
        goto cleanup;
    }

    if (!control.Inject_home_to_zero()) {
        result = running ? 1 : 0;
        goto cleanup;
    }

    if (!control.Inject_run_until_contact(direction, speed_rpm)) {
        result = running ? 1 : 0;
        goto cleanup;
    }

    printf("contact=%u, average=%.3f, baseline_delta=%.3f\n",
           control.Is_contact() ? 1U : 0U,
           control.Inject_current_average(),
           control.Inject_current_baseline_delta());

    if (run_terminal_relative_position_loop(&control, direction,
                                            speed_rpm) != 0) {
        result = 1;
    }

cleanup:
    control.Rotate_stop(ROTATE_DEFAULT_DIRECTION);
    control.Inject_stop(direction);
    return result;
}
