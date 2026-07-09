#include "All_control.h"

#include <cstdio>
#include <cstring>
#include <signal.h>

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

int main(void)
{
    int result = 0;

    setvbuf(stdout, NULL, _IOLBF, 0);

    if (install_signal_handlers() != 0) {
        return 1;
    }

    printf("uart=%s, rotate_addr=%u, speed=%.1f RPM, direction=%u, angle=%.1f deg\n",
           UART_INJECTION_PIN,
           (unsigned int)ROTATION_MOTOR_ADD,
           INJECT_DEFAULT_SPEED_RPM,
           (unsigned int)ROTATE_DEFAULT_DIRECTION,
           ROTATE_INIT_RELATIVE_DEGREES);

    All_control control;
    control.Inject_set_running_flag(&running);

    if (!control.Rotate_control()) {
        result = running ? 1 : 0;
        goto cleanup;
    }

    printf("rotate_position_arrived=%u\n",
           control.Is_rotate_position_arrived() ? 1U : 0U);

cleanup:
    control.Rotate_stop(ROTATE_DEFAULT_DIRECTION);
    return result;
}
