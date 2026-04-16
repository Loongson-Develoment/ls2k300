#define _POSIX_C_SOURCE 200809L

#include <stdio.h>
#include <time.h>

#include "LS2K0300_DRV_INC.h"

static volatile int g_timer_ticks = 0;

static void example_timer_cb(void *user_data)
{
    (void)user_data;
    g_timer_ticks++;
}

static void sleep_ms(uint32_t ms)
{
    struct timespec req;

    req.tv_sec = (time_t)(ms / 1000U);
    req.tv_nsec = (long)((ms % 1000U) * 1000000UL);
    (void)nanosleep(&req, NULL);
}

int main(void)
{
    ls2k0300_timer_t timer;
    int ret;

    g_timer_ticks = 0;

    if (ls2k0300_timer_init(&timer) != 0) {
        printf("[FAIL] timer init failed\n");
        return 1;
    }

    ret = ls2k0300_timer_set_seconds_ms(&timer, 20U, example_timer_cb, NULL);
    sleep_ms(120U);
    (void)ls2k0300_timer_stop(&timer);
    ls2k0300_timer_deinit(&timer);

    if (ret != 0 || g_timer_ticks <= 0) {
        printf("[FAIL] timer callback not invoked\n");
        return 1;
    }

    printf("[PASS] timer callback invoked, ticks=%d\n", g_timer_ticks);
    return 0;
}

