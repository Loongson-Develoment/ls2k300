#include <stdio.h>

#include "LS2K0300_DRV_INC.h"

int main(void)
{
    ls2k0300_atim_pwm_t atim_pwm;
    struct timespec ts = {0, 100 * 1000 * 1000}; /* 100ms */

    if (ls2k0300_atim_pwm_init(&atim_pwm, ATIM_PWM2_PIN83, 50, 0, ATIM_PWM_POL_NORMAL) != 0) {
        printf("[FAIL] atim_pwm init failed\n");
        return 1;
    }
    uint16_t duty = 0;
    for(int8_t i = 9; i >= 0; i--)
    {
        duty = i*1000;
        ls2k0300_atim_pwm_set_duty(&atim_pwm,duty);
        nanosleep(&ts,NULL);

    }
    for(uint8_t i = 1; i <= 10; i++)
    {
        duty = i*1000;
        ls2k0300_atim_pwm_set_duty(&atim_pwm,duty);
        nanosleep(&ts,NULL);
    }

    ls2k0300_atim_pwm_deinit(&atim_pwm);
    printf("[PASS] atim_pwm init/deinit ok\n");
    return 0;
}

