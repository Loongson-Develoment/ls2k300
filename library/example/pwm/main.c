#include <stdio.h>

#include "LS2K0300_DRV_INC.h"

int main(void)
{
    ls2k0300_pwm_t pwm;

    if (ls2k0300_pwm_init(&pwm, PWM0_PIN64, 1000U, 5000U, PWM_POL_NORMAL) != 0) {
        printf("[FAIL] pwm init failed\n");
        return 1;
    }

    ls2k0300_pwm_set_duty(&pwm, 2500U);
    ls2k0300_pwm_deinit(&pwm);

    printf("[PASS] pwm init/config/deinit ok\n");
    return 0;
}

