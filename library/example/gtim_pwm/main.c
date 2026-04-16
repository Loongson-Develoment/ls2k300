#include <stdio.h>

#include "LS2K0300_DRV_INC.h"

int main(void)
{
    ls2k0300_gtim_pwm_t gtim_pwm;

    if (ls2k0300_gtim_pwm_init(&gtim_pwm, GTIM_PWM0_PIN87, 1000U, 5000U, GTIM_PWM_POL_NORMAL) != 0) {
        printf("[FAIL] gtim_pwm init failed\n");
        return 1;
    }

    ls2k0300_gtim_pwm_deinit(&gtim_pwm);
    printf("[PASS] gtim_pwm init/deinit ok\n");
    return 0;
}

