#include <stdio.h>

#include "LS2K0300_DRV_INC.h"

int main(void)
{
    ls2k0300_pwm_encoder_t enc;

    if (ls2k0300_pwm_encoder_init(&enc, ENC_PWM0_PIN64, PIN_72) != 0) {
        printf("[FAIL] encoder init failed\n");
        return 1;
    }

    (void)ls2k0300_pwm_encoder_get_count(&enc);
    ls2k0300_pwm_encoder_deinit(&enc);

    printf("[PASS] encoder init/read/deinit ok\n");
    return 0;
}

