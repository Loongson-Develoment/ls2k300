#include <stdio.h>

#include "LS2K0300_DRV_INC.h"

int main(void)
{
    ls2k0300_i2c_t i2c;

    if (ls2k0300_i2c_init(&i2c, PIN_85, PIN_86, 0x50U) != 0) {
        printf("[FAIL] i2c init failed\n");
        return 1;
    }

    ls2k0300_i2c_deinit(&i2c);
    printf("[PASS] i2c init/deinit ok\n");
    return 0;
}

