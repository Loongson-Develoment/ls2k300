#include <stdio.h>

#include "LS2K0300_DRV_INC.h"

int main(void)
{
    ls2k0300_soft_spi_t soft_spi;

    if (ls2k0300_soft_spi_init(&soft_spi, PIN_64, PIN_65, PIN_66, PIN_67, LS_SOFT_SPI_MODE_0) != 0) {
        printf("[FAIL] soft_spi init failed\n");
        return 1;
    }

    (void)ls2k0300_soft_spi_read_byte(&soft_spi, 0x00U);
    ls2k0300_soft_spi_deinit(&soft_spi);

    printf("[PASS] soft_spi init/read/deinit ok\n");
    return 0;
}

