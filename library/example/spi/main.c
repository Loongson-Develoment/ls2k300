#include <stdint.h>
#include <stdio.h>

#include "LS2K0300_DRV_INC.h"

int main(void)
{
    ls2k0300_spi_t spi;
    uint8_t tx[2] = {0xA5U, 0x5AU};
    uint8_t rx[2] = {0U, 0U};
    ssize_t ret;

    if (ls2k0300_spi_init(&spi, LS_SPI2, 1000000U, LS_SPI_MODE_0) != 0) {
        printf("[FAIL] spi init failed\n");
        return 1;
    }

    ret = ls2k0300_spi_transfer(&spi, tx, rx, sizeof(tx));
    ls2k0300_spi_deinit(&spi);

    if (ret != 0) {
        printf("[FAIL] spi transfer failed\n");
        return 1;
    }

    printf("[PASS] spi init/transfer/deinit ok\n");
    return 0;
}

