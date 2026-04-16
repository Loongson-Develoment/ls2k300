#include <stdint.h>
#include <stdio.h>
#include <termios.h>

#include "LS2K0300_DRV_INC.h"

int main(void)
{
    ls2k0300_uart_t uart;
    static const uint8_t msg[] = "LS2K0300 uart example\r\n";
    ssize_t written;

    if (ls2k0300_uart_init(&uart, UART1, B115200, LS_UART_STOP1, LS_UART_DATA8, LS_UART_PARITY_NONE) != 0) {
        printf("[FAIL] uart init failed\n");
        return 1;
    }

    written = ls2k0300_uart_write(&uart, msg, sizeof(msg) - 1U);
    ls2k0300_uart_deinit(&uart);

    if (written < 0) {
        printf("[FAIL] uart write failed\n");
        return 1;
    }

    printf("[PASS] uart init/write/deinit ok\n");
    return 0;
}

