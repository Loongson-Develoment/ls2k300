#include <stdint.h>
#include <stdio.h>
#include <termios.h>

#include "LS2K0300_DRV_INC.h"

char rx_buf[20];
int running = 1;
void signal_handler(int signum) {
    running = 0;
}

int main(void)
{
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    ls2k0300_uart_t uart;

    if (ls2k0300_uart_init(&uart, UART1, B115200, LS_UART_STOP1, LS_UART_DATA8, LS_UART_PARITY_NONE, LS_UART_MODE_BLOCKING) != 0) {
        printf("[FAIL] uart init failed\n");
        return 1;
    }
    while(running){
        printf("[INFO] waiting for 1 char from UART1...\n");
        // ssize_t n = ls2k0300_uart_read_var(&uart, (uint8_t *)rx_buf, sizeof(rx_buf) - 1U, 1);
        // if (n > 0) {
        //     rx_buf[n] = '\0';
            ls2k0300_uart_write(&uart, "LD2 Togging", 11);
            sleep(1);

        // } else {
        //     printf("[FAIL] uart read failed\n");
        // }
    }


    ls2k0300_uart_deinit(&uart);
    printf("[PASS] uart deinit ok\n");
    return 0;
}
