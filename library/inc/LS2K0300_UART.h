#ifndef __LS2K0300_UART_H
#define __LS2K0300_UART_H

#include <stdint.h>
#include <stddef.h>
#include <unistd.h>
#include <termios.h>
#include <pthread.h>

#ifdef __cplusplus
extern "C" {
#endif

#define UART1 "/dev/ttyS1"

// 停止位枚举
typedef enum ls_uart_stop_bits {
    LS_UART_STOP1 = 0x00,
    LS_UART_STOP2,
} ls_uart_stop_bits_t;

// 数据位枚举
typedef enum ls_uart_data_bits {
    LS_UART_DATA5 = 0x00,
    LS_UART_DATA6,
    LS_UART_DATA7,
    LS_UART_DATA8,
} ls_uart_data_bits_t;

// 校验位枚举
typedef enum ls_uart_parity {
    LS_UART_PARITY_NONE = 0x00,
    LS_UART_PARITY_ODD,
    LS_UART_PARITY_EVEN,
} ls_uart_parity_t;

// UART句柄
typedef struct {
    int fd;
    struct termios ts;
    pthread_mutex_t mtx;
} ls2k0300_uart_t;

int ls2k0300_uart_init(ls2k0300_uart_t* uart, const char* path, speed_t baud, ls_uart_stop_bits_t stop, ls_uart_data_bits_t data, ls_uart_parity_t parity);
void ls2k0300_uart_deinit(ls2k0300_uart_t* uart);

ssize_t ls2k0300_uart_write(ls2k0300_uart_t* uart, const uint8_t* buf, size_t len);
ssize_t ls2k0300_uart_read(ls2k0300_uart_t* uart, uint8_t* buf, size_t len);
int ls2k0300_uart_flush(ls2k0300_uart_t* uart);

#ifdef __cplusplus
}
#endif

#endif // __LS2K0300_UART_H
