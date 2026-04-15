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

/********************************************************************************
 * @brief   UART 停止位枚举.
 ********************************************************************************/
typedef enum ls_uart_stop_bits {
    LS_UART_STOP1 = 0x00,
    LS_UART_STOP2,
} ls_uart_stop_bits_t;

/********************************************************************************
 * @brief   UART 数据位枚举.
 ********************************************************************************/
typedef enum ls_uart_data_bits {
    LS_UART_DATA5 = 0x00,
    LS_UART_DATA6,
    LS_UART_DATA7,
    LS_UART_DATA8,
} ls_uart_data_bits_t;

/********************************************************************************
 * @brief   UART 校验位枚举.
 ********************************************************************************/
typedef enum ls_uart_parity {
    LS_UART_PARITY_NONE = 0x00,
    LS_UART_PARITY_ODD,
    LS_UART_PARITY_EVEN,
} ls_uart_parity_t;

/********************************************************************************
 * @brief   UART 句柄结构体.
 ********************************************************************************/
typedef struct {
    int             fd;
    struct termios  ts;
    pthread_mutex_t mtx;
    int             initialized;
} ls2k0300_uart_t;

/********************************************************************************
 * @brief   初始化 UART 设备.
 * @param   uart   : UART 句柄.
 * @param   path   : 设备路径，例如 UART1.
 * @param   baud   : 波特率，例如 B115200.
 * @param   stop   : 停止位配置.
 * @param   data   : 数据位配置.
 * @param   parity : 校验位配置.
 * @return  成功返回 0，失败返回 -1.
 * @example ls2k0300_uart_t uart;
 *          ls2k0300_uart_init(&uart, UART1, B115200, LS_UART_STOP1, LS_UART_DATA8, LS_UART_PARITY_NONE);
 ********************************************************************************/
int ls2k0300_uart_init(ls2k0300_uart_t *uart, const char *path, speed_t baud,
                       ls_uart_stop_bits_t stop, ls_uart_data_bits_t data, ls_uart_parity_t parity);

/********************************************************************************
 * @brief   释放 UART 资源.
 * @param   uart : UART 句柄.
 * @return  none.
 * @example ls2k0300_uart_deinit(&uart);
 ********************************************************************************/
void ls2k0300_uart_deinit(ls2k0300_uart_t *uart);

/********************************************************************************
 * @brief   UART 发送数据.
 * @param   uart : UART 句柄.
 * @param   buf  : 待发送数据缓冲区.
 * @param   len  : 数据长度.
 * @return  成功返回写入字节数，失败返回 -1.
 * @example const uint8_t msg[] = "OK\r\n";
 *          ls2k0300_uart_write(&uart, msg, sizeof(msg)-1);
 ********************************************************************************/
ssize_t ls2k0300_uart_write(ls2k0300_uart_t *uart, const uint8_t *buf, size_t len);

/********************************************************************************
 * @brief   UART 接收数据.
 * @param   uart : UART 句柄.
 * @param   buf  : 接收缓冲区.
 * @param   len  : 最大读取长度.
 * @return  成功返回读取字节数，失败返回 -1.
 * @example uint8_t rx[64];
 *          ssize_t n = ls2k0300_uart_read(&uart, rx, sizeof(rx));
 ********************************************************************************/
ssize_t ls2k0300_uart_read(ls2k0300_uart_t *uart, uint8_t *buf, size_t len);

/********************************************************************************
 * @brief   清空 UART 输入输出缓冲区.
 * @param   uart : UART 句柄.
 * @return  成功返回 0，失败返回 -1.
 * @example ls2k0300_uart_flush(&uart);
 ********************************************************************************/
int ls2k0300_uart_flush(ls2k0300_uart_t *uart);

#ifdef __cplusplus
}
#endif

#endif
