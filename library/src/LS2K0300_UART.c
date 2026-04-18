#include "LS2K0300_UART.h"

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <termios.h>

#ifndef CRTSCTS
#define CRTSCTS 020000000000 /* Flow control.  */
#endif

/********************************************************************************
 * @brief   初始化 UART 句柄.
 * @param   uart   : UART 句柄.
 * @param   path   : 设备路径.
 * @param   baud   : 波特率.
 * @param   stop   : 停止位.
 * @param   data   : 数据位.
 * @param   parity : 校验位.
 * @param   mode   : 模式配置.
 * @return  成功返回 0，失败返回 -1.
 * @example ls2k0300_uart_t uart;
 *          ls2k0300_uart_init(&uart, UART1, B115200, LS_UART_STOP1, LS_UART_DATA8, LS_UART_PARITY_NONE, LS_UART_MODE_BLOCKING);
 ********************************************************************************/
int ls2k0300_uart_init(ls2k0300_uart_t *uart, const char *path, speed_t baud,
                       ls_uart_stop_bits_t stop, ls_uart_data_bits_t data, ls_uart_parity_t parity, ls_uart_mode_t mode)
{
    if (uart == NULL || path == NULL) { 
        return -1;
    }

    memset(uart, 0, sizeof(*uart));
    uart->fd = -1;
    pthread_mutex_init(&uart->mtx, NULL);

    pthread_mutex_lock(&uart->mtx);

    /* 打开串口设备，使用非阻塞打开策略 */
    if (mode == LS_UART_MODE_NON_BLOCKING) {
        uart->fd = open(path, O_RDWR | O_NOCTTY | O_NDELAY);
    } else {
        uart->fd = open(path, O_RDWR | O_NOCTTY);
    }
    if (uart->fd < 0) {
        printf("Open %s failed\n", path);
        pthread_mutex_unlock(&uart->mtx);
        pthread_mutex_destroy(&uart->mtx);
        return -1;
    }

    if (tcgetattr(uart->fd, &uart->ts) != 0) {
        printf("tcgetattr failed\n");
    }

    /* 配置基础串口属性：原始模式、禁用软硬件流控 */
    uart->ts.c_cflag |= CREAD | CLOCAL;
    uart->ts.c_cflag &= ~CRTSCTS;
    uart->ts.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    uart->ts.c_iflag &= ~(IXON | IXOFF | IXANY);
    uart->ts.c_oflag &= ~OPOST;

    cfsetispeed(&uart->ts, baud);
    cfsetospeed(&uart->ts, baud);

    uart->ts.c_cflag &= ~CSIZE;
    switch (data) {
        case LS_UART_DATA5: uart->ts.c_cflag |= CS5; break;
        case LS_UART_DATA6: uart->ts.c_cflag |= CS6; break;
        case LS_UART_DATA7: uart->ts.c_cflag |= CS7; break;
        case LS_UART_DATA8: uart->ts.c_cflag |= CS8; break;
        default:            uart->ts.c_cflag |= CS8; break;
    }

    switch (parity) {
        case LS_UART_PARITY_NONE:
            uart->ts.c_cflag &= ~PARENB;
            break;
        case LS_UART_PARITY_ODD:
            uart->ts.c_cflag |= PARENB;
            uart->ts.c_cflag |= PARODD;
            break;
        case LS_UART_PARITY_EVEN:
            uart->ts.c_cflag |= PARENB;
            uart->ts.c_cflag &= ~PARODD;
            break;
        default:
            uart->ts.c_cflag &= ~PARENB;
            break;
    }

    switch (stop) {
        case LS_UART_STOP1: uart->ts.c_cflag &= ~CSTOPB; break;
        case LS_UART_STOP2: uart->ts.c_cflag |= CSTOPB; break;
        default:            uart->ts.c_cflag &= ~CSTOPB; break;
    }

    if (mode == LS_UART_MODE_NON_BLOCKING) {
        uart->ts.c_cc[VMIN] = 0;
    } else {
        uart->ts.c_cc[VMIN] = 1;
    }
    uart->ts.c_cc[VTIME] = 0;


    if (tcsetattr(uart->fd, TCSANOW, &uart->ts) != 0) {
        printf("tcsetattr failed\n");
    }

    /* 清空历史缓存，避免旧数据干扰 */
    tcflush(uart->fd, TCIOFLUSH);
    uart->initialized = 1;

    pthread_mutex_unlock(&uart->mtx);
    return 0;
}

/********************************************************************************
 * @brief   初始化 阻塞UART 句柄.
 * @param   uart   : UART 句柄.
 * @param   path   : 设备路径.
 * @param   baud   : 波特率.
 * @param   stop   : 停止位.
 * @param   data   : 数据位.
 * @param   parity : 校验位.
 * @param   char_num : 需要读取的字符数量.
 * @return  成功返回 0，失败返回 -1.
 * @example ls2k0300_uart_t uart;
 *          ls2k0300_uart_init(&uart, UART1, B115200, LS_UART_STOP1, LS_UART_DATA8, LS_UART_PARITY_NONE，5);
 ********************************************************************************/
int ls2k0300_uart_block_init(ls2k0300_uart_t *uart, const char *path, speed_t baud,
                               ls_uart_stop_bits_t stop, ls_uart_data_bits_t data, ls_uart_parity_t parity, uint32_t char_num)
{
    int ret = ls2k0300_uart_init(uart, path, baud, stop, data, parity, LS_UART_MODE_BLOCKING);
    if (ret != 0) {
        return ret;
    }

    /* 切换到阻塞模式 */
    fcntl(uart->fd, F_SETFL, fcntl(uart->fd, F_GETFL) & ~O_NDELAY);
    uart->ts.c_cc[VMIN] = char_num;
    if (tcsetattr(uart->fd, TCSANOW, &uart->ts) != 0) {
    printf("tcsetattr failed\n");
    }
    return 0;
}

/********************************************************************************
 * @brief   释放 UART 句柄.
 * @param   uart : UART 句柄.
 * @return  none.
 * @example ls2k0300_uart_deinit(&uart);
 ********************************************************************************/
void ls2k0300_uart_deinit(ls2k0300_uart_t *uart)
{
    if (uart == NULL) {
        return;
    }

    pthread_mutex_lock(&uart->mtx);

    if (uart->fd >= 0) {
        close(uart->fd);
        uart->fd = -1;
    }
    uart->initialized = 0;

    pthread_mutex_unlock(&uart->mtx);
    pthread_mutex_destroy(&uart->mtx);
}

/********************************************************************************
 * @brief   UART 发送函数.
 * @param   uart : UART 句柄.
 * @param   buf  : 待发送数据.
 * @param   len  : 发送长度.
 * @return  成功返回写入字节数，失败返回 -1.
 * @example const uint8_t msg[] = "OK\r\n";
 *          ls2k0300_uart_write(&uart, msg, sizeof(msg)-1);
 ********************************************************************************/
ssize_t ls2k0300_uart_write(ls2k0300_uart_t *uart, const uint8_t *buf, size_t len)
{
    ssize_t ret;

    if (uart == NULL || uart->initialized == 0 || uart->fd < 0 || buf == NULL || len == 0U) {
        return -1;
    }

    pthread_mutex_lock(&uart->mtx);
    ret = write(uart->fd, buf, len);
    pthread_mutex_unlock(&uart->mtx);

    return ret;
}

/********************************************************************************
 * @brief   UART 接收函数.
 * @param   uart : UART 句柄.
 * @param   buf  : 接收缓存.
 * @param   len  : 最大读取长度.
 * @return  成功返回读取字节数，失败返回 -1.
 * @example uint8_t rx[64];
 *          ssize_t n = ls2k0300_uart_read(&uart, rx, sizeof(rx));
 ********************************************************************************/
ssize_t ls2k0300_uart_read(ls2k0300_uart_t *uart, uint8_t *buf, size_t len)
{
    ssize_t ret;

    if (uart == NULL || uart->initialized == 0 || uart->fd < 0 || buf == NULL || len == 0U) {
        return -1;
    }

    pthread_mutex_lock(&uart->mtx);
    ret = read(uart->fd, buf, len);
    pthread_mutex_unlock(&uart->mtx);

    return ret;
}

/********************************************************************************
 * @brief   UART 不定长接收（字节间超时分帧，毫秒级）.
 * @param   uart : UART 句柄.
 * @param   buf  : 接收缓存.
 * @param   len  : 最大读取长度.
 * @param   inter_timeout_ms : 字节间超时（毫秒）.
 * @return  成功返回读取字节数，失败返回 -1.
 * @note    首字节阻塞等待，后续字节采用 poll 毫秒级超时分帧.
 ********************************************************************************/
ssize_t ls2k0300_uart_read_var(ls2k0300_uart_t *uart, uint8_t *buf, size_t len,
                               int inter_timeout_ms)
{
    size_t total = 0U;
    struct pollfd pfd;

    if (uart == NULL || uart->initialized == 0 || uart->fd < 0 || buf == NULL || len == 0U ||
        inter_timeout_ms < 0) {
        return -1;
    }

    pfd.fd = uart->fd;
    pfd.events = POLLIN;
    pfd.revents = 0;

    while (total < len) {
        ssize_t n;
        int timeout_ms = (total == 0U) ? -1 : inter_timeout_ms;
        int poll_ret = poll(&pfd, 1, timeout_ms);

        if (poll_ret == 0) {
            break;
        }

        if (poll_ret < 0) {
            if (errno == EINTR) {
                continue;
            }
            return -1;
        }

        if ((pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
            return (total > 0U) ? (ssize_t)total : -1;
        }

        if ((pfd.revents & POLLIN) == 0) {
            continue;
        }

        n = ls2k0300_uart_read(uart, buf + total, len - total);

        if (n > 0) {
            total += (size_t)n;
            continue;
        }

        if (n == 0) {
            continue;
        }

        if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
            continue;
        }

        return -1;
    }

    return (ssize_t)total;
}

/********************************************************************************
 * @brief   清空 UART 缓冲区.
 * @param   uart : UART 句柄.
 * @return  成功返回 0，失败返回 -1.
 * @example ls2k0300_uart_flush(&uart);
 ********************************************************************************/
int ls2k0300_uart_flush(ls2k0300_uart_t *uart)
{
    int ret;

    if (uart == NULL || uart->initialized == 0 || uart->fd < 0) {
        return -1;
    }

    pthread_mutex_lock(&uart->mtx);
    ret = tcflush(uart->fd, TCIOFLUSH);
    pthread_mutex_unlock(&uart->mtx);

    return ret;
}
