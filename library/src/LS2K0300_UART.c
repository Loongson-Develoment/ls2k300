#include "LS2K0300_UART.h"
#include <stdio.h>
#include <string.h>
#include <fcntl.h>

int ls2k0300_uart_init(ls2k0300_uart_t* uart, const char* path, speed_t baud, ls_uart_stop_bits_t stop, ls_uart_data_bits_t data, ls_uart_parity_t parity) {
    if (uart == NULL || path == NULL) return -1;

    pthread_mutex_init(&uart->mtx, NULL);
    pthread_mutex_lock(&uart->mtx);

    uart->fd = open(path, O_RDWR | O_NOCTTY | O_NDELAY);
    if (uart->fd < 0) {
        printf("Open %s failed\n", path);
        pthread_mutex_unlock(&uart->mtx);
        return -1;
    }

    memset(&uart->ts, 0, sizeof(uart->ts));
    if (tcgetattr(uart->fd, &uart->ts) != 0) {
        printf("tcgetattr failed\n");
    }

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
        default: printf("Invalid data bits\n"); break;
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
        default: printf("Invalid parity\n"); break;
    }

    switch (stop) {
        case LS_UART_STOP1: uart->ts.c_cflag &= ~CSTOPB; break;
        case LS_UART_STOP2: uart->ts.c_cflag |= CSTOPB; break;
        default: printf("Invalid stop bits\n"); break;
    }

    uart->ts.c_cc[VMIN] = 0;
    uart->ts.c_cc[VTIME] = 5;

    if (tcsetattr(uart->fd, TCSANOW, &uart->ts) != 0) {
        printf("tcsetattr failed\n");
    }

    tcflush(uart->fd, TCIOFLUSH);
    pthread_mutex_unlock(&uart->mtx);
    return 0;
}

void ls2k0300_uart_deinit(ls2k0300_uart_t* uart) {
    if (uart == NULL) return;
    pthread_mutex_lock(&uart->mtx);
    if (uart->fd >= 0) {
        close(uart->fd);
        uart->fd = -1;
    }
    pthread_mutex_unlock(&uart->mtx);
    pthread_mutex_destroy(&uart->mtx);
}

ssize_t ls2k0300_uart_write(ls2k0300_uart_t* uart, const uint8_t* buf, size_t len) {
    if (uart == NULL || uart->fd < 0 || buf == NULL) return -1;
    pthread_mutex_lock(&uart->mtx);
    ssize_t ret = write(uart->fd, buf, len);
    pthread_mutex_unlock(&uart->mtx);
    return ret;
}

ssize_t ls2k0300_uart_read(ls2k0300_uart_t* uart, uint8_t* buf, size_t len) {
    if (uart == NULL || uart->fd < 0 || buf == NULL) return -1;
    pthread_mutex_lock(&uart->mtx);
    ssize_t ret = read(uart->fd, buf, len);
    pthread_mutex_unlock(&uart->mtx);
    return ret;
}

int ls2k0300_uart_flush(ls2k0300_uart_t* uart) {
    if (uart == NULL || uart->fd < 0) return -1;
    pthread_mutex_lock(&uart->mtx);
    int ret = tcflush(uart->fd, TCIOFLUSH);
    pthread_mutex_unlock(&uart->mtx);
    return ret;
}
