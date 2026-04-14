#ifndef __LS2K0300_SPI_H
#define __LS2K0300_SPI_H

#include <stdint.h>
#include <sys/types.h>
#include <pthread.h>
#include <stdbool.h>
#include "LS2K0300_MAP.h"
#include "LS2K0300_GPIO.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    LS_SPI0 = 0x00,
    LS_SPI1,
    LS_SPI2,
    LS_SPI3,
    LS_SPI_INVALID
} ls_spi_port_t;

typedef enum {
    LS_SPI_MODE_0 = 0x00,
    LS_SPI_MODE_1,
    LS_SPI_MODE_2,
    LS_SPI_MODE_3,
    LS_SPI_MODE_INVALID
} ls_reg_spi_mode_t;

typedef enum {
    LS_SPI_BPW_4  = 4,
    LS_SPI_BPW_8  = 8,
    LS_SPI_BPW_16 = 16,
    LS_SPI_BPW_32 = 32,
    LS_SPI_BPW_INVALID
} ls_spi_bits_per_word_t;

typedef enum {
    LS_SPI_MSB_FIRST = 0x00,
    LS_SPI_LSB_FIRST = (1 << 7),
    LS_SPI_DATA_ORDER_INVALID
} ls_spi_data_order_t;

typedef enum {
    LS_SPI_FIFO_THRESH_1 = 0x00,
    LS_SPI_FIFO_THRESH_2 = 0x01,
    LS_SPI_FIFO_THRESH_4 = 0x03,
    LS_SPI_FIFO_THRESH_INVALID,
} ls_spi_fifo_threshould_t;

typedef struct {
    ls_spi_port_t port;
    uint32_t speed_hz;
    ls_reg_spi_mode_t spi_mode;
    ls_spi_bits_per_word_t bpw;
    ls_spi_data_order_t ord;
    ls_spi_fifo_threshould_t tx_thresh;
    ls_spi_fifo_threshould_t rx_thresh;

    gpio_pin_t cs_gpio;
    bool is_initialized;

    ls_reg32_addr_t spi_base;
    ls_reg32_addr_t spi_cr1;
    ls_reg32_addr_t spi_cr2;
    ls_reg32_addr_t spi_cr3;
    ls_reg32_addr_t spi_cr4;
    ls_reg32_addr_t spi_ier;
    ls_reg32_addr_t spi_sr1;
    ls_reg32_addr_t spi_sr2;
    ls_reg32_addr_t spi_cfg1;
    ls_reg32_addr_t spi_cfg2;
    ls_reg32_addr_t spi_cfg3;
    ls_reg32_addr_t spi_dr;

    pthread_mutex_t mtx;
    pthread_mutex_t mtx_transfer;
} ls2k0300_spi_t;

int ls2k0300_spi_init(ls2k0300_spi_t* spi, ls_spi_port_t port, uint32_t speed, ls_reg_spi_mode_t mode);
void ls2k0300_spi_deinit(ls2k0300_spi_t* spi);
void ls2k0300_spi_enable(ls2k0300_spi_t* spi);
void ls2k0300_spi_disable(ls2k0300_spi_t* spi);
ssize_t ls2k0300_spi_transfer(ls2k0300_spi_t* spi, const void* tx_buf, void* rx_buf, size_t len);

#ifdef __cplusplus
}
#endif

#endif // __LS2K0300_SPI_H
