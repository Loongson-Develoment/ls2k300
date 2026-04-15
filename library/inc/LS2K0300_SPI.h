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

/********************************************************************************
 * @brief   SPI 端口枚举.
 ********************************************************************************/
typedef enum ls_spi_port {
    LS_SPI0 = 0x00,
    LS_SPI1,
    LS_SPI2,
    LS_SPI3,
    LS_SPI_INVALID,
} ls_spi_port_t;

/********************************************************************************
 * @brief   SPI 模式枚举.
 ********************************************************************************/
typedef enum ls_reg_spi_mode {
    LS_SPI_MODE_0 = 0x00,
    LS_SPI_MODE_1,
    LS_SPI_MODE_2,
    LS_SPI_MODE_3,
    LS_SPI_MODE_INVALID,
} ls_reg_spi_mode_t;

/********************************************************************************
 * @brief   SPI 数据位宽枚举.
 ********************************************************************************/
typedef enum ls_spi_bits_per_word {
    LS_SPI_BPW_4  = 4,
    LS_SPI_BPW_8  = 8,
    LS_SPI_BPW_16 = 16,
    LS_SPI_BPW_32 = 32,
    LS_SPI_BPW_INVALID,
} ls_spi_bits_per_word_t;

/********************************************************************************
 * @brief   SPI 数据顺序枚举.
 ********************************************************************************/
typedef enum ls_spi_data_order {
    LS_SPI_MSB_FIRST = 0x00,
    LS_SPI_LSB_FIRST = (1 << 7),
    LS_SPI_DATA_ORDER_INVALID,
} ls_spi_data_order_t;

/********************************************************************************
 * @brief   SPI FIFO 阈值枚举.
 ********************************************************************************/
typedef enum ls_spi_fifo_threshould {
    LS_SPI_FIFO_THRESH_1 = 0x00,
    LS_SPI_FIFO_THRESH_2 = 0x01,
    LS_SPI_FIFO_THRESH_4 = 0x03,
    LS_SPI_FIFO_THRESH_INVALID,
} ls_spi_fifo_threshould_t;

/********************************************************************************
 * @brief   SPI 句柄结构体.
 ********************************************************************************/
typedef struct {
    ls_spi_port_t            port;
    uint32_t                 speed_hz;
    ls_reg_spi_mode_t        spi_mode;
    ls_spi_bits_per_word_t   bpw;
    ls_spi_data_order_t      ord;
    ls_spi_fifo_threshould_t tx_thresh;
    ls_spi_fifo_threshould_t rx_thresh;

    ls2k0300_gpio_t          cs_gpio;
    int                      is_initialized;

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

/********************************************************************************
 * @brief   初始化 SPI 控制器.
 * @param   spi   : SPI 句柄指针.
 * @param   port  : SPI 端口（当前支持 LS_SPI2/LS_SPI3）.
 * @param   speed : 目标时钟频率(Hz).
 * @param   mode  : SPI 时序模式.
 * @return  成功返回 0，失败返回 -1.
 * @example ls2k0300_spi_init(&spi, LS_SPI2, 1000000, LS_SPI_MODE_0);
 ********************************************************************************/
int ls2k0300_spi_init(ls2k0300_spi_t *spi, ls_spi_port_t port, uint32_t speed, ls_reg_spi_mode_t mode);

/********************************************************************************
 * @brief   释放 SPI 控制器资源.
 * @param   spi : SPI 句柄指针.
 * @return  none.
 * @example ls2k0300_spi_deinit(&spi);
 ********************************************************************************/
void ls2k0300_spi_deinit(ls2k0300_spi_t *spi);

/********************************************************************************
 * @brief   使能 SPI 外设.
 * @param   spi : SPI 句柄指针.
 * @return  none.
 * @example ls2k0300_spi_enable(&spi);
 ********************************************************************************/
void ls2k0300_spi_enable(ls2k0300_spi_t *spi);

/********************************************************************************
 * @brief   禁用 SPI 外设.
 * @param   spi : SPI 句柄指针.
 * @return  none.
 * @example ls2k0300_spi_disable(&spi);
 ********************************************************************************/
void ls2k0300_spi_disable(ls2k0300_spi_t *spi);

/********************************************************************************
 * @brief   SPI 全双工传输.
 * @param   spi    : SPI 句柄指针.
 * @param   tx_buf : 发送缓冲区，可传 NULL 仅接收.
 * @param   rx_buf : 接收缓冲区，可传 NULL 仅发送.
 * @param   len    : 传输长度（字节）.
 * @return  成功返回 0，失败返回 -1.
 * @example ls2k0300_spi_transfer(&spi, tx, rx, sizeof(tx));
 ********************************************************************************/
ssize_t ls2k0300_spi_transfer(ls2k0300_spi_t *spi, const void *tx_buf, void *rx_buf, size_t len);

/********************************************************************************
 * @brief   SPI 发送数据.
 * @param   spi    : SPI 句柄指针.
 * @param   tx_buf : 发送缓冲区.
 * @param   len    : 发送长度（字节）.
 * @return  成功返回 0，失败返回 -1.
 * @example ls2k0300_spi_write(&spi, tx, tx_len);
 ********************************************************************************/
ssize_t ls2k0300_spi_write(ls2k0300_spi_t *spi, const void *tx_buf, size_t len);

/********************************************************************************
 * @brief   SPI 接收数据.
 * @param   spi    : SPI 句柄指针.
 * @param   rx_buf : 接收缓冲区.
 * @param   len    : 接收长度（字节）.
 * @return  成功返回 0，失败返回 -1.
 * @example ls2k0300_spi_read(&spi, rx, rx_len);
 ********************************************************************************/
ssize_t ls2k0300_spi_read(ls2k0300_spi_t *spi, void *rx_buf, size_t len);

/********************************************************************************
 * @brief   获取当前 SPI 实际速率.
 * @param   spi : SPI 句柄指针.
 * @return  成功返回速率(Hz)，失败返回 0.
 * @example uint32_t hz = ls2k0300_spi_get_speed(&spi);
 ********************************************************************************/
uint32_t ls2k0300_spi_get_speed(ls2k0300_spi_t *spi);

/********************************************************************************
 * @brief   获取当前 SPI 模式.
 * @param   spi : SPI 句柄指针.
 * @return  成功返回模式值，失败返回 LS_SPI_MODE_INVALID.
 * @example ls_reg_spi_mode_t mode = ls2k0300_spi_get_mode(&spi);
 ********************************************************************************/
ls_reg_spi_mode_t ls2k0300_spi_get_mode(ls2k0300_spi_t *spi);

/********************************************************************************
 * @brief   获取当前数据位宽.
 * @param   spi : SPI 句柄指针.
 * @return  成功返回位宽配置，失败返回 LS_SPI_BPW_INVALID.
 * @example ls_spi_bits_per_word_t bpw = ls2k0300_spi_get_bits_per_word(&spi);
 ********************************************************************************/
ls_spi_bits_per_word_t ls2k0300_spi_get_bits_per_word(ls2k0300_spi_t *spi);

/********************************************************************************
 * @brief   获取当前位序配置.
 * @param   spi : SPI 句柄指针.
 * @return  成功返回位序，失败返回 LS_SPI_DATA_ORDER_INVALID.
 * @example ls_spi_data_order_t ord = ls2k0300_spi_get_data_order(&spi);
 ********************************************************************************/
ls_spi_data_order_t ls2k0300_spi_get_data_order(ls2k0300_spi_t *spi);

/********************************************************************************
 * @brief   检查 SPI 状态是否异常.
 * @param   spi : SPI 句柄指针.
 * @return  正常返回 1，异常返回 0.
 * @example int ok = ls2k0300_spi_check_status(&spi);
 ********************************************************************************/
int ls2k0300_spi_check_status(ls2k0300_spi_t *spi);

#ifdef __cplusplus
}
#endif

#endif
