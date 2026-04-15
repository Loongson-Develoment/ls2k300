#ifndef __LS2K0300_SOFT_SPI_H
#define __LS2K0300_SOFT_SPI_H

#include <stdint.h>
#include "LS2K0300_GPIO.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LS_SOFT_SPI_MAX_TRANS_LEN  (256)

/********************************************************************************
 * @brief   软件 SPI 模式枚举.
 ********************************************************************************/
typedef enum ls2k0300_soft_spi_mode {
    LS_SOFT_SPI_MODE_0 = 0x00,
    LS_SOFT_SPI_MODE_1,
    LS_SOFT_SPI_MODE_2,
    LS_SOFT_SPI_MODE_3,
} ls2k0300_soft_spi_mode_t;

/********************************************************************************
 * @brief   软件 SPI 句柄结构体.
 ********************************************************************************/
typedef struct {
    ls2k0300_gpio_t           sck;
    ls2k0300_gpio_t           miso;
    ls2k0300_gpio_t           mosi;
    ls2k0300_gpio_t           cs;
    ls2k0300_soft_spi_mode_t  mode;
    int                       initialized;
} ls2k0300_soft_spi_t;

/********************************************************************************
 * @brief   初始化软件 SPI.
 * @param   spi  : 软件 SPI 句柄指针.
 * @param   sck  : 时钟引脚.
 * @param   miso : 输入引脚.
 * @param   mosi : 输出引脚.
 * @param   cs   : 片选引脚.
 * @param   mode : SPI 模式（0~3）.
 * @return  成功返回 0，失败返回 -1.
 * @example ls2k0300_soft_spi_init(&spi, PIN_64, PIN_65, PIN_66, PIN_67, LS_SOFT_SPI_MODE_0);
 ********************************************************************************/
int ls2k0300_soft_spi_init(ls2k0300_soft_spi_t *spi,
                           gpio_pin_t sck,
                           gpio_pin_t miso,
                           gpio_pin_t mosi,
                           gpio_pin_t cs,
                           ls2k0300_soft_spi_mode_t mode);

/********************************************************************************
 * @brief   释放软件 SPI.
 * @param   spi : 软件 SPI 句柄指针.
 * @return  none.
 * @example ls2k0300_soft_spi_deinit(&spi);
 ********************************************************************************/
void ls2k0300_soft_spi_deinit(ls2k0300_soft_spi_t *spi);

/********************************************************************************
 * @brief   读取寄存器单字节.
 * @param   spi : 软件 SPI 句柄指针.
 * @param   reg : 目标寄存器地址.
 * @return  成功返回读取字节，失败返回 0.
 * @example uint8_t id = ls2k0300_soft_spi_read_byte(&spi, 0x0F);
 ********************************************************************************/
uint8_t ls2k0300_soft_spi_read_byte(ls2k0300_soft_spi_t *spi, uint8_t reg);

/********************************************************************************
 * @brief   连续读取寄存器数据.
 * @param   spi : 软件 SPI 句柄指针.
 * @param   reg : 起始寄存器地址.
 * @param   buf : 输出缓冲区.
 * @param   len : 读取长度.
 * @return  none.
 * @example ls2k0300_soft_spi_read_n_byte(&spi, 0x20, rx, 6);
 ********************************************************************************/
void ls2k0300_soft_spi_read_n_byte(ls2k0300_soft_spi_t *spi, uint8_t reg, uint8_t *buf, uint16_t len);

/********************************************************************************
 * @brief   写寄存器单字节.
 * @param   spi : 软件 SPI 句柄指针.
 * @param   reg : 目标寄存器地址.
 * @param   val : 要写入的数据.
 * @return  none.
 * @example ls2k0300_soft_spi_write_byte(&spi, 0x20, 0x57);
 ********************************************************************************/
void ls2k0300_soft_spi_write_byte(ls2k0300_soft_spi_t *spi, uint8_t reg, uint8_t val);

/********************************************************************************
 * @brief   连续写寄存器数据.
 * @param   spi : 软件 SPI 句柄指针.
 * @param   reg : 起始寄存器地址.
 * @param   buf : 输入数据缓冲区.
 * @param   len : 写入长度.
 * @return  none.
 * @example ls2k0300_soft_spi_write_n_byte(&spi, 0x20, tx, 6);
 ********************************************************************************/
void ls2k0300_soft_spi_write_n_byte(ls2k0300_soft_spi_t *spi, uint8_t reg, const uint8_t *buf, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif
