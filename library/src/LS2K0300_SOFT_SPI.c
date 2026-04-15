#include "LS2K0300_SOFT_SPI.h"

#include <string.h>

/********************************************************************************
 * @brief   软件 SPI 收发内部函数.
 * @param   spi : 软件 SPI 句柄指针.
 * @param   buf : 收发缓冲区（原地覆盖接收数据）.
 * @param   len : 收发长度（字节）.
 * @return  none.
 * @note    该函数内部自动拉低/拉高片选信号.
 ********************************************************************************/
static void soft_spi_read_write_n_byte(ls2k0300_soft_spi_t *spi, uint8_t *buf, uint16_t len)
{
    uint8_t i;

    if (spi == NULL || buf == NULL || len == 0U || spi->initialized == 0 || spi->mode > LS_SOFT_SPI_MODE_3) {
        return;
    }

    /* 一次事务开始: CS 拉低并根据模式准备空闲时钟电平 */
    ls2k0300_gpio_level_set(&spi->cs, GPIO_LOW);

    if (spi->mode == LS_SOFT_SPI_MODE_0 || spi->mode == LS_SOFT_SPI_MODE_1) {
        ls2k0300_gpio_level_set(&spi->sck, GPIO_LOW);
    } else {
        ls2k0300_gpio_level_set(&spi->sck, GPIO_HIGH);
    }

    do {
        for (i = 0; i < 8U; i++) {
            if ((*buf & 0x80U) != 0U) {
                ls2k0300_gpio_level_set(&spi->mosi, GPIO_HIGH);
            } else {
                ls2k0300_gpio_level_set(&spi->mosi, GPIO_LOW);
            }

            /* 按 CPOL/CPHA 对应模式驱动时钟并在采样点读取 MISO */
            if (spi->mode == LS_SOFT_SPI_MODE_0) {
                ls2k0300_gpio_level_set(&spi->sck, GPIO_HIGH);
                *buf <<= 1;
                *buf |= (uint8_t)ls2k0300_gpio_level_get(&spi->miso);
                ls2k0300_gpio_level_set(&spi->sck, GPIO_LOW);
            } else if (spi->mode == LS_SOFT_SPI_MODE_1) {
                ls2k0300_gpio_level_set(&spi->sck, GPIO_HIGH);
                *buf <<= 1;
                ls2k0300_gpio_level_set(&spi->sck, GPIO_LOW);
                *buf |= (uint8_t)ls2k0300_gpio_level_get(&spi->miso);
            } else if (spi->mode == LS_SOFT_SPI_MODE_2) {
                ls2k0300_gpio_level_set(&spi->sck, GPIO_LOW);
                *buf <<= 1;
                *buf |= (uint8_t)ls2k0300_gpio_level_get(&spi->miso);
                ls2k0300_gpio_level_set(&spi->sck, GPIO_HIGH);
            } else {
                ls2k0300_gpio_level_set(&spi->sck, GPIO_LOW);
                *buf <<= 1;
                ls2k0300_gpio_level_set(&spi->sck, GPIO_HIGH);
                *buf |= (uint8_t)ls2k0300_gpio_level_get(&spi->miso);
            }
        }

        buf++;
    } while (--len > 0U);

    /* 一次事务结束: CS 拉高 */
    ls2k0300_gpio_level_set(&spi->cs, GPIO_HIGH);
}

/********************************************************************************
 * @brief   初始化软件 SPI.
 * @param   spi  : 软件 SPI 句柄指针.
 * @param   sck  : SCK 引脚.
 * @param   miso : MISO 引脚.
 * @param   mosi : MOSI 引脚.
 * @param   cs   : CS 引脚.
 * @param   mode : SPI 模式.
 * @return  成功返回 0，失败返回 -1.
 * @example ls2k0300_soft_spi_init(&spi, PIN_64, PIN_65, PIN_66, PIN_67, LS_SOFT_SPI_MODE_0);
 ********************************************************************************/
int ls2k0300_soft_spi_init(ls2k0300_soft_spi_t *spi,
                           gpio_pin_t sck,
                           gpio_pin_t miso,
                           gpio_pin_t mosi,
                           gpio_pin_t cs,
                           ls2k0300_soft_spi_mode_t mode)
{
    if (spi == NULL || mode > LS_SOFT_SPI_MODE_3) {
        return -1;
    }

    /* 四个 GPIO 任一初始化失败都回滚前序资源 */
    memset(spi, 0, sizeof(*spi));

    if (ls2k0300_gpio_init(&spi->sck, sck, GPIO_MODE_OUT, GPIO_MUX_GPIO) != 0) {
        return -1;
    }
    if (ls2k0300_gpio_init(&spi->miso, miso, GPIO_MODE_IN, GPIO_MUX_GPIO) != 0) {
        ls2k0300_gpio_deinit(&spi->sck);
        return -1;
    }
    if (ls2k0300_gpio_init(&spi->mosi, mosi, GPIO_MODE_OUT, GPIO_MUX_GPIO) != 0) {
        ls2k0300_gpio_deinit(&spi->sck);
        ls2k0300_gpio_deinit(&spi->miso);
        return -1;
    }
    if (ls2k0300_gpio_init(&spi->cs, cs, GPIO_MODE_OUT, GPIO_MUX_GPIO) != 0) {
        ls2k0300_gpio_deinit(&spi->sck);
        ls2k0300_gpio_deinit(&spi->miso);
        ls2k0300_gpio_deinit(&spi->mosi);
        return -1;
    }

    spi->mode = mode;
    spi->initialized = 1;

    /* 空闲状态默认片选无效 */
    ls2k0300_gpio_level_set(&spi->cs, GPIO_HIGH);
    return 0;
}

/********************************************************************************
 * @brief   释放软件 SPI.
 * @param   spi : 软件 SPI 句柄指针.
 * @return  none.
 * @example ls2k0300_soft_spi_deinit(&spi);
 ********************************************************************************/
void ls2k0300_soft_spi_deinit(ls2k0300_soft_spi_t *spi)
{
    if (spi == NULL || spi->initialized == 0) {
        return;
    }

    ls2k0300_gpio_deinit(&spi->sck);
    ls2k0300_gpio_deinit(&spi->miso);
    ls2k0300_gpio_deinit(&spi->mosi);
    ls2k0300_gpio_deinit(&spi->cs);

    spi->initialized = 0;
}

/********************************************************************************
 * @brief   软件 SPI 读单字节.
 * @param   spi : 软件 SPI 句柄指针.
 * @param   reg : 寄存器地址.
 * @return  成功返回读取值，失败返回 0.
 * @example uint8_t id = ls2k0300_soft_spi_read_byte(&spi, 0x0F);
 ********************************************************************************/
uint8_t ls2k0300_soft_spi_read_byte(ls2k0300_soft_spi_t *spi, uint8_t reg)
{
    uint8_t buf[2];

    if (spi == NULL || spi->initialized == 0) {
        return 0;
    }

    buf[0] = (uint8_t)(reg | 0x80U);
    buf[1] = 0U;

    /* 首字节发命令，次字节回读数据 */
    soft_spi_read_write_n_byte(spi, buf, 2);
    return buf[1];
}

/********************************************************************************
 * @brief   软件 SPI 读多字节.
 * @param   spi : 软件 SPI 句柄指针.
 * @param   reg : 起始寄存器地址.
 * @param   buf : 输出缓冲区.
 * @param   len : 读取长度.
 * @return  none.
 * @example ls2k0300_soft_spi_read_n_byte(&spi, 0x20, rx, 6);
 ********************************************************************************/
void ls2k0300_soft_spi_read_n_byte(ls2k0300_soft_spi_t *spi, uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint8_t trans_buf[LS_SOFT_SPI_MAX_TRANS_LEN];
    uint16_t i;

    if (spi == NULL || buf == NULL || len == 0U || spi->initialized == 0) {
        return;
    }

    if (len > (LS_SOFT_SPI_MAX_TRANS_LEN - 1U)) {
        len = (LS_SOFT_SPI_MAX_TRANS_LEN - 1U);
    }

    /* 第 0 字节为读命令，后续字节用于接收 */
    memset(trans_buf, 0, sizeof(trans_buf));
    trans_buf[0] = (uint8_t)(reg | 0x80U);

    soft_spi_read_write_n_byte(spi, trans_buf, (uint16_t)(len + 1U));

    for (i = 0U; i < len; i++) {
        buf[i] = trans_buf[i + 1U];
    }
}

/********************************************************************************
 * @brief   软件 SPI 写单字节.
 * @param   spi : 软件 SPI 句柄指针.
 * @param   reg : 寄存器地址.
 * @param   val : 写入值.
 * @return  none.
 * @example ls2k0300_soft_spi_write_byte(&spi, 0x20, 0x57);
 ********************************************************************************/
void ls2k0300_soft_spi_write_byte(ls2k0300_soft_spi_t *spi, uint8_t reg, uint8_t val)
{
    uint8_t buf[2];

    if (spi == NULL || spi->initialized == 0) {
        return;
    }

    buf[0] = (uint8_t)(reg & 0x7FU);
    buf[1] = val;

    /* 写操作首字节清读标志位 */
    soft_spi_read_write_n_byte(spi, buf, 2);
}

/********************************************************************************
 * @brief   软件 SPI 写多字节.
 * @param   spi : 软件 SPI 句柄指针.
 * @param   reg : 起始寄存器地址.
 * @param   buf : 输入数据缓冲区.
 * @param   len : 写入长度.
 * @return  none.
 * @example ls2k0300_soft_spi_write_n_byte(&spi, 0x20, tx, 6);
 ********************************************************************************/
void ls2k0300_soft_spi_write_n_byte(ls2k0300_soft_spi_t *spi, uint8_t reg, const uint8_t *buf, uint16_t len)
{
    uint8_t trans_buf[LS_SOFT_SPI_MAX_TRANS_LEN];
    uint16_t i;

    if (spi == NULL || buf == NULL || len == 0U || spi->initialized == 0) {
        return;
    }

    if (len > (LS_SOFT_SPI_MAX_TRANS_LEN - 1U)) {
        len = (LS_SOFT_SPI_MAX_TRANS_LEN - 1U);
    }

    /* 第 0 字节为写命令，后续连续发送 payload */
    memset(trans_buf, 0, sizeof(trans_buf));
    trans_buf[0] = (uint8_t)(reg & 0x7FU);
    for (i = 0U; i < len; i++) {
        trans_buf[i + 1U] = buf[i];
    }

    soft_spi_read_write_n_byte(spi, trans_buf, (uint16_t)(len + 1U));
}
