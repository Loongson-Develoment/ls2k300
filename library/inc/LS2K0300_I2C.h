#ifndef __LS2K0300_I2C_H
#define __LS2K0300_I2C_H

#include <stdint.h>
#include "LS2K0300_GPIO.h"

#ifdef __cplusplus
extern "C" {
#endif

/********************************************************************************
 * @brief   软件 I2C 句柄结构体.
 * @note    scl/sda 在 init 时由内部初始化，外部无需单独初始化.
 ********************************************************************************/
typedef struct {
    ls2k0300_gpio_t scl;
    ls2k0300_gpio_t sda;
    uint8_t         addr;
    int             initialized;
} ls2k0300_i2c_t;

/********************************************************************************
 * @brief   初始化软件 I2C.
 * @param   i2c  : I2C 句柄.
 * @param   scl  : 时钟引脚.
 * @param   sda  : 数据引脚.
 * @param   addr : 7bit 设备地址.
 * @return  成功返回 0，失败返回 -1.
 * @example ls2k0300_i2c_t i2c;
 *          ls2k0300_i2c_init(&i2c, PIN_85, PIN_86, 0x50);
 ********************************************************************************/
int ls2k0300_i2c_init(ls2k0300_i2c_t *i2c, gpio_pin_t scl, gpio_pin_t sda, uint8_t addr);

/********************************************************************************
 * @brief   释放软件 I2C 资源.
 * @param   i2c : I2C 句柄.
 * @return  none.
 * @example ls2k0300_i2c_deinit(&i2c);
 ********************************************************************************/
void ls2k0300_i2c_deinit(ls2k0300_i2c_t *i2c);

/********************************************************************************
 * @brief   读取单字节寄存器.
 * @param   i2c : I2C 句柄.
 * @param   reg : 寄存器地址.
 * @param   val : 输出数据指针.
 * @return  成功返回 0，失败返回 1.
 * @example uint8_t id = 0;
 *          ls2k0300_i2c_read_byte(&i2c, 0x00, &id);
 ********************************************************************************/
uint8_t ls2k0300_i2c_read_byte(ls2k0300_i2c_t *i2c, uint8_t reg, uint8_t *val);

/********************************************************************************
 * @brief   写入单字节寄存器.
 * @param   i2c : I2C 句柄.
 * @param   reg : 寄存器地址.
 * @param   val : 写入数据.
 * @return  成功返回 0，失败返回 1.
 * @example ls2k0300_i2c_write_byte(&i2c, 0x10, 0x5A);
 ********************************************************************************/
uint8_t ls2k0300_i2c_write_byte(ls2k0300_i2c_t *i2c, uint8_t reg, uint8_t val);

/********************************************************************************
 * @brief   连续读取多个寄存器字节.
 * @param   i2c : I2C 句柄.
 * @param   reg : 起始寄存器.
 * @param   buf : 输出缓冲区.
 * @param   len : 读取长度.
 * @return  成功返回 0，失败返回 1.
 * @example uint8_t buf[6];
 *          ls2k0300_i2c_read_n_byte(&i2c, 0x20, buf, 6);
 ********************************************************************************/
uint8_t ls2k0300_i2c_read_n_byte(ls2k0300_i2c_t *i2c, uint8_t reg, uint8_t *buf, uint8_t len);

/********************************************************************************
 * @brief   连续写入多个寄存器字节.
 * @param   i2c : I2C 句柄.
 * @param   reg : 起始寄存器.
 * @param   buf : 输入缓冲区.
 * @param   len : 写入长度.
 * @return  成功返回 0，失败返回 1.
 * @example uint8_t cfg[2] = {0x01, 0x02};
 *          ls2k0300_i2c_write_n_byte(&i2c, 0x30, cfg, 2);
 ********************************************************************************/
uint8_t ls2k0300_i2c_write_n_byte(ls2k0300_i2c_t *i2c, uint8_t reg, uint8_t *buf, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif
