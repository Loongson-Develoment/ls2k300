#ifndef __LS2K0300_I2C_H
#define __LS2K0300_I2C_H

#include "LS2K0300_GPIO.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    ls2k0300_gpio_t scl;
    ls2k0300_gpio_t sda;
    uint8_t addr;
    bool initialized;
} ls2k0300_i2c_t;

int ls2k0300_i2c_init(ls2k0300_i2c_t* i2c, gpio_pin_t scl, gpio_pin_t sda, uint8_t addr);
void ls2k0300_i2c_deinit(ls2k0300_i2c_t* i2c);
uint8_t ls2k0300_i2c_read_byte(ls2k0300_i2c_t* i2c, const uint8_t reg, uint8_t* val);
uint8_t ls2k0300_i2c_write_byte(ls2k0300_i2c_t* i2c, const uint8_t reg, uint8_t val);
uint8_t ls2k0300_i2c_read_n_byte(ls2k0300_i2c_t* i2c, const uint8_t reg, uint8_t* buf, uint8_t len);
uint8_t ls2k0300_i2c_write_n_byte(ls2k0300_i2c_t* i2c, const uint8_t reg, uint8_t* buf, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif // __LS2K0300_I2C_H
