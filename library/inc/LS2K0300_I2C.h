#ifndef __LS2K0300_I2C_H
#define __LS2K0300_I2C_H

#include "LS2K0300_GPIO.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    gpio_pin_t scl;
    gpio_pin_t sda;
    uint8_t addr;
} ls2k0300_i2c_t;

int ls2k0300_i2c_init(ls2k0300_i2c_t* i2c, gpio_pin_t scl, gpio_pin_t sda, uint8_t addr);
uint8_t ls2k0300_i2c_read_byte(ls2k0300_i2c_t* i2c, const uint8_t reg, uint8_t* val);
uint8_t ls2k0300_i2c_write_byte(ls2k0300_i2c_t* i2c, const uint8_t reg, uint8_t val);
uint8_t ls2k0300_i2c_read_n_byte(ls2k0300_i2c_t* i2c, const uint8_t reg, uint8_t* buf, uint8_t len);
uint8_t ls2k0300_i2c_write_n_byte(ls2k0300_i2c_t* i2c, const uint8_t reg, uint8_t* buf, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif // __LS2K0300_I2C_H
