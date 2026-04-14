#include "LS2K0300_I2C.h"
#include <unistd.h>

static void soft_i2c_delay_us(uint16_t us) { usleep(us); }

static void soft_i2c_start(ls2k0300_i2c_t* i2c) {
    ls2k0300_gpio_direction_set(i2c->sda, GPIO_MODE_OUT);
    ls2k0300_gpio_level_set(i2c->sda, GPIO_HIGH);
    ls2k0300_gpio_level_set(i2c->scl, GPIO_HIGH);
    soft_i2c_delay_us(4);
    ls2k0300_gpio_level_set(i2c->sda, GPIO_LOW);
    soft_i2c_delay_us(2);
    ls2k0300_gpio_level_set(i2c->scl, GPIO_LOW);
}

static void soft_i2c_stop(ls2k0300_i2c_t* i2c) {
    ls2k0300_gpio_direction_set(i2c->sda, GPIO_MODE_OUT);
    soft_i2c_delay_us(2);
    ls2k0300_gpio_level_set(i2c->scl, GPIO_LOW);
    ls2k0300_gpio_level_set(i2c->sda, GPIO_LOW);
    soft_i2c_delay_us(2);
    ls2k0300_gpio_level_set(i2c->scl, GPIO_HIGH);
    soft_i2c_delay_us(2);
    ls2k0300_gpio_level_set(i2c->sda, GPIO_HIGH);
    soft_i2c_delay_us(2);
}

static uint8_t soft_i2c_wait_ack(ls2k0300_i2c_t* i2c) {
    uint8_t errTime = 0;
    ls2k0300_gpio_direction_set(i2c->sda, GPIO_MODE_IN);
    ls2k0300_gpio_level_set(i2c->sda, GPIO_HIGH);
    soft_i2c_delay_us(2);
    ls2k0300_gpio_level_set(i2c->scl, GPIO_HIGH);
    soft_i2c_delay_us(2);
    while (ls2k0300_gpio_level_get(i2c->sda)) {
        errTime++;
        if (errTime > 100) {
            soft_i2c_stop(i2c);
            return 1;
        }
    }
    ls2k0300_gpio_level_set(i2c->scl, GPIO_LOW);
    return 0;
}

static void soft_i2c_ack(ls2k0300_i2c_t* i2c) {
    ls2k0300_gpio_level_set(i2c->scl, GPIO_LOW);
    ls2k0300_gpio_direction_set(i2c->sda, GPIO_MODE_OUT);
    ls2k0300_gpio_level_set(i2c->sda, GPIO_LOW);
    soft_i2c_delay_us(2);
    ls2k0300_gpio_level_set(i2c->scl, GPIO_HIGH);
    soft_i2c_delay_us(2);
    ls2k0300_gpio_level_set(i2c->scl, GPIO_LOW);
}

static void soft_i2c_nack(ls2k0300_i2c_t* i2c) {
    ls2k0300_gpio_level_set(i2c->scl, GPIO_LOW);
    ls2k0300_gpio_direction_set(i2c->sda, GPIO_MODE_OUT);
    ls2k0300_gpio_level_set(i2c->sda, GPIO_HIGH);
    soft_i2c_delay_us(2);
    ls2k0300_gpio_level_set(i2c->scl, GPIO_HIGH);
    soft_i2c_delay_us(2);
    ls2k0300_gpio_level_set(i2c->scl, GPIO_LOW);
}

static void soft_i2c_send_byte(ls2k0300_i2c_t* i2c, uint8_t byte) {
    ls2k0300_gpio_direction_set(i2c->sda, GPIO_MODE_OUT);
    ls2k0300_gpio_level_set(i2c->scl, GPIO_LOW);
    for (uint8_t t = 0; t < 8; t++) {
        ls2k0300_gpio_level_set(i2c->sda, (byte & 0x80) ? GPIO_HIGH : GPIO_LOW);
        ls2k0300_gpio_level_set(i2c->scl, GPIO_HIGH);
        soft_i2c_delay_us(2);
        byte <<= 1;
        soft_i2c_delay_us(2);
        ls2k0300_gpio_level_set(i2c->scl, GPIO_LOW);
        soft_i2c_delay_us(2);
    }
    soft_i2c_delay_us(2);
}

static uint8_t soft_i2c_read_byte_internal(ls2k0300_i2c_t* i2c, uint8_t ack) {
    uint8_t receive = 0;
    ls2k0300_gpio_direction_set(i2c->sda, GPIO_MODE_IN);
    for (uint8_t i = 0; i < 8; i++) {
        ls2k0300_gpio_level_set(i2c->scl, GPIO_LOW);
        soft_i2c_delay_us(2);
        ls2k0300_gpio_level_set(i2c->scl, GPIO_HIGH);
        receive <<= 1;
        if (ls2k0300_gpio_level_get(i2c->sda)) receive++;
        soft_i2c_delay_us(2);
    }
    if (ack) soft_i2c_ack(i2c);
    else soft_i2c_nack(i2c);
    return receive;
}

int ls2k0300_i2c_init(ls2k0300_i2c_t* i2c, gpio_pin_t scl, gpio_pin_t sda, uint8_t addr) {
    if (!i2c) return -1;
    i2c->scl = scl;
    i2c->sda = sda;
    i2c->addr = addr;
    ls2k0300_gpio_direction_set(scl, GPIO_MODE_OUT);
    ls2k0300_gpio_direction_set(sda, GPIO_MODE_OUT);
    ls2k0300_gpio_level_set(scl, GPIO_HIGH);
    ls2k0300_gpio_level_set(sda, GPIO_HIGH);
    return 0;
}

uint8_t ls2k0300_i2c_read_byte(ls2k0300_i2c_t* i2c, const uint8_t reg, uint8_t* val) {
    if (!i2c || !val) return 1;
    soft_i2c_start(i2c);
    soft_i2c_send_byte(i2c, i2c->addr << 1);
    soft_i2c_wait_ack(i2c);
    soft_i2c_send_byte(i2c, reg);
    soft_i2c_wait_ack(i2c);
    soft_i2c_start(i2c);
    soft_i2c_send_byte(i2c, (i2c->addr << 1) + 1);
    soft_i2c_wait_ack(i2c);
    *val = soft_i2c_read_byte_internal(i2c, 0);
    soft_i2c_stop(i2c);
    return 0;
}

uint8_t ls2k0300_i2c_write_byte(ls2k0300_i2c_t* i2c, const uint8_t reg, uint8_t val) {
    if (!i2c) return 1;
    soft_i2c_start(i2c);
    soft_i2c_send_byte(i2c, i2c->addr << 1);
    soft_i2c_wait_ack(i2c);
    soft_i2c_send_byte(i2c, reg);
    soft_i2c_wait_ack(i2c);
    soft_i2c_send_byte(i2c, val);
    soft_i2c_wait_ack(i2c);
    soft_i2c_stop(i2c);
    return 0;
}

uint8_t ls2k0300_i2c_read_n_byte(ls2k0300_i2c_t* i2c, const uint8_t reg, uint8_t* buf, uint8_t len) {
    if (!i2c || !buf) return 1;
    soft_i2c_start(i2c);
    soft_i2c_send_byte(i2c, i2c->addr << 1);
    soft_i2c_wait_ack(i2c);
    soft_i2c_send_byte(i2c, reg);
    soft_i2c_wait_ack(i2c);
    soft_i2c_start(i2c);
    soft_i2c_send_byte(i2c, (i2c->addr << 1) + 1);
    soft_i2c_wait_ack(i2c);
    for (uint8_t count = 0; count < len; count++) {
        buf[count] = soft_i2c_read_byte_internal(i2c, (count == len - 1) ? 0 : 1);
    }
    soft_i2c_stop(i2c);
    return 0;
}

uint8_t ls2k0300_i2c_write_n_byte(ls2k0300_i2c_t* i2c, const uint8_t reg, uint8_t* buf, uint8_t len) {
    if (!i2c || !buf) return 1;
    soft_i2c_start(i2c);
    soft_i2c_send_byte(i2c, i2c->addr << 1);
    soft_i2c_wait_ack(i2c);
    soft_i2c_send_byte(i2c, reg);
    soft_i2c_wait_ack(i2c);
    for (uint8_t count = 0; count < len; count++) {
        soft_i2c_send_byte(i2c, buf[count]);
        soft_i2c_wait_ack(i2c);
    }
    soft_i2c_stop(i2c);
    return 0;
}
