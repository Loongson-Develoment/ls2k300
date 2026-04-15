#ifndef __LS2K0300_PWM_ENCODER_H
#define __LS2K0300_PWM_ENCODER_H

#include <stdint.h>
#include <pthread.h>

#include "LS2K0300_CLOCK.h"
#include "LS2K0300_GPIO.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LS2K0300_ENCODER_LINE  (512)

/********************************************************************************
 * @brief   编码器 PWM 通道枚举.
 ********************************************************************************/
typedef enum enc_pwm_channel {
    ENC_PWM_CH0 = 0x00,
    ENC_PWM_CH1,
    ENC_PWM_CH2,
    ENC_PWM_CH3,
    ENC_PWM_CH_INVALID,
} enc_pwm_channel_t;

/********************************************************************************
 * @brief   编码器 PWM 引脚定义.
 ********************************************************************************/
typedef enum ls_enc_pwm_pin {
    ENC_PWM0_PIN64 = (GPIO_MUX_ALT1 << 10) | (ENC_PWM_CH0 << 8) | PIN_64,
    ENC_PWM1_PIN65 = (GPIO_MUX_ALT1 << 10) | (ENC_PWM_CH1 << 8) | PIN_65,
    ENC_PWM2_PIN66 = (GPIO_MUX_ALT1 << 10) | (ENC_PWM_CH2 << 8) | PIN_66,
    ENC_PWM3_PIN67 = (GPIO_MUX_ALT1 << 10) | (ENC_PWM_CH3 << 8) | PIN_67,

    ENC_PWM0_PIN86 = (GPIO_MUX_ALT2 << 10) | (ENC_PWM_CH0 << 8) | PIN_86,
} ls_enc_pwm_pin_t;

/********************************************************************************
 * @brief   编码器 PWM 句柄结构体.
 ********************************************************************************/
typedef struct {
    gpio_pin_t        gpio;
    ls2k0300_gpio_t   dir;
    gpio_mux_mode_t   mux;
    enc_pwm_channel_t ch;

    ls_reg32_addr_t   enc_pwm_base;
    ls_reg32_addr_t   enc_pwm_low_buf;
    ls_reg32_addr_t   enc_pwm_full_buf;
    ls_reg32_addr_t   enc_pwm_ctrl;

    pthread_mutex_t   mtx;
    int               initialized;
} ls2k0300_pwm_encoder_t;

/********************************************************************************
 * @brief   初始化编码器输入通道.
 * @param   enc     : 编码器句柄指针.
 * @param   pin     : 脉冲输入引脚定义.
 * @param   dir_pin : 方向输入引脚.
 * @return  成功返回 0，失败返回 -1.
 * @example ls2k0300_pwm_encoder_init(&enc, ENC_PWM0_PIN64, PIN_10);
 ********************************************************************************/
int ls2k0300_pwm_encoder_init(ls2k0300_pwm_encoder_t *enc, ls_enc_pwm_pin_t pin, gpio_pin_t dir_pin);

/********************************************************************************
 * @brief   释放编码器资源.
 * @param   enc : 编码器句柄指针.
 * @return  none.
 * @example ls2k0300_pwm_encoder_deinit(&enc);
 ********************************************************************************/
void ls2k0300_pwm_encoder_deinit(ls2k0300_pwm_encoder_t *enc);

/********************************************************************************
 * @brief   触发计数器复位.
 * @param   enc : 编码器句柄指针.
 * @return  none.
 * @example ls2k0300_pwm_encoder_reset_counter(&enc);
 ********************************************************************************/
void ls2k0300_pwm_encoder_reset_counter(ls2k0300_pwm_encoder_t *enc);

/********************************************************************************
 * @brief   关闭计数器复位位.
 * @param   enc : 编码器句柄指针.
 * @return  none.
 * @example ls2k0300_pwm_encoder_close_reset_counter(&enc);
 ********************************************************************************/
void ls2k0300_pwm_encoder_close_reset_counter(ls2k0300_pwm_encoder_t *enc);

/********************************************************************************
 * @brief   获取当前估算转速.
 * @param   enc : 编码器句柄指针.
 * @return  成功返回转速，失败返回 0.
 * @example float speed = ls2k0300_pwm_encoder_get_count(&enc);
 ********************************************************************************/
float ls2k0300_pwm_encoder_get_count(ls2k0300_pwm_encoder_t *enc);

/********************************************************************************
 * @brief   获取脉冲输入引脚.
 * @param   enc : 编码器句柄指针.
 * @return  成功返回引脚号，失败返回 PIN_INVALID.
 * @example gpio_pin_t p = ls2k0300_pwm_encoder_get_pulse(&enc);
 ********************************************************************************/
gpio_pin_t ls2k0300_pwm_encoder_get_pulse(ls2k0300_pwm_encoder_t *enc);

/********************************************************************************
 * @brief   获取方向输入引脚.
 * @param   enc : 编码器句柄指针.
 * @return  成功返回引脚号，失败返回 PIN_INVALID.
 * @example gpio_pin_t d = ls2k0300_pwm_encoder_get_dir(&enc);
 ********************************************************************************/
gpio_pin_t ls2k0300_pwm_encoder_get_dir(ls2k0300_pwm_encoder_t *enc);

/********************************************************************************
 * @brief   获取脉冲引脚复用模式.
 * @param   enc : 编码器句柄指针.
 * @return  成功返回复用模式，失败返回 GPIO_MUX_INVALID.
 * @example gpio_mux_mode_t mux = ls2k0300_pwm_encoder_get_mux(&enc);
 ********************************************************************************/
gpio_mux_mode_t ls2k0300_pwm_encoder_get_mux(ls2k0300_pwm_encoder_t *enc);

/********************************************************************************
 * @brief   获取编码器通道号.
 * @param   enc : 编码器句柄指针.
 * @return  成功返回通道号，失败返回 ENC_PWM_CH_INVALID.
 * @example enc_pwm_channel_t ch = ls2k0300_pwm_encoder_get_channel(&enc);
 ********************************************************************************/
enc_pwm_channel_t ls2k0300_pwm_encoder_get_channel(ls2k0300_pwm_encoder_t *enc);

#ifdef __cplusplus
}
#endif

#endif
