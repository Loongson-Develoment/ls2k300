#ifndef __LS2K0300_PWM_H
#define __LS2K0300_PWM_H

#include "LS2K0300_GPIO.h"

#ifdef __cplusplus
extern "C" {
#endif

/********************************************************************************
 * @brief   PWM 极性枚举.
 ********************************************************************************/
typedef enum pwm_polarity {
    PWM_POL_NORMAL = 0x00,
    PWM_POL_INV,
    PWM_POL_INVALID,
} pwm_polarity_t;

/********************************************************************************
 * @brief   PWM 通道枚举.
 ********************************************************************************/
typedef enum pwm_channel {
    PWM_CH0 = 0x00,
    PWM_CH1,
    PWM_CH2,
    PWM_CH3,
    PWM_CH_INVALID,
} pwm_channel_t;

/********************************************************************************
 * @brief   PWM 引脚定义.
 ********************************************************************************/
typedef enum pwm_pin {
    PWM0_PIN64 = (GPIO_MUX_ALT1 << 10) | (PWM_CH0 << 8) | PIN_64,
    PWM1_PIN65 = (GPIO_MUX_ALT1 << 10) | (PWM_CH1 << 8) | PIN_65,
    PWM2_PIN66 = (GPIO_MUX_ALT1 << 10) | (PWM_CH2 << 8) | PIN_66,
    PWM3_PIN67 = (GPIO_MUX_ALT1 << 10) | (PWM_CH3 << 8) | PIN_67,

    PWM0_PIN86 = (GPIO_MUX_ALT2 << 10) | (PWM_CH0 << 8) | PIN_86,
    PWM1_PIN87 = (GPIO_MUX_ALT2 << 10) | (PWM_CH1 << 8) | PIN_87,
    PWM2_PIN88 = (GPIO_MUX_ALT2 << 10) | (PWM_CH2 << 8) | PIN_88,
    PWM3_PIN89 = (GPIO_MUX_ALT2 << 10) | (PWM_CH3 << 8) | PIN_89,
} pwm_pin_t;

/********************************************************************************
 * @brief   PWM 句柄结构体.
 ********************************************************************************/
typedef struct {
    gpio_pin_t      gpio;
    gpio_mux_mode_t mux;
    pwm_channel_t   ch;
    pwm_polarity_t  pola;
    uint32_t        period;
    uint32_t        duty;

    ls_reg32_addr_t pwm_base;
    ls_reg32_addr_t pwm_low_buf;
    ls_reg32_addr_t pwm_full_buf;
    ls_reg32_addr_t pwm_ctrl;

    pthread_mutex_t mtx;
    int             initialized;
} ls2k0300_pwm_t;

/********************************************************************************
 * @brief   初始化 PWM 通道.
 * @param   pwm      : PWM 句柄指针.
 * @param   pin_info : PWM 引脚与通道复合定义.
 * @param   period   : 目标频率(Hz).
 * @param   duty     : 占空比，范围 0~10000.
 * @param   pola     : 输出极性.
 * @return  成功返回 0，失败返回 -1.
 * @example ls2k0300_pwm_init(&pwm, PWM0_PIN64, 1000, 5000, PWM_POL_NORMAL);
 ********************************************************************************/
int ls2k0300_pwm_init(ls2k0300_pwm_t *pwm, pwm_pin_t pin_info, uint32_t period, uint32_t duty, pwm_polarity_t pola);

/********************************************************************************
 * @brief   释放 PWM 资源.
 * @param   pwm : PWM 句柄指针.
 * @return  none.
 * @example ls2k0300_pwm_deinit(&pwm);
 ********************************************************************************/
void ls2k0300_pwm_deinit(ls2k0300_pwm_t *pwm);

/********************************************************************************
 * @brief   使能 PWM 输出.
 * @param   pwm : PWM 句柄指针.
 * @return  none.
 * @example ls2k0300_pwm_enable(&pwm);
 ********************************************************************************/
void ls2k0300_pwm_enable(ls2k0300_pwm_t *pwm);

/********************************************************************************
 * @brief   禁用 PWM 输出.
 * @param   pwm : PWM 句柄指针.
 * @return  none.
 * @example ls2k0300_pwm_disable(&pwm);
 ********************************************************************************/
void ls2k0300_pwm_disable(ls2k0300_pwm_t *pwm);

/********************************************************************************
 * @brief   设置 PWM 频率.
 * @param   pwm    : PWM 句柄指针.
 * @param   period : 目标频率(Hz).
 * @return  none.
 * @example ls2k0300_pwm_set_period(&pwm, 2000);
 ********************************************************************************/
void ls2k0300_pwm_set_period(ls2k0300_pwm_t *pwm, uint32_t period);

/********************************************************************************
 * @brief   设置 PWM 占空比.
 * @param   pwm  : PWM 句柄指针.
 * @param   duty : 占空比，范围 0~10000.
 * @return  none.
 * @example ls2k0300_pwm_set_duty(&pwm, 2500);
 ********************************************************************************/
void ls2k0300_pwm_set_duty(ls2k0300_pwm_t *pwm, uint32_t duty);

/********************************************************************************
 * @brief   设置 PWM 极性.
 * @param   pwm  : PWM 句柄指针.
 * @param   pola : 极性配置.
 * @return  none.
 * @example ls2k0300_pwm_set_polarity(&pwm, PWM_POL_INV);
 ********************************************************************************/
void ls2k0300_pwm_set_polarity(ls2k0300_pwm_t *pwm, pwm_polarity_t pola);

/********************************************************************************
 * @brief   获取当前输出引脚.
 * @param   pwm : PWM 句柄指针.
 * @return  成功返回引脚号，失败返回 PIN_INVALID.
 * @example gpio_pin_t pin = ls2k0300_pwm_get_gpio(&pwm);
 ********************************************************************************/
gpio_pin_t ls2k0300_pwm_get_gpio(ls2k0300_pwm_t *pwm);

/********************************************************************************
 * @brief   获取当前复用模式.
 * @param   pwm : PWM 句柄指针.
 * @return  成功返回复用模式，失败返回 GPIO_MUX_INVALID.
 * @example gpio_mux_mode_t mux = ls2k0300_pwm_get_mux(&pwm);
 ********************************************************************************/
gpio_mux_mode_t ls2k0300_pwm_get_mux(ls2k0300_pwm_t *pwm);

/********************************************************************************
 * @brief   获取当前通道.
 * @param   pwm : PWM 句柄指针.
 * @return  成功返回通道号，失败返回 PWM_CH_INVALID.
 * @example pwm_channel_t ch = ls2k0300_pwm_get_channel(&pwm);
 ********************************************************************************/
pwm_channel_t ls2k0300_pwm_get_channel(ls2k0300_pwm_t *pwm);

/********************************************************************************
 * @brief   获取当前极性.
 * @param   pwm : PWM 句柄指针.
 * @return  成功返回极性，失败返回 PWM_POL_INVALID.
 * @example pwm_polarity_t p = ls2k0300_pwm_get_polarity(&pwm);
 ********************************************************************************/
pwm_polarity_t ls2k0300_pwm_get_polarity(ls2k0300_pwm_t *pwm);

/********************************************************************************
 * @brief   获取当前频率配置.
 * @param   pwm : PWM 句柄指针.
 * @return  成功返回频率(Hz)，失败返回 0.
 * @example uint32_t hz = ls2k0300_pwm_get_period(&pwm);
 ********************************************************************************/
uint32_t ls2k0300_pwm_get_period(ls2k0300_pwm_t *pwm);

/********************************************************************************
 * @brief   获取当前占空比配置.
 * @param   pwm : PWM 句柄指针.
 * @return  成功返回占空比(0~10000)，失败返回 0.
 * @example uint32_t duty = ls2k0300_pwm_get_duty(&pwm);
 ********************************************************************************/
uint32_t ls2k0300_pwm_get_duty(ls2k0300_pwm_t *pwm);

#ifdef __cplusplus
}
#endif

#endif
