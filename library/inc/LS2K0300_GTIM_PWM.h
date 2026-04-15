#ifndef __LS2K0300_GTIM_PWM_H
#define __LS2K0300_GTIM_PWM_H

#include <stdint.h>
#include <pthread.h>

#include "LS2K0300_GPIO.h"

#ifdef __cplusplus
extern "C" {
#endif

/********************************************************************************
 * @brief   GTIM PWM 极性枚举.
 ********************************************************************************/
typedef enum gtim_pwm_polarity {
    GTIM_PWM_POL_NORMAL = 0x00,
    GTIM_PWM_POL_INV,
    GTIM_PWM_POL_INVALID,
} gtim_pwm_polarity_t;

/********************************************************************************
 * @brief   GTIM PWM 模式枚举.
 ********************************************************************************/
typedef enum gtim_pwm_mode {
    GTIM_PWM_MODE_1 = 0x06,
    GTIM_PWM_MODE_2 = 0x07,
    GTIM_PWM_MODE_INVALID,
} gtim_pwm_mode_t;

/********************************************************************************
 * @brief   GTIM PWM 通道枚举.
 ********************************************************************************/
typedef enum gtim_pwm_channel {
    GTIM_PWM_CH1 = 0x00,
    GTIM_PWM_CH2,
    GTIM_PWM_CH3,
    GTIM_PWM_CH4,
    GTIM_PWM_CH_INVALID,
} gtim_pwm_channel_t;

/********************************************************************************
 * @brief   GTIM PWM 引脚定义.
 ********************************************************************************/
typedef enum gtim_pwm_pin {
    GTIM_PWM0_PIN34 = (GPIO_MUX_ALT2 << 10) | (GTIM_PWM_CH1 << 8) | PIN_34,
    GTIM_PWM1_PIN35 = (GPIO_MUX_ALT2 << 10) | (GTIM_PWM_CH2 << 8) | PIN_35,
    GTIM_PWM2_PIN36 = (GPIO_MUX_ALT2 << 10) | (GTIM_PWM_CH3 << 8) | PIN_36,
    GTIM_PWM3_PIN77 = (GPIO_MUX_ALT1 << 10) | (GTIM_PWM_CH4 << 8) | PIN_77,

    GTIM_PWM0_PIN87 = (GPIO_MUX_MAIN << 10) | (GTIM_PWM_CH1 << 8) | PIN_87,
    GTIM_PWM1_PIN88 = (GPIO_MUX_MAIN << 10) | (GTIM_PWM_CH2 << 8) | PIN_88,
    GTIM_PWM2_PIN89 = (GPIO_MUX_MAIN << 10) | (GTIM_PWM_CH3 << 8) | PIN_89,
} gtim_pwm_pin_t;

/********************************************************************************
 * @brief   GTIM PWM 句柄结构体.
 ********************************************************************************/
typedef struct {
    gpio_pin_t           gpio;
    gpio_mux_mode_t      mux;
    gtim_pwm_channel_t   ch;
    gtim_pwm_polarity_t  pola;
    gtim_pwm_mode_t      mode;
    uint32_t             period;
    uint32_t             duty;

    ls_reg32_addr_t      gtim_base;
    ls_reg32_addr_t      gtim_arr;
    ls_reg32_addr_t      gtim_ccrx;
    ls_reg32_addr_t      gtim_ccmr[2];
    ls_reg32_addr_t      gtim_ccer;
    ls_reg32_addr_t      gtim_cnt;
    ls_reg32_addr_t      gtim_egr;
    ls_reg32_addr_t      gtim_cr1;

    pthread_mutex_t      mtx;
    pthread_mutex_t      enable_mtx;
    int                  initialized;
} ls2k0300_gtim_pwm_t;

/********************************************************************************
 * @brief   初始化 GTIM PWM 通道.
 * @param   pwm    : PWM 句柄指针.
 * @param   pin    : PWM 引脚与通道复合定义.
 * @param   period : 目标频率(Hz).
 * @param   duty   : 占空比，范围 0~10000.
 * @param   pola   : 输出极性.
 * @return  成功返回 0，失败返回 -1.
 * @example ls2k0300_gtim_pwm_init(&pwm, GTIM_PWM0_PIN34, 1000, 5000, GTIM_PWM_POL_NORMAL);
 ********************************************************************************/
int ls2k0300_gtim_pwm_init(ls2k0300_gtim_pwm_t *pwm, gtim_pwm_pin_t pin, uint32_t period, uint32_t duty, gtim_pwm_polarity_t pola);

/********************************************************************************
 * @brief   释放 GTIM PWM 资源.
 * @param   pwm : PWM 句柄指针.
 * @return  none.
 * @example ls2k0300_gtim_pwm_deinit(&pwm);
 ********************************************************************************/
void ls2k0300_gtim_pwm_deinit(ls2k0300_gtim_pwm_t *pwm);

/********************************************************************************
 * @brief   打开 GTIM PWM 输出.
 * @param   pwm : PWM 句柄指针.
 * @return  none.
 * @example ls2k0300_gtim_pwm_enable(&pwm);
 ********************************************************************************/
void ls2k0300_gtim_pwm_enable(ls2k0300_gtim_pwm_t *pwm);

/********************************************************************************
 * @brief   关闭 GTIM PWM 输出.
 * @param   pwm : PWM 句柄指针.
 * @return  none.
 * @example ls2k0300_gtim_pwm_disable(&pwm);
 ********************************************************************************/
void ls2k0300_gtim_pwm_disable(ls2k0300_gtim_pwm_t *pwm);

/********************************************************************************
 * @brief   设置 PWM 频率.
 * @param   pwm    : PWM 句柄指针.
 * @param   period : 目标频率(Hz).
 * @return  none.
 * @example ls2k0300_gtim_pwm_set_period(&pwm, 2000);
 ********************************************************************************/
void ls2k0300_gtim_pwm_set_period(ls2k0300_gtim_pwm_t *pwm, uint32_t period);

/********************************************************************************
 * @brief   设置 PWM 占空比.
 * @param   pwm  : PWM 句柄指针.
 * @param   duty : 占空比，范围 0~10000.
 * @return  none.
 * @example ls2k0300_gtim_pwm_set_duty(&pwm, 2500);
 ********************************************************************************/
void ls2k0300_gtim_pwm_set_duty(ls2k0300_gtim_pwm_t *pwm, uint32_t duty);

/********************************************************************************
 * @brief   设置输出极性.
 * @param   pwm  : PWM 句柄指针.
 * @param   pola : 极性配置.
 * @return  none.
 * @example ls2k0300_gtim_pwm_set_polarity(&pwm, GTIM_PWM_POL_INV);
 ********************************************************************************/
void ls2k0300_gtim_pwm_set_polarity(ls2k0300_gtim_pwm_t *pwm, gtim_pwm_polarity_t pola);

/********************************************************************************
 * @brief   设置 PWM 模式.
 * @param   pwm  : PWM 句柄指针.
 * @param   mode : PWM 模式.
 * @return  none.
 * @example ls2k0300_gtim_pwm_set_mode(&pwm, GTIM_PWM_MODE_2);
 ********************************************************************************/
void ls2k0300_gtim_pwm_set_mode(ls2k0300_gtim_pwm_t *pwm, gtim_pwm_mode_t mode);

/********************************************************************************
 * @brief   获取当前输出引脚.
 * @param   pwm : PWM 句柄指针.
 * @return  成功返回引脚号，失败返回 PIN_INVALID.
 * @example gpio_pin_t pin = ls2k0300_gtim_pwm_get_gpio(&pwm);
 ********************************************************************************/
gpio_pin_t ls2k0300_gtim_pwm_get_gpio(ls2k0300_gtim_pwm_t *pwm);

/********************************************************************************
 * @brief   获取当前复用模式.
 * @param   pwm : PWM 句柄指针.
 * @return  成功返回复用模式，失败返回 GPIO_MUX_INVALID.
 * @example gpio_mux_mode_t mux = ls2k0300_gtim_pwm_get_mux(&pwm);
 ********************************************************************************/
gpio_mux_mode_t ls2k0300_gtim_pwm_get_mux(ls2k0300_gtim_pwm_t *pwm);

/********************************************************************************
 * @brief   获取当前通道.
 * @param   pwm : PWM 句柄指针.
 * @return  成功返回通道号，失败返回 GTIM_PWM_CH_INVALID.
 * @example gtim_pwm_channel_t ch = ls2k0300_gtim_pwm_get_channel(&pwm);
 ********************************************************************************/
gtim_pwm_channel_t ls2k0300_gtim_pwm_get_channel(ls2k0300_gtim_pwm_t *pwm);

/********************************************************************************
 * @brief   获取当前极性.
 * @param   pwm : PWM 句柄指针.
 * @return  成功返回极性，失败返回 GTIM_PWM_POL_INVALID.
 * @example gtim_pwm_polarity_t p = ls2k0300_gtim_pwm_get_polarity(&pwm);
 ********************************************************************************/
gtim_pwm_polarity_t ls2k0300_gtim_pwm_get_polarity(ls2k0300_gtim_pwm_t *pwm);

/********************************************************************************
 * @brief   获取当前频率配置.
 * @param   pwm : PWM 句柄指针.
 * @return  成功返回频率(Hz)，失败返回 0.
 * @example uint32_t hz = ls2k0300_gtim_pwm_get_period(&pwm);
 ********************************************************************************/
uint32_t ls2k0300_gtim_pwm_get_period(ls2k0300_gtim_pwm_t *pwm);

/********************************************************************************
 * @brief   获取当前占空比配置.
 * @param   pwm : PWM 句柄指针.
 * @return  成功返回占空比(0~10000)，失败返回 0.
 * @example uint32_t duty = ls2k0300_gtim_pwm_get_duty(&pwm);
 ********************************************************************************/
uint32_t ls2k0300_gtim_pwm_get_duty(ls2k0300_gtim_pwm_t *pwm);

#ifdef __cplusplus
}
#endif

#endif
