#ifndef __LS2K0300_ATIM_PWM_H
#define __LS2K0300_ATIM_PWM_H

#include <stdint.h>
#include <pthread.h>

#include "LS2K0300_GPIO.h"

#ifdef __cplusplus
extern "C" {
#endif

/********************************************************************************
 * @brief   ATIM PWM 极性枚举.
 * @note    对应手册 ATIM_CCER.CCxP 位：
 *          - CCxP=0：OCx = OCxREF，输出不反相；
 *          - CCxP=1：OCx = !OCxREF，输出反相。
 * @note    本库初始化时默认使用 ATIM_PWM_MODE_2，且向上计数。
 *          在默认模式下，ATIM_PWM_POL_NORMAL 的 duty 对应低电平宽度；
 *          ATIM_PWM_POL_INV 的 duty 对应高电平宽度。
 * @note    ATIM_PWM_POL_INVALID 不是硬件极性，只用于参数检查/错误返回。
 ********************************************************************************/
typedef enum atim_pwm_polarity {
    ATIM_PWM_POL_NORMAL = 0x00, /* CCxP=0：不反相；默认 MODE_2 下 duty 为低电平有效宽度 */
    ATIM_PWM_POL_INV,           /* CCxP=1：反相；默认 MODE_2 下 duty 为高电平有效宽度 */
    ATIM_PWM_POL_INVALID,       /* 无效极性，用于参数检查/错误返回 */
} atim_pwm_polarity_t;

/********************************************************************************
 * @brief   ATIM PWM 模式枚举.
 * @note    对应手册 CCMRx.OCxM：
 *          - PWM 模式 1(110)：向上计数时 CNT < CCRx，OCxREF 为高；
 *          - PWM 模式 2(111)：向上计数时 CNT < CCRx，OCxREF 为低。
 * @note    本库 duty 会写入 CCRx，初始化默认使用 ATIM_PWM_MODE_2。
 *          若切换到 ATIM_PWM_MODE_1，则 duty 与有效电平的对应关系会反过来：
 *          POL_NORMAL 下 duty 为高电平宽度，POL_INV 下 duty 为低电平宽度。
 ********************************************************************************/
typedef enum atim_pwm_mode {
    ATIM_PWM_MODE_1 = 0x06,     /* PWM 模式 1：向上计数时 CNT < CCRx 输出 OCxREF 高电平 */
    ATIM_PWM_MODE_2 = 0x07,     /* PWM 模式 2：向上计数时 CNT < CCRx 输出 OCxREF 低电平 */
    ATIM_PWM_MODE_INVALID,      /* 无效模式，用于参数检查/错误返回 */
} atim_pwm_mode_t;

/********************************************************************************
 * @brief   ATIM PWM 通道枚举.
 ********************************************************************************/
typedef enum atim_pwm_channel {
    ATIM_PWM_CH1 = 0x00,
    ATIM_PWM_CH2,
    ATIM_PWM_CH3,
    ATIM_PWM_CH4,
    ATIM_PWM_CH_INVALID,
} atim_pwm_channel_t;

/********************************************************************************
 * @brief   ATIM PWM 引脚定义.
 ********************************************************************************/
typedef enum atim_pwm_pin {
    ATIM_PWM0_PIN28 = (GPIO_MUX_ALT2 << 10) | (ATIM_PWM_CH1 << 8) | PIN_28,
    ATIM_PWM1_PIN29 = (GPIO_MUX_ALT2 << 10) | (ATIM_PWM_CH2 << 8) | PIN_29,
    ATIM_PWM2_PIN30 = (GPIO_MUX_ALT2 << 10) | (ATIM_PWM_CH3 << 8) | PIN_30,
    ATIM_PWM3_PIN76 = (GPIO_MUX_ALT1 << 10) | (ATIM_PWM_CH4 << 8) | PIN_76,

    ATIM_PWM0_PIN81 = (GPIO_MUX_MAIN << 10) | (ATIM_PWM_CH1 << 8) | PIN_81,
    ATIM_PWM1_PIN82 = (GPIO_MUX_MAIN << 10) | (ATIM_PWM_CH2 << 8) | PIN_82,
    ATIM_PWM2_PIN83 = (GPIO_MUX_MAIN << 10) | (ATIM_PWM_CH3 << 8) | PIN_83,
} atim_pwm_pin_t;

/********************************************************************************
 * @brief   ATIM PWM 句柄结构体.
 ********************************************************************************/
typedef struct {
    gpio_pin_t           gpio;
    gpio_mux_mode_t      mux;
    atim_pwm_channel_t   ch;
    atim_pwm_polarity_t  pola;
    atim_pwm_mode_t      mode;
    uint32_t             period;
    uint32_t             duty;

    ls_reg32_addr_t      atim_base;
    ls_reg32_addr_t      atim_arr;
    ls_reg32_addr_t      atim_ccrx;
    ls_reg32_addr_t      atim_ccmr[2];
    ls_reg32_addr_t      atim_ccer;
    ls_reg32_addr_t      atim_cnt;
    ls_reg32_addr_t      atim_bdtr;
    ls_reg32_addr_t      atim_egr;
    ls_reg32_addr_t      atim_cr1;

    pthread_mutex_t      mtx;
    pthread_mutex_t      enable_mtx;
    int                  initialized;
} ls2k0300_atim_pwm_t;

/********************************************************************************
 * @brief   初始化 ATIM PWM 通道.
 * @param   pwm    : PWM 句柄指针.
 * @param   pin    : PWM 引脚与通道复合定义.
 * @param   period : 目标频率(Hz).
 * @param   duty   : 占空比，范围 0~10000.
 * @param   pola   : 输出极性.
 * @return  成功返回 0，失败返回 -1.
 * @example ls2k0300_atim_pwm_init(&pwm, ATIM_PWM0_PIN28, 1000, 5000, ATIM_PWM_POL_NORMAL);
 ********************************************************************************/
int ls2k0300_atim_pwm_init(ls2k0300_atim_pwm_t *pwm, atim_pwm_pin_t pin, uint32_t period, uint32_t duty, atim_pwm_polarity_t pola);

/********************************************************************************
 * @brief   释放 ATIM PWM 资源.
 * @param   pwm : PWM 句柄指针.
 * @return  none.
 * @example ls2k0300_atim_pwm_deinit(&pwm);
 ********************************************************************************/
void ls2k0300_atim_pwm_deinit(ls2k0300_atim_pwm_t *pwm);

/********************************************************************************
 * @brief   打开 ATIM PWM 输出.
 * @param   pwm : PWM 句柄指针.
 * @return  none.
 * @example ls2k0300_atim_pwm_enable(&pwm);
 ********************************************************************************/
void ls2k0300_atim_pwm_enable(ls2k0300_atim_pwm_t *pwm);

/********************************************************************************
 * @brief   关闭 ATIM PWM 输出.
 * @param   pwm : PWM 句柄指针.
 * @return  none.
 * @example ls2k0300_atim_pwm_disable(&pwm);
 ********************************************************************************/
void ls2k0300_atim_pwm_disable(ls2k0300_atim_pwm_t *pwm);

/********************************************************************************
 * @brief   设置 PWM 频率.
 * @param   pwm    : PWM 句柄指针.
 * @param   period : 目标频率(Hz).
 * @return  none.
 * @example ls2k0300_atim_pwm_set_period(&pwm, 2000);
 ********************************************************************************/
void ls2k0300_atim_pwm_set_period(ls2k0300_atim_pwm_t *pwm, uint32_t period);

/********************************************************************************
 * @brief   设置 PWM 占空比.
 * @param   pwm  : PWM 句柄指针.
 * @param   duty : 占空比，范围 0~10000.
 * @return  none.
 * @example ls2k0300_atim_pwm_set_duty(&pwm, 2500);
 ********************************************************************************/
void ls2k0300_atim_pwm_set_duty(ls2k0300_atim_pwm_t *pwm, uint32_t duty);

/********************************************************************************
 * @brief   设置输出极性.
 * @param   pwm  : PWM 句柄指针.
 * @param   pola : 极性配置.
 * @return  none.
 * @example ls2k0300_atim_pwm_set_polarity(&pwm, ATIM_PWM_POL_INV);
 ********************************************************************************/
void ls2k0300_atim_pwm_set_polarity(ls2k0300_atim_pwm_t *pwm, atim_pwm_polarity_t pola);

/********************************************************************************
 * @brief   设置 PWM 模式.
 * @param   pwm  : PWM 句柄指针.
 * @param   mode : PWM 模式.
 * @return  none.
 * @example ls2k0300_atim_pwm_set_mode(&pwm, ATIM_PWM_MODE_2);
 ********************************************************************************/
void ls2k0300_atim_pwm_set_mode(ls2k0300_atim_pwm_t *pwm, atim_pwm_mode_t mode);

/********************************************************************************
 * @brief   获取当前输出引脚.
 * @param   pwm : PWM 句柄指针.
 * @return  成功返回引脚号，失败返回 PIN_INVALID.
 * @example gpio_pin_t pin = ls2k0300_atim_pwm_get_gpio(&pwm);
 ********************************************************************************/
gpio_pin_t ls2k0300_atim_pwm_get_gpio(ls2k0300_atim_pwm_t *pwm);

/********************************************************************************
 * @brief   获取当前复用模式.
 * @param   pwm : PWM 句柄指针.
 * @return  成功返回复用模式，失败返回 GPIO_MUX_INVALID.
 * @example gpio_mux_mode_t mux = ls2k0300_atim_pwm_get_mux(&pwm);
 ********************************************************************************/
gpio_mux_mode_t ls2k0300_atim_pwm_get_mux(ls2k0300_atim_pwm_t *pwm);

/********************************************************************************
 * @brief   获取当前通道.
 * @param   pwm : PWM 句柄指针.
 * @return  成功返回通道号，失败返回 ATIM_PWM_CH_INVALID.
 * @example atim_pwm_channel_t ch = ls2k0300_atim_pwm_get_channel(&pwm);
 ********************************************************************************/
atim_pwm_channel_t ls2k0300_atim_pwm_get_channel(ls2k0300_atim_pwm_t *pwm);

/********************************************************************************
 * @brief   获取当前极性.
 * @param   pwm : PWM 句柄指针.
 * @return  成功返回极性，失败返回 ATIM_PWM_POL_INVALID.
 * @example atim_pwm_polarity_t p = ls2k0300_atim_pwm_get_polarity(&pwm);
 ********************************************************************************/
atim_pwm_polarity_t ls2k0300_atim_pwm_get_polarity(ls2k0300_atim_pwm_t *pwm);

/********************************************************************************
 * @brief   获取当前频率配置.
 * @param   pwm : PWM 句柄指针.
 * @return  成功返回频率(Hz)，失败返回 0.
 * @example uint32_t hz = ls2k0300_atim_pwm_get_period(&pwm);
 ********************************************************************************/
uint32_t ls2k0300_atim_pwm_get_period(ls2k0300_atim_pwm_t *pwm);

/********************************************************************************
 * @brief   获取当前占空比配置.
 * @param   pwm : PWM 句柄指针.
 * @return  成功返回占空比(0~10000)，失败返回 0.
 * @example uint32_t duty = ls2k0300_atim_pwm_get_duty(&pwm);
 ********************************************************************************/
uint32_t ls2k0300_atim_pwm_get_duty(ls2k0300_atim_pwm_t *pwm);

#ifdef __cplusplus
}
#endif

#endif
