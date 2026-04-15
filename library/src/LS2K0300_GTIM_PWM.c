#include "LS2K0300_GTIM_PWM.h"
#include "LS2K0300_CLOCK.h"
#include "LS2K0300_MAP.h"

#include <string.h>

/********************************************************************************
 * @brief   GTIM PWM 寄存器定义.
 ********************************************************************************/
#define LS_GTIM_BASE_ADDR       (0x16119000U)

#define LS_GTIM_CR1             (0x00U)
#define LS_GTIM_EGR             (0x14U)
#define LS_GTIM_CCMR1           (0x18U)
#define LS_GTIM_CCMR2           (0x1CU)
#define LS_GTIM_CCER            (0x20U)
#define LS_GTIM_CNT             (0x24U)
#define LS_GTIM_ARR             (0x2CU)
#define LS_GTIM_CCR1            (0x34U)

#define LS_GTIM_CCRX_OFS        (0x04U)
#define GTIM_PWM_DUTY_MAX       (10000U)

static pthread_mutex_t gtim_duty_mutex = PTHREAD_MUTEX_INITIALIZER;

/********************************************************************************
 * @brief   设置 GTIM PWM 模式.
 * @param   pwm  : PWM 句柄指针.
 * @param   mode : PWM 模式.
 * @return  none.
 * @example ls2k0300_gtim_pwm_set_mode(&pwm, GTIM_PWM_MODE_2);
 ********************************************************************************/
void ls2k0300_gtim_pwm_set_mode(ls2k0300_gtim_pwm_t *pwm, gtim_pwm_mode_t mode)
{
    uint32_t reg;

    if (pwm == NULL || pwm->initialized == 0 || mode >= GTIM_PWM_MODE_INVALID) {
        return;
    }

    pthread_mutex_lock(&pwm->mtx);

    pwm->mode = mode;
    /* 每个通道占 8bit，设置 OCxM 以及预装载使能位 */
    reg = ls_readl(pwm->gtim_ccmr[pwm->ch / 2U]);
    reg &= ~(0x7U << ((pwm->ch % 2U) * 8U + 4U));
    reg |= ((uint32_t)mode << ((pwm->ch % 2U) * 8U + 4U));
    reg &= ~(0x3U << ((pwm->ch % 2U) * 8U));
    reg &= ~(1U << ((pwm->ch % 2U) * 8U + 3U));
    reg |= (1U << ((pwm->ch % 2U) * 8U + 3U));
    ls_writel(pwm->gtim_ccmr[pwm->ch / 2U], reg);

    pthread_mutex_unlock(&pwm->mtx);
}

/********************************************************************************
 * @brief   设置 GTIM PWM 极性.
 * @param   pwm  : PWM 句柄指针.
 * @param   pola : PWM 极性.
 * @return  none.
 * @example ls2k0300_gtim_pwm_set_polarity(&pwm, GTIM_PWM_POL_NORMAL);
 ********************************************************************************/
void ls2k0300_gtim_pwm_set_polarity(ls2k0300_gtim_pwm_t *pwm, gtim_pwm_polarity_t pola)
{
    uint32_t reg;

    if (pwm == NULL || pwm->initialized == 0 || pola >= GTIM_PWM_POL_INVALID) {
        return;
    }

    pthread_mutex_lock(&pwm->mtx);

    pwm->pola = pola;
    /* CCxP 位控制通道极性 */
    reg = ls_readl(pwm->gtim_ccer);
    reg &= ~(0x1U << (pwm->ch * 4U + 1U));
    reg |= ((uint32_t)pola << (pwm->ch * 4U + 1U));
    ls_writel(pwm->gtim_ccer, reg);

    pthread_mutex_unlock(&pwm->mtx);
}

/********************************************************************************
 * @brief   使能 GTIM PWM 输出.
 * @param   pwm : PWM 句柄指针.
 * @return  none.
 * @example ls2k0300_gtim_pwm_enable(&pwm);
 ********************************************************************************/
void ls2k0300_gtim_pwm_enable(ls2k0300_gtim_pwm_t *pwm)
{
    if (pwm == NULL || pwm->initialized == 0) {
        return;
    }

    pthread_mutex_lock(&pwm->enable_mtx);
    /* 仅开启当前通道输出使能位 */
    ls_writel(pwm->gtim_ccer, ls_readl(pwm->gtim_ccer) | (0x1U << (pwm->ch * 4U)));
    pthread_mutex_unlock(&pwm->enable_mtx);
}

/********************************************************************************
 * @brief   禁用 GTIM PWM 输出.
 * @param   pwm : PWM 句柄指针.
 * @return  none.
 * @example ls2k0300_gtim_pwm_disable(&pwm);
 ********************************************************************************/
void ls2k0300_gtim_pwm_disable(ls2k0300_gtim_pwm_t *pwm)
{
    if (pwm == NULL || pwm->initialized == 0) {
        return;
    }

    pthread_mutex_lock(&pwm->enable_mtx);
    /* 清除当前通道输出使能位 */
    ls_writel(pwm->gtim_ccer, ls_readl(pwm->gtim_ccer) & ~(0x1U << (pwm->ch * 4U)));
    pthread_mutex_unlock(&pwm->enable_mtx);
}

/********************************************************************************
 * @brief   设置 GTIM PWM 周期.
 * @param   pwm    : PWM 句柄指针.
 * @param   period : 目标频率(Hz).
 * @return  none.
 * @example ls2k0300_gtim_pwm_set_period(&pwm, 1000);
 ********************************************************************************/
void ls2k0300_gtim_pwm_set_period(ls2k0300_gtim_pwm_t *pwm, uint32_t period)
{
    if (pwm == NULL || pwm->initialized == 0 || period == 0U || period > (uint32_t)LS_PMON_CLOCK_FREQ) {
        return;
    }

    pthread_mutex_lock(&pwm->mtx);

    pwm->period = period;
    /* 先关输出后改 ARR，避免修改周期时产生毛刺 */
    ls2k0300_gtim_pwm_disable(pwm);
    ls_writel(pwm->gtim_arr, (uint32_t)(LS_PMON_CLOCK_FREQ / (long)period) - 1U);
    ls2k0300_gtim_pwm_enable(pwm);

    pthread_mutex_unlock(&pwm->mtx);
}

/********************************************************************************
 * @brief   设置 GTIM PWM 占空比.
 * @param   pwm  : PWM 句柄指针.
 * @param   duty : 占空比，范围 0~10000.
 * @return  none.
 * @example ls2k0300_gtim_pwm_set_duty(&pwm, 3000);
 ********************************************************************************/
void ls2k0300_gtim_pwm_set_duty(ls2k0300_gtim_pwm_t *pwm, uint32_t duty)
{
    if (pwm == NULL || pwm->initialized == 0 || duty > GTIM_PWM_DUTY_MAX || pwm->period == 0U) {
        return;
    }

    pthread_mutex_lock(&pwm->mtx);
    pthread_mutex_lock(&gtim_duty_mutex);

    pwm->duty = duty;
    /* CCR = 周期计数 * 占空比比例 */
    ls_writel(pwm->gtim_ccrx,
              (uint32_t)((LS_PMON_CLOCK_FREQ / (long)pwm->period) * (long)duty / (long)GTIM_PWM_DUTY_MAX));

    pthread_mutex_unlock(&gtim_duty_mutex);
    pthread_mutex_unlock(&pwm->mtx);
}

/********************************************************************************
 * @brief   初始化 GTIM PWM.
 * @param   pwm    : PWM 句柄指针.
 * @param   pin    : 引脚与通道定义.
 * @param   period : 目标频率(Hz).
 * @param   duty   : 占空比，范围 0~10000.
 * @param   pola   : 输出极性.
 * @return  成功返回 0，失败返回 -1.
 * @example ls2k0300_gtim_pwm_init(&pwm, GTIM_PWM0_PIN34, 1000, 5000, GTIM_PWM_POL_NORMAL);
 ********************************************************************************/
int ls2k0300_gtim_pwm_init(ls2k0300_gtim_pwm_t *pwm, gtim_pwm_pin_t pin, uint32_t period, uint32_t duty, gtim_pwm_polarity_t pola)
{
    if (pwm == NULL || pola >= GTIM_PWM_POL_INVALID) {
        return -1;
    }

    memset(pwm, 0, sizeof(*pwm));
    pthread_mutex_init(&pwm->mtx, NULL);
    pthread_mutex_init(&pwm->enable_mtx, NULL);

    pwm->gpio = (gpio_pin_t)((uint32_t)pin & 0xFFU);
    pwm->ch = (gtim_pwm_channel_t)(((uint32_t)pin >> 8U) & 0x03U);
    pwm->mux = (gpio_mux_mode_t)(((uint32_t)pin >> 10U) & 0x03U);

    /* 先切换 GPIO 复用为定时器输出功能 */
    ls2k0300_gpio_mux_set(pwm->gpio, pwm->mux);

    pwm->gtim_base = (ls_reg32_addr_t)ls2k0300_mmap(LS_GTIM_BASE_ADDR, 0x1000);
    if (pwm->gtim_base == NULL) {
        pthread_mutex_destroy(&pwm->mtx);
        pthread_mutex_destroy(&pwm->enable_mtx);
        return -1;
    }

    pwm->gtim_arr = ls_reg_addr_calc(pwm->gtim_base, LS_GTIM_ARR);
    pwm->gtim_ccrx = ls_reg_addr_calc(pwm->gtim_base, LS_GTIM_CCR1 + pwm->ch * LS_GTIM_CCRX_OFS);
    pwm->gtim_ccmr[0] = ls_reg_addr_calc(pwm->gtim_base, LS_GTIM_CCMR1);
    pwm->gtim_ccmr[1] = ls_reg_addr_calc(pwm->gtim_base, LS_GTIM_CCMR2);
    pwm->gtim_ccer = ls_reg_addr_calc(pwm->gtim_base, LS_GTIM_CCER);
    pwm->gtim_cnt = ls_reg_addr_calc(pwm->gtim_base, LS_GTIM_CNT);
    pwm->gtim_egr = ls_reg_addr_calc(pwm->gtim_base, LS_GTIM_EGR);
    pwm->gtim_cr1 = ls_reg_addr_calc(pwm->gtim_base, LS_GTIM_CR1);

    pwm->initialized = 1;

    /* UG 触发更新，保证 ARR/CCR 预装载立刻生效 */
    ls_writel(pwm->gtim_egr, 0x01U);

    ls2k0300_gtim_pwm_set_mode(pwm, GTIM_PWM_MODE_2);
    ls2k0300_gtim_pwm_set_polarity(pwm, pola);
    ls2k0300_gtim_pwm_set_period(pwm, period);
    ls2k0300_gtim_pwm_set_duty(pwm, duty);

    ls_writel(pwm->gtim_cr1, 0x81U);
    return 0;
}

/********************************************************************************
 * @brief   释放 GTIM PWM.
 * @param   pwm : PWM 句柄指针.
 * @return  none.
 * @example ls2k0300_gtim_pwm_deinit(&pwm);
 ********************************************************************************/
void ls2k0300_gtim_pwm_deinit(ls2k0300_gtim_pwm_t *pwm)
{
    if (pwm == NULL) {
        return;
    }

    ls2k0300_gtim_pwm_disable(pwm);

    if (pwm->gtim_base != NULL) {
        ls2k0300_munmap((void *)pwm->gtim_base, 0x1000);
    }

    pthread_mutex_destroy(&pwm->mtx);
    pthread_mutex_destroy(&pwm->enable_mtx);

    memset(pwm, 0, sizeof(*pwm));
}

/********************************************************************************
 * @brief   获取 GTIM PWM 引脚.
 * @param   pwm : PWM 句柄指针.
 * @return  成功返回引脚号，失败返回 PIN_INVALID.
 * @example gpio_pin_t pin = ls2k0300_gtim_pwm_get_gpio(&pwm);
 ********************************************************************************/
gpio_pin_t ls2k0300_gtim_pwm_get_gpio(ls2k0300_gtim_pwm_t *pwm)
{
    return (pwm == NULL) ? PIN_INVALID : pwm->gpio;
}

/********************************************************************************
 * @brief   获取 GTIM PWM 复用.
 * @param   pwm : PWM 句柄指针.
 * @return  成功返回复用模式，失败返回 GPIO_MUX_INVALID.
 * @example gpio_mux_mode_t mux = ls2k0300_gtim_pwm_get_mux(&pwm);
 ********************************************************************************/
gpio_mux_mode_t ls2k0300_gtim_pwm_get_mux(ls2k0300_gtim_pwm_t *pwm)
{
    return (pwm == NULL) ? GPIO_MUX_INVALID : pwm->mux;
}

/********************************************************************************
 * @brief   获取 GTIM PWM 通道.
 * @param   pwm : PWM 句柄指针.
 * @return  成功返回通道号，失败返回 GTIM_PWM_CH_INVALID.
 * @example gtim_pwm_channel_t ch = ls2k0300_gtim_pwm_get_channel(&pwm);
 ********************************************************************************/
gtim_pwm_channel_t ls2k0300_gtim_pwm_get_channel(ls2k0300_gtim_pwm_t *pwm)
{
    return (pwm == NULL) ? GTIM_PWM_CH_INVALID : pwm->ch;
}

/********************************************************************************
 * @brief   获取 GTIM PWM 极性.
 * @param   pwm : PWM 句柄指针.
 * @return  成功返回极性，失败返回 GTIM_PWM_POL_INVALID.
 * @example gtim_pwm_polarity_t p = ls2k0300_gtim_pwm_get_polarity(&pwm);
 ********************************************************************************/
gtim_pwm_polarity_t ls2k0300_gtim_pwm_get_polarity(ls2k0300_gtim_pwm_t *pwm)
{
    return (pwm == NULL) ? GTIM_PWM_POL_INVALID : pwm->pola;
}

/********************************************************************************
 * @brief   获取 GTIM PWM 周期.
 * @param   pwm : PWM 句柄指针.
 * @return  成功返回频率(Hz)，失败返回 0.
 * @example uint32_t hz = ls2k0300_gtim_pwm_get_period(&pwm);
 ********************************************************************************/
uint32_t ls2k0300_gtim_pwm_get_period(ls2k0300_gtim_pwm_t *pwm)
{
    return (pwm == NULL) ? 0U : pwm->period;
}

/********************************************************************************
 * @brief   获取 GTIM PWM 占空比.
 * @param   pwm : PWM 句柄指针.
 * @return  成功返回占空比，失败返回 0.
 * @example uint32_t duty = ls2k0300_gtim_pwm_get_duty(&pwm);
 ********************************************************************************/
uint32_t ls2k0300_gtim_pwm_get_duty(ls2k0300_gtim_pwm_t *pwm)
{
    return (pwm == NULL) ? 0U : pwm->duty;
}
