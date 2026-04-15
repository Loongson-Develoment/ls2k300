#include "LS2K0300_PWM.h"
#include "LS2K0300_CLOCK.h"

/********************************************************************************
 * @brief   PWM 寄存器定义.
 ********************************************************************************/
#define LS_PWM_BASE_ADDR      (0x1611B000U)
#define LS_PWM_OFF            (0x10U)
#define LS_PWM_LOW_BUF_OFF    (0x04U)
#define LS_PWM_FULL_BUF_OFF   (0x08U)
#define LS_PWM_CTRL_OFF       (0x0CU)

#define LS_PWM_CTRL_EN        BIT(0)
#define LS_PWM_CTRL_INVERT    BIT(9)

#define PWM_DUTY_MAX          (10000U)

/********************************************************************************
 * @brief   使能 PWM 输出.
 * @param   pwm : PWM 句柄指针.
 * @return  none.
 * @example ls2k0300_pwm_enable(&pwm);
 ********************************************************************************/
void ls2k0300_pwm_enable(ls2k0300_pwm_t *pwm)
{
    if (pwm == NULL || pwm->initialized == 0 || pwm->pwm_ctrl == NULL) {
        return;
    }

    pthread_mutex_lock(&pwm->mtx);
    /* 控制位 EN=1 后开始输出 PWM 波形 */
    ls_writel(pwm->pwm_ctrl, ls_readl(pwm->pwm_ctrl) | LS_PWM_CTRL_EN);
    pthread_mutex_unlock(&pwm->mtx);
}

/********************************************************************************
 * @brief   禁用 PWM 输出.
 * @param   pwm : PWM 句柄指针.
 * @return  none.
 * @example ls2k0300_pwm_disable(&pwm);
 ********************************************************************************/
void ls2k0300_pwm_disable(ls2k0300_pwm_t *pwm)
{
    if (pwm == NULL || pwm->initialized == 0 || pwm->pwm_ctrl == NULL) {
        return;
    }

    pthread_mutex_lock(&pwm->mtx);
    /* 清除 EN 位停止输出 */
    ls_writel(pwm->pwm_ctrl, ls_readl(pwm->pwm_ctrl) & ~LS_PWM_CTRL_EN);
    pthread_mutex_unlock(&pwm->mtx);
}

/********************************************************************************
 * @brief   设置 PWM 极性.
 * @param   pwm  : PWM 句柄指针.
 * @param   pola : 极性配置.
 * @return  none.
 * @example ls2k0300_pwm_set_polarity(&pwm, PWM_POL_INV);
 ********************************************************************************/
void ls2k0300_pwm_set_polarity(ls2k0300_pwm_t *pwm, pwm_polarity_t pola)
{
    if (pwm == NULL || pwm->initialized == 0 || pwm->pwm_ctrl == NULL || pola >= PWM_POL_INVALID) {
        return;
    }

    pthread_mutex_lock(&pwm->mtx);

    pwm->pola = pola;
    if (pola == PWM_POL_NORMAL) {
        /* INVERT=0 正常极性 */
        ls_writel(pwm->pwm_ctrl, ls_readl(pwm->pwm_ctrl) & ~LS_PWM_CTRL_INVERT);
    } else {
        /* INVERT=1 反相输出 */
        ls_writel(pwm->pwm_ctrl, ls_readl(pwm->pwm_ctrl) | LS_PWM_CTRL_INVERT);
    }

    pthread_mutex_unlock(&pwm->mtx);
}

/********************************************************************************
 * @brief   设置 PWM 周期.
 * @param   pwm    : PWM 句柄指针.
 * @param   period : 目标频率(Hz).
 * @return  none.
 * @example ls2k0300_pwm_set_period(&pwm, 1000);
 ********************************************************************************/
void ls2k0300_pwm_set_period(ls2k0300_pwm_t *pwm, uint32_t period)
{
    uint32_t reg;

    if (pwm == NULL || pwm->initialized == 0 || pwm->pwm_full_buf == NULL) {
        return;
    }
    if (period == 0U || period > (uint32_t)LS_PMON_CLOCK_FREQ) {
        return;
    }

    pthread_mutex_lock(&pwm->mtx);

    pwm->period = period;

    /* 先停止输出再更新周期寄存器，避免周期切换毛刺 */
    ls_writel(pwm->pwm_ctrl, ls_readl(pwm->pwm_ctrl) & ~LS_PWM_CTRL_EN);
    reg = (uint32_t)(LS_PMON_CLOCK_FREQ / (long)period) - 1U;
    ls_writel(pwm->pwm_full_buf, reg);
    ls_writel(pwm->pwm_ctrl, ls_readl(pwm->pwm_ctrl) | LS_PWM_CTRL_EN);

    pthread_mutex_unlock(&pwm->mtx);
}

/********************************************************************************
 * @brief   设置 PWM 占空比.
 * @param   pwm  : PWM 句柄指针.
 * @param   duty : 占空比，范围 0~10000.
 * @return  none.
 * @example ls2k0300_pwm_set_duty(&pwm, 3000);
 ********************************************************************************/
void ls2k0300_pwm_set_duty(ls2k0300_pwm_t *pwm, uint32_t duty)
{
    uint32_t reg;

    if (pwm == NULL || pwm->initialized == 0 || pwm->pwm_low_buf == NULL || duty > PWM_DUTY_MAX) {
        return;
    }
    if (pwm->period == 0U) {
        return;
    }

    pthread_mutex_lock(&pwm->mtx);

    pwm->duty = duty;
    /* LOW_BUF 对应低电平持续计数，按占空比比例计算 */
    reg = (uint32_t)((LS_PMON_CLOCK_FREQ / (long)pwm->period) * (long)duty / (long)PWM_DUTY_MAX);
    ls_writel(pwm->pwm_low_buf, reg);

    pthread_mutex_unlock(&pwm->mtx);
}

/********************************************************************************
 * @brief   初始化 PWM 句柄.
 * @param   pwm      : PWM 句柄指针.
 * @param   pin_info : 引脚与通道定义.
 * @param   period   : 目标频率(Hz).
 * @param   duty     : 占空比，范围 0~10000.
 * @param   pola     : 输出极性.
 * @return  成功返回 0，失败返回 -1.
 * @example ls2k0300_pwm_init(&pwm, PWM0_PIN64, 1000, 5000, PWM_POL_NORMAL);
 ********************************************************************************/
int ls2k0300_pwm_init(ls2k0300_pwm_t *pwm, pwm_pin_t pin_info, uint32_t period, uint32_t duty, pwm_polarity_t pola)
{
    if (pwm == NULL || pola >= PWM_POL_INVALID) {
        return -1;
    }

    pthread_mutex_init(&pwm->mtx, NULL);

    pwm->gpio = (gpio_pin_t)(pin_info & 0xFFU);
    pwm->ch = (pwm_channel_t)((pin_info >> 8U) & 0x03U);
    pwm->mux = (gpio_mux_mode_t)((pin_info >> 10U) & 0x03U);
    pwm->period = 0;
    pwm->duty = 0;
    pwm->initialized = 0;

    /* 先设置引脚复用到 PWM 功能 */
    ls2k0300_gpio_mux_set(pwm->gpio, pwm->mux);

    pwm->pwm_base = (ls_reg32_addr_t)ls2k0300_mmap(LS_PWM_BASE_ADDR + ((uint32_t)pwm->ch * LS_PWM_OFF), 0x1000);
    if (pwm->pwm_base == NULL) {
        pthread_mutex_destroy(&pwm->mtx);
        return -1;
    }

    pwm->pwm_low_buf = ls_reg_addr_calc(pwm->pwm_base, LS_PWM_LOW_BUF_OFF);
    pwm->pwm_full_buf = ls_reg_addr_calc(pwm->pwm_base, LS_PWM_FULL_BUF_OFF);
    pwm->pwm_ctrl = ls_reg_addr_calc(pwm->pwm_base, LS_PWM_CTRL_OFF);
    pwm->initialized = 1;

    /* 依次配置极性、频率、占空比 */
    ls2k0300_pwm_set_polarity(pwm, pola);
    ls2k0300_pwm_set_period(pwm, period);
    ls2k0300_pwm_set_duty(pwm, duty);

    return 0;
}

/********************************************************************************
 * @brief   释放 PWM 句柄.
 * @param   pwm : PWM 句柄指针.
 * @return  none.
 * @example ls2k0300_pwm_deinit(&pwm);
 ********************************************************************************/
void ls2k0300_pwm_deinit(ls2k0300_pwm_t *pwm)
{
    if (pwm == NULL) {
        return;
    }

    pthread_mutex_lock(&pwm->mtx);

    if (pwm->initialized != 0 && pwm->pwm_ctrl != NULL) {
        ls_writel(pwm->pwm_ctrl, ls_readl(pwm->pwm_ctrl) & ~LS_PWM_CTRL_EN);
    }

    if (pwm->pwm_base != NULL) {
        ls2k0300_munmap((void *)pwm->pwm_base, 0x1000);
    }

    pwm->pwm_base = NULL;
    pwm->pwm_low_buf = NULL;
    pwm->pwm_full_buf = NULL;
    pwm->pwm_ctrl = NULL;
    pwm->gpio = PIN_INVALID;
    pwm->mux = GPIO_MUX_INVALID;
    pwm->ch = PWM_CH_INVALID;
    pwm->pola = PWM_POL_INVALID;
    pwm->period = 0;
    pwm->duty = 0;
    pwm->initialized = 0;

    pthread_mutex_unlock(&pwm->mtx);
    pthread_mutex_destroy(&pwm->mtx);
}

/********************************************************************************
 * @brief   获取 PWM 引脚.
 * @param   pwm : PWM 句柄指针.
 * @return  成功返回引脚号，失败返回 PIN_INVALID.
 * @example gpio_pin_t pin = ls2k0300_pwm_get_gpio(&pwm);
 ********************************************************************************/
gpio_pin_t ls2k0300_pwm_get_gpio(ls2k0300_pwm_t *pwm)
{
    if (pwm == NULL) {
        return PIN_INVALID;
    }
    return pwm->gpio;
}

/********************************************************************************
 * @brief   获取 PWM 复用模式.
 * @param   pwm : PWM 句柄指针.
 * @return  成功返回复用模式，失败返回 GPIO_MUX_INVALID.
 * @example gpio_mux_mode_t mux = ls2k0300_pwm_get_mux(&pwm);
 ********************************************************************************/
gpio_mux_mode_t ls2k0300_pwm_get_mux(ls2k0300_pwm_t *pwm)
{
    if (pwm == NULL) {
        return GPIO_MUX_INVALID;
    }
    return pwm->mux;
}

/********************************************************************************
 * @brief   获取 PWM 通道.
 * @param   pwm : PWM 句柄指针.
 * @return  成功返回通道号，失败返回 PWM_CH_INVALID.
 * @example pwm_channel_t ch = ls2k0300_pwm_get_channel(&pwm);
 ********************************************************************************/
pwm_channel_t ls2k0300_pwm_get_channel(ls2k0300_pwm_t *pwm)
{
    if (pwm == NULL) {
        return PWM_CH_INVALID;
    }
    return pwm->ch;
}

/********************************************************************************
 * @brief   获取 PWM 极性.
 * @param   pwm : PWM 句柄指针.
 * @return  成功返回极性，失败返回 PWM_POL_INVALID.
 * @example pwm_polarity_t p = ls2k0300_pwm_get_polarity(&pwm);
 ********************************************************************************/
pwm_polarity_t ls2k0300_pwm_get_polarity(ls2k0300_pwm_t *pwm)
{
    if (pwm == NULL) {
        return PWM_POL_INVALID;
    }
    return pwm->pola;
}

/********************************************************************************
 * @brief   获取 PWM 周期.
 * @param   pwm : PWM 句柄指针.
 * @return  成功返回频率(Hz)，失败返回 0.
 * @example uint32_t hz = ls2k0300_pwm_get_period(&pwm);
 ********************************************************************************/
uint32_t ls2k0300_pwm_get_period(ls2k0300_pwm_t *pwm)
{
    if (pwm == NULL) {
        return 0;
    }
    return pwm->period;
}

/********************************************************************************
 * @brief   获取 PWM 占空比.
 * @param   pwm : PWM 句柄指针.
 * @return  成功返回占空比，失败返回 0.
 * @example uint32_t duty = ls2k0300_pwm_get_duty(&pwm);
 ********************************************************************************/
uint32_t ls2k0300_pwm_get_duty(ls2k0300_pwm_t *pwm)
{
    if (pwm == NULL) {
        return 0;
    }
    return pwm->duty;
}
