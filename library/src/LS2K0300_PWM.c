#include "LS2K0300_PWM.h"
#include <stdio.h>

#define LS_PWM_BASE_ADDR            ( 0x1611B000 )
#define LS_PWM_OFF                  ( 0x10 )
#define LS_PWM_LOW_BUF_OFF          ( 0x04 )
#define LS_PWM_FULL_BUF_OFF         ( 0x08 )
#define LS_PWM_CTRL_OFF             ( 0x0C )

#define LS_PWM_CTRL_EN              BIT(0)
#define LS_PWM_CTRL_INVERT          BIT(9)

#define CONFIG_USE_PMON         ( 1 )
#if (CONFIG_USE_PMON == 1)
    #define LS_PMON_CLOCK_FREQ  ( 160000000L )
#else
    #define LS_PMON_CLOCK_FREQ  ( 100000000L )
#endif

#define PWM_CLK_FRE                 ( LS_PMON_CLOCK_FREQ )
#define PWM_DUTY_MAX                ( 10000 )

void ls2k0300_pwm_enable(ls2k0300_pwm_t* pwm) {
    if (pwm == NULL || pwm->pwm_ctrl == NULL) return;
    ls_writel(pwm->pwm_ctrl, ls_readl(pwm->pwm_ctrl) | LS_PWM_CTRL_EN);
}

void ls2k0300_pwm_disable(ls2k0300_pwm_t* pwm) {
    if (pwm == NULL || pwm->pwm_ctrl == NULL) return;
    ls_writel(pwm->pwm_ctrl, ls_readl(pwm->pwm_ctrl) & ~LS_PWM_CTRL_EN);
}

void ls2k0300_pwm_set_polarity(ls2k0300_pwm_t* pwm, pwm_polarity_t pola) {
    if (pwm == NULL || pola >= PWM_POL_INVALID || pwm->pwm_ctrl == NULL) return;
    pthread_mutex_lock(&pwm->mtx);
    pwm->pola = pola;
    if (pola == PWM_POL_NORMAL) {
        ls_writel(pwm->pwm_ctrl, ls_readl(pwm->pwm_ctrl) & ~LS_PWM_CTRL_INVERT);
    } else {
        ls_writel(pwm->pwm_ctrl, ls_readl(pwm->pwm_ctrl) | LS_PWM_CTRL_INVERT);
    }
    pthread_mutex_unlock(&pwm->mtx);
}

void ls2k0300_pwm_set_period(ls2k0300_pwm_t* pwm, uint32_t period) {
    if (pwm == NULL || period > PWM_CLK_FRE || pwm->pwm_full_buf == NULL) return;
    pthread_mutex_lock(&pwm->mtx);
    pwm->period = period;
    ls2k0300_pwm_disable(pwm);
    uint32_t val = (period == 0) ? 0 : (PWM_CLK_FRE / period - 1);
    ls_writel(pwm->pwm_full_buf, val);
    ls2k0300_pwm_enable(pwm);
    pthread_mutex_unlock(&pwm->mtx);
}

void ls2k0300_pwm_set_duty(ls2k0300_pwm_t* pwm, uint32_t duty) {
    if (pwm == NULL || duty > PWM_DUTY_MAX || pwm->pwm_low_buf == NULL) return;
    pthread_mutex_lock(&pwm->mtx);
    pwm->duty = duty;
    ls2k0300_pwm_enable(pwm);
    uint32_t val = 0;
    if (pwm->period > 0) {
        val = (PWM_CLK_FRE / pwm->period) * duty / PWM_DUTY_MAX;
    }
    ls_writel(pwm->pwm_low_buf, val);
    ls2k0300_pwm_enable(pwm);
    pthread_mutex_unlock(&pwm->mtx);
}

int ls2k0300_pwm_init(ls2k0300_pwm_t* pwm, pwm_pin_t pin_info, uint32_t period, uint32_t duty, pwm_polarity_t pola) {
    if (pwm == NULL) return -1;
    
    pthread_mutex_init(&pwm->mtx, NULL);

    pwm->gpio = (gpio_pin_t)(pin_info & 0xFF);
    pwm->ch = (pwm_channel_t)((pin_info >> 8) & 0x03);
    pwm->mux = (gpio_mux_mode_t)((pin_info >> 10) & 0x03);

    ls2k0300_gpio_mux_set(pwm->gpio, pwm->mux);

    pwm->pwm_base = (ls_reg32_addr_t)ls2k0300_mmap(LS_PWM_BASE_ADDR + (pwm->ch * LS_PWM_OFF), 0x1000);
    if (pwm->pwm_base == NULL) {
        pthread_mutex_destroy(&pwm->mtx);
        return -1;
    }

    pwm->pwm_low_buf = ls_reg_addr_calc(pwm->pwm_base, LS_PWM_LOW_BUF_OFF);
    pwm->pwm_full_buf = ls_reg_addr_calc(pwm->pwm_base, LS_PWM_FULL_BUF_OFF);
    pwm->pwm_ctrl = ls_reg_addr_calc(pwm->pwm_base, LS_PWM_CTRL_OFF);

    ls2k0300_pwm_set_polarity(pwm, pola);
    ls2k0300_pwm_set_period(pwm, period);
    ls2k0300_pwm_set_duty(pwm, duty);
    
    return 0;
}

void ls2k0300_pwm_deinit(ls2k0300_pwm_t* pwm) {
    if (pwm == NULL) return;
    pthread_mutex_lock(&pwm->mtx);
    ls2k0300_pwm_disable(pwm);
    if (pwm->pwm_base) {
        ls2k0300_munmap((void*)pwm->pwm_base, 0x1000);
        pwm->pwm_base = NULL;
    }
    pthread_mutex_unlock(&pwm->mtx);
    pthread_mutex_destroy(&pwm->mtx);
}
