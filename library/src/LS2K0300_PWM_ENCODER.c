#include "LS2K0300_PWM_ENCODER.h"
#include "LS2K0300_MAP.h"

#include <string.h>

/********************************************************************************
 * @brief   编码器 PWM 寄存器定义.
 ********************************************************************************/
#define LS_ENC_PWM_BASE_ADDR        (0x1611B000U)
#define LS_ENC_PWM_OFFSET           (0x10U)
#define LS_ENC_PWM_LOW_BUF_OFFSET   (0x04U)
#define LS_ENC_PWM_FULL_BUF_OFFSET  (0x08U)
#define LS_ENC_PWM_CTRL_OFFSET      (0x0CU)

#define LS_ENC_PWM_CTRL_EN          BIT(0)
#define LS_ENC_PWM_CTRL_INTE        BIT(5)
#define LS_ENC_PWM_CTRL_RST         BIT(7)
#define LS_ENC_PWM_CTRL_CAPTE       BIT(8)

/********************************************************************************
 * @brief   初始化编码器 PWM.
 * @param   enc     : 编码器句柄指针.
 * @param   pin     : 编码器脉冲输入定义.
 * @param   dir_pin : 方向引脚.
 * @return  成功返回 0，失败返回 -1.
 * @example ls2k0300_pwm_encoder_init(&enc, ENC_PWM0_PIN64, PIN_10);
 ********************************************************************************/
int ls2k0300_pwm_encoder_init(ls2k0300_pwm_encoder_t *enc, ls_enc_pwm_pin_t pin, gpio_pin_t dir_pin)
{
    uint32_t reg;

    if (enc == NULL) {
        return -1;
    }

    memset(enc, 0, sizeof(*enc));
    pthread_mutex_init(&enc->mtx, NULL);

    enc->gpio = (gpio_pin_t)((uint32_t)pin & 0xFFU);
    enc->mux = (gpio_mux_mode_t)(((uint32_t)pin >> 10U) & 0x03U);
    enc->ch = (enc_pwm_channel_t)(((uint32_t)pin >> 8U) & 0x03U);

    /* 方向脚作为普通 GPIO 输入读取方向电平 */
    if (ls2k0300_gpio_init(&enc->dir, dir_pin, GPIO_MODE_IN, GPIO_MUX_GPIO) != 0) {
        pthread_mutex_destroy(&enc->mtx);
        return -1;
    }

    ls2k0300_gpio_mux_set(enc->gpio, enc->mux);

    enc->enc_pwm_base = (ls_reg32_addr_t)ls2k0300_mmap(LS_ENC_PWM_BASE_ADDR + ((uint32_t)enc->ch * LS_ENC_PWM_OFFSET), 0x1000);
    if (enc->enc_pwm_base == NULL) {
        ls2k0300_gpio_deinit(&enc->dir);
        pthread_mutex_destroy(&enc->mtx);
        return -1;
    }

    enc->enc_pwm_low_buf = ls_reg_addr_calc(enc->enc_pwm_base, LS_ENC_PWM_LOW_BUF_OFFSET);
    enc->enc_pwm_full_buf = ls_reg_addr_calc(enc->enc_pwm_base, LS_ENC_PWM_FULL_BUF_OFFSET);
    enc->enc_pwm_ctrl = ls_reg_addr_calc(enc->enc_pwm_base, LS_ENC_PWM_CTRL_OFFSET);

    /* 开启捕获与中断逻辑，让 full_buf 持续更新周期计数 */
    ls_writel(enc->enc_pwm_ctrl, 0);
    reg = LS_ENC_PWM_CTRL_EN | LS_ENC_PWM_CTRL_CAPTE | LS_ENC_PWM_CTRL_INTE;
    ls_writel(enc->enc_pwm_ctrl, ls_readl(enc->enc_pwm_ctrl) | reg);

    enc->initialized = 1;
    return 0;
}

/********************************************************************************
 * @brief   释放编码器 PWM.
 * @param   enc : 编码器句柄指针.
 * @return  none.
 * @example ls2k0300_pwm_encoder_deinit(&enc);
 ********************************************************************************/
void ls2k0300_pwm_encoder_deinit(ls2k0300_pwm_encoder_t *enc)
{
    if (enc == NULL) {
        return;
    }

    if (enc->enc_pwm_base != NULL) {
        ls2k0300_munmap((void *)enc->enc_pwm_base, 0x1000);
    }

    ls2k0300_gpio_deinit(&enc->dir);
    pthread_mutex_destroy(&enc->mtx);

    memset(enc, 0, sizeof(*enc));
}

/********************************************************************************
 * @brief   重置编码器计数器.
 * @param   enc : 编码器句柄指针.
 * @return  none.
 * @example ls2k0300_pwm_encoder_reset_counter(&enc);
 ********************************************************************************/
void ls2k0300_pwm_encoder_reset_counter(ls2k0300_pwm_encoder_t *enc)
{
    if (enc == NULL || enc->initialized == 0) {
        return;
    }

    pthread_mutex_lock(&enc->mtx);
    /* RST=1 触发硬件计数器复位 */
    ls_writel(enc->enc_pwm_ctrl, ls_readl(enc->enc_pwm_ctrl) | LS_ENC_PWM_CTRL_RST);
    pthread_mutex_unlock(&enc->mtx);
}

/********************************************************************************
 * @brief   关闭编码器计数器重置.
 * @param   enc : 编码器句柄指针.
 * @return  none.
 * @example ls2k0300_pwm_encoder_close_reset_counter(&enc);
 ********************************************************************************/
void ls2k0300_pwm_encoder_close_reset_counter(ls2k0300_pwm_encoder_t *enc)
{
    if (enc == NULL || enc->initialized == 0) {
        return;
    }

    pthread_mutex_lock(&enc->mtx);
    /* 清除 RST 位，恢复正常计数 */
    ls_writel(enc->enc_pwm_ctrl, ls_readl(enc->enc_pwm_ctrl) & ~LS_ENC_PWM_CTRL_RST);
    pthread_mutex_unlock(&enc->mtx);
}

/********************************************************************************
 * @brief   获取编码器计数值.
 * @param   enc : 编码器句柄指针.
 * @return  成功返回估算转速，失败返回 0.
 * @example float speed = ls2k0300_pwm_encoder_get_count(&enc);
 ********************************************************************************/
float ls2k0300_pwm_encoder_get_count(ls2k0300_pwm_encoder_t *enc)
{
    uint32_t val;
    gpio_level_t dir;

    if (enc == NULL || enc->initialized == 0) {
        return 0.0f;
    }

    pthread_mutex_lock(&enc->mtx);

    val = ls_readl(enc->enc_pwm_full_buf);
    if (val == 0U) {
        pthread_mutex_unlock(&enc->mtx);
        return 0.0f;
    }

    dir = ls2k0300_gpio_level_get(&enc->dir);
    pthread_mutex_unlock(&enc->mtx);

    /* 频率法换算速度，方向脚电平用于符号判定 */
    return (float)((float)LS_PMON_CLOCK_FREQ / (float)val / (float)LS2K0300_ENCODER_LINE * ((int)dir * 2 - 1));
}

/********************************************************************************
 * @brief   获取脉冲引脚.
 * @param   enc : 编码器句柄指针.
 * @return  成功返回引脚号，失败返回 PIN_INVALID.
 * @example gpio_pin_t pin = ls2k0300_pwm_encoder_get_pulse(&enc);
 ********************************************************************************/
gpio_pin_t ls2k0300_pwm_encoder_get_pulse(ls2k0300_pwm_encoder_t *enc)
{
    return (enc == NULL) ? PIN_INVALID : enc->gpio;
}

/********************************************************************************
 * @brief   获取方向引脚.
 * @param   enc : 编码器句柄指针.
 * @return  成功返回引脚号，失败返回 PIN_INVALID.
 * @example gpio_pin_t dir = ls2k0300_pwm_encoder_get_dir(&enc);
 ********************************************************************************/
gpio_pin_t ls2k0300_pwm_encoder_get_dir(ls2k0300_pwm_encoder_t *enc)
{
    if (enc == NULL) {
        return PIN_INVALID;
    }
    return ls2k0300_gpio_get_pin(&enc->dir);
}

/********************************************************************************
 * @brief   获取复用模式.
 * @param   enc : 编码器句柄指针.
 * @return  成功返回复用模式，失败返回 GPIO_MUX_INVALID.
 * @example gpio_mux_mode_t mux = ls2k0300_pwm_encoder_get_mux(&enc);
 ********************************************************************************/
gpio_mux_mode_t ls2k0300_pwm_encoder_get_mux(ls2k0300_pwm_encoder_t *enc)
{
    return (enc == NULL) ? GPIO_MUX_INVALID : enc->mux;
}

/********************************************************************************
 * @brief   获取编码器 PWM 通道.
 * @param   enc : 编码器句柄指针.
 * @return  成功返回通道号，失败返回 ENC_PWM_CH_INVALID.
 * @example enc_pwm_channel_t ch = ls2k0300_pwm_encoder_get_channel(&enc);
 ********************************************************************************/
enc_pwm_channel_t ls2k0300_pwm_encoder_get_channel(ls2k0300_pwm_encoder_t *enc)
{
    return (enc == NULL) ? ENC_PWM_CH_INVALID : enc->ch;
}
