#ifndef __LS2K0300_PWM_H
#define __LS2K0300_PWM_H

#include "LS2K0300_GPIO.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief PWM极性配置枚举
 */
typedef enum {
    PWM_POL_NORMAL = 0x00,  /**< 正常极性 */
    PWM_POL_INV,            /**< 反转极性 */
    PWM_POL_INVALID         /**< 无效极性 */
} pwm_polarity_t;

/**
 * @brief PWM通道枚举
 */
typedef enum {
    PWM_CH0 = 0x00,     /**< PWM 通道 0 */
    PWM_CH1,            /**< PWM 通道 1 */
    PWM_CH2,            /**< PWM 通道 2 */
    PWM_CH3,            /**< PWM 通道 3 */
    PWM_CH_INVALID      /**< 无效通道 */
} pwm_channel_t;

/**
 * @brief PWM引脚及复用配置枚举
 * 包含引脚号、PWM通道以及GPIO复用模式信息
 */
typedef enum {
    PWM0_PIN64 = (GPIO_MUX_ALT1<<10)|(PWM_CH0<<8)|PIN_64,   /**< PWM0 映射到 PIN 64, 复用 1 */
    PWM1_PIN65 = (GPIO_MUX_ALT1<<10)|(PWM_CH1<<8)|PIN_65,   /**< PWM1 映射到 PIN 65, 复用 1 */
    PWM2_PIN66 = (GPIO_MUX_ALT1<<10)|(PWM_CH2<<8)|PIN_66,   /**< PWM2 映射到 PIN 66, 复用 1 */
    PWM3_PIN67 = (GPIO_MUX_ALT1<<10)|(PWM_CH3<<8)|PIN_67,   /**< PWM3 映射到 PIN 67, 复用 1 */

    PWM0_PIN86 = (GPIO_MUX_ALT2<<10)|(PWM_CH0<<8)|PIN_86,   /**< PWM0 映射到 PIN 86, 复用 2 */
    PWM1_PIN87 = (GPIO_MUX_ALT2<<10)|(PWM_CH1<<8)|PIN_87,   /**< PWM1 映射到 PIN 87, 复用 2 */
    PWM2_PIN88 = (GPIO_MUX_ALT2<<10)|(PWM_CH2<<8)|PIN_88,   /**< PWM2 映射到 PIN 88, 复用 2 */
    PWM3_PIN89 = (GPIO_MUX_ALT2<<10)|(PWM_CH3<<8)|PIN_89,   /**< PWM3 映射到 PIN 89, 复用 2 */
} pwm_pin_t;

/**
 * @brief PWM 控制句柄结构体
 */
typedef struct {
    gpio_pin_t      gpio;       /**< 使用的 GPIO 引脚 */
    gpio_mux_mode_t mux;        /**< GPIO 复用模式 */
    pwm_channel_t   ch;         /**< PWM 通道号 */
    pwm_polarity_t  pola;       /**< PWM 极性 */
    uint32_t        period;     /**< PWM 周期 (频率对应) */
    uint32_t        duty;       /**< PWM 占空比 (放大10000倍) */

    ls_reg32_addr_t pwm_base;       /**< PWM 基地址映射指针 */
    ls_reg32_addr_t pwm_low_buf;    /**< 占空比寄存器地址 */
    ls_reg32_addr_t pwm_full_buf;   /**< 周期寄存器地址 */
    ls_reg32_addr_t pwm_ctrl;       /**< 控制寄存器地址 */

    pthread_mutex_t mtx;            /**< 线程同步互斥锁 */
} ls2k0300_pwm_t;

/**
 * @brief 初始化 PWM 设备
 * 
 * @param pwm PWM 设备句柄
 * @param pin_info PWM 引脚及时钟通道信息配置
 * @param period PWM 输出周期/频率，对应频率的倒数
 * @param duty PWM 占空比 (0-10000)
 * @param pola PWM 极性设置
 * @return int 成功返回 0，失败返回 -1
 */
int ls2k0300_pwm_init(ls2k0300_pwm_t* pwm, pwm_pin_t pin_info, uint32_t period, uint32_t duty, pwm_polarity_t pola);

/**
 * @brief 释放 PWM 设备资源
 * 
 * @param pwm PWM 设备句柄
 */
void ls2k0300_pwm_deinit(ls2k0300_pwm_t* pwm);

/**
 * @brief 使能 PWM 输出
 * 
 * @param pwm PWM 设备句柄
 */
void ls2k0300_pwm_enable(ls2k0300_pwm_t* pwm);

/**
 * @brief 禁用 PWM 输出
 * 
 * @param pwm PWM 设备句柄
 */
void ls2k0300_pwm_disable(ls2k0300_pwm_t* pwm);

/**
 * @brief 设置 PWM 周期
 * 
 * @param pwm PWM 设备句柄
 * @param period 新的周期
 */
void ls2k0300_pwm_set_period(ls2k0300_pwm_t* pwm, uint32_t period);

/**
 * @brief 设置 PWM 占空比
 * 
 * @param pwm PWM 设备句柄
 * @param duty 新的占空比 (0-10000)
 */
void ls2k0300_pwm_set_duty(ls2k0300_pwm_t* pwm, uint32_t duty);

/**
 * @brief 设置 PWM 极性
 * 
 * @param pwm PWM 设备句柄
 * @param pola 新的极性
 */
void ls2k0300_pwm_set_polarity(ls2k0300_pwm_t* pwm, pwm_polarity_t pola);

#ifdef __cplusplus
}
#endif

#endif // __LS2K0300_PWM_H
