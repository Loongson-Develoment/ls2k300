#ifndef __LS2K0300_GPIO_H
#define __LS2K0300_GPIO_H

#include <pthread.h>
#include "LS2K0300_MAP.h"

#ifdef __cplusplus
extern "C" {
#endif

/********************************************************************************
 * @brief   GPIO 引脚枚举类型.
 ********************************************************************************/
typedef enum gpio_pin {
    PIN_0  = 0x00, PIN_1,  PIN_2,  PIN_3,  PIN_4,   PIN_5,   PIN_6,   PIN_7,   PIN_8,   PIN_9,   PIN_10,  PIN_11,  PIN_12,  PIN_13,  PIN_14,  PIN_15,
    PIN_16 = 0x10, PIN_17, PIN_18, PIN_19, PIN_20,  PIN_21,  PIN_22,  PIN_23,  PIN_24,  PIN_25,  PIN_26,  PIN_27,  PIN_28,  PIN_29,  PIN_30,  PIN_31,
    PIN_32 = 0x20, PIN_33, PIN_34, PIN_35, PIN_36,  PIN_37,  PIN_38,  PIN_39,  PIN_40,  PIN_41,  PIN_42,  PIN_43,  PIN_44,  PIN_45,  PIN_46,  PIN_47,
    PIN_48 = 0x30, PIN_49, PIN_50, PIN_51, PIN_52,  PIN_53,  PIN_54,  PIN_55,  PIN_56,  PIN_57,  PIN_58,  PIN_59,  PIN_60,  PIN_61,  PIN_62,  PIN_63,
    PIN_64 = 0x40, PIN_65, PIN_66, PIN_67, PIN_68,  PIN_69,  PIN_70,  PIN_71,  PIN_72,  PIN_73,  PIN_74,  PIN_75,  PIN_76,  PIN_77,  PIN_78,  PIN_79,
    PIN_80 = 0x50, PIN_81, PIN_82, PIN_83, PIN_84,  PIN_85,  PIN_86,  PIN_87,  PIN_88,  PIN_89,  PIN_90,  PIN_91,  PIN_92,  PIN_93,  PIN_94,  PIN_95,
    PIN_96 = 0x60, PIN_97, PIN_98, PIN_99, PIN_100, PIN_101, PIN_102, PIN_103, PIN_104, PIN_105,
    PIN_INVALID
} gpio_pin_t;

/********************************************************************************
 * @brief   GPIO 模式枚举类型.
 ********************************************************************************/
typedef enum gpio_mode {
    GPIO_MODE_OUT = 0x00,
    GPIO_MODE_IN  = 0x01,
    GPIO_MODE_INVALID,
} gpio_mode_t;

/********************************************************************************
 * @brief   GPIO 电平枚举类型.
 ********************************************************************************/
typedef enum gpio_level {
    GPIO_LOW  = 0x00,
    GPIO_HIGH = 0x01,
    GPIO_LEVEL_INVALID,
} gpio_level_t;

/********************************************************************************
 * @brief   GPIO 复用模式枚举类型.
 ********************************************************************************/
typedef enum gpio_mux_mode {
    GPIO_MUX_GPIO = 0x00,
    GPIO_MUX_ALT1 = 0x01,
    GPIO_MUX_ALT2 = 0x02,
    GPIO_MUX_MAIN = 0x03,
    GPIO_MUX_INVALID,
} gpio_mux_mode_t;

/********************************************************************************
 * @brief   GPIO 控制句柄.
 ********************************************************************************/
typedef struct {
    gpio_pin_t      pin;
    gpio_mode_t     mode;
    gpio_mux_mode_t mux;

    ls_reg32_addr_t gpio_base;
    ls_reg32_addr_t gpio_one;
    ls_reg32_addr_t gpio_o;
    ls_reg32_addr_t gpio_i;

    pthread_mutex_t mtx;
    int             initialized;
} ls2k0300_gpio_t;

/********************************************************************************
 * @brief   配置指定引脚复用功能.
 * @param   pin : GPIO 引脚号.
 * @param   mux : 复用模式.
 * @return  none.
 * @example ls2k0300_gpio_mux_set(PIN_64, GPIO_MUX_GPIO);
 ********************************************************************************/
void ls2k0300_gpio_mux_set(gpio_pin_t pin, gpio_mux_mode_t mux);

/********************************************************************************
 * @brief   初始化 GPIO 句柄并映射寄存器.
 * @param   gpio : GPIO 句柄指针.
 * @param   pin  : GPIO 引脚号.
 * @param   mode : 输入/输出模式.
 * @param   mux  : 复用模式.
 * @return  成功返回 0，失败返回 -1.
 * @example ls2k0300_gpio_t led;
 *          ls2k0300_gpio_init(&led, PIN_64, GPIO_MODE_OUT, GPIO_MUX_GPIO);
 ********************************************************************************/
int ls2k0300_gpio_init(ls2k0300_gpio_t *gpio, gpio_pin_t pin, gpio_mode_t mode, gpio_mux_mode_t mux);

/********************************************************************************
 * @brief   释放 GPIO 句柄及映射资源.
 * @param   gpio : GPIO 句柄指针.
 * @return  none.
 * @example ls2k0300_gpio_deinit(&led);
 ********************************************************************************/
void ls2k0300_gpio_deinit(ls2k0300_gpio_t *gpio);

/********************************************************************************
 * @brief   设置 GPIO 方向.
 * @param   gpio : GPIO 句柄.
 * @param   mode : GPIO_MODE_IN 或 GPIO_MODE_OUT.
 * @return  none.
 * @example ls2k0300_gpio_direction_set(&led, GPIO_MODE_OUT);
 ********************************************************************************/
void ls2k0300_gpio_direction_set(ls2k0300_gpio_t *gpio, gpio_mode_t mode);

/********************************************************************************
 * @brief   设置 GPIO 输出电平.
 * @param   gpio : GPIO 句柄.
 * @param   val  : GPIO_LOW 或 GPIO_HIGH.
 * @return  none.
 * @example ls2k0300_gpio_level_set(&led, GPIO_HIGH);
 ********************************************************************************/
void ls2k0300_gpio_level_set(ls2k0300_gpio_t *gpio, gpio_level_t val);

/********************************************************************************
 * @brief   读取 GPIO 输入电平.
 * @param   gpio : GPIO 句柄.
 * @return  成功返回 GPIO_LOW 或 GPIO_HIGH，失败返回 GPIO_LEVEL_INVALID.
 * @example gpio_level_t lv = ls2k0300_gpio_level_get(&key);
 ********************************************************************************/
gpio_level_t ls2k0300_gpio_level_get(ls2k0300_gpio_t *gpio);

/********************************************************************************
 * @brief   获取 GPIO 当前引脚号.
 * @param   gpio : GPIO 句柄.
 * @return  成功返回引脚号，失败返回 PIN_INVALID.
 * @example gpio_pin_t pin = ls2k0300_gpio_get_pin(&led);
 ********************************************************************************/
gpio_pin_t ls2k0300_gpio_get_pin(ls2k0300_gpio_t *gpio);

/********************************************************************************
 * @brief   获取 GPIO 当前方向模式.
 * @param   gpio : GPIO 句柄.
 * @return  成功返回 GPIO_MODE_IN/OUT，失败返回 GPIO_MODE_INVALID.
 * @example gpio_mode_t mode = ls2k0300_gpio_get_mode(&led);
 ********************************************************************************/
gpio_mode_t ls2k0300_gpio_get_mode(ls2k0300_gpio_t *gpio);

/********************************************************************************
 * @brief   获取 GPIO 当前复用模式.
 * @param   gpio : GPIO 句柄.
 * @return  成功返回复用值，失败返回 GPIO_MUX_INVALID.
 * @example gpio_mux_mode_t mux = ls2k0300_gpio_get_mux(&led);
 ********************************************************************************/
gpio_mux_mode_t ls2k0300_gpio_get_mux(ls2k0300_gpio_t *gpio);

#ifdef __cplusplus
}
#endif

#endif
