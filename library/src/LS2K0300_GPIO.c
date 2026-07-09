#include "LS2K0300_GPIO.h"

/********************************************************************************
 * @brief   GPIO 基地址与寄存器定义.
 ********************************************************************************/
#define LS_GPIO_BASE_ADDR    (0x16104000U)
#define LS_GPIO_REUSE_ADDR   (0x16000490U)
#define LS_GPIO_REUSE_OFS    (0x04U)

#define LS_GPIO_OEN_OFF(pin) (0x00U + ((pin) / 8U) * 0x01U)
#define LS_GPIO_O_OFF(pin)   (0x10U + ((pin) / 8U) * 0x01U)
#define LS_GPIO_I_OFF(pin)   (0x20U + ((pin) / 8U) * 0x01U)

/* 保护复用寄存器并发写 */
static pthread_mutex_t gpio_mux_mutex = PTHREAD_MUTEX_INITIALIZER;

/********************************************************************************
 * @brief   配置 GPIO 复用.
 * @param   pin : 引脚号.
 * @param   mux : 复用模式.
 * @return  none.
 * @example ls2k0300_gpio_mux_set(PIN_64, GPIO_MUX_GPIO);
 ********************************************************************************/
void ls2k0300_gpio_mux_set(gpio_pin_t pin, gpio_mux_mode_t mux)
{
    ls_reg32_addr_t reuse_reg;

    pthread_mutex_lock(&gpio_mux_mutex);

    if (pin >= PIN_INVALID || mux >= GPIO_MUX_INVALID) {
        printf("gpio_mux_set: gpio or mux is invalid\n");
        pthread_mutex_unlock(&gpio_mux_mutex);
        return;
    }

    /* 每 16 个引脚对应一个复用配置寄存器 */
    reuse_reg = (ls_reg32_addr_t)ls2k0300_mmap(LS_GPIO_REUSE_ADDR + (pin / 16U) * LS_GPIO_REUSE_OFS, 4);
    if (reuse_reg != NULL) {
        /* 先清除目标 pin 的 2bit 复用位，再写入新复用值 */
        ls_writel(reuse_reg,
                  (ls_readl(reuse_reg) & ~(0x3U << ((pin % 16U) * 2U))) |
                  ((uint32_t)mux << ((pin % 16U) * 2U)));
        ls2k0300_munmap((void *)reuse_reg, 4);
    }

    pthread_mutex_unlock(&gpio_mux_mutex);
}

/********************************************************************************
 * @brief   初始化 GPIO 句柄.
 * @param   gpio : GPIO 句柄.
 * @param   pin  : 引脚号.
 * @param   mode : 输入输出模式.
 * @param   mux  : 复用模式.
 * @return  成功返回 0，失败返回 -1.
 * @example ls2k0300_gpio_t led;
 *          ls2k0300_gpio_init(&led, PIN_64, GPIO_MODE_OUT, GPIO_MUX_GPIO);
 ********************************************************************************/
int ls2k0300_gpio_init(ls2k0300_gpio_t *gpio, gpio_pin_t pin, gpio_mode_t mode, gpio_mux_mode_t mux)
{
    if (gpio == NULL || pin >= PIN_INVALID || mode >= GPIO_MODE_INVALID || mux >= GPIO_MUX_INVALID) {
        printf("ls2k0300_gpio_init: invalid args\n");
        return -1;
    }

    pthread_mutex_init(&gpio->mtx, NULL);
    pthread_mutex_lock(&gpio->mtx);

    gpio->pin = pin;
    gpio->mode = mode;
    gpio->mux = mux;
    gpio->initialized = 0;

    /* 映射 GPIO 控制器地址空间 */
    gpio->gpio_base = (ls_reg32_addr_t)ls2k0300_mmap(LS_GPIO_BASE_ADDR, 0x1000);
    if (gpio->gpio_base == NULL) {
        pthread_mutex_unlock(&gpio->mtx);
        pthread_mutex_destroy(&gpio->mtx);
        printf("ls2k0300_gpio_init: map failed\n");
        return -1;
    }

    /* 根据引脚号计算方向/输出/输入寄存器地址 */
    gpio->gpio_one = ls_reg_addr_calc(gpio->gpio_base, LS_GPIO_OEN_OFF(pin));
    gpio->gpio_o = ls_reg_addr_calc(gpio->gpio_base, LS_GPIO_O_OFF(pin));
    gpio->gpio_i = ls_reg_addr_calc(gpio->gpio_base, LS_GPIO_I_OFF(pin));
    gpio->initialized = 1;

    pthread_mutex_unlock(&gpio->mtx);

    /* 先配复用，再设方向，确保寄存器配置顺序一致 */
    ls2k0300_gpio_mux_set(pin, mux);
    ls2k0300_gpio_direction_set(gpio, mode);
    return 0;
}

/********************************************************************************
 * @brief   释放 GPIO 句柄资源.
 * @param   gpio : GPIO 句柄.
 * @return  none.
 * @example ls2k0300_gpio_deinit(&led);
 ********************************************************************************/
void ls2k0300_gpio_deinit(ls2k0300_gpio_t *gpio)
{
    if (gpio == NULL) {
        return;
    }

    pthread_mutex_lock(&gpio->mtx);
    if (gpio->gpio_base != NULL) {
        ls2k0300_munmap((void *)gpio->gpio_base, 0x1000);
    }
    gpio_pin_t old_pin = gpio->pin;

    gpio->gpio_base = NULL;
    gpio->gpio_one = NULL;
    gpio->gpio_o = NULL;
    gpio->gpio_i = NULL;
    gpio->pin = PIN_INVALID;
    gpio->mode = GPIO_MODE_INVALID;
    gpio->mux = GPIO_MUX_INVALID;
    gpio->initialized = 0;

    pthread_mutex_unlock(&gpio->mtx);
    pthread_mutex_destroy(&gpio->mtx);
    printf("[INFO] deinit gpio %d\n", old_pin);
}

/********************************************************************************
 * @brief   设置 GPIO 方向.
 * @param   gpio : GPIO 句柄.
 * @param   mode : GPIO_MODE_IN 或 GPIO_MODE_OUT.
 * @return  none.
 * @example ls2k0300_gpio_direction_set(&led, GPIO_MODE_OUT);
 ********************************************************************************/
void ls2k0300_gpio_direction_set(ls2k0300_gpio_t *gpio, gpio_mode_t mode)
{
    if (gpio == NULL || mode >= GPIO_MODE_INVALID || gpio->initialized == 0) {
        return;
    }

    pthread_mutex_lock(&gpio->mtx);

    /* OEN 位为 1 表示输入，为 0 表示输出 */
    if (mode == GPIO_MODE_IN) {
        ls_writel(gpio->gpio_one, ls_readl(gpio->gpio_one) | BIT(gpio->pin % 8U));
    } else {
        ls_writel(gpio->gpio_one, ls_readl(gpio->gpio_one) & ~BIT(gpio->pin % 8U));
    }

    gpio->mode = mode;

    pthread_mutex_unlock(&gpio->mtx);
}

/********************************************************************************
 * @brief   设置 GPIO 输出电平.
 * @param   gpio : GPIO 句柄.
 * @param   val  : GPIO_LOW 或 GPIO_HIGH.
 * @return  none.
 * @example ls2k0300_gpio_level_set(&led, GPIO_HIGH);
 ********************************************************************************/
void ls2k0300_gpio_level_set(ls2k0300_gpio_t *gpio, gpio_level_t val)
{
    if (gpio == NULL || val >= GPIO_LEVEL_INVALID || gpio->initialized == 0) {
        return;
    }

    pthread_mutex_lock(&gpio->mtx);

    /* 写输出寄存器对应 bit 控制实际电平 */
    if (val == GPIO_HIGH) {
        ls_writel(gpio->gpio_o, ls_readl(gpio->gpio_o) | BIT(gpio->pin % 8U));
    } else {
        ls_writel(gpio->gpio_o, ls_readl(gpio->gpio_o) & ~BIT(gpio->pin % 8U));
    }

    pthread_mutex_unlock(&gpio->mtx);
}

/********************************************************************************
 * @brief   获取 GPIO 输入电平.
 * @param   gpio : GPIO 句柄.
 * @return  GPIO_LOW/GPIO_HIGH，失败返回 GPIO_LEVEL_INVALID.
 * @example gpio_level_t lv = ls2k0300_gpio_level_get(&key);
 ********************************************************************************/
gpio_level_t ls2k0300_gpio_level_get(ls2k0300_gpio_t *gpio)
{
    gpio_level_t level;

    if (gpio == NULL || gpio->initialized == 0) {
        return GPIO_LEVEL_INVALID;
    }

    pthread_mutex_lock(&gpio->mtx);
    level = (gpio_level_t)((ls_readl(gpio->gpio_i) & BIT(gpio->pin % 8U)) == BIT(gpio->pin % 8U));
    pthread_mutex_unlock(&gpio->mtx);

    return level;
}

/********************************************************************************
 * @brief   获取 GPIO 当前引脚.
 * @param   gpio : GPIO 句柄.
 * @return  引脚号，失败返回 PIN_INVALID.
 ********************************************************************************/
gpio_pin_t ls2k0300_gpio_get_pin(ls2k0300_gpio_t *gpio)
{
    if (gpio == NULL) {
        return PIN_INVALID;
    }
    return gpio->pin;
}

/********************************************************************************
 * @brief   获取 GPIO 当前模式.
 * @param   gpio : GPIO 句柄.
 * @return  GPIO_MODE_IN/OUT，失败返回 GPIO_MODE_INVALID.
 ********************************************************************************/
gpio_mode_t ls2k0300_gpio_get_mode(ls2k0300_gpio_t *gpio)
{
    if (gpio == NULL) {
        return GPIO_MODE_INVALID;
    }
    return gpio->mode;
}

/********************************************************************************
 * @brief   获取 GPIO 当前复用模式.
 * @param   gpio : GPIO 句柄.
 * @return  复用模式，失败返回 GPIO_MUX_INVALID.
 ********************************************************************************/
gpio_mux_mode_t ls2k0300_gpio_get_mux(ls2k0300_gpio_t *gpio)
{
    if (gpio == NULL) {
        return GPIO_MUX_INVALID;
    }
    return gpio->mux;
}
