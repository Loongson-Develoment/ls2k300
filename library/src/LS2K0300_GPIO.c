#include "LS2K0300_GPIO.h"

// GPIO 基地址和复用地址配置
#define LS_GPIO_BASE_ADDR               ( 0x16104000 )
#define LS_GPIO_REUSE_ADDR              ( 0x16000490 )
#define LS_GPIO_REUSE_OFS               ( 0x04 )

#define LS_GPIO_OEN_OFF(_pin)           ( 0x00 + (_pin) / 8 * 0x01 )
#define LS_GPIO_O_OFF(_pin)             ( 0x10 + (_pin) / 8 * 0x01 )
#define LS_GPIO_I_OFF(_pin)             ( 0x20 + (_pin) / 8 * 0x01 )

// 全局复用锁
static pthread_mutex_t gpio_mux_mutex = PTHREAD_MUTEX_INITIALIZER;

void ls2k0300_gpio_mux_set(gpio_pin_t pin, gpio_mux_mode_t mux) {
    pthread_mutex_lock(&gpio_mux_mutex);
    if (pin >= PIN_INVALID || mux >= GPIO_MUX_INVALID) {
        printf("gpio_mux_set: gpio or mux is invalid\n");
        pthread_mutex_unlock(&gpio_mux_mutex);
        return;
    }
    
    ls_reg32_addr_t gpio_reuse_reg = (ls_reg32_addr_t)ls2k0300_mmap(LS_GPIO_REUSE_ADDR + (pin / 16) * LS_GPIO_REUSE_OFS, 4);
    if (gpio_reuse_reg != NULL) {
        ls_writel(gpio_reuse_reg, (ls_readl(gpio_reuse_reg) & ~(0b11 << ((pin % 16) * 2))) | (mux << ((pin % 16) * 2)));
        ls2k0300_munmap((void*)gpio_reuse_reg, 4);
    }
    pthread_mutex_unlock(&gpio_mux_mutex);
}

int ls2k0300_gpio_init(ls2k0300_gpio_t* gpio, gpio_pin_t pin, gpio_mode_t mode, gpio_mux_mode_t mux) {
    if (gpio == NULL || pin >= PIN_INVALID || mode >= GPIO_MODE_INVALID || mux >= GPIO_MUX_INVALID) {
        printf("ls2k0300_gpio_init: invalid args\n");
        return -1;
    }
    
    pthread_mutex_init(&gpio->mtx, NULL);
    pthread_mutex_lock(&gpio->mtx);
    
    gpio->pin = pin;
    gpio->mode = mode;
    gpio->mux = mux;

    // 默认映射大小 0x1000 以包含所需寄存器
    gpio->gpio_base = (ls_reg32_addr_t)ls2k0300_mmap(LS_GPIO_BASE_ADDR, 0x1000);
    if (gpio->gpio_base == NULL) {
        printf("ls2k0300_gpio_init: map failed\n");
        pthread_mutex_unlock(&gpio->mtx);
        return -1;
    }

    gpio->gpio_one = ls_reg_addr_calc(gpio->gpio_base, LS_GPIO_OEN_OFF(pin));
    gpio->gpio_o   = ls_reg_addr_calc(gpio->gpio_base, LS_GPIO_O_OFF(pin));
    gpio->gpio_i   = ls_reg_addr_calc(gpio->gpio_base, LS_GPIO_I_OFF(pin));

    pthread_mutex_unlock(&gpio->mtx);

    ls2k0300_gpio_mux_set(pin, mux);
    ls2k0300_gpio_direction_set(gpio, mode);

    return 0;
}

void ls2k0300_gpio_deinit(ls2k0300_gpio_t* gpio) {
    if (gpio == NULL) return;
    pthread_mutex_lock(&gpio->mtx);
    if (gpio->gpio_base) {
        ls2k0300_munmap((void*)gpio->gpio_base, 0x1000);
        gpio->gpio_base = NULL;
    }
    gpio->gpio_one = NULL;
    gpio->gpio_o = NULL;
    gpio->gpio_i = NULL;
    gpio->pin = PIN_INVALID;
    pthread_mutex_unlock(&gpio->mtx);
    pthread_mutex_destroy(&gpio->mtx);
}

void ls2k0300_gpio_direction_set(ls2k0300_gpio_t* gpio, gpio_mode_t mode) {
    if (gpio == NULL || mode >= GPIO_MODE_INVALID) return;
    pthread_mutex_lock(&gpio->mtx);
    
    if (gpio->gpio_one) {
        if (mode == GPIO_MODE_IN) {
            ls_writel(gpio->gpio_one, ls_readl(gpio->gpio_one) | BIT(gpio->pin % 8));
        } else if (mode == GPIO_MODE_OUT) {
            ls_writel(gpio->gpio_one, ls_readl(gpio->gpio_one) & ~BIT(gpio->pin % 8));
        }
        gpio->mode = mode;
    }
    pthread_mutex_unlock(&gpio->mtx);
}

void ls2k0300_gpio_level_set(ls2k0300_gpio_t* gpio, gpio_level_t val) {
    if (gpio == NULL || val >= GPIO_LEVEL_INVALID) return;
    pthread_mutex_lock(&gpio->mtx);
    
    if (gpio->gpio_o) {
        if (val == GPIO_HIGH) {
            ls_writel(gpio->gpio_o, ls_readl(gpio->gpio_o) | BIT(gpio->pin % 8));
        } else if (val == GPIO_LOW) {
            ls_writel(gpio->gpio_o, ls_readl(gpio->gpio_o) & ~BIT(gpio->pin % 8));
        }
    }
    pthread_mutex_unlock(&gpio->mtx);
}

gpio_level_t ls2k0300_gpio_level_get(ls2k0300_gpio_t* gpio) {
    if (gpio == NULL) return GPIO_LEVEL_INVALID;
    gpio_level_t lvl = GPIO_LEVEL_INVALID;
    
    pthread_mutex_lock(&gpio->mtx);
    if (gpio->gpio_i) {
        lvl = (gpio_level_t)((ls_readl(gpio->gpio_i) & BIT(gpio->pin % 8)) != 0);
    }
    pthread_mutex_unlock(&gpio->mtx);
    
    return lvl;
}
