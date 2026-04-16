#include <stdio.h>
#include <time.h>

#include "LS2K0300_DRV_INC.h"

int main(void)
{
    ls2k0300_gpio_t gpio;
    struct timespec ts = {0, 500 * 1000 * 1000}; /* 500ms */
    gpio_level_t level = GPIO_LOW;
    int i;

    if (ls2k0300_gpio_init(&gpio, PIN_83, GPIO_MODE_OUT, GPIO_MUX_GPIO) != 0) {
        printf("[FAIL] gpio init failed\n");
        return 1;
    }

    printf("[INFO] toggling PIN_88 for 5 seconds\n");

    for (i = 0; i < 10; i++) {
        level = (level == GPIO_LOW) ? GPIO_HIGH : GPIO_LOW;
        ls2k0300_gpio_level_set(&gpio, level);
        nanosleep(&ts, NULL);
    }

    ls2k0300_gpio_deinit(&gpio);

    printf("[PASS] 5s done, gpio deinit ok\n");
    return 0;
}
