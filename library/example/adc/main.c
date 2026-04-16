#include <stdio.h>

#include "LS2K0300_DRV_INC.h"

int main(void)
{
    int raw;
    float voltage;

    if (ls2k0300_adc_init() != 0) {
        printf("[FAIL] adc init failed\n");
        return 1;
    }

    raw = ls2k0300_adc_read_raw(LS_ADC_CH0);
    voltage = ls2k0300_adc_read_voltage(LS_ADC_CH0);
    ls2k0300_adc_deinit();

    if (raw < 0 || voltage < 0.0f) {
        printf("[FAIL] adc read failed\n");
        return 1;
    }

    printf("[PASS] adc raw=%d voltage=%.3fV\n", raw, voltage);
    return 0;
}

