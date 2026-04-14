#ifndef __LS2K0300_ADC_H
#define __LS2K0300_ADC_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ADC 通道枚举
typedef enum {
    LS_ADC_CH0 = 0x00,
    LS_ADC_CH1,
    LS_ADC_CH2,
    LS_ADC_CH3,
    LS_ADC_CH4,
    LS_ADC_CH5,
    LS_ADC_CH6,
    LS_ADC_CH7,
    LS_ADC_CH_INVALID,
} ls_adc_channel_t;

int ls2k0300_adc_init(void);
void ls2k0300_adc_deinit(void);

int ls2k0300_adc_read_raw(ls_adc_channel_t ch);
float ls2k0300_adc_read_voltage(ls_adc_channel_t ch);

#ifdef __cplusplus
}
#endif

#endif // __LS2K0300_ADC_H
