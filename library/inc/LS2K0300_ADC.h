#ifndef __LS2K0300_ADC_H
#define __LS2K0300_ADC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/********************************************************************************
 * @brief   ADC 通道枚举.
 ********************************************************************************/
typedef enum ls_adc_channel {
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

/********************************************************************************
 * @brief   初始化 ADC 硬件.
 * @note    使用前必须先调用一次，重复调用会直接返回成功.
 * @return  成功返回 0, 失败返回 -1.
 * @example int ret = ls2k0300_adc_init();
 ********************************************************************************/
int ls2k0300_adc_init(void);

/********************************************************************************
 * @brief   释放 ADC 硬件资源.
 * @return  none.
 * @example ls2k0300_adc_deinit();
 ********************************************************************************/
void ls2k0300_adc_deinit(void);

/********************************************************************************
 * @brief   读取 ADC 原始值.
 * @param   ch : 通道.
 * @return  成功返回原始值, 失败返回负数.
 * @example int raw = ls2k0300_adc_read_raw(LS_ADC_CH0);
 ********************************************************************************/
int ls2k0300_adc_read_raw(ls_adc_channel_t ch);

/********************************************************************************
 * @brief   读取 ADC 电压值 (V).
 * @param   ch : 通道.
 * @return  成功返回电压, 失败返回负数.
 * @example float v = ls2k0300_adc_read_voltage(LS_ADC_CH0);
 ********************************************************************************/
float ls2k0300_adc_read_voltage(ls_adc_channel_t ch);

#ifdef __cplusplus
}
#endif

#endif
