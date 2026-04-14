#include "LS2K0300_ADC.h"
#include "LS2K0300_MAP.h"
#include <unistd.h>
#include <pthread.h>

#define LS_ADC_BASE_ADDR            ( 0x1611c000 )
#define ADC_SR_OFFSET               ( 0x00 )
#define ADC_CR1_OFFSET              ( 0x04 )
#define ADC_CR2_OFFSET              ( 0x08 )
#define ADC_SMPR2_OFFSET            ( 0x10 )
#define ADC_SQR3_OFFSET             ( 0x34 )
#define ADC_DR_OFFSET               ( 0x4C )

#define ADC_SAMPLE_TIME             ( 0x06 )
#define ADC_RESOLUTION              ( 4096 )
#define ADC_REF_VOLTAGE             ( 1800 )
#define ADC_CONV_TIMEOUT            ( 10000 )

static bool adc_is_inited = false;
static pthread_mutex_t adc_mtx = PTHREAD_MUTEX_INITIALIZER;
static ls_reg32_addr_t adc_base = NULL;
static ls_reg32_addr_t adc_sr, adc_cr1, adc_cr2, adc_smpr2, adc_sqr3, adc_dr;

static bool ls2k0300_adc_hard_calibrate(void) {
    if (!adc_cr2) return false;
    ls_writel(adc_cr2, ls_readl(adc_cr2) | (1 << 0));
    usleep(20);
    ls_writel(adc_cr2, ls_readl(adc_cr2) | (1 << 0));
    usleep(20);
    ls_writel(adc_cr2, ls_readl(adc_cr2) | (1 << 3));
    usleep(10);
    while ((ls_readl(adc_cr2) & (1 << 3)) != 0) { usleep(1); }
    ls_writel(adc_cr2, ls_readl(adc_cr2) | (1 << 2));
    usleep(10);
    while ((ls_readl(adc_cr2) & (1 << 2)) != 0) { usleep(1); }
    usleep(20);
    return true;
}

static void ls2k0300_adc_switch_smap_channel(ls_adc_channel_t ch) {
    ls_writel(adc_smpr2, ls_readl(adc_smpr2) & ~(0x07 << (3 * ch)));
    ls_writel(adc_smpr2, ls_readl(adc_smpr2) | (ADC_SAMPLE_TIME << (3 * ch)));
    ls_writel(adc_sqr3, ls_readl(adc_sqr3) & ~(0x1F << 0));
    ls_writel(adc_sqr3, ls_readl(adc_sqr3) | (ch << 0));
}

int ls2k0300_adc_init(void) {
    pthread_mutex_lock(&adc_mtx);
    if (adc_is_inited) {
        pthread_mutex_unlock(&adc_mtx);
        return 0;
    }
    
    adc_base = (ls_reg32_addr_t)ls2k0300_mmap(LS_ADC_BASE_ADDR, 0x1000);
    if (!adc_base) {
        pthread_mutex_unlock(&adc_mtx);
        return -1;
    }

    adc_sr    = ls_reg_addr_calc(adc_base, ADC_SR_OFFSET);
    adc_cr1   = ls_reg_addr_calc(adc_base, ADC_CR1_OFFSET);
    adc_cr2   = ls_reg_addr_calc(adc_base, ADC_CR2_OFFSET);
    adc_smpr2 = ls_reg_addr_calc(adc_base, ADC_SMPR2_OFFSET);
    adc_sqr3  = ls_reg_addr_calc(adc_base, ADC_SQR3_OFFSET);
    adc_dr    = ls_reg_addr_calc(adc_base, ADC_DR_OFFSET);

    ls_writel(adc_cr1, 0);
    ls_writel(adc_cr1, ls_readl(adc_cr1) & ~(1 << 8));
    ls_writel(adc_cr1, ls_readl(adc_cr1) & ~(1 << 9));
    ls_writel(adc_cr1, ls_readl(adc_cr1) & ~(1 << 7));

    ls_writel(adc_cr2, 0);
    ls_writel(adc_cr2, ls_readl(adc_cr2) & ~(1 << 11));
    ls_writel(adc_cr2, ls_readl(adc_cr2) & ~(1 << 1));
    ls_writel(adc_cr2, ls_readl(adc_cr2) | (1 << 20));
    ls_writel(adc_cr2, ls_readl(adc_cr2) | (0x0E << 17));
    ls_writel(adc_cr2, ls_readl(adc_cr2) & ~(1 << 8));

    if (!ls2k0300_adc_hard_calibrate()) {
        ls2k0300_munmap((void*)adc_base, 0x1000);
        adc_base = NULL;
        pthread_mutex_unlock(&adc_mtx);
        return -1;
    }

    // Dummy config
    ls2k0300_adc_switch_smap_channel(LS_ADC_CH0);
    ls_writel(adc_sr, ls_readl(adc_sr) & ~(1 << 1));
    ls_writel(adc_cr2, ls_readl(adc_cr2) | (1 << 22));
    
    int dummy_timeout = 0;
    while (((ls_readl(adc_sr) & (1 << 1)) == 0) && (dummy_timeout < ADC_CONV_TIMEOUT * 2)) {
        usleep(1);
        dummy_timeout++;
    }
    if (dummy_timeout < ADC_CONV_TIMEOUT * 2) {
        ls_readl(adc_dr);
    }

    adc_is_inited = true;
    pthread_mutex_unlock(&adc_mtx);
    return 0;
}

void ls2k0300_adc_deinit(void) {
    pthread_mutex_lock(&adc_mtx);
    if (adc_base) {
        ls2k0300_munmap((void*)adc_base, 0x1000);
        adc_base = NULL;
    }
    adc_is_inited = false;
    pthread_mutex_unlock(&adc_mtx);
}

int ls2k0300_adc_read_raw(ls_adc_channel_t ch) {
    pthread_mutex_lock(&adc_mtx);
    if (!adc_is_inited || ch >= LS_ADC_CH_INVALID) {
        pthread_mutex_unlock(&adc_mtx);
        return -1;
    }

    ls2k0300_adc_switch_smap_channel(ch);
    ls_writel(adc_sr, ls_readl(adc_sr) & ~(1 << 1));
    ls_writel(adc_cr2, ls_readl(adc_cr2) | (1 << 22));

    int timeout = 0;
    while (((ls_readl(adc_sr) & (1 << 1)) == 0) && (timeout < ADC_CONV_TIMEOUT)) {
        usleep(1);
        timeout++;
    }

    if (timeout >= ADC_CONV_TIMEOUT) {
        pthread_mutex_unlock(&adc_mtx);
        return -1;
    }

    int val = ls_readl(adc_dr) & 0x0FFF;
    pthread_mutex_unlock(&adc_mtx);
    return val;
}

float ls2k0300_adc_read_voltage(ls_adc_channel_t ch) {
    int raw = ls2k0300_adc_read_raw(ch);
    if (raw < 0) return -1.0f;
    return (float)raw * ADC_REF_VOLTAGE / ADC_RESOLUTION / 1000.0f;
}
