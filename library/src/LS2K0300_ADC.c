#include "LS2K0300_ADC.h"
#include "LS2K0300_MAP.h"

#include <stdbool.h>
#include <unistd.h>
#include <pthread.h>

/********************************************************************************
 * @brief   ADC 寄存器定义.
 ********************************************************************************/
#define LS_ADC_BASE_ADDR     (0x1611c000U)
#define ADC_SR_OFFSET        (0x00U)
#define ADC_CR1_OFFSET       (0x04U)
#define ADC_CR2_OFFSET       (0x08U)
#define ADC_SMPR2_OFFSET     (0x10U)
#define ADC_SQR3_OFFSET      (0x34U)
#define ADC_DR_OFFSET        (0x4CU)

#define ADC_SAMPLE_TIME      (0x06U)
#define ADC_RESOLUTION       (4096.0f)
#define ADC_REF_VOLTAGE_MV   (1800.0f)
#define ADC_CONV_TIMEOUT     (10000)

static bool adc_is_inited = false;
static pthread_mutex_t adc_mtx = PTHREAD_MUTEX_INITIALIZER;

static ls_reg32_addr_t adc_base = NULL;
static ls_reg32_addr_t adc_sr = NULL;
static ls_reg32_addr_t adc_cr1 = NULL;
static ls_reg32_addr_t adc_cr2 = NULL;
static ls_reg32_addr_t adc_smpr2 = NULL;
static ls_reg32_addr_t adc_sqr3 = NULL;
static ls_reg32_addr_t adc_dr = NULL;

/********************************************************************************
 * @brief   ADC 硬件校准.
 * @return  校准成功返回 true，失败返回 false.
 * @note    上电后执行复位与双阶段校准，确保后续转换精度.
 ********************************************************************************/
static bool ls2k0300_adc_hard_calibrate(void)
{
    if (adc_cr2 == NULL) {
        return false;
    }

    /* 先复位 ADC 内部状态机，再开始校准流程 */
    ls_writel(adc_cr2, ls_readl(adc_cr2) | (1U << 0));
    usleep(20);
    ls_writel(adc_cr2, ls_readl(adc_cr2) | (1U << 0));
    usleep(20);

    /* 第一阶段硬件校准 */
    ls_writel(adc_cr2, ls_readl(adc_cr2) | (1U << 3));
    usleep(10);
    while ((ls_readl(adc_cr2) & (1U << 3)) != 0U) {
        usleep(1);
    }

    /* 第二阶段硬件校准 */
    ls_writel(adc_cr2, ls_readl(adc_cr2) | (1U << 2));
    usleep(10);
    while ((ls_readl(adc_cr2) & (1U << 2)) != 0U) {
        usleep(1);
    }

    usleep(20);
    return true;
}

/********************************************************************************
 * @brief   切换 ADC 通道采样配置.
 * @param   ch : 目标 ADC 通道.
 * @return  none.
 ********************************************************************************/
static void ls2k0300_adc_switch_smap_channel(ls_adc_channel_t ch)
{
    /* 配置采样时间寄存器对应的通道字段 */
    ls_writel(adc_smpr2, ls_readl(adc_smpr2) & ~(0x7U << (3U * (uint32_t)ch)));
    ls_writel(adc_smpr2, ls_readl(adc_smpr2) | (ADC_SAMPLE_TIME << (3U * (uint32_t)ch)));

    /* 把规则序列第一位指向目标通道 */
    ls_writel(adc_sqr3, ls_readl(adc_sqr3) & ~(0x1FU << 0));
    ls_writel(adc_sqr3, ls_readl(adc_sqr3) | ((uint32_t)ch << 0));
}

/********************************************************************************
 * @brief   初始化 ADC.
 * @return  成功返回 0，失败返回 -1.
 * @example ls2k0300_adc_init();
 ********************************************************************************/
int ls2k0300_adc_init(void)
{
    int dummy_timeout;

    pthread_mutex_lock(&adc_mtx);

    if (adc_is_inited) {
        pthread_mutex_unlock(&adc_mtx);
        return 0;
    }

    /* 映射 ADC 控制器基地址，并缓存关键寄存器地址 */
    adc_base = (ls_reg32_addr_t)ls2k0300_mmap(LS_ADC_BASE_ADDR, 0x1000);
    if (adc_base == NULL) {
        pthread_mutex_unlock(&adc_mtx);
        return -1;
    }

    adc_sr = ls_reg_addr_calc(adc_base, ADC_SR_OFFSET);
    adc_cr1 = ls_reg_addr_calc(adc_base, ADC_CR1_OFFSET);
    adc_cr2 = ls_reg_addr_calc(adc_base, ADC_CR2_OFFSET);
    adc_smpr2 = ls_reg_addr_calc(adc_base, ADC_SMPR2_OFFSET);
    adc_sqr3 = ls_reg_addr_calc(adc_base, ADC_SQR3_OFFSET);
    adc_dr = ls_reg_addr_calc(adc_base, ADC_DR_OFFSET);

    /* 配置分辨率、触发与工作模式 */
    ls_writel(adc_cr1, 0);
    ls_writel(adc_cr1, ls_readl(adc_cr1) & ~(1U << 8));
    ls_writel(adc_cr1, ls_readl(adc_cr1) & ~(1U << 9));
    ls_writel(adc_cr1, ls_readl(adc_cr1) & ~(1U << 7));

    ls_writel(adc_cr2, 0);
    ls_writel(adc_cr2, ls_readl(adc_cr2) & ~(1U << 11));
    ls_writel(adc_cr2, ls_readl(adc_cr2) & ~(1U << 1));
    ls_writel(adc_cr2, ls_readl(adc_cr2) | (1U << 20));
    ls_writel(adc_cr2, ls_readl(adc_cr2) | (0x0EU << 17));
    ls_writel(adc_cr2, ls_readl(adc_cr2) & ~(1U << 8));

    /* 校准失败时回收映射资源并返回错误 */
    if (!ls2k0300_adc_hard_calibrate()) {
        ls2k0300_munmap((void *)adc_base, 0x1000);
        adc_base = NULL;
        adc_sr = adc_cr1 = adc_cr2 = adc_smpr2 = adc_sqr3 = adc_dr = NULL;
        pthread_mutex_unlock(&adc_mtx);
        return -1;
    }

    ls2k0300_adc_switch_smap_channel(LS_ADC_CH0);
    ls_writel(adc_sr, ls_readl(adc_sr) & ~(1U << 1));
    ls_writel(adc_cr2, ls_readl(adc_cr2) | (1U << 22));

    /* 做一次空转换，清除首次采样不稳定影响 */
    dummy_timeout = 0;
    while (((ls_readl(adc_sr) & (1U << 1)) == 0U) && (dummy_timeout < ADC_CONV_TIMEOUT * 2)) {
        usleep(1);
        dummy_timeout++;
    }
    if (dummy_timeout < ADC_CONV_TIMEOUT * 2) {
        (void)ls_readl(adc_dr);
    }

    adc_is_inited = true;
    pthread_mutex_unlock(&adc_mtx);
    return 0;
}

/********************************************************************************
 * @brief   释放 ADC.
 * @return  none.
 * @example ls2k0300_adc_deinit();
 ********************************************************************************/
void ls2k0300_adc_deinit(void)
{
    pthread_mutex_lock(&adc_mtx);

    if (adc_base != NULL) {
        ls2k0300_munmap((void *)adc_base, 0x1000);
    }

    adc_base = NULL;
    adc_sr = adc_cr1 = adc_cr2 = adc_smpr2 = adc_sqr3 = adc_dr = NULL;
    adc_is_inited = false;

    pthread_mutex_unlock(&adc_mtx);
}

/********************************************************************************
 * @brief   读取 ADC 原始值.
 * @param   ch : ADC 通道.
 * @return  成功返回原始值，失败返回 -1.
 * @example int raw = ls2k0300_adc_read_raw(LS_ADC_CH0);
 ********************************************************************************/
int ls2k0300_adc_read_raw(ls_adc_channel_t ch)
{
    int timeout;
    int value;

    pthread_mutex_lock(&adc_mtx);

    if (!adc_is_inited || ch >= LS_ADC_CH_INVALID) {
        pthread_mutex_unlock(&adc_mtx);
        return -1;
    }

    /* 切换通道并触发一次转换 */
    ls2k0300_adc_switch_smap_channel(ch);
    ls_writel(adc_sr, ls_readl(adc_sr) & ~(1U << 1));
    ls_writel(adc_cr2, ls_readl(adc_cr2) | (1U << 22));

    /* 轮询 EOC 位，超时直接返回错误 */
    timeout = 0;
    while (((ls_readl(adc_sr) & (1U << 1)) == 0U) && (timeout < ADC_CONV_TIMEOUT)) {
        usleep(1);
        timeout++;
    }

    if (timeout >= ADC_CONV_TIMEOUT) {
        pthread_mutex_unlock(&adc_mtx);
        return -1;
    }

    value = (int)(ls_readl(adc_dr) & 0x0FFFU);
    pthread_mutex_unlock(&adc_mtx);

    return value;
}

/********************************************************************************
 * @brief   读取 ADC 电压值.
 * @param   ch : ADC 通道.
 * @return  成功返回电压值(V)，失败返回负数.
 * @example float v = ls2k0300_adc_read_voltage(LS_ADC_CH1);
 ********************************************************************************/
float ls2k0300_adc_read_voltage(ls_adc_channel_t ch)
{
    int raw;

    raw = ls2k0300_adc_read_raw(ch);
    if (raw < 0) {
        return -1.0f;
    }

    /* 原始值转换为伏特: raw/4096*1.8V */
    return ((float)raw * ADC_REF_VOLTAGE_MV / ADC_RESOLUTION) / 1000.0f;
}
