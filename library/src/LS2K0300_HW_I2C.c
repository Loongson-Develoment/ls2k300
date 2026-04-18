#include "LS2K0300_HW_I2C.h"
#include "LS2K0300_GPIO.h"

#include <errno.h>
#include <string.h>
#include <time.h>

/********************************************************************************
 * @brief   硬件 I2C 基地址与寄存器定义.
 ********************************************************************************/
#define LS_HW_I2C0_BASE_ADDR        (0x16108000U)
#define LS_HW_I2C1_BASE_ADDR        (0x16109000U)
#define LS_HW_I2C2_BASE_ADDR        (0x1610A000U)
#define LS_HW_I2C3_BASE_ADDR        (0x1610B000U)
#define LS_HW_I2C_MMAP_SIZE         (0x400U)

#define I2C_CR1_OFS                 (0x00U)
#define I2C_CR2_OFS                 (0x04U)
#define I2C_OAR_OFS                 (0x08U)
#define I2C_DR_OFS                  (0x10U)
#define I2C_SR1_OFS                 (0x14U)
#define I2C_SR2_OFS                 (0x18U)
#define I2C_CCR_OFS                 (0x1CU)
#define I2C_TRISE_OFS               (0x20U)

/********************************************************************************
 * @brief   I2C CR1 位定义.
 ********************************************************************************/
#define I2C_CR1_SWRST               BIT(15)
#define I2C_CR1_RECOVER             BIT(14)
#define I2C_CR1_POS                 BIT(11)
#define I2C_CR1_ACK                 BIT(10)
#define I2C_CR1_STOP                BIT(9)
#define I2C_CR1_START               BIT(8)
#define I2C_CR1_PE                  BIT(0)

/********************************************************************************
 * @brief   I2C CR2 位定义.
 ********************************************************************************/
#define I2C_CR2_DMAEN               BIT(11)
#define I2C_CR2_ITBUFEN             BIT(10)
#define I2C_CR2_ITEVTEN             BIT(9)
#define I2C_CR2_ITERREN             BIT(8)
#define I2C_CR2_FREQ_MASK           (0x3FU)

/********************************************************************************
 * @brief   I2C SR1/SR2 位定义.
 ********************************************************************************/
#define I2C_SR1_OVR                 BIT(11)
#define I2C_SR1_AF                  BIT(10)
#define I2C_SR1_ARLO                BIT(9)
#define I2C_SR1_BERR                BIT(8)
#define I2C_SR1_TXE                 BIT(7)
#define I2C_SR1_RXNE                BIT(6)
#define I2C_SR1_BTF                 BIT(2)
#define I2C_SR1_ADDR                BIT(1)
#define I2C_SR1_SB                  BIT(0)

#define I2C_SR2_BUSY                BIT(1)

#define I2C_SR1_ERROR_MASK          (I2C_SR1_OVR | I2C_SR1_AF | I2C_SR1_ARLO | I2C_SR1_BERR)

/********************************************************************************
 * @brief   I2C CCR/TRISE 位定义.
 ********************************************************************************/
#define I2C_CCR_FS                  BIT(15)
#define I2C_CCR_DUTY                BIT(14)
#define I2C_CCR_CCR_MASK            (0x0FFFU)

#define I2C_TRISE_MASK              (0x3FU)

/********************************************************************************
 * @brief   寄存器位操作辅助函数.
 ********************************************************************************/
static inline void hw_i2c_set_bits(ls_reg32_addr_t reg, uint32_t mask)
{
    ls_writel(reg, ls_readl(reg) | mask);
}

static inline void hw_i2c_clr_bits(ls_reg32_addr_t reg, uint32_t mask)
{
    ls_writel(reg, ls_readl(reg) & ~mask);
}

/********************************************************************************
 * @brief   获取微秒时间戳（单调时钟）.
 ********************************************************************************/
static uint64_t hw_i2c_now_us(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ((uint64_t)ts.tv_sec * 1000000ULL) + ((uint64_t)ts.tv_nsec / 1000ULL);
}

/********************************************************************************
 * @brief   微秒级延时（基于 nanosleep）.
 ********************************************************************************/
static void hw_i2c_delay_us(uint32_t us)
{
    struct timespec req;
    struct timespec rem;

    req.tv_sec = (time_t)(us / 1000000U);
    req.tv_nsec = (long)((us % 1000000U) * 1000U);

    while (nanosleep(&req, &rem) != 0) {
        if (errno != EINTR) {
            break;
        }
        req = rem;
    }
}

/********************************************************************************
 * @brief   根据端口获取 I2C 控制器基地址.
 ********************************************************************************/
static uint32_t hw_i2c_get_base_addr(ls_hw_i2c_port_t port)
{
    switch (port) {
        case LS_HW_I2C0: return LS_HW_I2C0_BASE_ADDR;
        case LS_HW_I2C1: return LS_HW_I2C1_BASE_ADDR;
        case LS_HW_I2C2: return LS_HW_I2C2_BASE_ADDR;
        case LS_HW_I2C3: return LS_HW_I2C3_BASE_ADDR;
        default:         return 0U;
    }
}

/********************************************************************************
 * @brief   I2C 端口复用映射项.
 ********************************************************************************/
typedef struct {
    ls_hw_i2c_port_t port;
    ls_hw_i2c_mux_t  mux_sel;
    gpio_pin_t       scl_pin;
    gpio_pin_t       sda_pin;
    gpio_mux_mode_t  mux_mode;
} hw_i2c_pinmux_item_t;

static const hw_i2c_pinmux_item_t g_hw_i2c_pinmux_table[] = {
    /* I2C0: m0=gpa3[0:1] MAIN, m1=gpa3[12:13] ALT1 */
    {LS_HW_I2C0, LS_HW_I2C_MUX_M0, PIN_48, PIN_49, GPIO_MUX_MAIN},
    {LS_HW_I2C0, LS_HW_I2C_MUX_M1, PIN_60, PIN_61, GPIO_MUX_ALT1},
    /* I2C1: m0=gpa3[2:3] MAIN, m1=gpa3[14:15] ALT1 */
    {LS_HW_I2C1, LS_HW_I2C_MUX_M0, PIN_50, PIN_51, GPIO_MUX_MAIN},
    {LS_HW_I2C1, LS_HW_I2C_MUX_M1, PIN_62, PIN_63, GPIO_MUX_ALT1},
    /* I2C2: m0=gpa3[4:5] MAIN, m1=gpa5[2:3] ALT2 */
    {LS_HW_I2C2, LS_HW_I2C_MUX_M0, PIN_52, PIN_53, GPIO_MUX_MAIN},
    {LS_HW_I2C2, LS_HW_I2C_MUX_M1, PIN_82, PIN_83, GPIO_MUX_ALT2},
    /* I2C3: m0=gpa3[6:7] MAIN, m1=gpa5[4:5] ALT2 */
    {LS_HW_I2C3, LS_HW_I2C_MUX_M0, PIN_54, PIN_55, GPIO_MUX_MAIN},
    {LS_HW_I2C3, LS_HW_I2C_MUX_M1, PIN_84, PIN_85, GPIO_MUX_ALT2},
};

/********************************************************************************
 * @brief   根据端口+复用组设置 I2C 引脚复用.
 * @return  成功返回 0，失败返回 -1.
 ********************************************************************************/
static int hw_i2c_apply_pinmux(ls_hw_i2c_port_t port, ls_hw_i2c_mux_t mux)
{
    size_t i;

    for (i = 0U; i < (sizeof(g_hw_i2c_pinmux_table) / sizeof(g_hw_i2c_pinmux_table[0])); ++i) {
        if (g_hw_i2c_pinmux_table[i].port == port && g_hw_i2c_pinmux_table[i].mux_sel == mux) {
            ls2k0300_gpio_mux_set(g_hw_i2c_pinmux_table[i].scl_pin, g_hw_i2c_pinmux_table[i].mux_mode);
            ls2k0300_gpio_mux_set(g_hw_i2c_pinmux_table[i].sda_pin, g_hw_i2c_pinmux_table[i].mux_mode);
            return 0;
        }
    }

    return -1;
}

/********************************************************************************
 * @brief   清理 SR1 错误状态位.
 * @param   i2c : I2C 句柄.
 * @return  none.
 ********************************************************************************/
static void hw_i2c_clear_error_flags(ls2k0300_hw_i2c_t *i2c)
{
    uint32_t sr1;
    uint32_t err;

    sr1 = ls_readl(i2c->i2c_sr1);
    err = sr1 & I2C_SR1_ERROR_MASK;
    if (err != 0U) {
        ls_writel(i2c->i2c_sr1, sr1 & ~err);
    }
}

/********************************************************************************
 * @brief   等待 SR1 指定位被置位.
 * @param   i2c        : I2C 句柄.
 * @param   flag       : 目标状态位.
 * @param   timeout_us : 超时时间.
 * @return  成功返回 0，失败返回 -1.
 ********************************************************************************/
static int hw_i2c_wait_sr1_flag(ls2k0300_hw_i2c_t *i2c, uint32_t flag, uint32_t timeout_us)
{
    uint64_t start;
    uint32_t sr1;

    start = hw_i2c_now_us();
    do {
        sr1 = ls_readl(i2c->i2c_sr1);
        if ((sr1 & I2C_SR1_ERROR_MASK) != 0U) {
            hw_i2c_clear_error_flags(i2c);
            return -1;
        }
        if ((sr1 & flag) != 0U) {
            return 0;
        }
        hw_i2c_delay_us(10U);
    } while ((hw_i2c_now_us() - start) < (uint64_t)timeout_us);

    return -1;
}

/********************************************************************************
 * @brief   等待总线空闲.
 * @param   i2c        : I2C 句柄.
 * @param   timeout_us : 超时.
 * @return  成功返回 0，失败返回 -1.
 ********************************************************************************/
static int hw_i2c_wait_bus_free(ls2k0300_hw_i2c_t *i2c, uint32_t timeout_us)
{
    uint64_t start;

    start = hw_i2c_now_us();
    while ((ls_readl(i2c->i2c_sr2) & I2C_SR2_BUSY) != 0U) {
        if ((hw_i2c_now_us() - start) >= (uint64_t)timeout_us) {
            return -1;
        }
        hw_i2c_delay_us(10U);
    }

    return 0;
}

/********************************************************************************
 * @brief   产生 START 条件并等待 SB.
 * @param   i2c : I2C 句柄.
 * @return  成功返回 0，失败返回 -1.
 ********************************************************************************/
static int hw_i2c_generate_start(ls2k0300_hw_i2c_t *i2c)
{
    hw_i2c_set_bits(i2c->i2c_cr1, I2C_CR1_START);
    return hw_i2c_wait_sr1_flag(i2c, I2C_SR1_SB, i2c->timeout_us);
}

/********************************************************************************
 * @brief   发送 8bit 地址并等待 ADDR.
 * @param   i2c      : I2C 句柄.
 * @param   addr_8b  : 8bit 地址（最低位为 R/W）.
 * @return  成功返回 0，失败返回 -1.
 ********************************************************************************/
static int hw_i2c_send_addr(ls2k0300_hw_i2c_t *i2c, uint8_t addr_8b)
{
    ls_writel(i2c->i2c_dr, (uint32_t)addr_8b);
    return hw_i2c_wait_sr1_flag(i2c, I2C_SR1_ADDR, i2c->timeout_us);
}

/********************************************************************************
 * @brief   清除 ADDR 状态位（读 SR1 后读 SR2）.
 * @param   i2c : I2C 句柄.
 * @return  none.
 ********************************************************************************/
static void hw_i2c_clear_addr_flag(ls2k0300_hw_i2c_t *i2c)
{
    (void)ls_readl(i2c->i2c_sr1);
    (void)ls_readl(i2c->i2c_sr2);
}

/********************************************************************************
 * @brief   产生 STOP 条件.
 * @param   i2c : I2C 句柄.
 * @return  none.
 ********************************************************************************/
static void hw_i2c_generate_stop(ls2k0300_hw_i2c_t *i2c)
{
    hw_i2c_set_bits(i2c->i2c_cr1, I2C_CR1_STOP);
}

/********************************************************************************
 * @brief   配置时序寄存器（CR2/CCR/TRISE）.
 * @param   i2c      : I2C 句柄.
 * @param   speed_hz : 目标速率.
 * @return  成功返回 0，失败返回 -1.
 ********************************************************************************/
static int hw_i2c_apply_timing(ls2k0300_hw_i2c_t *i2c, uint32_t speed_hz)
{
    uint32_t clk_hz;
    uint32_t clk_mhz;
    uint32_t cr2;
    uint32_t ccr;
    uint32_t trise;

    if (speed_hz == 0U) {
        return -1;
    }

    if (speed_hz > 400000U) {
        speed_hz = 400000U;
    }

    clk_hz = i2c->input_clk_hz;
    if (clk_hz == 0U) {
        return -1;
    }

    clk_mhz = clk_hz / 1000000U;
    if (clk_mhz == 0U) {
        clk_mhz = 1U;
    }
    if (clk_mhz > I2C_CR2_FREQ_MASK) {
        clk_mhz = I2C_CR2_FREQ_MASK;
    }

    cr2 = ls_readl(i2c->i2c_cr2);
    cr2 &= ~(I2C_CR2_DMAEN | I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN | I2C_CR2_ITERREN | I2C_CR2_FREQ_MASK);
    cr2 |= (clk_mhz & I2C_CR2_FREQ_MASK);
    ls_writel(i2c->i2c_cr2, cr2);

    if (speed_hz <= 100000U) {
        /* 标准模式：T_high = T_low = CCR * T_pclk */
        ccr = clk_hz / (2U * speed_hz);
        if (ccr == 0U) {
            ccr = 1U;
        }
        if (ccr > I2C_CCR_CCR_MASK) {
            ccr = I2C_CCR_CCR_MASK;
        }

        trise = clk_mhz + 1U; /* 约等于 1000ns / T_pclk + 1 */
        if (trise > I2C_TRISE_MASK) {
            trise = I2C_TRISE_MASK;
        }

        ls_writel(i2c->i2c_ccr, ccr & I2C_CCR_CCR_MASK);
        ls_writel(i2c->i2c_trise, trise & I2C_TRISE_MASK);
    } else {
        /* 快速模式（duty=0）：T_high = CCR*T_pclk, T_low = 2*CCR*T_pclk */
        ccr = clk_hz / (3U * speed_hz);
        if (ccr == 0U) {
            ccr = 1U;
        }
        if (ccr > I2C_CCR_CCR_MASK) {
            ccr = I2C_CCR_CCR_MASK;
        }

        trise = ((clk_mhz * 300U) / 1000U) + 1U; /* 约等于 300ns / T_pclk + 1 */
        if (trise == 0U) {
            trise = 1U;
        }
        if (trise > I2C_TRISE_MASK) {
            trise = I2C_TRISE_MASK;
        }

        ls_writel(i2c->i2c_ccr, I2C_CCR_FS | (ccr & I2C_CCR_CCR_MASK));
        ls_writel(i2c->i2c_trise, trise & I2C_TRISE_MASK);
    }

    i2c->speed_hz = speed_hz;
    return 0;
}

/********************************************************************************
 * @brief   内部：寄存器写事务.
 * @param   i2c : I2C 句柄（已加锁）.
 * @param   reg : 起始寄存器.
 * @param   buf : 写入数据.
 * @param   len : 写入长度.
 * @return  成功返回 0，失败返回 1.
 ********************************************************************************/
static uint8_t hw_i2c_write_reg_locked(ls2k0300_hw_i2c_t *i2c, uint8_t reg, const uint8_t *buf, uint16_t len)
{
    uint16_t i;

    if (hw_i2c_wait_bus_free(i2c, i2c->timeout_us) != 0) {
        return 1U;
    }

    hw_i2c_set_bits(i2c->i2c_cr1, I2C_CR1_ACK);
    hw_i2c_clr_bits(i2c->i2c_cr1, I2C_CR1_POS);

    if (hw_i2c_generate_start(i2c) != 0) {
        return 1U;
    }

    if (hw_i2c_send_addr(i2c, (uint8_t)(i2c->addr << 1)) != 0) {
        hw_i2c_generate_stop(i2c);
        return 1U;
    }
    hw_i2c_clear_addr_flag(i2c);

    if (hw_i2c_wait_sr1_flag(i2c, I2C_SR1_TXE, i2c->timeout_us) != 0) {
        hw_i2c_generate_stop(i2c);
        return 1U;
    }
    ls_writel(i2c->i2c_dr, (uint32_t)reg);

    for (i = 0U; i < len; ++i) {
        if (hw_i2c_wait_sr1_flag(i2c, I2C_SR1_TXE, i2c->timeout_us) != 0) {
            hw_i2c_generate_stop(i2c);
            return 1U;
        }
        ls_writel(i2c->i2c_dr, (uint32_t)buf[i]);
    }

    if (hw_i2c_wait_sr1_flag(i2c, I2C_SR1_BTF, i2c->timeout_us) != 0) {
        hw_i2c_generate_stop(i2c);
        return 1U;
    }

    hw_i2c_generate_stop(i2c);
    return 0U;
}

/********************************************************************************
 * @brief   内部：单字节随机读事务.
 * @param   i2c : I2C 句柄（已加锁）.
 * @param   reg : 寄存器地址.
 * @param   val : 读取输出.
 * @return  成功返回 0，失败返回 1.
 ********************************************************************************/
static uint8_t hw_i2c_read_one_reg_locked(ls2k0300_hw_i2c_t *i2c, uint8_t reg, uint8_t *val)
{
    if (hw_i2c_wait_bus_free(i2c, i2c->timeout_us) != 0) {
        return 1U;
    }

    hw_i2c_set_bits(i2c->i2c_cr1, I2C_CR1_ACK);
    hw_i2c_clr_bits(i2c->i2c_cr1, I2C_CR1_POS);

    if (hw_i2c_generate_start(i2c) != 0) {
        return 1U;
    }

    if (hw_i2c_send_addr(i2c, (uint8_t)(i2c->addr << 1)) != 0) {
        hw_i2c_generate_stop(i2c);
        return 1U;
    }
    hw_i2c_clear_addr_flag(i2c);

    if (hw_i2c_wait_sr1_flag(i2c, I2C_SR1_TXE, i2c->timeout_us) != 0) {
        hw_i2c_generate_stop(i2c);
        return 1U;
    }
    ls_writel(i2c->i2c_dr, (uint32_t)reg);

    if (hw_i2c_wait_sr1_flag(i2c, I2C_SR1_BTF, i2c->timeout_us) != 0) {
        hw_i2c_generate_stop(i2c);
        return 1U;
    }

    /* 单字节接收：先关 ACK，再发送重复 START 并进入读地址阶段 */
    hw_i2c_clr_bits(i2c->i2c_cr1, I2C_CR1_ACK);
    if (hw_i2c_generate_start(i2c) != 0) {
        hw_i2c_set_bits(i2c->i2c_cr1, I2C_CR1_ACK);
        hw_i2c_generate_stop(i2c);
        return 1U;
    }

    if (hw_i2c_send_addr(i2c, (uint8_t)((i2c->addr << 1) | 0x01U)) != 0) {
        hw_i2c_set_bits(i2c->i2c_cr1, I2C_CR1_ACK);
        hw_i2c_generate_stop(i2c);
        return 1U;
    }

    hw_i2c_generate_stop(i2c);
    hw_i2c_clear_addr_flag(i2c);

    if (hw_i2c_wait_sr1_flag(i2c, I2C_SR1_RXNE, i2c->timeout_us) != 0) {
        hw_i2c_set_bits(i2c->i2c_cr1, I2C_CR1_ACK);
        return 1U;
    }
    *val = (uint8_t)(ls_readl(i2c->i2c_dr) & 0xFFU);

    hw_i2c_set_bits(i2c->i2c_cr1, I2C_CR1_ACK);
    hw_i2c_clr_bits(i2c->i2c_cr1, I2C_CR1_POS);
    return 0U;
}

/********************************************************************************
 * @brief   初始化硬件 I2C（可选复用组）.
 ********************************************************************************/
int ls2k0300_hw_i2c_init_ex(ls2k0300_hw_i2c_t *i2c, ls_hw_i2c_port_t port,
                            ls_hw_i2c_mux_t mux, uint8_t addr, uint32_t speed_hz)
{
    uint32_t base_addr;

    if (i2c == NULL || port >= LS_HW_I2C_INVALID || mux >= LS_HW_I2C_MUX_INVALID || addr > 0x7FU) {
        return -1;
    }

    memset(i2c, 0, sizeof(*i2c));
    pthread_mutex_init(&i2c->mtx, NULL);
    pthread_mutex_lock(&i2c->mtx);

    base_addr = hw_i2c_get_base_addr(port);
    if (base_addr == 0U) {
        pthread_mutex_unlock(&i2c->mtx);
        pthread_mutex_destroy(&i2c->mtx);
        return -1;
    }

    i2c->i2c_base = (ls_reg32_addr_t)ls2k0300_mmap(base_addr, LS_HW_I2C_MMAP_SIZE);
    if (i2c->i2c_base == NULL) {
        pthread_mutex_unlock(&i2c->mtx);
        pthread_mutex_destroy(&i2c->mtx);
        return -1;
    }

    i2c->i2c_cr1 = ls_reg_addr_calc(i2c->i2c_base, I2C_CR1_OFS);
    i2c->i2c_cr2 = ls_reg_addr_calc(i2c->i2c_base, I2C_CR2_OFS);
    i2c->i2c_oar = ls_reg_addr_calc(i2c->i2c_base, I2C_OAR_OFS);
    i2c->i2c_dr = ls_reg_addr_calc(i2c->i2c_base, I2C_DR_OFS);
    i2c->i2c_sr1 = ls_reg_addr_calc(i2c->i2c_base, I2C_SR1_OFS);
    i2c->i2c_sr2 = ls_reg_addr_calc(i2c->i2c_base, I2C_SR2_OFS);
    i2c->i2c_ccr = ls_reg_addr_calc(i2c->i2c_base, I2C_CCR_OFS);
    i2c->i2c_trise = ls_reg_addr_calc(i2c->i2c_base, I2C_TRISE_OFS);

    i2c->port = port;
    i2c->mux = mux;
    i2c->addr = addr;
    i2c->input_clk_hz = LS_HW_I2C_DEFAULT_INPUT_CLK_HZ;
    i2c->timeout_us = LS_HW_I2C_DEFAULT_TIMEOUT_US;

    /* 按端口+mux 显式设置 I2C 复用引脚 */
    if (hw_i2c_apply_pinmux(port, mux) != 0) {
        ls2k0300_munmap((void *)i2c->i2c_base, LS_HW_I2C_MMAP_SIZE);
        i2c->i2c_base = NULL;
        pthread_mutex_unlock(&i2c->mtx);
        pthread_mutex_destroy(&i2c->mtx);
        return -1;
    }

    /* 复位控制器状态机 */
    ls_writel(i2c->i2c_cr1, 0U);
    hw_i2c_set_bits(i2c->i2c_cr1, I2C_CR1_SWRST);
    hw_i2c_delay_us(100U);
    ls_writel(i2c->i2c_cr1, 0U);

    if (hw_i2c_apply_timing(i2c, speed_hz) != 0) {
        ls2k0300_munmap((void *)i2c->i2c_base, LS_HW_I2C_MMAP_SIZE);
        i2c->i2c_base = NULL;
        pthread_mutex_unlock(&i2c->mtx);
        pthread_mutex_destroy(&i2c->mtx);
        return -1;
    }

    hw_i2c_set_bits(i2c->i2c_cr1, I2C_CR1_PE | I2C_CR1_ACK);
    hw_i2c_clr_bits(i2c->i2c_cr1, I2C_CR1_POS);

    if (hw_i2c_wait_bus_free(i2c, i2c->timeout_us) != 0) {
        hw_i2c_set_bits(i2c->i2c_cr1, I2C_CR1_RECOVER);
        hw_i2c_delay_us(200U);
        hw_i2c_clr_bits(i2c->i2c_cr1, I2C_CR1_RECOVER);
        (void)hw_i2c_wait_bus_free(i2c, i2c->timeout_us);
    }

    i2c->initialized = 1;
    pthread_mutex_unlock(&i2c->mtx);
    return 0;
}

/********************************************************************************
 * @brief   初始化硬件 I2C（兼容入口，默认 m0）.
 ********************************************************************************/
int ls2k0300_hw_i2c_init(ls2k0300_hw_i2c_t *i2c, ls_hw_i2c_port_t port, uint8_t addr, uint32_t speed_hz)
{
    return ls2k0300_hw_i2c_init_ex(i2c, port, LS_HW_I2C_MUX_M0, addr, speed_hz);
}

/********************************************************************************
 * @brief   释放硬件 I2C.
 ********************************************************************************/
void ls2k0300_hw_i2c_deinit(ls2k0300_hw_i2c_t *i2c)
{
    if (i2c == NULL || i2c->initialized == 0) {
        return;
    }

    pthread_mutex_lock(&i2c->mtx);

    if (i2c->i2c_base != NULL) {
        ls_writel(i2c->i2c_cr1, 0U);
        ls2k0300_munmap((void *)i2c->i2c_base, LS_HW_I2C_MMAP_SIZE);
    }

    i2c->i2c_base = NULL;
    i2c->i2c_cr1 = NULL;
    i2c->i2c_cr2 = NULL;
    i2c->i2c_oar = NULL;
    i2c->i2c_dr = NULL;
    i2c->i2c_sr1 = NULL;
    i2c->i2c_sr2 = NULL;
    i2c->i2c_ccr = NULL;
    i2c->i2c_trise = NULL;
    i2c->initialized = 0;
    i2c->addr = 0xFFU;
    i2c->mux = LS_HW_I2C_MUX_INVALID;
    i2c->port = LS_HW_I2C_INVALID;

    pthread_mutex_unlock(&i2c->mtx);
    pthread_mutex_destroy(&i2c->mtx);
}

/********************************************************************************
 * @brief   设置从设备地址.
 ********************************************************************************/
int ls2k0300_hw_i2c_set_addr(ls2k0300_hw_i2c_t *i2c, uint8_t addr)
{
    if (i2c == NULL || addr > 0x7FU || i2c->initialized == 0) {
        return -1;
    }

    pthread_mutex_lock(&i2c->mtx);
    i2c->addr = addr;
    pthread_mutex_unlock(&i2c->mtx);

    return 0;
}

/********************************************************************************
 * @brief   设置 I2C 速率.
 ********************************************************************************/
int ls2k0300_hw_i2c_set_speed(ls2k0300_hw_i2c_t *i2c, uint32_t speed_hz)
{
    int ret;

    if (i2c == NULL || i2c->initialized == 0) {
        return -1;
    }

    pthread_mutex_lock(&i2c->mtx);

    /* 先关闭外设后重配时序，再重新使能 */
    hw_i2c_clr_bits(i2c->i2c_cr1, I2C_CR1_PE);
    ret = hw_i2c_apply_timing(i2c, speed_hz);
    if (ret == 0) {
        hw_i2c_set_bits(i2c->i2c_cr1, I2C_CR1_PE | I2C_CR1_ACK);
        hw_i2c_clr_bits(i2c->i2c_cr1, I2C_CR1_POS);
    }

    pthread_mutex_unlock(&i2c->mtx);
    return ret;
}

/********************************************************************************
 * @brief   总线恢复.
 ********************************************************************************/
int ls2k0300_hw_i2c_recover(ls2k0300_hw_i2c_t *i2c)
{
    if (i2c == NULL || i2c->initialized == 0) {
        return -1;
    }

    pthread_mutex_lock(&i2c->mtx);
    hw_i2c_set_bits(i2c->i2c_cr1, I2C_CR1_RECOVER);
    hw_i2c_delay_us(200U);
    hw_i2c_clr_bits(i2c->i2c_cr1, I2C_CR1_RECOVER);
    hw_i2c_clear_error_flags(i2c);
    pthread_mutex_unlock(&i2c->mtx);

    return 0;
}

/********************************************************************************
 * @brief   单字节读寄存器.
 ********************************************************************************/
uint8_t ls2k0300_hw_i2c_read_byte(ls2k0300_hw_i2c_t *i2c, uint8_t reg, uint8_t *val)
{
    uint8_t ret;

    if (i2c == NULL || val == NULL || i2c->initialized == 0) {
        return 1U;
    }

    pthread_mutex_lock(&i2c->mtx);
    ret = hw_i2c_read_one_reg_locked(i2c, reg, val);
    pthread_mutex_unlock(&i2c->mtx);

    return ret;
}

/********************************************************************************
 * @brief   单字节写寄存器.
 ********************************************************************************/
uint8_t ls2k0300_hw_i2c_write_byte(ls2k0300_hw_i2c_t *i2c, uint8_t reg, uint8_t val)
{
    uint8_t ret;

    if (i2c == NULL || i2c->initialized == 0) {
        return 1U;
    }

    pthread_mutex_lock(&i2c->mtx);
    ret = hw_i2c_write_reg_locked(i2c, reg, &val, 1U);
    pthread_mutex_unlock(&i2c->mtx);

    return ret;
}

/********************************************************************************
 * @brief   连续读寄存器.
 ********************************************************************************/
uint8_t ls2k0300_hw_i2c_read_n_byte(ls2k0300_hw_i2c_t *i2c, uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint16_t i;

    if (i2c == NULL || buf == NULL || len == 0U || i2c->initialized == 0) {
        return 1U;
    }

    pthread_mutex_lock(&i2c->mtx);
    for (i = 0U; i < len; ++i) {
        if (hw_i2c_read_one_reg_locked(i2c, (uint8_t)(reg + i), &buf[i]) != 0U) {
            pthread_mutex_unlock(&i2c->mtx);
            return 1U;
        }
    }
    pthread_mutex_unlock(&i2c->mtx);

    return 0U;
}

/********************************************************************************
 * @brief   连续写寄存器.
 ********************************************************************************/
uint8_t ls2k0300_hw_i2c_write_n_byte(ls2k0300_hw_i2c_t *i2c, uint8_t reg, const uint8_t *buf, uint16_t len)
{
    uint8_t ret;

    if (i2c == NULL || buf == NULL || len == 0U || i2c->initialized == 0) {
        return 1U;
    }

    pthread_mutex_lock(&i2c->mtx);
    ret = hw_i2c_write_reg_locked(i2c, reg, buf, len);
    pthread_mutex_unlock(&i2c->mtx);

    return ret;
}
