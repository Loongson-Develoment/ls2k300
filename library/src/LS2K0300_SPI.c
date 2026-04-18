#include "LS2K0300_SPI.h"
#include "LS2K0300_CLOCK.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>

/********************************************************************************
 * @brief   SPI1(SPI-FLASH 控制器) 与 SPI2/3(SPI-IO 控制器) 基地址定义.
 ********************************************************************************/
#define LS_SPI1_BASE_ADDR           (0x16018000U)
#define LS_SPI2_BASE_ADDR           (0x1610C000U)
#define LS_SPI3_BASE_ADDR           (0x1610E000U)
#define LS_SPI_MMAP_SIZE            (0x1000U)

/********************************************************************************
 * @brief   SPI1(SPI0/1 控制器) 寄存器偏移定义.
 * @note    参考用户手册第 9 章：SPCR/SPSR/TxFIFO-RxFIFO/SPER/SFC_*.
 ********************************************************************************/
#define LS_SPI01_SPCR               (0x00U)
#define LS_SPI01_SPSR               (0x01U)
#define LS_SPI01_FIFO               (0x02U)
#define LS_SPI01_SPER               (0x03U)
#define LS_SPI01_SFC_PARAM          (0x04U)
#define LS_SPI01_SFC_SOFTCS         (0x05U)
#define LS_SPI01_SFC_TIMING         (0x06U)

#define LS_SPI01_SPCR_SPIE          (1U << 7)
#define LS_SPI01_SPCR_SPE           (1U << 6)
#define LS_SPI01_SPCR_MSTR          (1U << 4)
#define LS_SPI01_SPCR_CPOL          (1U << 3)
#define LS_SPI01_SPCR_CPHA          (1U << 2)
#define LS_SPI01_SPCR_SPR_MASK      (0x03U)

#define LS_SPI01_SPSR_SPIF          (1U << 7)
#define LS_SPI01_SPSR_WCOL          (1U << 6)
#define LS_SPI01_SPSR_WFFULL        (1U << 3)
#define LS_SPI01_SPSR_WFEMPTY       (1U << 2)
#define LS_SPI01_SPSR_RFFULL        (1U << 1)
#define LS_SPI01_SPSR_RFEMPTY       (1U << 0)

#define LS_SPI01_SPER_MODE          (1U << 2)
#define LS_SPI01_SPER_SPRE_MASK     (0x03U)

#define LS_SPI01_SFC_PARAM_DUAL_IO  (1U << 3)
#define LS_SPI01_SFC_PARAM_FAST_RD  (1U << 2)
#define LS_SPI01_SFC_PARAM_BURST_EN (1U << 1)
#define LS_SPI01_SFC_PARAM_MEMORY_EN (1U << 0)

#define LS_SPI01_BYTE_TIMEOUT_US    (100000U)

/********************************************************************************
 * @brief   SPI2/3 寄存器偏移定义.
 ********************************************************************************/
#define LS_SPI_CR1                  (0x00U)
#define LS_SPI_CR2                  (0x04U)
#define LS_SPI_CR3                  (0x08U)
#define LS_SPI_CR4                  (0x0CU)
#define LS_SPI_IER                  (0x10U)
#define LS_SPI_SR1                  (0x14U)
#define LS_SPI_SR2                  (0x18U)
#define LS_SPI_CFG1                 (0x20U)
#define LS_SPI_CFG2                 (0x24U)
#define LS_SPI_CFG3                 (0x28U)
#define LS_SPI_DR                   (0x40U)

/********************************************************************************
 * @brief   SPI 控制位定义.
 ********************************************************************************/
#define LS_SPI_CR1_AUTOSUS          (1U << 2)
#define LS_SPI_CR1_CSTART           (1U << 1)
#define LS_SPI_CR1_SPE              (1U << 0)

#define LS_SPI_CR2_TXFTHLV_SHIFT    (8U)
#define LS_SPI_CR2_TXFTHLV          (0x3U << 8)
#define LS_SPI_CR2_RXFTHLV_SHIFT    (0U)
#define LS_SPI_CR2_RXFTHLV          (0x3U << 0)

#define LS_SPI_CFG1_DSIZE_SHIFT     (8U)
#define LS_SPI_CFG1_DSIZE           (0x1FU << 8)
#define LS_SPI_CFG1_LSBFRST         (1U << 7)
#define LS_SPI_CFG1_CPHA            (1U << 1)
#define LS_SPI_CFG1_CPOL            (1U << 0)

#define LS_SPI_CFG2_BRINT_SHIFT     (8U)
#define LS_SPI_CFG2_BRINT           (0xFFU << 8)

#define LS_SPI_CFG3_DOE             (1U << 3)
#define LS_SPI_CFG3_DIE             (1U << 2)
#define LS_SPI_CFG3_DIOSWP          (1U << 1)
#define LS_SPI_CFG3_MSTR            (1U << 0)

#define LS_SPI_SR1_EOT              (1U << 15)
#define LS_SPI_SR1_MODF             (1U << 11)
#define LS_SPI_SR1_OVR              (1U << 8)
#define LS_SPI_SR1_TXA              (1U << 1)
#define LS_SPI_SR1_RXA              (1U << 0)

typedef struct {
    uint16_t div;
    uint8_t spr;
    uint8_t spre;
} ls_spi01_div_cfg_t;

static const ls_spi01_div_cfg_t g_spi01_div_table[] = {
    {2U,    0U, 0U},
    {4U,    1U, 0U},
    {8U,    0U, 1U},
    {16U,   2U, 0U},
    {32U,   3U, 0U},
    {64U,   1U, 1U},
    {128U,  2U, 1U},
    {256U,  3U, 1U},
    {512U,  0U, 2U},
    {1024U, 1U, 2U},
    {2048U, 2U, 2U},
    {4096U, 3U, 2U},
};

/********************************************************************************
 * @brief   根据目标速率选择 SPI1 的 spr/spre 分频配置.
 * @param   speed_hz    : 目标速率.
 * @param   spr         : 输出 SPCR.spr.
 * @param   spre        : 输出 SPER.spre.
 * @param   real_speed  : 输出实际速率.
 * @return  none.
 ********************************************************************************/
static void spi01_pick_divider(uint32_t speed_hz, uint8_t *spr, uint8_t *spre, uint32_t *real_speed)
{
    uint32_t need_div;
    const ls_spi01_div_cfg_t *sel;
    size_t i;

    if (speed_hz == 0U) {
        speed_hz = 1U;
    }

    need_div = ((uint32_t)LS_PMON_CLOCK_FREQ + speed_hz - 1U) / speed_hz;
    sel = &g_spi01_div_table[sizeof(g_spi01_div_table) / sizeof(g_spi01_div_table[0]) - 1U];

    for (i = 0U; i < (sizeof(g_spi01_div_table) / sizeof(g_spi01_div_table[0])); ++i) {
        if (g_spi01_div_table[i].div >= need_div) {
            sel = &g_spi01_div_table[i];
            break;
        }
    }

    *spr = sel->spr;
    *spre = sel->spre;
    *real_speed = (uint32_t)LS_PMON_CLOCK_FREQ / (uint32_t)sel->div;
}

/********************************************************************************
 * @brief   等待 SPI1 状态位达到期望值.
 * @param   spi        : SPI 句柄指针.
 * @param   mask       : 状态位掩码.
 * @param   expect_set : 1 表示等待置位，0 表示等待清零.
 * @param   timeout_us : 超时时间(us).
 * @return  达到期望返回 1，超时返回 0.
 ********************************************************************************/
static int spi01_wait_spsr(ls2k0300_spi_t *spi, uint8_t mask, int expect_set, uint32_t timeout_us)
{
    uint32_t waited_us;
    const uint32_t step_us = 10U;
    uint8_t spsr;

    waited_us = 0U;
    while (waited_us < timeout_us) {
        spsr = ls_readb(spi->spi_sr1);
        if (expect_set != 0) {
            if ((spsr & mask) != 0U) {
                return 1;
            }
        } else {
            if ((spsr & mask) == 0U) {
                return 1;
            }
        }
        usleep(step_us);
        waited_us += step_us;
    }

    return 0;
}

/********************************************************************************
 * @brief   应用 SPI1 的模式和速率设置.
 * @param   spi   : SPI 句柄指针.
 * @param   mode  : SPI 模式.
 * @param   speed : 目标速率(Hz).
 * @return  none.
 ********************************************************************************/
static void spi01_set_mode_and_speed(ls2k0300_spi_t *spi, ls_reg_spi_mode_t mode, uint32_t speed)
{
    uint8_t spcr;
    uint8_t sper;
    uint8_t spr;
    uint8_t spre;
    uint32_t real_speed;

    spi01_pick_divider(speed, &spr, &spre, &real_speed);

    spcr = ls_readb(spi->spi_cr1);
    spcr &= (uint8_t)~(LS_SPI01_SPCR_SPIE | LS_SPI01_SPCR_CPOL | LS_SPI01_SPCR_CPHA | LS_SPI01_SPCR_SPR_MASK);
    spcr |= LS_SPI01_SPCR_MSTR;
    if (mode == LS_SPI_MODE_1 || mode == LS_SPI_MODE_3) {
        spcr |= LS_SPI01_SPCR_CPHA;
    }
    if (mode == LS_SPI_MODE_2 || mode == LS_SPI_MODE_3) {
        spcr |= LS_SPI01_SPCR_CPOL;
    }
    spcr |= spr;
    ls_writeb(spi->spi_cr1, spcr);

    sper = ls_readb(spi->spi_cfg1);
    sper &= (uint8_t)~(LS_SPI01_SPER_MODE | LS_SPI01_SPER_SPRE_MASK);
    sper |= (uint8_t)(LS_SPI01_SPER_MODE | (spre & LS_SPI01_SPER_SPRE_MASK));
    ls_writeb(spi->spi_cfg1, sper);

    spi->spi_mode = mode;
    spi->speed_hz = real_speed;
}

/********************************************************************************
 * @brief   设置 SPI TX/RX FIFO 阈值.
 * @param   spi : SPI 句柄指针.
 * @param   tx  : TX FIFO 阈值.
 * @param   rx  : RX FIFO 阈值.
 * @return  none.
 ********************************************************************************/
static void spi_set_fifo_threshold(ls2k0300_spi_t *spi, ls_spi_fifo_threshould_t tx, ls_spi_fifo_threshould_t rx)
{
    uint32_t val;

    /* 同时更新 TX/RX FIFO 阈值，避免两次写寄存器的中间态 */
    val = ls_readl(spi->spi_cr2);
    val &= ~(LS_SPI_CR2_TXFTHLV | LS_SPI_CR2_RXFTHLV);
    val |= ((uint32_t)tx << LS_SPI_CR2_TXFTHLV_SHIFT);
    val |= ((uint32_t)rx << LS_SPI_CR2_RXFTHLV_SHIFT);
    ls_writel(spi->spi_cr2, val);

    spi->tx_thresh = tx;
    spi->rx_thresh = rx;
}

/********************************************************************************
 * @brief   配置 SPI 模式（Mode0~3）.
 * @param   spi  : SPI 句柄指针.
 * @param   mode : SPI 模式.
 * @return  none.
 ********************************************************************************/
static void spi_set_mode(ls2k0300_spi_t *spi, ls_reg_spi_mode_t mode)
{
    uint32_t cfg1;

    /* CPOL/CPHA 组合决定 SPI Mode0~3 */
    cfg1 = ls_readl(spi->spi_cfg1);
    cfg1 &= ~(LS_SPI_CFG1_CPOL | LS_SPI_CFG1_CPHA);
    cfg1 |= (uint32_t)mode;
    ls_writel(spi->spi_cfg1, cfg1);

    spi->spi_mode = mode;
}

/********************************************************************************
 * @brief   配置 SPI 数据位宽.
 * @param   spi : SPI 句柄指针.
 * @param   bpw : 位宽配置.
 * @return  none.
 ********************************************************************************/
static void spi_set_bits_per_word(ls2k0300_spi_t *spi, ls_spi_bits_per_word_t bpw)
{
    uint32_t cfg1;

    /* DSIZE 写入的是位宽减 1 */
    cfg1 = ls_readl(spi->spi_cfg1);
    cfg1 &= ~LS_SPI_CFG1_DSIZE;
    cfg1 |= (((uint32_t)bpw - 1U) << LS_SPI_CFG1_DSIZE_SHIFT);
    ls_writel(spi->spi_cfg1, cfg1);

    spi->bpw = bpw;
}

/********************************************************************************
 * @brief   配置 SPI 数据位序.
 * @param   spi : SPI 句柄指针.
 * @param   ord : 位序配置.
 * @return  none.
 ********************************************************************************/
static void spi_set_data_order(ls2k0300_spi_t *spi, ls_spi_data_order_t ord)
{
    uint32_t cfg1;

    /* LSBFRST=1 表示低位先发，默认是高位先发 */
    cfg1 = ls_readl(spi->spi_cfg1);
    if (ord == LS_SPI_LSB_FIRST) {
        cfg1 |= LS_SPI_CFG1_LSBFRST;
    } else {
        cfg1 &= ~LS_SPI_CFG1_LSBFRST;
    }
    ls_writel(spi->spi_cfg1, cfg1);

    spi->ord = ord;
}

/********************************************************************************
 * @brief   配置 SPI 通信速率.
 * @param   spi   : SPI 句柄指针.
 * @param   speed : 目标速率(Hz).
 * @return  none.
 ********************************************************************************/
static void spi_set_speed(ls2k0300_spi_t *spi, uint32_t speed)
{
    uint32_t div;
    uint32_t cfg2;

    if (speed == 0U) {
        speed = 1U;
    }

    /* 根据 PMON 时钟计算最接近的分频值 */
    div = (uint32_t)(LS_PMON_CLOCK_FREQ / (long)speed);
    if (div < 2U) {
        div = 2U;
    }
    if (div > 255U) {
        div = 255U;
    }

    cfg2 = ls_readl(spi->spi_cfg2);
    cfg2 &= ~LS_SPI_CFG2_BRINT;
    cfg2 |= (div << LS_SPI_CFG2_BRINT_SHIFT);
    ls_writel(spi->spi_cfg2, cfg2);

    spi->speed_hz = (uint32_t)(LS_PMON_CLOCK_FREQ / (long)div);
}

/********************************************************************************
 * @brief   等待本次 SPI 事务完成.
 * @param   spi        : SPI 句柄指针.
 * @param   timeout_ms : 超时时间（毫秒）.
 * @return  完成返回 1，超时返回 0.
 ********************************************************************************/
static int spi_wait_transfer_complete(ls2k0300_spi_t *spi, uint32_t timeout_ms)
{
    uint32_t waited_us;

    waited_us = 0U;
    /* 轮询 EOT 结束标志，超时则返回失败 */
    while ((ls_readl(spi->spi_sr1) & LS_SPI_SR1_EOT) == 0U) {
        if (waited_us >= timeout_ms * 1000U) {
            return 0;
        }
        usleep(10);
        waited_us += 10U;
    }

    return 1;
}

/********************************************************************************
 * @brief   向 SPI FIFO 写入数据.
 * @param   spi    : SPI 句柄指针.
 * @param   tx_buf : 发送缓存.
 * @param   len    : 期望写入长度（字节）.
 * @return  实际写入字节数.
 ********************************************************************************/
static ssize_t spi_write_fifo(ls2k0300_spi_t *spi, const uint8_t *tx_buf, size_t len)
{
    ssize_t written = 0;

    /* 按位宽批量写 FIFO，尽量减少总线访问次数 */
    while ((size_t)written < len && (ls_readl(spi->spi_sr1) & LS_SPI_SR1_TXA) != 0U) {
        if (spi->bpw == LS_SPI_BPW_32 && (len - (size_t)written) >= 4U) {
            uint32_t data = *((const uint32_t *)(tx_buf + written));
            ls_writel(spi->spi_dr, data);
            written += 4;
        } else if (spi->bpw == LS_SPI_BPW_16 && (len - (size_t)written) >= 2U) {
            uint16_t data = *((const uint16_t *)(tx_buf + written));
            ls_writel(spi->spi_dr, data);
            written += 2;
        } else {
            ls_writeb(spi->spi_dr, tx_buf[written]);
            written += 1;
        }
    }

    return written;
}

/********************************************************************************
 * @brief   从 SPI FIFO 读取数据.
 * @param   spi    : SPI 句柄指针.
 * @param   rx_buf : 接收缓存.
 * @param   len    : 期望读取长度（字节）.
 * @return  实际读取字节数.
 ********************************************************************************/
static ssize_t spi_read_fifo(ls2k0300_spi_t *spi, uint8_t *rx_buf, size_t len)
{
    ssize_t read_len = 0;

    /* 按位宽读取 FIFO，与写入端保持一致的数据打包方式 */
    while ((size_t)read_len < len && (ls_readl(spi->spi_sr1) & LS_SPI_SR1_RXA) != 0U) {
        if (spi->bpw == LS_SPI_BPW_32 && (len - (size_t)read_len) >= 4U) {
            uint32_t data = ls_readl(spi->spi_dr);
            *((uint32_t *)(rx_buf + read_len)) = data;
            read_len += 4;
        } else if (spi->bpw == LS_SPI_BPW_16 && (len - (size_t)read_len) >= 2U) {
            uint16_t data = (uint16_t)ls_readl(spi->spi_dr);
            *((uint16_t *)(rx_buf + read_len)) = data;
            read_len += 2;
        } else {
            rx_buf[read_len] = (uint8_t)ls_readb(spi->spi_dr);
            read_len += 1;
        }
    }

    return read_len;
}

/********************************************************************************
 * @brief   使能 SPI 控制器.
 * @param   spi : SPI 句柄指针.
 * @return  none.
 * @example ls2k0300_spi_enable(&spi);
 ********************************************************************************/
void ls2k0300_spi_enable(ls2k0300_spi_t *spi)
{
    uint8_t spcr;

    if (spi == NULL || spi->spi_cr1 == NULL) {
        return;
    }

    pthread_mutex_lock(&spi->mtx);
    if (spi->port == LS_SPI1) {
        /* SPI1(SPI-FLASH 主控器) 的使能位在 SPCR[6] */
        spcr = ls_readb(spi->spi_cr1);
        spcr |= (uint8_t)(LS_SPI01_SPCR_MSTR | LS_SPI01_SPCR_SPE);
        ls_writeb(spi->spi_cr1, spcr);
    } else {
        /* SPI2/3 使用 CR1.SPE */
        ls_writel(spi->spi_cr1, ls_readl(spi->spi_cr1) | LS_SPI_CR1_SPE);
    }
    pthread_mutex_unlock(&spi->mtx);
}

/********************************************************************************
 * @brief   禁用 SPI 控制器.
 * @param   spi : SPI 句柄指针.
 * @return  none.
 * @example ls2k0300_spi_disable(&spi);
 ********************************************************************************/
void ls2k0300_spi_disable(ls2k0300_spi_t *spi)
{
    uint32_t cr1;
    uint8_t spcr;

    if (spi == NULL || spi->spi_cr1 == NULL) {
        return;
    }

    pthread_mutex_lock(&spi->mtx);

    if (spi->port == LS_SPI1) {
        /* SPI1 仅需清 SPCR.SPE */
        spcr = ls_readb(spi->spi_cr1);
        spcr &= (uint8_t)~LS_SPI01_SPCR_SPE;
        ls_writeb(spi->spi_cr1, spcr);
    } else {
        /* SPI2/3：先清 CSTART 停止当前事务，再清 SPE 关闭外设 */
        cr1 = ls_readl(spi->spi_cr1);
        cr1 &= ~LS_SPI_CR1_CSTART;
        ls_writel(spi->spi_cr1, cr1);

        cr1 = ls_readl(spi->spi_cr1);
        cr1 &= ~LS_SPI_CR1_SPE;
        ls_writel(spi->spi_cr1, cr1);
    }

    pthread_mutex_unlock(&spi->mtx);
}

/********************************************************************************
 * @brief   SPI1 字节流传输实现.
 * @param   spi    : SPI 句柄指针.
 * @param   tx_buf : 发送缓冲区，可为 NULL.
 * @param   rx_buf : 接收缓冲区，可为 NULL.
 * @param   len    : 传输字节数.
 * @return  成功返回 0，失败返回 -1.
 ********************************************************************************/
static ssize_t spi_transfer_spi1(ls2k0300_spi_t *spi, const uint8_t *tx_buf, uint8_t *rx_buf, size_t len)
{
    size_t i;
    uint8_t tx;
    uint8_t rx;

    /* 每次事务开始前清 SPIF/WCOL 状态位 */
    ls_writeb(spi->spi_sr1, (uint8_t)(LS_SPI01_SPSR_SPIF | LS_SPI01_SPSR_WCOL));
    ls2k0300_spi_enable(spi);

    for (i = 0U; i < len; ++i) {
        tx = (tx_buf != NULL) ? tx_buf[i] : 0U;

        if (!spi01_wait_spsr(spi, (uint8_t)LS_SPI01_SPSR_WFFULL, 0, LS_SPI01_BYTE_TIMEOUT_US)) {
            ls2k0300_spi_disable(spi);
            return -1;
        }

        ls_writeb(spi->spi_dr, tx);

        if (!spi01_wait_spsr(spi, (uint8_t)LS_SPI01_SPSR_RFEMPTY, 0, LS_SPI01_BYTE_TIMEOUT_US)) {
            ls2k0300_spi_disable(spi);
            return -1;
        }

        rx = ls_readb(spi->spi_dr);
        if (rx_buf != NULL) {
            rx_buf[i] = rx;
        }
    }

    ls2k0300_spi_disable(spi);
    return 0;
}

/********************************************************************************
 * @brief   初始化 SPI 句柄.
 * @param   spi   : SPI 句柄指针.
 * @param   port  : SPI 端口号.
 * @param   speed : 目标速率(Hz).
 * @param   mode  : SPI 模式.
 * @return  成功返回 0，失败返回 -1.
 * @example ls2k0300_spi_init(&spi, LS_SPI2, 1000000, LS_SPI_MODE_0);
 ********************************************************************************/
int ls2k0300_spi_init(ls2k0300_spi_t *spi, ls_spi_port_t port, uint32_t speed, ls_reg_spi_mode_t mode)
{
    ls_reg_base_t base_addr;
    uint32_t cfg3;
    uint32_t cr1;
    uint8_t sfc_param;

    if (spi == NULL || mode >= LS_SPI_MODE_INVALID || speed == 0U) {
        return -1;
    }

    memset(spi, 0, sizeof(*spi));
    pthread_mutex_init(&spi->mtx, NULL);
    pthread_mutex_init(&spi->mtx_transfer, NULL);

    spi->port = port;
    spi->speed_hz = speed;
    spi->spi_mode = mode;
    spi->bpw = LS_SPI_BPW_8;
    spi->ord = LS_SPI_MSB_FIRST;
    spi->tx_thresh = LS_SPI_FIFO_THRESH_1;
    spi->rx_thresh = LS_SPI_FIFO_THRESH_1;

    /* 按端口配置固定引脚复用，并使用软件 GPIO 控制 CS */
    if (port == LS_SPI1) {
        /* 手册第 1.3 节：SPI1 使用 GPIO60~63 */
        ls2k0300_gpio_mux_set(PIN_60, GPIO_MUX_MAIN);
        ls2k0300_gpio_mux_set(PIN_61, GPIO_MUX_MAIN);
        ls2k0300_gpio_mux_set(PIN_62, GPIO_MUX_MAIN);
        if (ls2k0300_gpio_init(&spi->cs_gpio, PIN_63, GPIO_MODE_OUT, GPIO_MUX_GPIO) != 0) {
            goto err_out;
        }
        base_addr = LS_SPI1_BASE_ADDR;
    } else if (port == LS_SPI2) {
        ls2k0300_gpio_mux_set(PIN_64, GPIO_MUX_MAIN);
        ls2k0300_gpio_mux_set(PIN_65, GPIO_MUX_MAIN);
        ls2k0300_gpio_mux_set(PIN_66, GPIO_MUX_MAIN);
        if (ls2k0300_gpio_init(&spi->cs_gpio, PIN_67, GPIO_MODE_OUT, GPIO_MUX_GPIO) != 0) {
            goto err_out;
        }
        base_addr = LS_SPI2_BASE_ADDR;
    } else if (port == LS_SPI3) {
        ls2k0300_gpio_mux_set(PIN_82, GPIO_MUX_ALT1);
        ls2k0300_gpio_mux_set(PIN_83, GPIO_MUX_ALT1);
        ls2k0300_gpio_mux_set(PIN_84, GPIO_MUX_ALT1);
        if (ls2k0300_gpio_init(&spi->cs_gpio, PIN_85, GPIO_MODE_OUT, GPIO_MUX_GPIO) != 0) {
            goto err_out;
        }
        base_addr = LS_SPI3_BASE_ADDR;
    } else {
        goto err_out;
    }

    ls2k0300_gpio_level_set(&spi->cs_gpio, GPIO_HIGH);

    /* 映射 SPI 控制器寄存器空间 */
    spi->spi_base = (ls_reg32_addr_t)ls2k0300_mmap(base_addr, LS_SPI_MMAP_SIZE);
    if (spi->spi_base == NULL) {
        goto err_gpio;
    }

    if (port == LS_SPI1) {
        /* SPI1(SPI-FLASH 控制器) 寄存器映射 */
        spi->spi_cr1 = ls_reg_addr_calc(spi->spi_base, LS_SPI01_SPCR);
        spi->spi_sr1 = ls_reg_addr_calc(spi->spi_base, LS_SPI01_SPSR);
        spi->spi_dr = ls_reg_addr_calc(spi->spi_base, LS_SPI01_FIFO);
        spi->spi_cfg1 = ls_reg_addr_calc(spi->spi_base, LS_SPI01_SPER);
        spi->spi_cfg2 = ls_reg_addr_calc(spi->spi_base, LS_SPI01_SFC_PARAM);
        spi->spi_cfg3 = ls_reg_addr_calc(spi->spi_base, LS_SPI01_SFC_SOFTCS);

        ls2k0300_spi_disable(spi);
        ls_writeb(spi->spi_sr1, (uint8_t)(LS_SPI01_SPSR_SPIF | LS_SPI01_SPSR_WCOL));

        /* 关闭 flash memory 映射读，进入主控器直连 SPI 总线模式 */
        sfc_param = ls_readb(spi->spi_cfg2);
        sfc_param &= (uint8_t)~(LS_SPI01_SFC_PARAM_MEMORY_EN |
                                LS_SPI01_SFC_PARAM_DUAL_IO |
                                LS_SPI01_SFC_PARAM_FAST_RD |
                                LS_SPI01_SFC_PARAM_BURST_EN);
        ls_writeb(spi->spi_cfg2, sfc_param);

        /* 这里使用独立 GPIO 片选，禁用控制器内部 softcs 控制 */
        ls_writeb(spi->spi_cfg3, 0U);

        /* SPI1 主控器只支持 8-bit 帧 */
        spi->bpw = LS_SPI_BPW_8;
        spi->ord = LS_SPI_MSB_FIRST;
        spi01_set_mode_and_speed(spi, mode, speed);
    } else {
        /* SPI2/3(SPI-IO 控制器) 寄存器映射 */
        spi->spi_cr1 = ls_reg_addr_calc(spi->spi_base, LS_SPI_CR1);
        spi->spi_cr2 = ls_reg_addr_calc(spi->spi_base, LS_SPI_CR2);
        spi->spi_cr3 = ls_reg_addr_calc(spi->spi_base, LS_SPI_CR3);
        spi->spi_cr4 = ls_reg_addr_calc(spi->spi_base, LS_SPI_CR4);
        spi->spi_ier = ls_reg_addr_calc(spi->spi_base, LS_SPI_IER);
        spi->spi_sr1 = ls_reg_addr_calc(spi->spi_base, LS_SPI_SR1);
        spi->spi_sr2 = ls_reg_addr_calc(spi->spi_base, LS_SPI_SR2);
        spi->spi_cfg1 = ls_reg_addr_calc(spi->spi_base, LS_SPI_CFG1);
        spi->spi_cfg2 = ls_reg_addr_calc(spi->spi_base, LS_SPI_CFG2);
        spi->spi_cfg3 = ls_reg_addr_calc(spi->spi_base, LS_SPI_CFG3);
        spi->spi_dr = ls_reg_addr_calc(spi->spi_base, LS_SPI_DR);

        /* 先清状态再配主机模式，保证初始化过程可重复 */
        ls2k0300_spi_disable(spi);
        ls_writel(spi->spi_sr1, 0xFFFFFFFFU);
        ls_writel(spi->spi_ier, 0);

        cfg3 = ls_readl(spi->spi_cfg3);
        cfg3 |= LS_SPI_CFG3_MSTR;
        cfg3 &= ~LS_SPI_CFG3_DIOSWP;
        ls_writel(spi->spi_cfg3, cfg3);

        cr1 = ls_readl(spi->spi_cr1);
        cr1 &= ~LS_SPI_CR1_AUTOSUS;
        ls_writel(spi->spi_cr1, cr1);

        /* 统一应用默认阈值/模式/位宽/位序/速率 */
        spi_set_fifo_threshold(spi, spi->tx_thresh, spi->rx_thresh);
        spi_set_mode(spi, mode);
        spi_set_bits_per_word(spi, spi->bpw);
        spi_set_data_order(spi, spi->ord);
        spi_set_speed(spi, speed);
    }

    spi->is_initialized = 1;
    return 0;

err_gpio:
    ls2k0300_gpio_deinit(&spi->cs_gpio);
err_out:
    /* 初始化失败时回收互斥锁与句柄状态 */
    pthread_mutex_destroy(&spi->mtx);
    pthread_mutex_destroy(&spi->mtx_transfer);
    memset(spi, 0, sizeof(*spi));
    return -1;
}

/********************************************************************************
 * @brief   释放 SPI 句柄.
 * @param   spi : SPI 句柄指针.
 * @return  none.
 * @example ls2k0300_spi_deinit(&spi);
 ********************************************************************************/
void ls2k0300_spi_deinit(ls2k0300_spi_t *spi)
{
    if (spi == NULL) {
        return;
    }

    if (spi->is_initialized) {
        ls2k0300_spi_disable(spi);
    }

    if (spi->spi_base != NULL) {
        ls2k0300_munmap((void *)spi->spi_base, LS_SPI_MMAP_SIZE);
    }

    ls2k0300_gpio_deinit(&spi->cs_gpio);

    pthread_mutex_destroy(&spi->mtx);
    pthread_mutex_destroy(&spi->mtx_transfer);

    memset(spi, 0, sizeof(*spi));
}

/********************************************************************************
 * @brief   SPI 全双工传输.
 * @param   spi    : SPI 句柄指针.
 * @param   tx_buf : 发送缓冲区，可为 NULL.
 * @param   rx_buf : 接收缓冲区，可为 NULL.
 * @param   len    : 传输长度（字节）.
 * @return  成功返回 0，失败返回 -1.
 * @example ls2k0300_spi_transfer(&spi, tx, rx, len);
 ********************************************************************************/
ssize_t ls2k0300_spi_transfer(ls2k0300_spi_t *spi, const void *tx_buf, void *rx_buf, size_t len)
{
    const uint8_t *tx_data;
    uint8_t *rx_data;
    size_t tx_target;
    size_t rx_target;
    size_t tx_idx;
    size_t rx_idx;
    uint32_t cfg3;
    uint32_t cr1;

    if (spi == NULL || spi->is_initialized == 0 || len == 0U) {
        return -1;
    }

    if (spi->port == LS_SPI1) {
        tx_data = (const uint8_t *)tx_buf;
        rx_data = (uint8_t *)rx_buf;

        pthread_mutex_lock(&spi->mtx_transfer);
        ls2k0300_gpio_level_set(&spi->cs_gpio, GPIO_LOW);

        if (spi_transfer_spi1(spi, tx_data, rx_data, len) != 0) {
            ls2k0300_gpio_level_set(&spi->cs_gpio, GPIO_HIGH);
            pthread_mutex_unlock(&spi->mtx_transfer);
            return -1;
        }

        ls2k0300_gpio_level_set(&spi->cs_gpio, GPIO_HIGH);
        pthread_mutex_unlock(&spi->mtx_transfer);
        return 0;
    }

    pthread_mutex_lock(&spi->mtx_transfer);

    /* 事务开始: CS 拉低 */
    ls2k0300_gpio_level_set(&spi->cs_gpio, GPIO_LOW);

    tx_data = (const uint8_t *)tx_buf;
    rx_data = (uint8_t *)rx_buf;
    tx_target = len;
    rx_target = (rx_buf != NULL) ? len : 0U;
    tx_idx = 0U;
    rx_idx = 0U;

    ls_writel(spi->spi_sr1, 0xFFFFFFFFU);
    ls_writel(spi->spi_sr2, 0xFFFFFFFFU);

    /* 每次传输都重置控制器状态，确保本次方向配置生效 */
    ls2k0300_spi_disable(spi);

    cfg3 = ls_readl(spi->spi_cfg3);
    if (tx_buf != NULL && rx_buf != NULL) {
        cfg3 |= (LS_SPI_CFG3_DOE | LS_SPI_CFG3_DIE);
    } else if (tx_buf != NULL) {
        cfg3 |= LS_SPI_CFG3_DOE;
        cfg3 &= ~LS_SPI_CFG3_DIE;
    } else {
        cfg3 |= (LS_SPI_CFG3_DOE | LS_SPI_CFG3_DIE);
    }
    cfg3 &= ~LS_SPI_CFG3_DIOSWP;
    ls_writel(spi->spi_cfg3, cfg3);

    /* 按位宽折算当前事务帧数 */
    ls_writel(spi->spi_cr3, (uint32_t)(((len * 8U) + (size_t)spi->bpw - 1U) / (size_t)spi->bpw) - 1U);
    ls_writel(spi->spi_cr4, 0);

    ls2k0300_spi_enable(spi);

    if (tx_data != NULL) {
        while ((ls_readl(spi->spi_sr1) & LS_SPI_SR1_TXA) != 0U && tx_idx < tx_target) {
            ssize_t written = spi_write_fifo(spi, tx_data + tx_idx, tx_target - tx_idx);
            if (written <= 0) {
                break;
            }
            tx_idx += (size_t)written;
        }
    }

    cr1 = ls_readl(spi->spi_cr1);
    cr1 |= LS_SPI_CR1_CSTART;
    ls_writel(spi->spi_cr1, cr1);

    /* 轮询收发 FIFO 直到完成或 EOT 置位 */
    while (tx_idx < tx_target || rx_idx < rx_target) {
        if (tx_idx < tx_target && (ls_readl(spi->spi_sr1) & LS_SPI_SR1_TXA) != 0U) {
            if (tx_data != NULL) {
                ssize_t written = spi_write_fifo(spi, tx_data + tx_idx, tx_target - tx_idx);
                if (written > 0) {
                    tx_idx += (size_t)written;
                }
            } else {
                ls_writeb(spi->spi_dr, 0);
                tx_idx++;
            }
        }

        if (rx_idx < rx_target && (ls_readl(spi->spi_sr1) & LS_SPI_SR1_RXA) != 0U) {
            ssize_t read_len = spi_read_fifo(spi, rx_data + rx_idx, rx_target - rx_idx);
            if (read_len > 0) {
                rx_idx += (size_t)read_len;
            }
        }

        if ((ls_readl(spi->spi_sr1) & LS_SPI_SR1_EOT) != 0U) {
            break;
        }
    }

    if (!spi_wait_transfer_complete(spi, 1000U)) {
        /* 超时时立即撤销事务并释放锁 */
        ls2k0300_spi_disable(spi);
        ls2k0300_gpio_level_set(&spi->cs_gpio, GPIO_HIGH);
        pthread_mutex_unlock(&spi->mtx_transfer);
        return -1;
    }

    while (rx_idx < rx_target && (ls_readl(spi->spi_sr1) & LS_SPI_SR1_RXA) != 0U) {
        ssize_t read_len = spi_read_fifo(spi, rx_data + rx_idx, rx_target - rx_idx);
        if (read_len <= 0) {
            break;
        }
        rx_idx += (size_t)read_len;
    }

    ls_writel(spi->spi_sr1, LS_SPI_SR1_EOT);

    cr1 = ls_readl(spi->spi_cr1);
    cr1 &= ~LS_SPI_CR1_CSTART;
    ls_writel(spi->spi_cr1, cr1);

    /* 事务结束: 关 SPI 并释放 CS */
    ls2k0300_spi_disable(spi);
    ls2k0300_gpio_level_set(&spi->cs_gpio, GPIO_HIGH);

    pthread_mutex_unlock(&spi->mtx_transfer);

    if (tx_idx != tx_target || rx_idx != rx_target) {
        return -1;
    }

    return 0;
}

/********************************************************************************
 * @brief   SPI 写数据.
 * @param   spi    : SPI 句柄指针.
 * @param   tx_buf : 发送缓冲区.
 * @param   len    : 发送长度（字节）.
 * @return  成功返回 0，失败返回 -1.
 * @example ls2k0300_spi_write(&spi, tx, tx_len);
 ********************************************************************************/
ssize_t ls2k0300_spi_write(ls2k0300_spi_t *spi, const void *tx_buf, size_t len)
{
    return ls2k0300_spi_transfer(spi, tx_buf, NULL, len);
}

/********************************************************************************
 * @brief   SPI 读数据.
 * @param   spi    : SPI 句柄指针.
 * @param   rx_buf : 接收缓冲区.
 * @param   len    : 接收长度（字节）.
 * @return  成功返回 0，失败返回 -1.
 * @example ls2k0300_spi_read(&spi, rx, rx_len);
 ********************************************************************************/
ssize_t ls2k0300_spi_read(ls2k0300_spi_t *spi, void *rx_buf, size_t len)
{
    return ls2k0300_spi_transfer(spi, NULL, rx_buf, len);
}

/********************************************************************************
 * @brief   获取 SPI 当前速率.
 * @param   spi : SPI 句柄指针.
 * @return  成功返回速率(Hz)，失败返回 0.
 * @example uint32_t hz = ls2k0300_spi_get_speed(&spi);
 ********************************************************************************/
uint32_t ls2k0300_spi_get_speed(ls2k0300_spi_t *spi)
{
    if (spi == NULL) {
        return 0U;
    }
    return spi->speed_hz;
}

/********************************************************************************
 * @brief   获取 SPI 当前模式.
 * @param   spi : SPI 句柄指针.
 * @return  成功返回模式值，失败返回 LS_SPI_MODE_INVALID.
 * @example ls_reg_spi_mode_t mode = ls2k0300_spi_get_mode(&spi);
 ********************************************************************************/
ls_reg_spi_mode_t ls2k0300_spi_get_mode(ls2k0300_spi_t *spi)
{
    if (spi == NULL) {
        return LS_SPI_MODE_INVALID;
    }
    return spi->spi_mode;
}

/********************************************************************************
 * @brief   获取 SPI 当前数据位宽.
 * @param   spi : SPI 句柄指针.
 * @return  成功返回位宽，失败返回 LS_SPI_BPW_INVALID.
 * @example ls_spi_bits_per_word_t bpw = ls2k0300_spi_get_bits_per_word(&spi);
 ********************************************************************************/
ls_spi_bits_per_word_t ls2k0300_spi_get_bits_per_word(ls2k0300_spi_t *spi)
{
    if (spi == NULL) {
        return LS_SPI_BPW_INVALID;
    }
    return spi->bpw;
}

/********************************************************************************
 * @brief   获取 SPI 当前数据顺序.
 * @param   spi : SPI 句柄指针.
 * @return  成功返回位序，失败返回 LS_SPI_DATA_ORDER_INVALID.
 * @example ls_spi_data_order_t ord = ls2k0300_spi_get_data_order(&spi);
 ********************************************************************************/
ls_spi_data_order_t ls2k0300_spi_get_data_order(ls2k0300_spi_t *spi)
{
    if (spi == NULL) {
        return LS_SPI_DATA_ORDER_INVALID;
    }
    return spi->ord;
}

/********************************************************************************
 * @brief   检查 SPI 状态寄存器错误位.
 * @param   spi : SPI 句柄指针.
 * @return  正常返回 1, 异常返回 0.
 * @example int ok = ls2k0300_spi_check_status(&spi);
 ********************************************************************************/
int ls2k0300_spi_check_status(ls2k0300_spi_t *spi)
{
    uint32_t sr1;
    uint8_t spsr;

    if (spi == NULL || spi->spi_sr1 == NULL) {
        return 0;
    }

    if (spi->port == LS_SPI1) {
        spsr = ls_readb(spi->spi_sr1);
        if ((spsr & LS_SPI01_SPSR_WCOL) != 0U) {
            return 0;
        }
        return 1;
    }

    sr1 = ls_readl(spi->spi_sr1);
    if ((sr1 & (LS_SPI_SR1_MODF | LS_SPI_SR1_OVR)) != 0U) {
        return 0;
    }

    return 1;
}
