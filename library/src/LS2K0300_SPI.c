#include "LS2K0300_SPI.h"
#include "LS2K0300_CLOCK.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>

/********************************************************************************
 * @brief   SPI2/3 基地址定义.
 ********************************************************************************/
#define LS_SPI2_BASE_ADDR           (0x1610C000U)
#define LS_SPI3_BASE_ADDR           (0x1610E000U)

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
    if (spi == NULL || spi->spi_cr1 == NULL) {
        return;
    }

    pthread_mutex_lock(&spi->mtx);
    /* SPE 位置位后 SPI 外设进入工作态 */
    ls_writel(spi->spi_cr1, ls_readl(spi->spi_cr1) | LS_SPI_CR1_SPE);
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

    if (spi == NULL || spi->spi_cr1 == NULL) {
        return;
    }

    pthread_mutex_lock(&spi->mtx);

    /* 先清 CSTART 停止当前事务，再清 SPE 关闭外设 */
    cr1 = ls_readl(spi->spi_cr1);
    cr1 &= ~LS_SPI_CR1_CSTART;
    ls_writel(spi->spi_cr1, cr1);

    cr1 = ls_readl(spi->spi_cr1);
    cr1 &= ~LS_SPI_CR1_SPE;
    ls_writel(spi->spi_cr1, cr1);

    pthread_mutex_unlock(&spi->mtx);
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
    if (port == LS_SPI2) {
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
    spi->spi_base = (ls_reg32_addr_t)ls2k0300_mmap(base_addr, 0x1000);
    if (spi->spi_base == NULL) {
        goto err_gpio;
    }

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
        ls2k0300_munmap((void *)spi->spi_base, 0x1000);
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

    if (spi == NULL || spi->spi_sr1 == NULL) {
        return 0;
    }

    sr1 = ls_readl(spi->spi_sr1);
    if ((sr1 & (LS_SPI_SR1_MODF | LS_SPI_SR1_OVR)) != 0U) {
        return 0;
    }

    return 1;
}
