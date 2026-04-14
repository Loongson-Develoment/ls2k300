#include "LS2K0300_SPI.h"
#include <unistd.h>
#include <stdio.h>

#define LS_SPI2_BASE_ADDR           0x1610C000
#define LS_SPI3_BASE_ADDR           0x1610E000

#define LS_SPI_CR1                  ( 0x00 )
#define LS_SPI_CR2                  ( 0x04 )
#define LS_SPI_CR3                  ( 0x08 )
#define LS_SPI_CR4                  ( 0x0C )
#define LS_SPI_IER                  ( 0x10 )
#define LS_SPI_SR1                  ( 0x14 )
#define LS_SPI_SR2                  ( 0x18 )
#define LS_SPI_CFG1                 ( 0x20 )
#define LS_SPI_CFG2                 ( 0x24 )
#define LS_SPI_CFG3                 ( 0x28 )
#define LS_SPI_DR                   ( 0x40 )

#define LS_SPI_CR1_AUTOSUS          ( 1 << 2 )
#define LS_SPI_CR1_CSTART           ( 1 << 1 )
#define LS_SPI_CR1_SPE              ( 1 << 0 )

#define LS_SPI_CR2_TXFTHLV_SHIFT    ( 8 )
#define LS_SPI_CR2_TXFTHLV          ( 0x3 << 8 )
#define LS_SPI_CR2_RXFTHLV_SHIFT    ( 0 )
#define LS_SPI_CR2_RXFTHLV          ( 0x3 << 0 )

#define LS_SPI_CFG1_DSIZE_SHIFT     ( 8 )
#define LS_SPI_CFG1_DSIZE           ( 0x1F << 8 )
#define LS_SPI_CFG1_LSBFRST         ( 1 << 7 )
#define LS_SPI_CFG1_CPHA            ( 1 << 1 )
#define LS_SPI_CFG1_CPOL            ( 1 << 0 )

#define LS_SPI_CFG2_BRINT_SHIFT     ( 8 )
#define LS_SPI_CFG2_BRINT           ( 0xFF << 8 )

#define LS_SPI_CFG3_DOE             ( 1 << 3 )
#define LS_SPI_CFG3_DIE             ( 1 << 2 )
#define LS_SPI_CFG3_DIOSWP          ( 1 << 1 )
#define LS_SPI_CFG3_MSTR            ( 1 << 0 )

#define LS_SPI_SR1_EOT             ( 1 << 15 )
#define LS_SPI_SR1_MODF            ( 1 << 11 )
#define LS_SPI_SR1_OVR             ( 1 << 8 )
#define LS_SPI_SR1_TXA             ( 1 << 1 )
#define LS_SPI_SR1_RXA             ( 1 << 0 )

#define LS_SPI_CLK_FRE              ( 160000000L )

static void spi_set_fifo_threshold(ls2k0300_spi_t* spi, ls_spi_fifo_threshould_t tx, ls_spi_fifo_threshould_t rx) {
    uint32_t val = ls_readl(spi->spi_cr2);
    val &= ~(LS_SPI_CR2_TXFTHLV | LS_SPI_CR2_RXFTHLV);
    val |= (tx << LS_SPI_CR2_TXFTHLV_SHIFT) | (rx << LS_SPI_CR2_RXFTHLV_SHIFT);
    ls_writel(spi->spi_cr2, val);
}

void ls2k0300_spi_enable(ls2k0300_spi_t* spi) {
    pthread_mutex_lock(&spi->mtx);
    ls_writel(spi->spi_cr1, ls_readl(spi->spi_cr1) | LS_SPI_CR1_SPE);
    pthread_mutex_unlock(&spi->mtx);
}

void ls2k0300_spi_disable(ls2k0300_spi_t* spi) {
    pthread_mutex_lock(&spi->mtx);
    uint32_t val = ls_readl(spi->spi_cr1);
    val &= ~LS_SPI_CR1_CSTART;
    ls_writel(spi->spi_cr1, val);
    val &= ~LS_SPI_CR1_SPE;
    ls_writel(spi->spi_cr1, val);
    pthread_mutex_unlock(&spi->mtx);
}

int ls2k0300_spi_init(ls2k0300_spi_t* spi, ls_spi_port_t port, uint32_t speed, ls_reg_spi_mode_t mode) {
    if (!spi) return -1;
    pthread_mutex_init(&spi->mtx, NULL);
    pthread_mutex_init(&spi->mtx_transfer, NULL);
    
    spi->port = port;
    spi->speed_hz = speed;
    spi->spi_mode = mode;
    spi->bpw = LS_SPI_BPW_8;
    spi->ord = LS_SPI_MSB_FIRST;
    spi->tx_thresh = LS_SPI_FIFO_THRESH_1;
    spi->rx_thresh = LS_SPI_FIFO_THRESH_1;
    
    ls_reg_base_t base_addr;
    if (port == LS_SPI2) {
        ls2k0300_gpio_mux_set(PIN_64, GPIO_MUX_MAIN);
        ls2k0300_gpio_mux_set(PIN_65, GPIO_MUX_MAIN);
        ls2k0300_gpio_mux_set(PIN_66, GPIO_MUX_MAIN);
        spi->cs_gpio = PIN_67;
        base_addr = LS_SPI2_BASE_ADDR;
    } else if (port == LS_SPI3) {
        ls2k0300_gpio_mux_set(PIN_82, GPIO_MUX_ALT1);
        ls2k0300_gpio_mux_set(PIN_83, GPIO_MUX_ALT1);
        ls2k0300_gpio_mux_set(PIN_84, GPIO_MUX_ALT1);
        spi->cs_gpio = PIN_85;
        base_addr = LS_SPI3_BASE_ADDR;
    } else {
        return -1;
    }
    ls2k0300_gpio_direction_set(spi->cs_gpio, GPIO_MODE_OUT);
    ls2k0300_gpio_level_set(spi->cs_gpio, GPIO_HIGH);

    spi->spi_base = (ls_reg32_addr_t)ls2k0300_mmap(base_addr, 0x1000);
    if (!spi->spi_base) return -1;

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

    ls2k0300_spi_disable(spi);
    ls_writel(spi->spi_sr1, 0xFFFFFFFF);
    ls_writel(spi->spi_ier, 0);

    uint32_t cfg3 = ls_readl(spi->spi_cfg3);
    cfg3 |= LS_SPI_CFG3_MSTR;
    cfg3 &= ~LS_SPI_CFG3_DIOSWP;
    ls_writel(spi->spi_cfg3, cfg3);

    uint32_t cr1 = ls_readl(spi->spi_cr1);
    cr1 &= ~LS_SPI_CR1_AUTOSUS;
    ls_writel(spi->spi_cr1, cr1);

    spi_set_fifo_threshold(spi, spi->tx_thresh, spi->rx_thresh);

    uint32_t cfg1 = ls_readl(spi->spi_cfg1);
    cfg1 &= ~(LS_SPI_CFG1_CPOL | LS_SPI_CFG1_CPHA);
    cfg1 |= mode;
    cfg1 &= ~LS_SPI_CFG1_DSIZE;
    cfg1 |= ((spi->bpw - 1) << LS_SPI_CFG1_DSIZE_SHIFT);
    cfg1 &= ~LS_SPI_CFG1_LSBFRST;
    ls_writel(spi->spi_cfg1, cfg1);

    uint32_t div = LS_SPI_CLK_FRE / speed;
    if (div < 2) div = 2;
    if (div > 255) div = 255;
    uint32_t cfg2 = ls_readl(spi->spi_cfg2);
    cfg2 &= ~LS_SPI_CFG2_BRINT;
    cfg2 |= (div << LS_SPI_CFG2_BRINT_SHIFT);
    ls_writel(spi->spi_cfg2, cfg2);
    
    spi->is_initialized = true;
    return 0;
}

void ls2k0300_spi_deinit(ls2k0300_spi_t* spi) {
    if (!spi) return;
    if (spi->is_initialized) ls2k0300_spi_disable(spi);
    if (spi->spi_base) ls2k0300_munmap((void*)spi->spi_base, 0x1000);
    pthread_mutex_destroy(&spi->mtx);
    pthread_mutex_destroy(&spi->mtx_transfer);
    spi->is_initialized = false;
}

ssize_t ls2k0300_spi_transfer(ls2k0300_spi_t* spi, const void* tx_buf, void* rx_buf, size_t len) {
    if (!spi || !spi->is_initialized || len == 0) return -1;
    
    pthread_mutex_lock(&spi->mtx_transfer);
    ls2k0300_gpio_level_set(spi->cs_gpio, GPIO_LOW);

    ls_writel(spi->spi_sr1, 0xFFFFFFFF);
    ls_writel(spi->spi_sr2, 0xFFFFFFFF);
    
    uint32_t cr1 = ls_readl(spi->spi_cr1);
    cr1 &= ~LS_SPI_CR1_CSTART;
    ls_writel(spi->spi_cr1, cr1);
    cr1 &= ~LS_SPI_CR1_SPE;
    ls_writel(spi->spi_cr1, cr1);

    uint32_t cfg3 = ls_readl(spi->spi_cfg3);
    if (tx_buf && rx_buf) {
        cfg3 |= (LS_SPI_CFG3_DOE | LS_SPI_CFG3_DIE);
    } else if (tx_buf) {
        cfg3 |= LS_SPI_CFG3_DOE;
        cfg3 &= ~LS_SPI_CFG3_DIE;
    } else if (rx_buf) {
        cfg3 |= LS_SPI_CFG3_DIE;
        cfg3 &= ~LS_SPI_CFG3_DOE;
    }
    cfg3 &= ~LS_SPI_CFG3_DIOSWP;
    ls_writel(spi->spi_cfg3, cfg3);

    uint32_t tsize = (len * 8 + spi->bpw - 1) / spi->bpw;
    ls_writel(spi->spi_cr3, tsize - 1);
    ls_writel(spi->spi_cr4, 0);

    cr1 = ls_readl(spi->spi_cr1);
    cr1 |= LS_SPI_CR1_SPE;
    ls_writel(spi->spi_cr1, cr1);

    const uint8_t* tx_data = (const uint8_t*)tx_buf;
    uint8_t* rx_data = (uint8_t*)rx_buf;
    size_t tx_idx = 0;

    if (tx_buf) {
        while ((ls_readl(spi->spi_sr1) & LS_SPI_SR1_TXA) && (tx_idx < len)) {
            ls_writeb(spi->spi_dr, tx_data[tx_idx++]);
        }
    }

    cr1 = ls_readl(spi->spi_cr1);
    cr1 |= LS_SPI_CR1_CSTART;
    ls_writel(spi->spi_cr1, cr1);

    size_t rx_idx = 0;
    while (tx_idx < len || rx_idx < len) {
        if (tx_idx < len && (ls_readl(spi->spi_sr1) & LS_SPI_SR1_TXA)) {
            ls_writeb(spi->spi_dr, tx_data ? tx_data[tx_idx] : 0);
            tx_idx++;
        }
        if (rx_idx < len && (ls_readl(spi->spi_sr1) & LS_SPI_SR1_RXA)) {
            uint8_t d = (uint8_t)ls_readb(spi->spi_dr);
            if (rx_data) rx_data[rx_idx] = d;
            rx_idx++;
        }
        if (ls_readl(spi->spi_sr1) & LS_SPI_SR1_EOT) break;
    }

    int timeout = 100000;
    while (!(ls_readl(spi->spi_sr1) & LS_SPI_SR1_EOT) && timeout > 0) {
        usleep(10);
        timeout--;
    }

    while ((ls_readl(spi->spi_sr1) & LS_SPI_SR1_RXA) && (rx_idx < len)) {
        uint8_t d = (uint8_t)ls_readb(spi->spi_dr);
        if (rx_data) rx_data[rx_idx] = d;
        rx_idx++;
    }

    ls_writel(spi->spi_sr1, LS_SPI_SR1_EOT);
    cr1 = ls_readl(spi->spi_cr1);
    cr1 &= ~LS_SPI_CR1_CSTART;
    cr1 &= ~LS_SPI_CR1_SPE;
    ls_writel(spi->spi_cr1, cr1);

    ls2k0300_gpio_level_set(spi->cs_gpio, GPIO_HIGH);
    pthread_mutex_unlock(&spi->mtx_transfer);

    return (tx_idx == len && rx_idx == len) ? 0 : -1;
}
