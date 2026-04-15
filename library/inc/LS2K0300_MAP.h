#ifndef __LS2K0300_MAP_H
#define __LS2K0300_MAP_H

#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>

#ifdef __cplusplus
extern "C" {
#endif

/********************************************************************************
 * @brief   基础寄存器类型定义.
 ********************************************************************************/
typedef uint32_t ls_reg_base_t;
typedef volatile uint32_t *ls_reg32_addr_t;

/********************************************************************************
 * @brief   寄存器读写宏.
 ********************************************************************************/
#define ls_readb(addr)          (*(volatile uint8_t *)(addr))
#define ls_writeb(addr, val)    (*(volatile uint8_t *)(addr) = (val))

#define ls_readl(addr)          (*(ls_reg32_addr_t)(addr))
#define ls_writel(addr, val)    (*(ls_reg32_addr_t)(addr) = (val))

#define BIT(x)                  (1U << (x))

/********************************************************************************
 * @brief   计算寄存器偏移地址.
 * @param   base   : 基地址.
 * @param   offset : 偏移.
 * @return  计算后的寄存器地址.
 * @example ls_reg32_addr_t reg = ls_reg_addr_calc(base, 0x10);
 ********************************************************************************/
static inline ls_reg32_addr_t ls_reg_addr_calc(ls_reg32_addr_t base, uint32_t offset)
{
    return (ls_reg32_addr_t)((uint8_t *)base + offset);
}

/********************************************************************************
 * @brief   物理地址映射.
 * @param   phys_addr : 物理地址.
 * @param   size      : 映射长度.
 * @return  成功返回虚拟地址, 失败返回 NULL.
 * @example void *addr = ls2k0300_mmap(0x16104000U, 0x1000);
 ********************************************************************************/
void *ls2k0300_mmap(uint32_t phys_addr, size_t size);

/********************************************************************************
 * @brief   解除物理地址映射.
 * @param   virt_addr : 虚拟地址.
 * @param   size      : 映射长度.
 * @return  none.
 * @example ls2k0300_munmap(addr, 0x1000);
 ********************************************************************************/
void ls2k0300_munmap(void *virt_addr, size_t size);

#ifdef __cplusplus
}
#endif

#endif
