#ifndef __LS2K0300_MAP_H
#define __LS2K0300_MAP_H

#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <pthread.h> // User requested retaining mutexes

#ifdef __cplusplus
extern "C" {
#endif

// 基础类型定义
typedef uint32_t ls_reg_base_t;
typedef volatile uint32_t* ls_reg32_addr_t;

// 寄存器读写宏
#define ls_readb(addr)          (*(volatile uint8_t*)(addr))
#define ls_writeb(addr, val)    (*(volatile uint8_t*)(addr) = (val))

#define ls_readl(addr)          (*(ls_reg32_addr_t)(addr))
#define ls_writel(addr, val)    (*(ls_reg32_addr_t)(addr) = (val))

#define BIT(x) (1U << (x))

// 地址偏移计算内联函数
static inline ls_reg32_addr_t ls_reg_addr_calc(ls_reg32_addr_t base, uint32_t offset) {
    return (ls_reg32_addr_t)((uint8_t*)base + offset);
}

// 内存映射接口
void* ls2k0300_mmap(uint32_t phys_addr, size_t size);
void ls2k0300_munmap(void* virt_addr, size_t size);

#ifdef __cplusplus
}
#endif

#endif // __LS2K0300_MAP_H
