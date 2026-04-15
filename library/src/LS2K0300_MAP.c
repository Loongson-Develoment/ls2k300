#include "LS2K0300_MAP.h"

#define LS2K0300_MAP_PAGE_SIZE  (0x10000U)

/********************************************************************************
 * @brief   物理地址映射.
 * @param   phys_addr : 物理地址.
 * @param   size      : 映射长度.
 * @return  成功返回虚拟地址, 失败返回 NULL.
 * @example void *base = ls2k0300_mmap(0x16104000U, 0x1000);
 ********************************************************************************/
void *ls2k0300_mmap(uint32_t phys_addr, size_t size)
{
    int mem_fd;
    uint32_t aligned_addr;
    uint32_t offset;
    size_t aligned_size;
    void *virt_addr;

    mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (mem_fd < 0) {
        printf("open /dev/mem error\n");
        return NULL;
    }

    /* 按页对齐映射起始地址与长度，兼容任意寄存器偏移 */
    aligned_addr = phys_addr & ~(LS2K0300_MAP_PAGE_SIZE - 1U);
    offset = phys_addr & (LS2K0300_MAP_PAGE_SIZE - 1U);
    aligned_size = ((size + offset + LS2K0300_MAP_PAGE_SIZE - 1U) / LS2K0300_MAP_PAGE_SIZE) * LS2K0300_MAP_PAGE_SIZE;

    virt_addr = mmap(NULL, aligned_size, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, aligned_addr);
    close(mem_fd);

    if (virt_addr == MAP_FAILED) {
        printf("mmap error\n");
        return NULL;
    }

    /* 返回带 offset 的实际寄存器地址 */
    return (void *)((uint8_t *)virt_addr + offset);
}

/********************************************************************************
 * @brief   解除物理地址映射.
 * @param   virt_addr : 虚拟地址.
 * @param   size      : 映射长度.
 * @return  none.
 * @example ls2k0300_munmap(base, 0x1000);
 ********************************************************************************/
void ls2k0300_munmap(void *virt_addr, size_t size)
{
    uintptr_t virt;
    uintptr_t aligned_addr;
    uintptr_t offset;
    size_t aligned_size;

    if (virt_addr == NULL || virt_addr == MAP_FAILED) {
        return;
    }

    virt = (uintptr_t)virt_addr;
    /* 反推原始页对齐地址与映射长度，保证 munmap 参数匹配 mmap */
    aligned_addr = virt & ~(uintptr_t)(LS2K0300_MAP_PAGE_SIZE - 1U);
    offset = virt & (uintptr_t)(LS2K0300_MAP_PAGE_SIZE - 1U);
    aligned_size = ((size + offset + LS2K0300_MAP_PAGE_SIZE - 1U) / LS2K0300_MAP_PAGE_SIZE) * LS2K0300_MAP_PAGE_SIZE;

    if (munmap((void *)aligned_addr, aligned_size) == -1) {
        printf("munmap failed\n");
    }
}
