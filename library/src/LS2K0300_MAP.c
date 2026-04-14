#include "LS2K0300_MAP.h"

#define MAP_PAGE_SIZE 0x10000

void* ls2k0300_mmap(uint32_t phys_addr, size_t size) {
    int mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (mem_fd < 0) {
        printf("open /dev/mem error\n");
        return NULL;
    }

    uint32_t aligned_addr = phys_addr & ~(MAP_PAGE_SIZE - 1);
    uint32_t offset = phys_addr & (MAP_PAGE_SIZE - 1);
    size_t aligned_size = ((size + offset + MAP_PAGE_SIZE - 1) / MAP_PAGE_SIZE) * MAP_PAGE_SIZE;

    void *virt_addr = mmap(NULL, aligned_size, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, aligned_addr);
    close(mem_fd);

    if (virt_addr == MAP_FAILED) {
        printf("mmap error\n");
        return NULL;
    }

    return (void*)((uint8_t*)virt_addr + offset);
}

void ls2k0300_munmap(void* virt_addr, size_t size) {
    if (virt_addr == NULL || virt_addr == MAP_FAILED) {
        return;
    }

    uintptr_t virt = (uintptr_t)virt_addr;
    uintptr_t aligned_addr = virt & ~(MAP_PAGE_SIZE - 1);
    uintptr_t offset = virt & (MAP_PAGE_SIZE - 1);
    size_t aligned_size = ((size + offset + MAP_PAGE_SIZE - 1) / MAP_PAGE_SIZE) * MAP_PAGE_SIZE;

    if (munmap((void*)aligned_addr, aligned_size) == -1) {
        printf("munmap failed\n");
    }
}
