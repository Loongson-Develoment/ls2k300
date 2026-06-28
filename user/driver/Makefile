KDIR ?= /home/ch2/2k0300/linux-4.19
ARCH ?= loongarch
CROSS_COMPILE ?= loongarch64-linux-gnu-

all:
	$(MAKE) -C build KDIR=$(KDIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE)

clean:
	$(MAKE) -C build KDIR=$(KDIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) clean
