# SPDX-License-Identifier: GPL-2.0
#
# Anlogic PH1P100 PCIe EP 驱动（RK3588 主机侧）的 Kbuild 文件。
# 内核树外编译，架构与交叉工具链由调用方给出：
#   make -C <内核树> M=$PWD ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- modules
ifneq ($(KERNELRELEASE),)

obj-m := anlogic_pci.o

anlogic_pci-y := src/anlogic_pci_main.o \
                 src/anlogic_pci_hw.o \
                 src/anlogic_pci_frame.o \
                 src/anlogic_pci_chrdev.o \
                 src/anlogic_pci_ctrl.o \
                 src/anlogic_pci_link.o \
                 src/anlogic_pci_rka1.o

ccflags-y += -I$(src)/include

else

KDIR ?= /lib/modules/$(shell uname -r)/build
PWD  := $(shell pwd)

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

.PHONY: all clean

endif
