TARGET_MODULE:=anlogic_pci

$(TARGET_MODULE)-objs := anlogic_pci_lib.o anlogic_pci_cdev.o cdev_ctrl.o cdev_events.o anlogic_pci_drv.o 

ifneq ($(KERNELRELEASE),)
obj-m := $(TARGET_MODULE).o
else
PWD := $(shell pwd)
KVER := $(shell uname -r)
KDIR := /lib/modules/$(KVER)/build
all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules
clean:
	rm -rf *.o *.mod.c *.ko *.symvers *.order *.cmd *.maker .tmp_versions
endif

