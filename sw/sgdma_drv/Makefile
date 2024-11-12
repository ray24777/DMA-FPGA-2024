TARGET_MODULE:=anlogic_pci

$(TARGET_MODULE)-objs := anlogic_pci_lib.o anlogic_pci_cdev.o cdev_ctrl.o cdev_events.o cdev_sgdma.o anlogic_pci_drv.o anlogic_thread.o anlogic_ring.o anlogic_dbg.o anlogic_pci_dbg.o anlogic_pci_test.o anlogic_pci_c2h_test.o anlogic_pci_h2c_test.o

ccflags-y += -DAL_BD_RING -Wno-unused

ifneq ($(KERNELRELEASE),)
obj-m := $(TARGET_MODULE).o
else
PWD := $(shell pwd)
KVER := $(shell uname -r)
KDIR := /lib/modules/$(KVER)/build
all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules
clean:
	rm -rf *.o *.mod.c *.ko *.symvers *.order *.cmd *.ur-safe *.maker .tmp_versions .*.o.cmd .*.ko.cmd .cache.mk
endif

