ARCH=arm
COMPILER=arm-linux-gnueabihf-
KDIR := $(DST_KERNEL)/
UNAME_M := $(shell uname -m)
PWD := $(shell pwd)
DST_KERNEL := $(PWD)/linux-4.4.8-ti-rt-r22

ifeq ($(UNAME_M),armv7l)
        KDIR:="/lib/modules/$(shell uname -r)/build/"
else
        KDIR:=$(DST_KERNEL)/
        export CROSS_COMPILE=$(COMPILER)
endif

obj-m+=driver/spi-mcspi-slave.o driver/spi-slave-dev.o

default:
	$(MAKE) -C $(KDIR) M=$(PWD) ARCH=$(ARCH) modules
	gcc -o test_master_app/spidev_test test_master_app/spidev_test.c
	gcc -o slave_app/slave_app slave_app/slave_app.c

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) ARCH=$(ARCH) clean
