ARCH := arm
COMPILER := arm-linux-gnueabihf-
COMPILER_V := gcc
PWD := $(shell pwd)
KERN_V= 4.4.8
BUILD_V= -ti-r22
SOURCE=linux-stable-rcn-ee

KDIR := $(PWD)/$(SOURCE)-$(KERN_V)$(BUILD_V)

obj-m+=driver/spi-mcspi-slave.o driver/spi-slave-dev.o

default:
	$(MAKE) -C $(KDIR) M=$(PWD) ARCH=$(ARCH) CROSS_COMPILE=$(COMPILER)  modules
	$(COMPILER)$(COMPILER_V) -o test_master_app/spidev_test test_master_app/spidev_test.c
	$(COMPILER)$(COMPILER_V) -o slave_app/slave_app slave_app/slave_app.c

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) ARCH=$(ARCH) clean
