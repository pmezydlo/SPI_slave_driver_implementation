ARCH=arm
COMPILER=arm-linux-gnueabihf-
KDIR:=$(DST_KERNEL)/

obj-m+=driver/spi-mcspi-slave.o

PWD := $(shell pwd)

default:
	$(MAKE) -C $(KDIR) M=$(PWD) ARCH=$(ARCH) CROSS_COMPILE=$(COMPILER) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) ARCH=$(ARCH) clean
