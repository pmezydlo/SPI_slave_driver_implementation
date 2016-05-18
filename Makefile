ARCH=arm
COMPILER=arm-linux-gnueabihf-
KDIR:=$(DST_KERNEL)/
CFLAGS = -g3 -std=c99 -pedantic -Wall

obj-m+=driver/spi-mcspi-slave.o

PWD := $(shell pwd)

default:
	$(MAKE) -C $(KDIR) M=$(PWD) ${CFLAGS} ARCH=$(ARCH) CROSS_COMPILE=$(COMPILER) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) ${CFLAGS} ARCH=$(ARCH) clean
