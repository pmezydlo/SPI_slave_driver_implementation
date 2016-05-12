#!/bin/bash

echo SPI_slave>/sys/devices/bone_capemgr.9/slots
cat /sys/devices/bone_capemgr.9/slots
echo Uninstall omap driver:

ls /sys/bus/platform/drivers/omap2_mcspi
echo 481a0000.spi>/sys/bus/platform/drivers/omap2_mcspi/unbind

ls /sys/bus/platform/drivers/omap2_mcspi

