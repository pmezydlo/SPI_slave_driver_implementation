#!/bin/bash

echo Remove spi-omap2-mcspi driver
rmmod spi_omap2_mcspi

echo Install DTS for SPI1
echo SPI1_slave>/sys/devices/platform/bone_capemgr/slots
cat /sys/devices/platform/bone_capemgr/slots

echo Virtual file system for the dynamic debug
mount -o rw,remount -t debugfs none /sys/kernel/debug

echo Add driver to the dynamic debug
echo -n 'file ~/SPI_slave_driver_implementation/driver/spi-mcspi-slave.c +p'>/sys/kernel/debug/dynamic_debug/control

