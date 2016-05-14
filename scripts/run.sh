#!/bin/bash


echo Install driver
insmod ~/SPI_slave_driver_implementation/driver/spi-mcspi-slave.ko

echo Dynamic debug message
grep -i spi_mcspi_slave /sys/kernel/debug/dynamic_debug/control

echo Remove driver
rmmod spi_mcspi_slave
