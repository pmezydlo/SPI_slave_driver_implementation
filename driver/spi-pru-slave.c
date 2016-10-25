/*
 * SPI slave driver for PRU processor
 *
 * Copyright (C) 2016 Patryk Mężydło <mezydlo.p@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/pm_runtime.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/uaccess.h>

#define DRIVER_NAME "spi-pru-slave"

#include "spi-slave-core.h"

irq_handler_t pruspi_slave_irq(unsigned int irq, void *dev_id)
{
	return (irq_handler_t) IRQ_HANDLED;
}

const struct of_device_id pruspi_slave_of_match[] = {
	{
		.compatible = "ti,pru-spi-slave",
	},
	{ }
};
MODULE_DEVICE_TABLE(of, pruspi_slave_of_match);

static int pruspi_slave_probe(struct platform_device *pdev)
{
	pr_info("pru driver probe");
	return 0;
}

static int pruspi_slave_remove(struct platform_device *pdev)
{
	pr_info("pru driver remove");
	return 0;
}

static struct platform_driver pruspi_slave_driver = {
	.probe	= pruspi_slave_probe,
	.remove = pruspi_slave_remove,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = of_match_ptr(pruspi_slave_of_match),
	},
};

module_platform_driver(pruspi_slave_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Patryk Mezydlo, <mezydlo.p@gmail.com>");
MODULE_DESCRIPTION("SPI slave driver for PRU processor.");
MODULE_VERSION("1.0");
