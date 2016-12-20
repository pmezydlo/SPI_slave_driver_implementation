/*
 * SPI slave driver for debugging API
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

#define DRIVER_NAME "spi-slave-debug"

#include "spi-slave-core.h"

const struct of_device_id debspi_slave_of_match[] = {
	{
		.compatible = "spislave,spi-slave-debug",
	},
	{ }
};
MODULE_DEVICE_TABLE(of, debspi_slave_of_match);

static int debspi_slave_probe(struct platform_device *pdev)
{
	pr_info("debug driver probe");
	return 0;
}

static int debspi_slave_remove(struct platform_device *pdev)
{
	pr_info("debug driver remove");
	return 0;
}

static struct platform_driver debspi_slave_driver = {
	.probe	= debspi_slave_probe,
	.remove = debspi_slave_remove,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = of_match_ptr(debspi_slave_of_match),
	},
};

module_platform_driver(debspi_slave_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Patryk Mezydlo, <mezydlo.p@gmail.com>");
MODULE_DESCRIPTION("SPI slave driver for debugging API");
MODULE_VERSION("1.0");
