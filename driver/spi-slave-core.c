/*
 * A device driver for the SPI slave mode bus interface.
 *
 * Copyright (C) 2016 Patryk Mężydło <mezydlo.p@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/of_device.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/pm_domain.h>
#include <linux/slab.h>

#include "spi-slave-core.h"
#define DRIVER_NAME "spislavecore"

/*============================================================================*/
struct spislave_message *spislave_msg_alloc(struct spislave *slave)
{
	struct spislave_message *msg;

	pr_info("%s: function: msg alloc\n", DRIVER_NAME);

	msg = slave->msg;

	msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	if (!msg)
		return NULL;

	mutex_init(&msg->msg_lock);
	spin_lock_init(&msg->wait_lock);
	init_waitqueue_head(&msg->wait);
	slave->msg = msg;

	return msg;
}
EXPORT_SYMBOL_GPL(spislave_msg_alloc);

void spislave_msg_remove(struct spislave *slave)
{
	struct spislave_message *msg;

	pr_info("%s: function: msg remove\n", DRIVER_NAME);

	if (slave->clear_msg != NULL)
		slave->clear_msg(slave);

	msg = slave->msg;
	kfree(msg);
}
EXPORT_SYMBOL_GPL(spislave_msg_remove);

int spislave_transfer_msg(struct spislave *slave)
{
	int ret;

	pr_info("%s: function: transfer msg\n", DRIVER_NAME);

	if (slave->transfer_msg != NULL)
		ret = slave->transfer_msg(slave);

	return ret;
}
EXPORT_SYMBOL_GPL(spislave_transfer_msg);

int spislave_clear_transfer(struct spislave *slave)
{
	pr_info("%s: function: clear transfer\n", DRIVER_NAME);

	if (slave->clear_msg != NULL)
		slave->clear_msg(slave);

	return 0;
}
EXPORT_SYMBOL_GPL(spislave_clear_transfer);

/*============================================================================*/
static int spislave_drv_probe(struct device *dev)
{
	int ret = 0;
	const struct spislave_driver *sdrv;
	struct spislave_device *sdev;

	pr_info("%s: probe\n", DRIVER_NAME);

	sdrv = to_spislave_drv(dev->driver);
	sdev = to_spislave_dev(dev);

	ret = dev_pm_domain_attach(dev, true);
	if (ret != -EPROBE_DEFER) {
		ret = sdrv->probe(sdev);
		if (ret)
			dev_pm_domain_detach(dev, true);
	}

	return ret;
}

static int spislave_drv_remove(struct device *dev)
{
	int ret = 0;
	const struct spislave_driver *sdrv;
	struct spislave_device *sdev;

	pr_info("%s: remove\n", DRIVER_NAME);

	sdrv = to_spislave_drv(dev->driver);

	if (!sdrv)
		return -ENODEV;

	sdev = to_spislave_dev(dev);

	if (!sdev)
		return -ENODEV;

	ret = sdrv->remove(sdev);
	dev_pm_domain_detach(dev, true);

	return ret;
}

int spislave_register_driver(struct spislave_driver *sdrv)
{
	pr_info("%s: register driver\n", DRIVER_NAME);

	sdrv->driver.owner = THIS_MODULE;
	sdrv->driver.bus = &spislave_bus_type;

	sdrv->driver.probe = spislave_drv_probe;
	sdrv->driver.remove = spislave_drv_remove;

	return driver_register(&sdrv->driver);
}
EXPORT_SYMBOL_GPL(spislave_register_driver);

void spislave_unregister_driver(struct spislave_driver *sdrv)
{
	if (sdrv)
		driver_unregister(&sdrv->driver);
}
EXPORT_SYMBOL_GPL(spislave_unregister_driver);

/*============================================================================*/
static int __unregister(struct device *dev, void *null)
{
	spislave_unregister_device(to_spislave_dev(dev));
	return 0;
}

void spislave_unregister_slave(struct spislave *slave)
{
	int				dummy;

	pr_info("%s: function: unregister slave\n", DRIVER_NAME);

	dummy = device_for_each_child(&slave->dev, NULL, __unregister);
	device_unregister(&slave->dev);
}
EXPORT_SYMBOL_GPL(spislave_unregister_slave);

static void devm_spislave_unregister_slave(struct device *dev, void *res)
{
	spislave_unregister_slave(*(struct spislave **)res);
}
EXPORT_SYMBOL_GPL(devm_spislave_unregister_slave);

static void spislave_dev_release(struct device *dev)
{
	struct spislave_device *slave_dev = to_spislave_dev(dev);
	struct spislave *slave;

	pr_info("%s: function: adev release\n", DRIVER_NAME);

	slave = slave_dev->slave;

	put_device(&slave->dev);
	kfree(slave_dev);
}

static void spislave_release(struct device *dev)
{
	struct spislave *slave;

	pr_info("%s: function: release\n", DRIVER_NAME);

	slave = container_of(dev, struct spislave, dev);
	kfree(slave);
}

static struct class spislave_class = {
	.name = "spi_slave",
	.owner = THIS_MODULE,
	.dev_release = spislave_release,
};

struct spislave *spislave_alloc_slave(struct device *dev, unsigned int size)
{
	struct spislave *slave;

	pr_info("%s: function: alloc slave\n", DRIVER_NAME);

	slave = kzalloc(size + sizeof(*slave), GFP_KERNEL);
	if (!slave)
		return NULL;

	device_initialize(&slave->dev);
	slave->dev.class = &spislave_class;
	slave->dev.parent = get_device(dev);
	dev_set_drvdata(&slave->dev, (void *)&slave[1]);

	return slave;
}
EXPORT_SYMBOL_GPL(spislave_alloc_slave);

static struct spislave_device *spislave_register_device(struct spislave *slave,
							struct device_node *nc)
{
	struct spislave_device *slave_dev;
	int ret;

	pr_info("%s: function: register device\n", DRIVER_NAME);

	dev_dbg(&slave->dev, "chid node is found: %s\n", nc->full_name);

	slave_dev = kzalloc(sizeof(*slave_dev), GFP_KERNEL);

	if (!slave_dev) {
		put_device(&slave->dev);
		return NULL;
	}

	slave_dev->slave = slave;
	slave_dev->dev.parent = &slave->dev;
	slave_dev->dev.bus = &spislave_bus_type;
	slave_dev->dev.release = spislave_dev_release;

	ret = of_modalias_node(nc, slave_dev->modalias,
			      sizeof(slave_dev->modalias));
	if (ret < 0) {
		dev_dbg(&slave->dev, "cannot find modalias\n");
		return NULL;
	}

	of_node_get(nc);
	slave_dev->dev.of_node = nc;
	dev_set_name(&slave_dev->dev, "%s", nc->name);
	ret = device_register(&slave_dev->dev);
	if (ret) {
		dev_dbg(&slave->dev, "register child device erroc\n");
		return NULL;
	}

	return slave_dev;
}

int spislave_register_devices(struct spislave *slave)
{
	struct spislave_device *slave_dev;
	struct device_node *nc;

	pr_info("%s: function: register devices\n", DRIVER_NAME);

	if (!slave->dev.of_node)
		return -ENODEV;

	for_each_available_child_of_node(slave->dev.of_node, nc) {
		if (of_node_test_and_set_flag(nc, OF_POPULATED))
			continue;

		slave_dev = spislave_register_device(slave, nc);
		if (IS_ERR(slave_dev))
			dev_dbg(&slave->dev,
				"filed to create spislave device\n");
	}
	return 0;
}


int spislave_register_slave(struct spislave *slave, struct device *dev)
{
	int ret = 0;
	struct device_node *node;

	pr_info("%s: function: register slave\n", DRIVER_NAME);

	if (!dev)
		return -ENODEV;

	node = dev->of_node;
	slave->dev.of_node = dev ? node : NULL;
	dev_set_name(&slave->dev, "%s", node->name);

	ret = device_add(&slave->dev);
	if (ret < 0) {
		dev_dbg(&slave->dev, "device add error\n");
		return -ENODEV;
	}

	ret = spislave_register_devices(slave);
	if (ret < 0)
		dev_dbg(&slave->dev, " child device add erroc\n");

	return ret;
}

int devm_spislave_register_slave(struct device *dev,
				 struct spislave *slave)
{
	int ret = 0;
	struct spislave **ptr;

	pr_info("%s: function: devm register slave\n", DRIVER_NAME);

	ptr = devres_alloc(devm_spislave_unregister_slave, sizeof(*ptr),
			   GFP_KERNEL);
	if (!ptr) {
		dev_dbg(&slave->dev, "devers alloc error\n");
		return -ENOMEM;
	}

	ret = spislave_register_slave(slave, dev);
	if (!ret) {
		*ptr = slave;
		devres_add(dev, ptr);
	} else
		devres_free(ptr);

	return ret;
}
EXPORT_SYMBOL_GPL(devm_spislave_register_slave);

void spislave_unregister_device(struct spislave_device *sdev)
{
	pr_info("%s: function: function: unregister device\n", DRIVER_NAME);

	if (!sdev)
		return;

	if (sdev->dev.of_node)
		of_node_clear_flag(sdev->dev.of_node, OF_POPULATED);

	device_unregister(&sdev->dev);
}
EXPORT_SYMBOL_GPL(spislave_unregister_device);
/*============================================================================*/

static const struct spislave_device_id *spislave_match_id(const struct
		    spislave_device_id * id, const struct spislave_device *sdev)
{
	while (id->name[0]) {
		if (!strcmp(sdev->modalias, id->name))
			return id;
		id++;
	}
	return NULL;
}

static int spislave_device_match(struct device *dev,
				struct device_driver *drv)
{
	const struct spislave_driver *sdrv;
	const struct spislave_device *sdev;

	pr_info("%s: function: device match\n", DRIVER_NAME);

	sdrv = to_spislave_drv(drv);
	sdev = to_spislave_dev(dev);

	if (of_driver_match_device(dev, drv))
		return 1;

	if (sdrv->id_table)
		return !!spislave_match_id(sdrv->id_table, sdev);

	return strcmp(sdev->modalias, sdrv->driver.name) == 0;
}

struct bus_type spislave_bus_type = {
	.name = "spislave",
	.match = spislave_device_match,
};

static int __init spislave_init(void)
{
	int ret = 0;

	pr_info("%s: function: init\n", DRIVER_NAME);

	ret = bus_register(&spislave_bus_type);
	if (ret < 0)
		return ret;

	ret = class_register(&spislave_class);
	if (ret < 0)
		bus_unregister(&spislave_bus_type);

	return ret;
}

static void __exit spislave_exit(void)
{
	pr_info("%s: function: exit\n", DRIVER_NAME);

	bus_unregister(&spislave_bus_type);
}

module_init(spislave_init);
module_exit(spislave_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Patryk Mezydlo, <mezydlo.p@gmail.com>");
MODULE_DESCRIPTION("bus driver");
MODULE_VERSION("1.0");
