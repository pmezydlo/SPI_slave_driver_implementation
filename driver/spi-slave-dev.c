/*
 * Simple userspace interface to SPI devices in slave mode.
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
#include <linux/idr.h>
#include <linux/of_device.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/uaccess.h>
#include <linux/poll.h>

#include "spi-slave-dev.h"
#include "spi-slave-core.h"

#define DRIVER_NAME		"spislavedev"
#define SPISLAVE_MAJOR		156

static LIST_HEAD(spislave_dev_list);
static struct class *spislave_class;
static DEFINE_SPINLOCK(spislave_lock);
static DEFINE_IDR(spislave_idr);
static DEFINE_SPINLOCK(spislave_idr_lock);

struct spislave_data {
	dev_t devt;
	struct	list_head device_entry;
	unsigned int users;
	struct spi_slave *slave;
};

static ssize_t spislave_read(struct file *filp, char __user *buf, size_t count,
			     loff_t *f_pos)
{
	struct spi_slave *slave;
	int error_count = 0;
	struct spislave_data *data;

	data = filp->private_data;
	slave = data->slave;

	error_count = copy_to_user(buf, slave->rx, slave->rx_offset);

	if (error_count != 0)
		return -EFAULT;


	pr_info("%s: read end count:%d rx_offset:%d\n", DRIVER_NAME,
		error_count, slave->rx_offset);

	slave->rx_offset = 0;
	memset(slave->rx, 0, slave->buf_depth);

	return 0;
}

static ssize_t spislave_write(struct file *filp, const char __user *buf,
			      size_t count, loff_t *f_pos)
{
	ssize_t ret = 0;
	struct spi_slave *slave;
	unsigned long missing;
	struct spislave_data *data;

	data = filp->private_data;
	slave = data->slave;

	memset(slave->tx, 0, slave->buf_depth);

	missing = copy_from_user(slave->tx, buf, count);

	if (missing == 0)
		ret = count;
	else
		return -EFAULT;

	slave->tx_offset = 0;
	slave->enable(slave);
	slave->transfer(slave);

	return ret;
}

static int spislave_release(struct inode *inode, struct file *filp)
{
	int ret = 0;
	struct spi_slave *slave;
	struct spislave_data *data;

	spin_lock(&spislave_lock);

	data = filp->private_data;
	slave = data->slave;

	data->users--;

	if (!data->users)

	filp->private_data = NULL;
	slave->clr_transfer(slave);

	spin_unlock(&spislave_lock);

	return ret;
}

static int spislave_open(struct inode *inode, struct file *filp)
{
	int ret = -ENXIO;
	struct spislave_data *data;
	struct spi_slave *slave;

	list_for_each_entry(data, &spislave_dev_list, device_entry) {
		if (data->devt == inode->i_rdev) {
			ret = 0;
			break;
		}
	}

	data->users++;
	filp->private_data = data;
	nonseekable_open(inode, filp);
	slave = data->slave;
	init_waitqueue_head(&slave->wait);

	spin_unlock(&spislave_lock);

	return ret;
}

static long spislave_ioctl(struct file *filp, unsigned int cmd,
			   unsigned long arg)
{
	int ret = 0;
	int err = 0;
	struct spi_slave *slave;
	struct spislave_data *data;

	spin_lock(&spislave_lock);

	data = filp->private_data;
	slave = data->slave;

	if (_IOC_TYPE(cmd) != SPISLAVE_IOC_MAGIC)
		return -ENOTTY;

	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,
				 (void __user *)arg, _IOC_SIZE(cmd));

	if (err == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_WRITE,
				 (void __user *)arg, _IOC_SIZE(cmd));

	if (err)
		return -EFAULT;

	switch (cmd) {
	case SPISLAVE_RD_TX_OFFSET:
		ret = __put_user(slave->tx_offset, (__u32 __user *)arg);
		break;

	case SPISLAVE_RD_RX_OFFSET:
		ret = __put_user(slave->rx_offset, (__u32 __user *)arg);
		break;

	case SPISLAVE_RD_BITS_PER_WORD:
		ret = __put_user(slave->bits_per_word, (__u32 __user *)arg);
		break;

	case SPISLAVE_RD_BYTES_PER_LOAD:
		ret = __put_user(slave->bytes_per_load, (__u32 __user *)arg);
		break;

	case SPISLAVE_RD_MODE:
		ret = __put_user(slave->mode, (__u32 __user *)arg);
		break;

	case SPISLAVE_RD_BUF_DEPTH:
		ret = __put_user(slave->buf_depth, (__u32 __user *)arg);
		break;

	case SPISLAVE_ENABLED:
		slave->enable(slave);
		break;

	case SPISLAVE_DISABLED:
		slave->disable(slave);
		break;

	case SPISLAVE_SET_TRANSFER:
		slave->set_transfer(slave);
		break;

	case SPISLAVE_CLR_TRANSFER:
		slave->clr_transfer(slave);
		break;

	case SPISLAVE_WR_BITS_PER_WORD:
		ret = __get_user(slave->bits_per_word, (__u32 __user *)arg);
		break;

	case SPISLAVE_WR_MODE:
		ret = __get_user(slave->mode, (__u32 __user *)arg);
		break;

	case SPISLAVE_WR_BUF_DEPTH:
		ret = __get_user(slave->buf_depth, (__u32 __user *)arg);
		break;

	case SPISLAVE_WR_BYTES_PER_LOAD:
		ret = __get_user(slave->bytes_per_load, (__u32 __user *)arg);
		break;

	default:

		break;
	}
	return ret;
}

static unsigned int spislave_event_poll(struct file *filp,
					struct poll_table_struct *wait)
{
	struct spi_slave *slave;
	unsigned int events = 0;
	struct spislave_data *data;

	data = filp->private_data;
	slave = data->slave;

	poll_wait(filp, &slave->wait, wait);
	if (slave->rx_offset != 0)
		events = POLLIN | POLLRDNORM;

	return events;
}

static const struct file_operations spislave_fops = {
	.owner		= THIS_MODULE,
	.open		= spislave_open,
	.read		= spislave_read,
	.write		= spislave_write,
	.release	= spislave_release,
	.unlocked_ioctl = spislave_ioctl,
	.poll		= spislave_event_poll,
};

static int spislave_probe(struct spislave_device *spi)
{
	int ret = 0;
	struct spislave_data *data;
	struct spi_slave *slave;
	int data_id;
	struct device *dev;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENODEV;

	slave = spi->slave;
	data->slave = slave;

	if (slave == NULL)
		return -ENODEV;
	do {
		if (!idr_pre_get(&spislave_idr, GFP_KERNEL) == 0) {
			dev_dbg(&slave->dev, "can't reserve idr resources.\n");
			return -ENOMEM;
		}

		spin_lock(&spislave_idr_lock);
		ret = idr_get_new(&spislave_idr, data, &data_id);
		spin_unlock(&spislave_idr_lock);
	} while (ret == -EAGAIN);


	data->devt = MKDEV(SPISLAVE_MAJOR, data_id);
	dev = device_create(spislave_class, &spi->dev,
			    data->devt, data, "%s.%d",
			    DRIVER_NAME, slave->bus_num);

	if (IS_ERR(&spislave_idr))
		return PTR_ERR(dev);

	spislave_set_drv_data(spi, data);

	return ret;
}

static int spislave_remove(struct spislave_device *spi)
{
	int ret = 0;
	struct spislave_data *data = spislave_get_drv_data(spi);

	data->slave = NULL;

	spin_lock(&spislave_lock);
	list_del(&data->device_entry);
	spin_lock(&spislave_lock);

	device_destroy(spislave_class, data->devt);

	kfree(data);

	return ret;
}

static const struct of_device_id spislave_dt_ids[] = {
	{ .compatible = "linux,spislave"},
	{},
};
MODULE_DEVICE_TABLE(of, spislave_dt_ids);

static struct spislave_driver slave_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = of_match_ptr(spislave_dt_ids),
	},
	.probe = spislave_probe,
	.remove = spislave_remove,
};

static int __init spislave_init(void)
{
	int ret = 0;

	ret = register_chrdev(SPISLAVE_MAJOR, "spislave", &spislave_fops);
	if (ret < 0)
		return ret;

	spislave_class = class_create(THIS_MODULE, DRIVER_NAME);
	if (IS_ERR(spislave_class)) {
		unregister_chrdev(SPISLAVE_MAJOR, DRIVER_NAME);
		return PTR_ERR(spislave_class);
	}

	ret = spislave_register_driver(&slave_driver);
	if (ret) {
		class_unregister(spislave_class);
		class_destroy(spislave_class);
		return ret;
	}

	idr_init(&spislave_idr);

	return ret;
}

static void __exit spislave_exit(void)
{
	idr_destroy(&spislave_idr);
	spislave_unregister_driver(&slave_driver);
	class_destroy(spislave_class);
	unregister_chrdev(SPISLAVE_MAJOR, DRIVER_NAME);
}

module_init(spislave_init);
module_exit(spislave_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Patryk Mezydlo, <mezydlo.p@gmail.com>");
MODULE_DESCRIPTION("User mode SPI slave device interface");
MODULE_VERSION("1.0");
