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
#include <asm/current.h>

#include "spi-slave-dev.h"
#include "spi-slave-core.h"

#define DRIVER_NAME		"spislavedev"
#define SPISLAVE_MAJOR		156
#define SPISLAVE_MAX_MINOR	64

static int buf_depth = 64;
module_param(buf_depth, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(buf_depth, "Size of each tx and rx buffer[default 64 bytes]");

static int word_after_data = 1;
module_param(word_after_data, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(word_after_data, "The number of words after which slave starts to send data [default 64 bytes]");

static int bits_per_word = 8;
module_param(bits_per_word, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(word_after_data, "The number of bits per word [default 8 bits]");

static int mode;
module_param(mode, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(mode, "Selection of mode [default ?]");

static LIST_HEAD(spislave_dev_list);
static struct class *spislave_class;
static DEFINE_MUTEX(spislave_dev_list_lock);
static DEFINE_IDR(spislave_idr);
static DEFINE_SPINLOCK(spislave_idr_lock);

struct spislave_data {
	dev_t devt;
	struct	list_head device_entry;
	unsigned int users;
	struct spislave *slave;
	int id;
};

static ssize_t spislave_read(struct file *filp, char __user *buf, size_t count,
			     loff_t *f_pos)
{
	struct spislave_data *data = filp->private_data;
	struct spislave *slave = data->slave;
	struct spislave_message *msg = slave->msg;
	ssize_t status;
	unsigned long missing;
	DECLARE_WAITQUEUE(wait, current);
	unsigned long flags;
	int ret = 0;

	pr_info("%s: function: read\n", DRIVER_NAME);

	if (msg->mode == SPISLAVE_SLAVE_MODE) {
		ret = spislave_transfer_msg(slave);
		if (ret < 0)
			status = -EFAULT;
	}

	spin_lock_irqsave(&msg->wait_lock, flags);

	if (filp->f_flags & O_NONBLOCK) {
		spin_unlock_irqrestore(&msg->wait_lock, flags);
		return -EAGAIN;
	}

	add_wait_queue(&msg->wait, &wait);
	for (;;) {
		set_current_state(TASK_INTERRUPTIBLE);
		if (signal_pending(current))
			break;

		spin_unlock_irqrestore(&msg->wait_lock, flags);
		schedule();
		spin_lock_irqsave(&msg->wait_lock, flags);
	}
	set_current_state(TASK_RUNNING);
	remove_wait_queue(&msg->wait, &wait);
	spin_unlock_irqrestore(&msg->wait_lock, flags);

	mutex_lock(&msg->msg_lock);

	if (count > msg->buf_depth)
		return -EMSGSIZE;

	if (!msg->tx) {
		msg->tx = kzalloc(msg->buf_depth, GFP_KERNEL);
		if (!msg->tx)
			return -ENOMEM;
	}

	if (!msg->rx) {
		msg->rx = kzalloc(msg->buf_depth, GFP_KERNEL);
		if (!msg->rx)
			return -ENOMEM;
	}

	status = count;
	missing = copy_to_user(buf, msg->rx, msg->rx_actual_length);
	if (missing == status)
		status = -EFAULT;
	else
		status = status - missing;

	mutex_unlock(&msg->msg_lock);

	return status;
}

static ssize_t spislave_write(struct file *filp, const char __user *buf,
			      size_t count, loff_t *f_pos)
{
	ssize_t status = 0;
	int ret = 0;
	unsigned long missing;
	struct spislave_data *data = filp->private_data;
	struct spislave *slave = data->slave;
	struct spislave_message *msg = slave->msg;

	pr_info("%s: function: write\n", DRIVER_NAME);

	mutex_lock(&msg->msg_lock);

	if (!msg->tx) {
		msg->tx = kzalloc(msg->buf_depth, GFP_KERNEL);
		if (!msg->tx)
			return -ENOMEM;
	} else
		memset(msg->tx, 0, msg->buf_depth);

	if (!msg->rx) {
		msg->rx = kzalloc(msg->buf_depth, GFP_KERNEL);
		if (!msg->rx)
			return -ENOMEM;
	}

	if (count > msg->buf_depth)
		return -EMSGSIZE;

	missing = copy_from_user(msg->tx, buf, count);

	if (missing == 0)
		status = count;
	else
		status = -EFAULT;

	mutex_unlock(&msg->msg_lock);

	if (msg->mode == SPISLAVE_MASTER_MODE) {
		ret = spislave_transfer_msg(slave);
		if (ret < 0)
			status = -EFAULT;
	}

	return status;
}

static int spislave_release(struct inode *inode, struct file *filp)
{
	struct spislave_data *data = filp->private_data;
	struct spislave *slave = data->slave;

	pr_info("%s: function: release\n", DRIVER_NAME);

	mutex_lock(&spislave_dev_list_lock);

	spislave_msg_remove(slave);

	data->users--;
	mutex_unlock(&spislave_dev_list_lock);

	return 0;
}

static int spislave_open(struct inode *inode, struct file *filp)
{
	struct spislave_data *data;
	struct spislave *slave;
	struct spislave_message *msg;
	int ret = -ENXIO;

	pr_info("%s: function: open\n", DRIVER_NAME);

	mutex_lock(&spislave_dev_list_lock);

	list_for_each_entry(data, &spislave_dev_list, device_entry) {
		if (data->devt == inode->i_rdev) {
			ret = 0;
			break;
		}
	}

	if (ret) {
		mutex_unlock(&spislave_dev_list_lock);
		return ret;
	}

	data->users++;
	if (data->users > 1)
		return -EBUSY;

	filp->private_data = data;
	nonseekable_open(inode, filp);
	slave = data->slave;

	msg = spislave_msg_alloc(slave);
	if (!msg)
		ret = -ENOMEM;

	mutex_unlock(&spislave_dev_list_lock);

	return 0;
}

static long spislave_ioctl(struct file *filp, unsigned int cmd,
			   unsigned long arg)
{
	struct spislave_data *data = filp->private_data;
	struct spislave *slave = data->slave;
	struct spislave_message *msg = slave->msg;
	int ret = 0;

	pr_info("%s: function: ioctl\n", DRIVER_NAME);

	mutex_lock(&msg->msg_lock);

	switch (cmd) {
	case SPISLAVE_RD_TX_ACTUAL_LENGTH:
		ret = __put_user(msg->tx_actual_length, (__u32 __user *)arg);
		break;

	case SPISLAVE_RD_RX_ACTUAL_LENGTH:
		ret = __put_user(msg->rx_actual_length, (__u32 __user *)arg);
		break;

	case SPISLAVE_RD_BITS_PER_WORD:
		ret = __put_user(msg->bits_per_word, (__u32 __user *)arg);
		break;

	case SPISLAVE_RD_WORD_AFTER_DATA:
		ret = __put_user(msg->word_after_data, (__u32 __user *)arg);
		break;

	case SPISLAVE_RD_MODE:
		ret = __put_user(msg->mode, (__u8 __user *)arg);
		break;

	case SPISLAVE_RD_BUF_DEPTH:
		ret = __put_user(msg->buf_depth, (__u32 __user *)arg);
		break;

	case SPISLAVE_WR_BITS_PER_WORD:
		ret = __get_user(msg->bits_per_word, (__u32 __user *)arg);
		break;

	case SPISLAVE_WR_MODE:
		ret = __get_user(msg->mode, (__u8 __user *)arg);
		break;

	case SPISLAVE_WR_BUF_DEPTH:
		ret = __get_user(msg->buf_depth, (__u32 __user *)arg);
		break;

	case SPISLAVE_WR_WORD_AFTER_DATA:
		ret = __get_user(msg->word_after_data, (__u32 __user *)arg);
		break;

	default:

		break;
	}

	mutex_unlock(&msg->msg_lock);
	return ret;
}

static unsigned int spislave_event_poll(struct file *filp,
					struct poll_table_struct *wait)
{
	struct spislave_data *data = filp->private_data;
	struct spislave *slave = data->slave;
	struct spislave_message *msg = slave->msg;

	pr_info("%s: function: poll\n", DRIVER_NAME);

	poll_wait(filp, &msg->wait, wait);
	if (msg->rx_actual_length > 0)
		return POLLIN | POLLRDNORM;

	return 0;
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
	struct spislave *slave;
	struct device *dev;
	struct device_node *node;

	pr_info("%s: function: probe\n", DRIVER_NAME);

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENODEV;

	slave = spi->slave;
	data->slave = slave;
	node = spi->dev.of_node;

	if (!slave) {
		ret = -ENODEV;
		goto err_out;
	}

	INIT_LIST_HEAD(&data->device_entry);

	mutex_lock(&spislave_dev_list_lock);
	spin_lock(&spislave_idr_lock);

	ret = idr_alloc(&spislave_idr, data, 0, SPISLAVE_MAX_MINOR,
			     GFP_KERNEL);

	spin_unlock(&spislave_idr_lock);

	if (ret < 0) {
		ret = -EBUSY;
		goto err_out;
	}

	data->id = ret;
	data->users = 0;
	data->devt = MKDEV(SPISLAVE_MAJOR, data->id);
	dev = device_create(spislave_class, &spi->dev, data->devt, data, "%s",
			    node->name);

	if (IS_ERR(&dev)) {
		ret =  PTR_ERR(dev);
		goto err_out;
	}

	list_add(&data->device_entry, &spislave_dev_list);
	spislave_set_drv_data(spi, data);
	mutex_unlock(&spislave_dev_list_lock);

	return 0;

err_out:
	kfree(data);
	mutex_unlock(&spislave_dev_list_lock);
	return ret;
}

static int spislave_remove(struct spislave_device *spi)
{
	struct spislave_data *data = spislave_get_drv_data(spi);

	pr_info("%s: function: remove\n", DRIVER_NAME);

	data->slave = NULL;

	mutex_lock(&spislave_dev_list_lock);
	list_del(&data->device_entry);
	mutex_unlock(&spislave_dev_list_lock);

	device_destroy(spislave_class, data->devt);

	kfree(data);

	return 0;
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

	pr_info("%s: function: init\n", DRIVER_NAME);

	ret = register_chrdev(SPISLAVE_MAJOR, DRIVER_NAME, &spislave_fops);
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

	pr_info("%s: function: exit\n", DRIVER_NAME);

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
