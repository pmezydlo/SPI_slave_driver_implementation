#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/of_device.h>
#include <linux/device.h>
#include <linux/spi/spi.h>

#include "spi-slave-dev.h"

#define DRIVER_NAME		"spislave"
#define SPISLAVE_MAJOR		154
#define N_SPI_MINORS		32

static DECLARE_BITMAP(minors, N_SPI_MINORS);

static struct class	*spislave_class;
dev_t			devt;

struct spislave {
	struct device		*dev;
	dev_t			devt;
};

static ssize_t spislave_read(struct file *flip, char __user *buf, size_t count,
			     loff_t *f_pos)
{
	ssize_t		ret = 0;

	pr_info("%s read\n", DRIVER_NAME);
	return ret;
}

static ssize_t spislave_write(struct file *file, const char __user *buf,
			      size_t count, loff_t *f_pos)
{
	ssize_t		ret = 0;

	pr_info("%s write\n", DRIVER_NAME);
	return ret;
}

static int spislave_release(struct inode *inode, struct file *filp)
{
	int	ret = 0;

	pr_info("%s release\n", DRIVER_NAME);
	return ret;
}

static int spislave_open(struct inode *inode, struct file *filp)
{
	int	ret = 0;

	pr_info("%s open\n", DRIVER_NAME);
	return ret;
}

static const struct file_operations spislave_fops = {
	.owner		= THIS_MODULE,
	.open		= spislave_open,
	.read		= spislave_read,
	.write		= spislave_write,
	.release	= spislave_release,
};

static const struct of_device_id spislave_of_match[] = {
	{
		.compatible = "linux,spislave",
	},
	{ }
};
MODULE_DEVICE_TABLE(of, spislave_of_match);

static int spislave_probe(struct platform_device *pdev)
{
	int			ret = 0;

	pr_info("%s: Entry probe\n", DRIVER_NAME);

	return ret;
}

static int spislave_remove(struct platform_device *pdev)
{
	int		ret = 0;

	pr_info("%s: remove\n", DRIVER_NAME);

	return ret;
}

static struct platform_driver spislave_driver = {
	.probe = spislave_probe,
	.remove = spislave_remove,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = of_match_ptr(spislave_of_match),
	},
};

static int __init spislave_init(void)
{
	int			ret = 0;
	unsigned long		minor;

	pr_info("%s: init\n", DRIVER_NAME);

	BUILD_BUG_ON(N_SPI_MINORS > 256);

	ret = register_chrdev(SPISLAVE_MAJOR, "spi", &spislave_fops);
	if (ret < 0)
		return ret;

	spislave_class = class_create(THIS_MODULE, DRIVER_NAME);
	if (IS_ERR(spislave_class)) {
		unregister_chrdev(SPISLAVE_MAJOR, DRIVER_NAME);
		return PTR_ERR(spislave_class);
	}

	minor = find_first_zero_bit(minors, N_SPI_MINORS);

	if (minor < N_SPI_MINORS) {
		struct device *dev;

		devt = MKDEV(SPISLAVE_MAJOR, minor);
		dev = device_create(spislave_class, NULL,
				    devt,
				    NULL, DRIVER_NAME);

		ret = PTR_ERR_OR_ZERO(dev);
	} else {
		pr_err("%s: no minor number available!!\n",
		       DRIVER_NAME);
		ret = -ENODEV;
	}

	return ret;
}

static void __exit spislave_exit(void)
{
	device_destroy(spislave_class, devt);
	class_unregister(spislave_class);
	class_destroy(spislave_class);
	unregister_chrdev(SPISLAVE_MAJOR, DRIVER_NAME);


	pr_info("%s: exit\n", DRIVER_NAME);
}

module_init(spislave_init);
module_exit(spislave_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Patryk Mezydlo, <mezydlo.p@gmail.com>");
MODULE_DESCRIPTION("User mode SPI slave device interface");
MODULE_VERSION("1.0");
