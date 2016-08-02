#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/of_device.h>
#include <linux/device.h>
#include <linux/spi/spi.h>

#include "spi-slave-dev.h"

#define DRIVER_NAME		"spislave"

struct spislave {
	struct device		*dev;
	dev_t			devt;
};

static const struct of_device_id spislave_of_match[] = {
	{
		.compatible = "ti,omap4-mcspi",
	},
	{ }
};
MODULE_DEVICE_TABLE(of, spislave_of_match);

static int spislave_probe(struct device *pdev)
{
	int			ret = 0;

	pr_info("%s: Entry probe\n", DRIVER_NAME);

	return ret;
}

static int spislave_remove(struct device *pdev)
{
	int		ret = 0;

	pr_info("%s: remove\n", DRIVER_NAME);

	return ret;
}

struct bus_type spislave_bus = {
	.name = "spislave",
	.probe = spislave_probe,
	.remove = spislave_remove,
};

static int __init spislave_init(void)
{
	int			ret = 0;


	pr_info("%s: init\n", DRIVER_NAME);

	ret = bus_register(&spislave_bus);

	return ret;
}

static void __exit spislave_exit(void)
{
	bus_unregister(&spislave_bus);
	pr_info("%s: exit\n", DRIVER_NAME);
}

module_init(spislave_init);
module_exit(spislave_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Patryk Mezydlo, <mezydlo.p@gmail.com>");
MODULE_DESCRIPTION("User mode SPI slave device interface");
MODULE_VERSION("1.0");
