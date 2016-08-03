#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/of_device.h>
#include <linux/device.h>
#include <linux/kernel.h>

#define DRIVER_NAME "spislave_core"
#include "spi-slave-core.h"

static int spislave_device_match(struct device *dev,
				 struct device_driver *drv)
{
	pr_info("%s: device match", DRIVER_NAME);
	return 1;
}

struct bus_type spislave_bus = {
	.name = "spislave",
	.match = spislave_device_match,
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
