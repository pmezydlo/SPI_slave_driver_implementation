#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/of_device.h>
#include <linux/device.h>
#include <linux/kernel.h>

#include "spi-slave-core.h"
#define DRIVER_NAME "spislavecore"

int spislave_register_driver(struct spislave_driver *sdrv)
{
	sdrv->driver.owner = THIS_MODULE;
	sdrv->driver.bus = &spislave_bus_type;

	return driver_register(&sdrv->driver);
}
EXPORT_SYMBOL_GPL(spislave_register_driver);

void spislave_unregister_driver(struct spislave_driver *sdrv)
{
	driver_unregister(&sdrv->driver);
}
EXPORT_SYMBOL_GPL(spislave_unregister_driver);

int spislave_register_device(struct spislave_device *sdev)
{
	int			ret = 0;

	sdev->dev.bus = &spislave_bus_type;
	dev_set_name(&sdev->dev, "%s", sdev->name);

	ret = device_register(&sdev->dev);

	if (ret) {
		pr_err("%s: Failed to register device\n", DRIVER_NAME);
		return 0;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(spislave_register_device);

void spislave_unregister_device(struct spislave_device *sdev)
{
	device_unregister(&sdev->dev);
}
EXPORT_SYMBOL_GPL(spislave_unregister_device);

static int spislave_device_match(struct device *dev,
				 struct device_driver *drv)
{
	pr_info("%s: device match", DRIVER_NAME);
	return 1;
}

struct bus_type spislave_bus_type = {
	.name = "spislave",
	.match = spislave_device_match,
};

static int __init spislave_init(void)
{
	int			ret = 0;

	pr_info("%s: init\n", DRIVER_NAME);

	ret = bus_register(&spislave_bus_type);

	return ret;
}

static void __exit spislave_exit(void)
{
	bus_unregister(&spislave_bus_type);
	pr_info("%s: exit\n", DRIVER_NAME);
}

module_init(spislave_init);
module_exit(spislave_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Patryk Mezydlo, <mezydlo.p@gmail.com>");
MODULE_DESCRIPTION("User mode SPI slave device interface");
MODULE_VERSION("1.0");
