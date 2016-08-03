#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/of_device.h>
#include <linux/device.h>

#include "spi-slave-dev.h"
#include "spi-slave-core.h"

#define DRIVER_NAME			"spislavedev"

static struct spislave_driver slave_driver = {
	.driver = {
		.name = DRIVER_NAME,
	},
};

static int __init spislave_init(void)
{
	int			ret = 0;

	ret = spislave_register_driver(&slave_driver);
	if (ret)
		return ret;

	pr_info("%s: init\n", DRIVER_NAME);

	return ret;
}

static void __exit spislave_exit(void)
{
	pr_info("%s: exit\n", DRIVER_NAME);
}

module_init(spislave_init);
module_exit(spislave_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Patryk Mezydlo, <mezydlo.p@gmail.com>");
MODULE_DESCRIPTION("User mode SPI slave device interface");
MODULE_VERSION("1.0");
