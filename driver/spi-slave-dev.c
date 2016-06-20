#include <linux/init.h>
#include <linux/module.h>

#define DRIVER_NAME "spi-slave-dev"

static int __init spislave_init(void)
{
	int ret = 0;

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
