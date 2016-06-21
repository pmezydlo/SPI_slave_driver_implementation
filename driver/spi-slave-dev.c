#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/of_device.h>
#include <linux/device.h>
#include <linux/spi/spi.h>

#define DRIVER_NAME		"spislave"

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
	int ret = 0;

	pr_info("%s: init\n", DRIVER_NAME);

	ret = platform_driver_register(&spislave_driver);

	if (ret < 0)
		pr_info("%s: platform driver register error\n", DRIVER_NAME);
	else
		pr_info("%s: platform driver register ok\n", DRIVER_NAME);

	return ret;
}

static void __exit spislave_exit(void)
{
	platform_driver_unregister(&spislave_driver);

	pr_info("%s: exit\n", DRIVER_NAME);
}

module_init(spislave_init);
module_exit(spislave_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Patryk Mezydlo, <mezydlo.p@gmail.com>");
MODULE_DESCRIPTION("User mode SPI slave device interface");
MODULE_VERSION("1.0");
