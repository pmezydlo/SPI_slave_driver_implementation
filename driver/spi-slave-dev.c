#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/of_device.h>
#include <linux/device.h>

#define DRIVER_NAME		"spislave"
#define SPISLAVE_MAJOR		153


struct spislave_dev {
	int majorNumber;

};

static const struct file_operations spislave_fops = {
	.owner = THIS_MODULE,
};

static struct class *spislave_class;

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

	ret  = register_chrdev(SPISLAVE_MAJOR, "spi", &spislave_fops);

	if (ret < 0)
		return ret;

	pr_info("%s: register chrdev\n", DRIVER_NAME);

	spislave_class = class_create(THIS_MODULE, "spislave");
	if (IS_ERR(spislave_class)) {
		unregister_chrdev(SPISLAVE_MAJOR, spislave_driver.driver.name);
		return PTR_ERR(spislave_class);
	}

	pr_info("%s: class create\n", DRIVER_NAME);
	pr_info("%s: driver register\n", DRIVER_NAME);

	if (ret == 0)
		pr_info("%s: platform driver register ok\n", DRIVER_NAME);
	else
		pr_info("%s: platform driver register error\n", DRIVER_NAME);

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
