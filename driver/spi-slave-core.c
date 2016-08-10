#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/of_device.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/pm_domain.h>
#include <linux/slab.h>

#include "spi-slave-core.h"
#define DRIVER_NAME "spislavecore"

static int spislave_drv_probe(struct device *dev)
{
	int				ret = 0;
	const struct spislave_driver	*sdrv;
	struct spislave_device		*sdev;

	sdrv = to_spislave_drv(dev->driver);
	sdev = to_spislave_dev(dev);

	ret = dev_pm_domain_attach(dev, true);
	if (ret != -EPROBE_DEFER) {
		ret = sdrv->probe(sdev);
		if (ret)
			dev_pm_domain_detach(dev, true);
	}

	return ret;
}

static int spislave_drv_remove(struct device *dev)
{
	int			ret = 0;
	const struct spislave_driver	*sdrv;
	struct spislave_device		*sdev;

	sdrv = to_spislave_drv(dev->driver);
	sdev = to_spislave_dev(dev);

	ret = sdrv->remove(sdev);
	dev_pm_domain_detach(dev, true);

	return ret;
}

int spislave_register_driver(struct spislave_driver *sdrv)
{
	sdrv->driver.owner = THIS_MODULE;
	sdrv->driver.bus = &spislave_bus_type;

	sdrv->driver.probe = spislave_drv_probe;
	sdrv->driver.remove = spislave_drv_remove;

	return driver_register(&sdrv->driver);
}
EXPORT_SYMBOL_GPL(spislave_register_driver);

void spislave_unregister_driver(struct spislave_driver *sdrv)
{
	driver_unregister(&sdrv->driver);
}
EXPORT_SYMBOL_GPL(spislave_unregister_driver);


/*============================================================================*/
void spislave_unregister_pdev(struct spi_slave *slave)
{
	device_unregister(&slave->dev);
}

static void devm_spislave_unregister_pdev(struct device *dev, void *res)
{
	spislave_unregister_pdev(*(struct spi_slave **)res);
}



static void spislave_dev_release(struct device *dev)
{
	struct spislave_device *slave_dev = to_spislave_dev(dev);
	struct spi_slave *slave;

	slave = slave_dev->slave;

	put_device(&slave->dev);
	kfree(slave_dev);
}



static void spislave_release(struct device *dev)
{
	struct spi_slave *slave;

	slave = container_of(dev, struct spi_slave, dev);
	kfree(slave);
}

static struct class spislave_class = {
	.name = "spi_slave",
	.owner = THIS_MODULE,
	.dev_release = spislave_release,
};

struct spi_slave *spislave_alloc_slave(struct device *dev, unsigned size)
{
	struct spi_slave		*slave;

	pr_info("%s: slave alloc\n", DRIVER_NAME);

	slave = kzalloc(size + sizeof(*slave), GFP_KERNEL);
	if (!slave)
		return NULL;

	pr_info("%s: register device\n", DRIVER_NAME);

	device_initialize(&slave->dev);
	slave->dev.class = &spislave_class;
	slave->dev.parent = get_device(dev);
	dev_set_drvdata(&slave->dev, (void *)&slave[1]);

	return slave;
}
EXPORT_SYMBOL_GPL(spislave_alloc_slave);

int spislave_register_device(struct device *dev, const char *name,
				  struct spi_slave *slave,
				  struct device_node *node)
{
	int					ret = 0;
	struct spi_slave			**ptr;
	struct device_node			*nc;
	struct spislave_device			*slave_dev;

	ptr = devres_alloc(devm_spislave_unregister_pdev, sizeof(*ptr),
			   GFP_KERNEL);
	if (!ptr) {
		pr_err("%s: devers alloc error\n", DRIVER_NAME);
		return -ENOMEM;
	}

	slave->dev.of_node = dev ? dev->of_node : NULL;
	slave->dev.bus = &spislave_bus_type;

	if ((slave->bus_num < 0) && slave->dev.of_node)
		slave->bus_num = of_alias_get_id(slave->dev.of_node, "spi");

	dev_set_name(&slave->dev, "%s.%u", name, slave->bus_num);
	ret = device_add(&slave->dev);
	if (!ret) {
		pr_info("%s: register device ok\n", DRIVER_NAME);
		*ptr = slave;
		devres_add(dev, ptr);
	} else
		devres_free(ptr);

	for_each_available_child_of_node(dev->of_node, nc) {
		if (of_node_test_and_set_flag(nc, OF_POPULATED))
			continue;

		pr_info("%s: chid node is found: %s\n", DRIVER_NAME,
			nc->full_name);

		slave_dev = kzalloc(sizeof(struct spislave_device), GFP_KERNEL);

		if (slave_dev == NULL) {
			put_device(&slave->dev);
			return -ENOMEM;
		}

		slave_dev->slave = slave;
		slave_dev->dev.parent = &slave->dev;
		slave_dev->dev.bus = &spislave_bus_type;
		slave_dev->dev.release = spislave_dev_release;

		ret = of_modalias_node(nc, slave_dev->modalias,
				      sizeof(slave_dev->modalias));

		if (ret < 0) {
			pr_err("%s: cannot find modalias\n", DRIVER_NAME);
			return ret;
		}

		pr_info("%s: modalias:%s\n", DRIVER_NAME, slave_dev->modalias);

		of_node_get(nc);
		slave_dev->dev.of_node = nc;
		dev_set_name(&slave_dev->dev, "slave_dev%d", slave->bus_num);
		ret = device_register(&slave_dev->dev);
		if (!ret) {
			pr_err("%s: register child device ok\n",
			       DRIVER_NAME);
		} else
			return ret;

	}

	return ret;
}
EXPORT_SYMBOL_GPL(spislave_register_device);

void spislave_unregister_device(struct spislave_device *sdev)
{
	device_unregister(&sdev->dev);
}
EXPORT_SYMBOL_GPL(spislave_unregister_device);
/*============================================================================*/

static const struct spislave_device_id *spislave_match_id(const struct
		    spislave_device_id * id, const struct spislave_device *sdev)
{
	pr_info("%s: spislave match id\n", DRIVER_NAME);

	while (id->name[0]) {
		if (!strcmp(sdev->modalias, id->name))
			return id;
		id++;
	}
	return NULL;
}

static int spislave_device_match(struct device *dev,
				struct device_driver *drv)
{
	const struct spislave_driver		*sdrv;
	const struct spislave_device		*sdev;

	pr_info("%s: device match\n", DRIVER_NAME);

	sdrv = to_spislave_drv(drv);
	sdev = to_spislave_dev(dev);

	pr_info("%s: to spi slave drv and dev", DRIVER_NAME);

	if (sdrv == NULL)
		pr_err("%s: sdrv is NULL\n", DRIVER_NAME);

	if (sdev == NULL)
		pr_err("%s: sdev is NULL\n", DRIVER_NAME);


	if (sdrv->id_table)
		return !!spislave_match_id(sdrv->id_table, sdev);

	return strcmp(sdev->modalias, sdrv->driver.name) == 0;
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
	if (ret < 0)
		return ret;

	ret = class_register(&spislave_class);
	if (ret < 0) {
		bus_unregister(&spislave_bus_type);
		pr_err("%s: class register erroc\n", DRIVER_NAME);
	}

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
