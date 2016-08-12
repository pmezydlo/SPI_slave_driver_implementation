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
#define SPISLAVE_MODULE_PREFIX "spislave:"

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

	pr_info("%s: spislave drv remove\n", DRIVER_NAME);

	sdrv = to_spislave_drv(dev->driver);

	if (sdrv == NULL) {
		pr_err("%s: sdrv pointer is NULL\n", DRIVER_NAME);
		return -ENODEV;
	}

	sdev = to_spislave_dev(dev);

	if (sdev == NULL) {
		pr_err("%s: sdev pointer is NULL", DRIVER_NAME);
		return -ENODEV;
	}

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
	if (sdrv)
		driver_unregister(&sdrv->driver);
}
EXPORT_SYMBOL_GPL(spislave_unregister_driver);


/*============================================================================*/
static int __unregister(struct device *dev, void *null)
{
	spislave_unregister_device(to_spislave_dev(dev));
	return 0;
}

void spislave_unregister_slave(struct spi_slave *slave)
{
	int				dummy;

	pr_info("%s: spislave_unregister slave", DRIVER_NAME);

	dummy = device_for_each_child(&slave->dev, NULL, __unregister);
	device_unregister(&slave->dev);
}
EXPORT_SYMBOL_GPL(spislave_unregister_slave);

static void devm_spislave_unregister_slave(struct device *dev, void *res)
{
	pr_info("%s: devm spislave unregister pdev\n", DRIVER_NAME);
	spislave_unregister_slave(*(struct spi_slave **)res);
}
EXPORT_SYMBOL_GPL(devm_spislave_unregister_slave);

static void spislave_dev_release(struct device *dev)
{
	struct spislave_device *slave_dev = to_spislave_dev(dev);
	struct spi_slave *slave;

	pr_info("%s: spislave dev release\n", DRIVER_NAME);

	slave = slave_dev->slave;

	pr_info("%s: put device\n", DRIVER_NAME);

	put_device(&slave->dev);
	kfree(slave_dev);
}

static void spislave_release(struct device *dev)
{
	struct spi_slave *slave;

	pr_info("%s: spislave release\n", DRIVER_NAME);
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

static struct spislave_device *spislave_register_device(struct spi_slave *slave,
							struct device_node *nc)
{
	struct spislave_device			*slave_dev;
	int					ret = 0;

	pr_info("%s: chid node is found: %s\n", DRIVER_NAME,
		nc->full_name);

	slave_dev = kzalloc(sizeof(*slave_dev), GFP_KERNEL);

	if (slave_dev == NULL) {
		put_device(&slave->dev);
		return NULL;
	}

	slave_dev->slave = slave;
	slave_dev->dev.parent = &slave->dev;
	slave_dev->dev.bus = &spislave_bus_type;
	slave_dev->dev.release = spislave_dev_release;

	ret = of_modalias_node(nc, slave_dev->modalias,
			      sizeof(slave_dev->modalias));
	if (ret < 0) {
		pr_err("%s: cannot find modalias\n", DRIVER_NAME);
		return NULL;
	}

	pr_info("%s: modalias:%s\n", DRIVER_NAME, slave_dev->modalias);
	of_node_get(nc);
	slave_dev->dev.of_node = nc;
	dev_set_name(&slave_dev->dev, "%s%d", slave->name, slave->bus_num);
	ret = device_register(&slave_dev->dev);
	if (!ret) {
		pr_info("%s: register child device ok\n",
		       DRIVER_NAME);
	}

	return slave_dev;
}

int spislave_register_devices(struct spi_slave *slave)
{
	struct spislave_device		*slave_dev;
	struct device_node		*nc;

	if (!slave->dev.of_node)
		return -ENODEV;

	for_each_available_child_of_node(slave->dev.of_node, nc) {
		if (of_node_test_and_set_flag(nc, OF_POPULATED))
			continue;

		slave_dev = spislave_register_device(slave, nc);
		if (IS_ERR(slave_dev))
			pr_err("%s: filed to create spislave device\n",
			DRIVER_NAME);
	}
	return 0;
}


int spislave_register_slave(struct spi_slave *slave, struct device *dev)
{
	int					ret = 0;

	if (!dev)
		return -ENODEV;

	pr_info("%s: spislave register slave\n", DRIVER_NAME);
	slave->dev.of_node = dev ? dev->of_node : NULL;
	dev_set_name(&slave->dev, "%s.%u", slave->name, slave->bus_num);

	if ((slave->bus_num < 0) && slave->dev.of_node)
		slave->bus_num = of_alias_get_id(slave->dev.of_node, "spi");

	ret = device_add(&slave->dev);
	if (ret < 0) {
		pr_err("%s:device add error\n", DRIVER_NAME);
		return -ENODEV;
	}

	ret = spislave_register_devices(slave);
	if (ret < 0)
		pr_err("%s: child device add erroc\n", DRIVER_NAME);

	return ret;
}

int devm_spislave_register_slave(struct device *dev,
				  struct spi_slave *slave)
{
	int					ret = 0;
	struct spi_slave			**ptr;

	ptr = devres_alloc(devm_spislave_unregister_slave, sizeof(*ptr),
			   GFP_KERNEL);
	if (!ptr) {
		pr_err("%s: devers alloc error\n", DRIVER_NAME);
		return -ENOMEM;
	}

	ret = spislave_register_slave(slave, dev);
	if (!ret) {
		pr_info("%s: register device ok\n", DRIVER_NAME);
		*ptr = slave;
		devres_add(dev, ptr);
	} else
		devres_free(ptr);

	return ret;
}
EXPORT_SYMBOL_GPL(devm_spislave_register_slave);

void spislave_unregister_device(struct spislave_device *sdev)
{
	if (!sdev)
		return;

	if (sdev->dev.of_node)
		of_node_clear_flag(sdev->dev.of_node, OF_POPULATED);

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

	if (of_driver_match_device(dev, drv))
		return 1;

	if (sdrv->id_table)
		return !!spislave_match_id(sdrv->id_table, sdev);

	return strcmp(sdev->modalias, sdrv->driver.name) == 0;
}

static int spislave_uevent(struct device *dev, struct kobj_uevent_env *env)
{
	const struct spislave_device *spi_dev = to_spislave_dev(dev);

	add_uevent_var(env, "MODALIAS=%s%s", SPISLAVE_MODULE_PREFIX,
		       spi_dev->modalias);

	return 0;
}

struct bus_type spislave_bus_type = {
	.name = "spislave",
	.match = spislave_device_match,
	.uevent = spislave_uevent,
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
MODULE_DESCRIPTION("bus driver");
MODULE_VERSION("1.0");
