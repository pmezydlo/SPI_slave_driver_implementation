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

static inline void *spislave_get_drv_data(struct spislave_device *sdev)
{
	return dev_get_drvdata(&sdev->dev);
}

static inline void spislave_set_drv_data(struct spislave_device *sdev,
					    void *data)
{
	dev_set_drvdata(&sdev->dev, data);
}

/*============================================================================*/
void spislave_unregister_pdev(struct spi_slave *slave)
{
	device_unregister(&slave->dev);
}

static void devm_spislave_unregister_pdev(struct device *dev, void *res)
{
	spislave_unregister_pdev(*(struct spi_slave **)res);
}

int spislave_register_device(struct device *dev, const char *name,
				  struct spi_slave *slave,
				  struct device_node *node)
{
	int					ret = 0;
	struct spi_slave			**ptr;
	struct device_node			*nc;
	struct spislave_device			*slave_dev;

	pr_info("%s: register device and driver start\n", DRIVER_NAME);

	ptr = devres_alloc(devm_spislave_unregister_pdev, sizeof(*ptr),
			   GFP_KERNEL);
	if (!ptr) {
		pr_err("%s: devers alloc error\n", DRIVER_NAME);
		return -ENOMEM;
	}

	slave->dev.bus = &spislave_bus_type;
	slave->dev.of_node = node;
	dev_set_name(&slave->dev, "%s", slave->name);
	ret = device_register(&slave->dev);
	if (!ret) {
		pr_info("%s: register device ok\n", DRIVER_NAME);
		*ptr = slave;
		devres_add(dev, ptr);
	} else {
		devres_free(ptr);
	}

	for_each_available_child_of_node(dev->of_node, nc) {
		if (of_node_test_and_set_flag(nc, OF_POPULATED))
			continue;

		pr_info("%s: chid node is found: %s\n", DRIVER_NAME,
			nc->full_name);

		/*TODO: dem_kzalloc, */
		slave_dev = kzalloc(sizeof(struct spislave_device), GFP_KERNEL);

		if (slave_dev == NULL) {
			put_device(&slave->dev);
			return -ENOMEM;
		}

		slave_dev->slave = slave;
		slave_dev->dev.parent = &slave->dev;
		slave_dev->dev.bus = &spislave_bus_type;
		/*slave_dev->dev.release = spislave_dev_release;*/

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
		ret = device_add(&slave_dev->dev);
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

static inline void *spislave_get_slave_data(struct spi_slave *slave)
{
	return dev_get_drvdata(&slave->dev);
}

static inline void spislave_set_slave_data(struct spi_slave *slave,
					    void *data)
{
	dev_set_drvdata(&slave->dev, data);
}

/*============================================================================*/

static const struct spislave_device_id *spislave_match_id(const struct
		    spislave_device_id *id, const struct spislave_device *sdev)
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

	pr_info("%s: device match \n", DRIVER_NAME);

	sdrv = to_spislave_drv(dev->driver);
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
