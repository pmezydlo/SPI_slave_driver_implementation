#ifndef SPI_SLAVE_CORE_H
#define SPI_SLAVE_CORE_H

#define SPISLAVE_NAME_SIZE 14

extern struct bus_type spislave_bus_type;

struct spislave_driver {
	struct module			*module;
	struct device_driver		driver;
};

struct spislave_device {
	char				name[SPISLAVE_NAME_SIZE];
	struct device			dev;
	unsigned int			id;
};

extern int spislave_register_driver(struct spislave_driver *sdrv);
extern void spislave_unregister_driver(struct spislave_driver *sdrv);
extern int spislave_register_device(struct spislave_device *sdev);
extern void spislave_unregister_device(struct spislave_device *sdev);

#endif
