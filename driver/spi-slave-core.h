#ifndef SPI_SLAVE_CORE_H
#define SPI_SLAVE_CORE_H

extern struct bus_type spislave_bus_type;

struct spislave_driver {
	struct module			*module;
	struct device_driver		driver;
};

extern int spislave_register_driver(struct spislave_driver *sdrv);




#endif
