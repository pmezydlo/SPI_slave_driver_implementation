#ifndef SPI_SLAVE_CORE_H
#define SPI_SLAVE_CORE_H


struct spislave_driver {
	char				*version;
	struct module			*module;
	struct device_driver		driver;
	struct driver_attribute		version_attr;

};

extern struct bus_type			spislave_bus_type;


#endif
