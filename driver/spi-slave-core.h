#ifndef SPI_SLAVE_CORE_H
#define SPI_SLAVE_CORE_H

#define SPISLAVE_NAME_SIZE 14

extern struct bus_type spislave_bus_type;

struct spi_slave {
	/*var defining device parameters*/
	struct	device				dev;
	void	__iomem				*base;
	u32					start;
	u32					end;
	unsigned int				reg_offset;
	s16					bus_num;
	char					name[SPISLAVE_NAME_SIZE];

	/*var defining cs and pin direct parameters*/
	unsigned int				pin_dir;
	u32					cs_sensitive;
	u32					cs_polarity;
	unsigned int				pha;
	unsigned int				pol;

	/*var defining interrupt*/
	unsigned int				irq;

	/*var defining msg */
	u32					tx_offset;
	u32					rx_offset;
	void  __iomem				*tx;
	void  __iomem				*rx;

	/*var defining the char driver parameters*/
	char					modalias[SPI_NAME_SIZE];
	dev_t					devt;
	struct	list_head			device_entry;
	unsigned int				users;
	wait_queue_head_t			wait;

	/*var defining the transfer parameters*/
	u32					mode;
	u32					bytes_per_load;
	u32					bits_per_word;
	u32					buf_depth;
};

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
extern int devm_spislave_register_device(struct device *dev, const char *name,
				  struct spi_slave *slave);
extern void spislave_unregister_device(struct spislave_device *sdev);

#endif
