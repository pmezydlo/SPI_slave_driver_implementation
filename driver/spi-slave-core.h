#ifndef SPI_SLAVE_CORE_H
#define SPI_SLAVE_CORE_H

#define SPISLAVE_NAME_SIZE 14

extern struct bus_type spislave_bus_type;

struct spi_slave {
	struct	device				dev;
	void	__iomem				*base;
	u32					start;
	u32					end;
	unsigned int				reg_offset;
	s16					bus_num;
	char					name[SPISLAVE_NAME_SIZE];

	unsigned int				pin_dir;
	u32					cs_sensitive;
	u32					cs_polarity;
	unsigned int				pha;
	unsigned int				pol;

	unsigned int				irq;

	u32					tx_offset;
	u32					rx_offset;
	void  __iomem				*tx;
	void  __iomem				*rx;

	u32					mode;
	u32					bytes_per_load;
	u32					bits_per_word;
	u32					buf_depth;

	wait_queue_head_t			wait;

	void			(*enable)(struct spi_slave *slave);
	void			(*disable)(struct spi_slave *slave);
	int			(*set_transfer)(struct spi_slave *slave);
	int			(*clr_transfer)(struct spi_slave *slave);
	void			(*transfer)(struct spi_slave *slave);
};

struct spislave_driver {
	int				(*probe)(struct spi_slave *spi);
	int				(*remove)(struct spi_slave *spi);
	struct device_driver		driver;
};

extern int spislave_register_driver(struct spislave_driver *sdrv);
extern void spislave_unregister_driver(struct spislave_driver *sdrv);
extern int devm_spislave_register_device(struct device *dev, const char *name,
				  struct spi_slave *slave);
extern void spislave_unregister_device(struct spi_slave *slave);

#endif
