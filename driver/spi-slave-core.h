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

struct spislave_device_id {
	char			name[SPISLAVE_NAME_SIZE];
	kernel_ulong_t		driver_data;
};

struct spislave_device {
	struct device                   dev;
        struct spi_slave                *slave;
	char				modalias[SPISLAVE_NAME_SIZE];
};

struct spislave_driver {
	const struct spislave_device_id *id_table;
	int				(*probe)(struct spislave_device *spi);
	int				(*remove)(struct spislave_device *spi);
	struct device_driver		driver;
};

extern int spislave_register_driver(struct spislave_driver *sdrv);
extern void spislave_unregister_driver(struct spislave_driver *sdrv);

extern int spislave_register_device(struct device *dev, const char *name,
					  struct spi_slave *slave,
					  struct device_node *node);

extern void spislave_unregister_device(struct spislave_device *dev);

static inline struct spislave_device *to_spislave_dev(struct device *dev)
{
	return dev ? container_of(dev, struct spislave_device, dev) : NULL;
}

static inline struct spislave_driver *to_spislave_drv(struct device_driver *drv)
{
	return drv ? container_of(drv, struct spislave_driver, driver) : NULL;
}

#endif
