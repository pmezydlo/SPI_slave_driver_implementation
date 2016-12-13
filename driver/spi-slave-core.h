/*
 * Interface between SPI slave-side drivers and SPI slave infrastructure.
 *
 * Copyright (C) 2016 Patryk Mężydło <mezydlo.p@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef SPI_SLAVE_CORE_H
#define SPI_SLAVE_CORE_H

#define SPISLAVE_NAME_SIZE 32

extern struct bus_type spislave_bus_type;

struct spislave_message {
	void  __iomem *tx;
	u32 tx_actual_length;
	void  __iomem *rx;
	u32 rx_actual_length;
	u8 mode;
	u32 bits_per_word;
	u32 buf_depth;
	u32 max_speed;
	u8 lsb_first;

	wait_queue_head_t wait;
	spinlock_t wait_lock;
	struct mutex msg_lock;
};

struct spislave {
	struct device dev;
	struct spislave_message *msg;
	void *spislave_gadget;

	int (*transfer_msg)(struct spislave *slave);
	void (*clear_msg)(struct spislave *slave);
};

struct spislave_device_id {
	char name[SPISLAVE_NAME_SIZE];
	kernel_ulong_t driver_data;
};

struct spislave_device {
	struct device dev;
	struct spislave *slave;
	char modalias[SPISLAVE_NAME_SIZE];
};

struct spislave_driver {
	const struct spislave_device_id *id_table;
	int (*probe)(struct spislave_device *spi);
	int (*remove)(struct spislave_device *spi);
	struct device_driver driver;
};

extern struct spislave_message *spislave_msg_alloc(struct spislave *slave);
extern void spislave_msg_remove(struct spislave *slave);
extern int spislave_transfer_msg(struct spislave *slave);

extern int spislave_register_driver(struct spislave_driver *sdrv);
extern void spislave_unregister_driver(struct spislave_driver *sdrv);
extern int devm_spislave_register_slave(struct device *dev,
					struct spislave *slave);
extern void spislave_unregister_device(struct spislave_device *dev);

static inline void *spislave_get_drv_data(struct spislave_device *sdev)
{
	return dev_get_drvdata(&sdev->dev);
}

static inline void spislave_set_drv_data(struct spislave_device *sdev,
					    void *data)
{
	dev_set_drvdata(&sdev->dev, data);
}

static inline void *spislave_get_slave_data(struct spislave *slave)
{
	return dev_get_drvdata(&slave->dev);
}

static inline void spislave_set_slave_data(struct spislave *slave,
					    void *data)
{
	dev_set_drvdata(&slave->dev, data);
}

extern struct spislave *spislave_alloc_slave(struct device *dev,
					      unsigned int size);
static inline struct spislave_device *to_spislave_dev(struct device *dev)
{
	return container_of(dev, struct spislave_device, dev);
}

static inline struct spislave_driver *to_spislave_drv(struct device_driver *drv)
{
	return container_of(drv, struct spislave_driver, driver);
}

#endif
