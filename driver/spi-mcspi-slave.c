/*
 * Driver for OMAP2 McSPI controller in slave mode.
 *
 * Copyright (C) 2016 Patryk Mężydło <mezydlo.p@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/pm_runtime.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/uaccess.h>

#define DRIVER_NAME "spi-mcspi-slave"

#include <linux/platform_data/spi-omap2-mcspi.h>

#include "spi-slave-core.h"

#define MCSPI_PIN_DIR_D0_IN_D1_OUT		0
#define MCSPI_PIN_DIR_D0_OUT_D1_IN		1
#define MCSPI_CS_POLARITY_ACTIVE_HIGH		1
#define MCSPI_CS_POLARITY_ACTIVE_LOW		0
#define MCSPI_CS_SENSITIVE_ENABLED		1
#define MCSPI_CS_SENSITIVE_DISABLED		0
#define MCSPI_MAX_FIFO_DEPTH			64
#define MCSPI_POL_HELD_HIGH			0
#define MCSPI_POL_HELD_LOW			1
#define MCSPI_PHA_ODD_NUMBERED_EDGES		0
#define MCSPI_PHA_EVEN_NUMBERED_EDGES		1

#define SPI_SLAVE_CS_SENSITIVE			MCSPI_CS_SENSITIVE_ENABLED
#define SPI_SLAVE_CS_POLARITY			MCSPI_CS_POLARITY_ACTIVE_LOW
#define SPI_SLAVE_PIN_DIR			MCSPI_PIN_DIR_D0_IN_D1_OUT

#define MCSPI_SYSCONFIG				0x10
#define MCSPI_SYSSTATUS				0x14
#define MCSPI_IRQSTATUS				0x18
#define MCSPI_IRQENABLE				0x1C
#define MCSPI_SYST				0x24
#define MCSPI_MODULCTRL				0x28
#define MCSPI_CH0CONF				0x2C
#define MCSPI_CH0STAT				0x30
#define MCSPI_CH0CTRL				0x34
#define MCSPI_TX0				0x38
#define MCSPI_RX0				0x3C
#define MCSPI_XFERLEVEL				0x7C
#define MCSPI_DAFTX				0x80
#define MCSPI_DAFRX				0xA0

#define SPI_AUTOSUSPEND_TIMEOUT			-1

#define MCSPI_SYSSTATUS_RESETDONE		BIT(0)
#define MCSPI_MODULCTRL_MS			BIT(2)
#define MCSPI_MODULCTRL_PIN34			BIT(1)
#define MCSPI_CHCTRL_EN				BIT(0)
#define MCSPI_CHCONF_EPOL			BIT(6)

#define MCSPI_CHCONF_TRM			(0x03 << 12)
#define MCSPI_CHCONF_TM				BIT(13)
#define MCSPI_CHCONF_RM				BIT(12)

#define MCSPI_CHCONF_WL				(0x1F << 7)

#define MCSPI_CHCONF_WL_8BIT_MASK		(0x07 << 7)
#define MCSPI_CHCONF_WL_16BIT_MASK		(0x0F << 7)
#define MCSPI_CHCONF_WL_32BIT_MASK		(0x1F << 7)

#define MCSPI_CHCONF_IS				BIT(18)
#define MCSPI_CHCONF_DPE0			BIT(16)
#define MCSPI_CHCONF_DPE1			BIT(17)
#define MCSPI_CHCONF_POL			BIT(1)
#define MCSPI_CHCONF_PHA			BIT(0)

#define MCSPI_IRQ_RX_OVERFLOW			BIT(3)
#define MCSPI_IRQ_RX_FULL			BIT(2)
#define MCSPI_IRQ_TX_UNDERFLOW			BIT(1)
#define MCSPI_IRQ_TX_EMPTY			BIT(0)
#define MCSPI_IRQ_EOW				BIT(17)

#define MCSPI_SYSCONFIG_CLOCKACTIVITY		(0x03 << 8)
#define MCSPI_SYSCONFIG_SIDLEMODE		(0x03 << 3)
#define MCSPI_SYSCONFIG_SOFTRESET		BIT(1)
#define MCSPI_SYSCONFIG_AUTOIDLE		BIT(0)

#define MCSPI_IRQ_RESET				0xFFFFFFFF

#define MCSPI_XFER_AFL				(0x7 << 8)
#define MCSPI_XFER_AEL				(0x7)
#define MCSPI_XFER_WCNT				(0xFFFF << 16)

#define MCSPI_CHCONF_FFER			BIT(28)
#define MCSPI_CHCONF_FFEW			BIT(27)

#define MCSPI_MODULCTRL_MOA			BIT(7)
#define MCSPI_MODULCTRL_FDAA			BIT(8)

#define MCSPI_CHSTAT_EOT			BIT(2)
#define MCSPI_CHSTAT_TXS			BIT(1)
#define MCSPI_CHSTAT_RXS			BIT(1)
#define MCSPI_CHSTAT_RXFFF			BIT(6)
#define MCSPI_CHSTAT_RXFFE			BIT(5)
#define MCSPI_CHSTAT_TXFFF			BIT(4)
#define MCSPI_CHSTAT_TXFFE			BIT(3)

struct mcspi_drv {
	void __iomem *base;

	unsigned int pin_dir;
	u32 cs_sensitive;
	u32 cs_polarity;
	unsigned int pha;
	unsigned int pol;
	unsigned int irq;
};

unsigned int mcspi_slave_read_reg(void __iomem *base, u32 idx)
{
	return ioread32(base + idx);
}

void mcspi_slave_write_reg(void __iomem *base,
		u32 idx, u32 val)
{
	iowrite32(val, base + idx);
}

int mcspi_slave_bytes_per_word(int word_len)
{
	if (word_len <= 8)
		return 1;
	else if (word_len <= 16)
		return 2;
	else
		return 4;
}

int mcspi_slave_wait_for_bit(void __iomem *reg, u32 bit)
{
	unsigned long timeout;

	timeout = jiffies + msecs_to_jiffies(1000);
	while (!(ioread32(reg) & bit)) {
		if (time_after(jiffies, timeout))
			return -ETIMEDOUT;
		else
			return 0;

		cpu_relax();
	}
	return 0;
}

void mcspi_slave_enable(struct mcspi_drv *mcspi)
{
	u32 val;

	val = mcspi_slave_read_reg(mcspi->base, MCSPI_CH0CTRL);
	val |= MCSPI_CHCTRL_EN;
	mcspi_slave_write_reg(mcspi->base, MCSPI_CH0CTRL, val);
}

void mcspi_slave_disable(struct mcspi_drv *mcspi)
{
	u32 val;

	val = mcspi_slave_read_reg(mcspi->base, MCSPI_CH0CTRL);
	val &= ~MCSPI_CHCTRL_EN;
	mcspi_slave_write_reg(mcspi->base, MCSPI_CH0CTRL, val);
}

void mcspi_slave_pio_rx_transfer(unsigned long data)
{

}
DECLARE_TASKLET(pio_rx_tasklet, mcspi_slave_pio_rx_transfer, 0);

void mcspi_slave_pio_tx_transfer(struct mcspi_drv *mcspi)
{

}

irq_handler_t mcspi_slave_irq(unsigned int irq, void *dev_id)
{

	return (irq_handler_t) IRQ_HANDLED;
}

int mcspi_slave_set_irq(struct mcspi_drv *mcspi)
{


	return 0;
}

int mcspi_slave_setup_pio_trnasfer(struct mcspi_drv *mcspi)
{

	return 0;
}

void mcspi_slave_set_mode(struct mcspi_drv *mcspi)
{

}

void mcspi_slave_set_cs(struct mcspi_drv *mcspi)
{

}

int mcspi_slave_setup(struct mcspi_drv *mcspi)
{

	return 0;
}

int mcspi_slave_transfer(struct spislave *slave)
{


	return 0;
}

void mcspi_slave_clear(struct spislave *slave)
{

}

struct omap2_mcspi_platform_config mcspi_slave_pdata = {
	.regs_offset	= OMAP4_MCSPI_REG_OFFSET,
};

const struct of_device_id mcspi_slave_of_match[] = {
	{
		.compatible = "ti,omap4-mcspi-slave",
		.data = &mcspi_slave_pdata,
	},
	{ }
};
MODULE_DEVICE_TABLE(of, mcspi_slave_of_match);

static int mcspi_slave_probe(struct platform_device *pdev)
{
	struct resource	*res;
	struct resource	cp_res;
	const struct of_device_id *match;
	const struct omap2_mcspi_platform_config *pdata;
	struct spislave *slave;
	struct mcspi_drv *mcspi;
	int ret = 0;
	u32 regs_offset = 0;

	u32 cs_sensitive;
	u32 cs_polarity;
	unsigned int pin_dir;
	unsigned int irq;
	unsigned int pha;
	unsigned int pol;

	slave = spislave_alloc_slave(&pdev->dev, sizeof(struct spislave));
	if (slave == NULL)
		return -ENOMEM;

	mcspi = kzalloc(sizeof(*mcspi), GFP_KERNEL);
	slave->spislave_gadget = mcspi;

	match = of_match_device(mcspi_slave_of_match, &pdev->dev);

	if (!match) {
		dev_dbg(&slave->dev, "failed to match, install DTS\n");
		ret = -EINVAL;
		goto free_slave;
	}

	pdata = match->data;

	if (of_get_property(pdev->dev.of_node, "cs_polarity", &cs_polarity))
		cs_polarity = MCSPI_CS_POLARITY_ACTIVE_HIGH;
	else
		cs_polarity = MCSPI_CS_POLARITY_ACTIVE_LOW;

	if (of_get_property(pdev->dev.of_node, "cs_sensitive", &cs_sensitive))
		cs_sensitive = MCSPI_CS_SENSITIVE_DISABLED;
	else
		cs_sensitive = MCSPI_CS_SENSITIVE_ENABLED;

	if (of_get_property(pdev->dev.of_node, "pindir-D0-out-D1-in", &pin_dir))
		pin_dir = MCSPI_PIN_DIR_D0_OUT_D1_IN;
	else
		pin_dir = MCSPI_PIN_DIR_D0_IN_D1_OUT;

	if (of_get_property(pdev->dev.of_node, "pha", &pha))
		pha = MCSPI_PHA_EVEN_NUMBERED_EDGES;
	else
		pha = MCSPI_PHA_ODD_NUMBERED_EDGES;

	if (of_get_property(pdev->dev.of_node, "pol", &pol))
		pol = MCSPI_POL_HELD_LOW;
	else
		pol = MCSPI_POL_HELD_HIGH;

	irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	regs_offset = pdata->regs_offset;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	memcpy(&cp_res, res, sizeof(struct resource));

	if (!res) {
		ret = -ENODEV;
		goto free_slave;
	}

	cp_res.start += regs_offset;
	cp_res.end += regs_offset;

	mcspi->base = devm_ioremap_resource(&pdev->dev, &cp_res);

	if (IS_ERR(mcspi->base)) {
		ret = PTR_ERR(mcspi->base);
		goto free_slave;
	}

	mcspi->cs_polarity = cs_polarity;
	mcspi->cs_sensitive = cs_sensitive;
	mcspi->pin_dir = pin_dir;
	mcspi->irq = irq;
	mcspi->pol = pol;
	mcspi->pha = pha;

	slave->transfer_msg = mcspi_slave_transfer;
	slave->clear_msg = mcspi_slave_clear;

	platform_set_drvdata(pdev, mcspi);

	pm_runtime_use_autosuspend(&pdev->dev);
	pm_runtime_set_autosuspend_delay(&pdev->dev, SPI_AUTOSUSPEND_TIMEOUT);
	pm_runtime_enable(&pdev->dev);

	ret = pm_runtime_get_sync(&pdev->dev);

	if (ret < 0)
		goto disable_pm;

	ret = mcspi_slave_setup(mcspi);
	if (ret < 0)
		goto disable_pm;

	ret = devm_spislave_register_slave(&pdev->dev, slave);
	if (ret) {
		dev_dbg(&slave->dev, "register device error\n");
		goto disable_pm;
	}

	return ret;

disable_pm:
	pm_runtime_dont_use_autosuspend(&pdev->dev);
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

free_slave:

	if (!slave) {
		put_device(&slave->dev);
		mcspi_slave_clear(slave);
	}

	return ret;
}

static int mcspi_slave_remove(struct platform_device *pdev)
{
	tasklet_kill(&pio_rx_tasklet);

	pm_runtime_dont_use_autosuspend(&pdev->dev);
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	return 0;
}

static struct platform_driver mcspi_slave_driver = {
	.probe	= mcspi_slave_probe,
	.remove = mcspi_slave_remove,
	.driver = {
		.name =	DRIVER_NAME,
		.of_match_table = of_match_ptr(mcspi_slave_of_match),
	},
};

module_platform_driver(mcspi_slave_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Patryk Mezydlo, <mezydlo.p@gmail.com>");
MODULE_DESCRIPTION("SPI slave for McSPI controller.");
MODULE_VERSION("1.0");
