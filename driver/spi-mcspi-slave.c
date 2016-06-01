#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/io.h>

#define DRIVER_NAME "spi-mcspi-slave"

#include "spi-mcspi-slave.h"

#define MCSPI_SYSCONFIG 0x10
#define MCSPI_SYSSTATUS 0x14
#define MCSPI_IRQSTATUS 0x18
#define MCSPI_IRQENABLE 0x1C
#define MCSPI_SYST	0x24
#define MCSPI_MODULCTRL	0x28
#define MCSPI_CH0CONF	0x2C
#define MCSPI_CH0STAT	0x30
#define MCSPI_CH0CTRL	0x34
#define MCSPI_TX0	0x38
#define MCSPI_RX0	0x3C
#define MCSPI_CH1CONF	0x40
#define MCSPI_CH1STAT	0x44
#define MCSPI_CH1CTRL	0x48
#define MCSPI_TX1	0x4C
#define MCSPI_RX1	0x50
#define MCSPI_CH2CONF	0x54
#define MCSPI_CH2STAT	0x58
#define MCSPI_CH2CTRL	0x5C
#define MCSPI_TX2	0x60
#define MCSPI_RX2	0x64
#define MCSPI_CH3CONF	0x68
#define MCSPI_CH3STAT	0x6C
#define MCSPI_CH3CTRL	0x70
#define MCSPI_TX3	0x74
#define MCSPI_RX3	0x78
#define MCSPI_XFERLEVEL	0x7C
#define MCSPI_DAFTX	0x80
#define MCSPI_DAFRX	0xA0

/*
 * this structure describe a device
 *
 */
struct spi_slave {
	struct	device		*dev;
	void	__iomem		*base;
	int			fifo_depth;
	int			num_cs;
	u32			start;
	u32			end;
	u32			reg_offset;
	u8			bits_per_word;
	void			*TX_buf;
	void			*RX_buf;
};

static inline unsigned int mcspi_slave_read_reg(void __iomem *base, int idx)
{
	return readl_relaxed(&base + idx);
}

static inline void mcspi_slave_write_reg(void __iomem *base,
		int idx, unsigned int val)
{
	writel_relaxed(val, &base + idx);
}

static int mcspi_slave_setup(struct spi_slave *slave)
{
	int		ret = 0;

	/*here set mcspi controller in slave mode and more setting*/

	return ret;
}

 /* default platform value located in .h file*/
static struct mcspi_slave_platform_config mcspi_slave_pdata = {
	.regs_offset	= OMAP4_MCSPI_SLAVE_REG_OFFSET,
	.fifo_depth	= SPI_MCSPI_SLAVE_FIFO_DEPTH,
	.num_cs		= SPI_MCSPI_SLAVE_NUM_CS,
};

static const struct of_device_id mcspi_slave_of_match[] = {
	{
		.compatible = "ti,omap4-mcspi",
		.data = &mcspi_slave_pdata,
	},
	{ }
};
MODULE_DEVICE_TABLE(of, mcspi_slave_of_match);

static int mcspi_slave_probe(struct platform_device *pdev)
{
	struct device					*dev;
	struct device_node				*node;

	struct resource					*res;
	struct resource					cp_res;
	const struct of_device_id			*match;
	const struct mcspi_slave_platform_config	*pdata;

	int						ret = 0;
	u32						regs_offset;

	struct spi_slave				*slave;

	unsigned int					fifo_depth;
	unsigned int					num_cs;

	void __iomem					*base;

	pr_info("mcspi_slave: Entry probe\n");

	dev  = &pdev->dev;
	node = pdev->dev.of_node;

	slave = kzalloc(sizeof(struct spi_slave), GFP_KERNEL);

	if (slave == NULL) {
		/*pr_err("slave allocation failed\n");*/
		return -ENOMEM;
	}

	/*
	 * here allocate memory for slave structure
	 * I have to write the structure of the slave device
	 * I don't know what to put in it
	 *
	 * and here I have to fill this structure
	 *
	 *  tell if an of_device structure has a metching
	 */

	match = of_match_device(mcspi_slave_of_match, dev);

	if (match) {/* user setting from dts*/
		pdata = match->data;

		/*
		 *default value num_cs and memory_depth
		 *when this value is not define in dts
		 */

		num_cs = 1;
		fifo_depth = 32;

		of_property_read_u32(node, "memory_depth", &fifo_depth);

		of_property_read_u32(node, "ti,spi-num-cs", &num_cs);

	} else {/*default setting from pdata*/
		pdata = dev_get_platdata(&pdev->dev);
		fifo_depth = pdata->fifo_depth;
		num_cs = pdata->num_cs;
	}

	regs_offset = pdata->regs_offset;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	/*copy resources because base address is changed*/
	memcpy(&cp_res, res, sizeof(struct resource));

	if (res == NULL) {
		pr_dbg("res not availablee\n");
		return -ENODEV;
	}

	/* driver is increment allways when omap2 driver is install
	 * base addres is not correct when install a driver more times
	 * but when resources is copied it's ok
	 */
	cp_res.start += regs_offset;
	cp_res.end   += regs_offset;

	base = devm_ioremap_resource(&pdev->dev, &cp_res);

	if (IS_ERR(&base)) {
		pr_err("base addres ioremap error!!");
		return -ENODEV;
	}

	slave->base		= base;
	slave->dev		= dev;
	slave->fifo_depth	= fifo_depth;
	slave->num_cs		= num_cs;
	slave->start		= cp_res.start;
	slave->end		= cp_res.end;
	slave->reg_offset	= regs_offset;

	platform_set_drvdata(pdev, slave);

	mcspi_slave_setup(slave);

	return ret;
}

static int mcspi_slave_remove(struct platform_device *pdev)
{
	struct spi_slave *slave;

	slave = platform_get_drvdata(pdev);

	pr_info("start:%x\n",	slave->start);
	pr_info("end:%x\n", slave->end);
	pr_info("regs_offset=%d\n", slave->reg_offset);
	pr_info("memory_depth=%d\n", slave->fifo_depth);
	pr_info("num_cs=%d\n", slave->num_cs);

	kfree(slave);

	pr_info("mcspi_slave: remove\n");
	return 0;
}

static struct platform_driver mcspi_slave_driver = {
	.probe =	mcspi_slave_probe,
	.remove =	mcspi_slave_remove,
	.driver = {
		.name =	DRIVER_NAME,
		.of_match_table = of_match_ptr(mcspi_slave_of_match),
	},
};

static int __init mcspi_slave_init(void)
{
	int ret;

	pr_info("mcspi_slave: init\n");
	ret = platform_driver_register(&mcspi_slave_driver);

	if (ret != 0)
		pr_info("mcspi_slave: platform driver register ok\n");
	else
		pr_err("mcspi_slave: platform driver register error\n");

	return ret;
}

static void __exit mcspi_slave_exit(void)
{
	platform_driver_unregister(&mcspi_slave_driver);

	pr_info("mcspi_slave: exit\n");
}

module_init(mcspi_slave_init);
module_exit(mcspi_slave_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Patryk Mezydlo");
MODULE_DESCRIPTION("SPI slave for McSPI controller.");
MODULE_VERSION("1.0");
