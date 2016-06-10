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
#include <linux/spi/spi.h>

#define DRIVER_NAME "spi-mcspi-slave"

#include <linux/platform_data/spi-omap2-mcspi.h>

#define MCSPI_PIN_DIR_D0_IN_D1_OUT	0
#define MCSPI_PIN_DIR_D0_OUT_D1_IN	1
#define MCSPI_CS_POLARITY_ACTIVE_HIGH	1
#define MCSPI_CS_POLARITY_ACTIVE_LOW	0
#define MCSPI_CS_SENSITIVE_ENABLED	1
#define MCSPI_CS_SENSITIVE_DISABLED	0

#define SPI_MCSPI_SLAVE_FIFO_DEPTH	32
#define SPI_MCSPI_SLAVE_BITS_PER_WORD	8
#define SPI_MCSPI_SLAVE_CS_SENSITIVE	MCSPI_CS_SENSITIVE_ENABLED
#define SPI_MCSPI_SLAVE_CS_POLARITY	MCSPI_CS_POLARITY_ACTIVE_LOW
#define SPI_MCSPI_SLAVE_PIN_DIR		MCSPI_PIN_DIR_D0_IN_D1_OUT

#define MCSPI_SYSCONFIG			0x10
#define MCSPI_SYSSTATUS			0x14
#define MCSPI_IRQSTATUS			0x18
#define MCSPI_IRQENABLE			0x1C
#define MCSPI_SYST			0x24
#define MCSPI_MODULCTRL			0x28
#define MCSPI_CH0CONF			0x2C
#define MCSPI_CH0STAT			0x30
#define MCSPI_CH0CTRL			0x34
#define MCSPI_TX0			0x38
#define MCSPI_RX0			0x3C
#define MCSPI_CH1CONF			0x40
#define MCSPI_CH1STAT			0x44
#define MCSPI_CH1CTRL			0x48
#define MCSPI_TX1			0x4C
#define MCSPI_RX1			0x50
#define MCSPI_CH2CONF			0x54
#define MCSPI_CH2STAT			0x58
#define MCSPI_CH2CTRL			0x5C
#define MCSPI_TX2			0x60
#define MCSPI_RX2			0x64
#define MCSPI_CH3CONF			0x68
#define MCSPI_CH3STAT			0x6C
#define MCSPI_CH3CTRL			0x70
#define MCSPI_TX3			0x74
#define MCSPI_RX3			0x78
#define MCSPI_XFERLEVEL			0x7C
#define MCSPI_DAFTX			0x80
#define MCSPI_DAFRX			0xA0

#define SPI_AUTOSUSPEND_TIMEOUT		2000

#define MCSPI_SYSSTATUS_RESETDONE	BIT(0)
#define MCSPI_MODULCTRL_MS		BIT(2)
#define MCSPI_MODULCTRL_PIN34		BIT(1)
#define MCSPI_CHCTRL_EN			BIT(0)
#define MCSPI_CHCONF_EPOL		BIT(6)
#define MCSPI_CHCONF_TRM		(0x03 << 12)
#define MCSPI_CHCONF_WL			(0x1F << 7)

#define MCSPI_CHCONF_WL_8BIT_MASK	(0x07 << 7)
#define MCSPI_CHCONF_WL_16BIT_MASK	(0x0F << 7)
#define MCSPI_CHCONF_WL_32BIT_MASK	(0x1F << 7)

#define MCSPI_CHCONF_IS			BIT(18)
#define MCSPI_CHCONF_DPE0		BIT(16)
#define MCSPI_CHCONF_DPE1		BIT(17)

#define MCSPI_IRQ_RX0_OVERFLOW		BIT(3)
#define MCSPI_IRQ_RX0_FULL		BIT(2)
#define MCSPI_IRQ_TX0_UNDERFLOW		BIT(1)
#define MCSPI_IRQ_TX0_EMPTY		BIT(0)
#define MCSPI_IRQ_EOWKE			BIT(17)

/*
 * this structure describe a device
 *
 */

struct spi_slave {
	struct	device			*dev;
	void	__iomem			*base;
	struct	spi_transfer		*spi_transfer;
	u32				start;
	u32				end;
	unsigned int			reg_offset;
	u32				bits_per_word;
	u32				fifo_depth;
	u32				cs_sensitive;
	u32				cs_polarity;
	unsigned int			irq;
	unsigned int			pin_dir;
};

static inline unsigned int mcspi_slave_read_reg(void __iomem *base, u32 idx)
{
	return readl_relaxed(base + idx);
}

static inline void mcspi_slave_write_reg(void __iomem *base,
		u32 idx, u32 val)
{
	writel_relaxed(val, base + idx);
}

static int mcspi_slave_setup_pio_transfer(struct spi_slave *slave)
{
	struct spi_transfer		*spi_transfer;
	u32				l;
	int				ret;

	spi_transfer = kzalloc(sizeof(struct spi_transfer), GFP_KERNEL);

	if (spi_transfer == NULL)
		return -ENOMEM;

	slave->spi_transfer = spi_transfer;

	spi_transfer->tx_buf = kzalloc(spi_transfer->len, GFP_KERNEL);
	if (spi_transfer->tx_buf == NULL)
		return -ENOMEM;

	spi_transfer->rx_buf = kzalloc(spi_transfer->len, GFP_KERNEL);
	if (spi_transfer->rx_buf == NULL)
		return -ENOMEM;

	return ret;
}

static void mcspi_slave_pio_transfer(struct spi_slave *slave)
{
	void __iomem			*rx_reg;
	void __iomem			*tx_reg;

	rx_reg = slave->base + MCSPI_TX0;
	tx_reg = slave->base + MCSPI_RX0;

}

static void mcspi_slave_enable(struct spi_slave *slave)
{
	u32		l;

	pr_info("%s: spi is enabled\n", DRIVER_NAME);
	l = mcspi_slave_read_reg(slave->base, MCSPI_CH0CTRL);

	/*set bit(0) in ch0ctrl, spi is enabled*/
	l |= MCSPI_CHCTRL_EN;
	pr_info("%s: MCSPI_CH0CTRL:0x%x\n", DRIVER_NAME, l);

	mcspi_slave_write_reg(slave->base, MCSPI_CH0CTRL, l);
}

static void mcspi_slave_disable(struct spi_slave *slave)
{
	u32		l;

	pr_info("%s: spi is disabled", DRIVER_NAME);
	l = mcspi_slave_read_reg(slave->base, MCSPI_CH0CTRL);

	/*clr bit(0) in ch0ctrl, spi is enabled*/
	l &= ~MCSPI_CHCTRL_EN;
	pr_info("%s: MCSPI_CH0CTRL:0x%x\n", DRIVER_NAME, l);

	mcspi_slave_write_reg(slave->base, MCSPI_CH0CTRL, l);
}

static void mcspi_slave_set_slave_mode(struct spi_slave *slave)
{
	u32		l;

	pr_info("%s: set slave mode\n", DRIVER_NAME);

	l = mcspi_slave_read_reg(slave->base, MCSPI_MODULCTRL);

	/*set bit(2) in modulctrl, spi is set in slave mode*/
	l |= MCSPI_MODULCTRL_MS;

	pr_info("%s: MCSPI_MODULCTRL:0x%x\n", DRIVER_NAME, l);


	/*
	 * clr bit(13 and 12) in chconf,
	 * spi is set in transmit and receive mode
	 */
	mcspi_slave_write_reg(slave->base, MCSPI_MODULCTRL, l);

	l = mcspi_slave_read_reg(slave->base, MCSPI_CH0CONF);
	l &= ~MCSPI_CHCONF_TRM;

	/*
	 * available is only 8 16 and 32 bits per word
	 * before setting clear all WL bits
	 */
	l &= ~MCSPI_CHCONF_WL;

	if (slave->bits_per_word == 32)
		l |= MCSPI_CHCONF_WL_8BIT_MASK;
	else if (slave->bits_per_word == 16)
		l |= MCSPI_CHCONF_WL_16BIT_MASK;
	else
		l |= MCSPI_CHCONF_WL_8BIT_MASK;

	/*setting a line which is selected for reception */
	if (slave->pin_dir == MCSPI_PIN_DIR_D0_IN_D1_OUT) {
		l &= ~MCSPI_CHCONF_IS;
		l &= ~MCSPI_CHCONF_DPE1;
		l |= MCSPI_CHCONF_DPE0;
	} else {
		l |= MCSPI_CHCONF_IS;
		l |= MCSPI_CHCONF_DPE1;
		l &= ~MCSPI_CHCONF_DPE0;
	}

	pr_info("%s: MCSPI_CH0CONF:0x%x\n", DRIVER_NAME, l);
	mcspi_slave_write_reg(slave->base, MCSPI_CH0CONF, l);
}

static void mcspi_slave_set_cs(struct spi_slave *slave)
{
	u32		l;

	pr_info("%s: set cs sensitive and polarity\n", DRIVER_NAME);

	l = mcspi_slave_read_reg(slave->base, MCSPI_CH0CONF);

	/*cs polatiry
	 * when cs_polarity is 0: MCSPI is enabled when cs line is 0
	 * (set EPOL bit)
	 * when cs_polarity is 1: MCSPI is enabled when cs line is 1
	 * (clr EPOL bit)
	 */
	if (slave->cs_polarity == MCSPI_CS_POLARITY_ACTIVE_LOW)
		l |= MCSPI_CHCONF_EPOL;
	else
		l &= ~MCSPI_CHCONF_EPOL;

	pr_info("%s: MCSPI_CH0CONF:0x%x\n", DRIVER_NAME, l);
	mcspi_slave_write_reg(slave->base, MCSPI_CH0CONF, l);
	l = mcspi_slave_read_reg(slave->base, MCSPI_MODULCTRL);
	/*
	 * set bit(1) in modulctrl, spi wtihout cs line, only enabled
	 * clear bit(1) in modulctrl, spi with cs line,
	 * enable if cs is set
	 */
	if (slave->cs_sensitive == MCSPI_CS_SENSITIVE_ENABLED)
		l &= ~MCSPI_MODULCTRL_PIN34;
	else
		l |= MCSPI_MODULCTRL_PIN34;

	pr_info("%s: MCSPI_MODULCTRL:0x%x\n", DRIVER_NAME, l);
	mcspi_slave_write_reg(slave->base, MCSPI_MODULCTRL, l);
}

static irq_handler_t mcspi_slave_irq(unsigned int irq, void *dev_id)
{
	struct spi_slave *slave = dev_id;

	(void)slave;

	pr_info("%s: interrupt\n", DRIVER_NAME);

	return (irq_handler_t) IRQ_HANDLED;
}

static int mcspi_slave_set_irq(struct spi_slave *slave)
{
	int		ret = 0;
	u32		l;

	pr_info("%s: set interrupt\n", DRIVER_NAME);

	l = mcspi_slave_read_reg(slave->base, MCSPI_IRQENABLE);

	l |= MCSPI_IRQ_RX0_FULL;
	l |= MCSPI_IRQ_RX0_OVERFLOW;

	pr_info("%s: MCSPI_IRQENABLE:0x%x\n", DRIVER_NAME, l);

	mcspi_slave_write_reg(slave->base, MCSPI_IRQENABLE, l);

	return ret;
}

static int mcspi_slave_setup(struct spi_slave *slave)
{
	int		ret = 0;
	u32		l;

	pr_info("%s: slave setup\n", DRIVER_NAME);

	ret = pm_runtime_get_sync(slave->dev);
	if (ret < 0)
		return ret;

	/*verification status bit(0) in MCSPI system status register*/
	l = mcspi_slave_read_reg(slave->base, MCSPI_SYSSTATUS);

	pr_info("%s: MCSPI_SYSSTATUS:0x%x\n", DRIVER_NAME, l);

	if (l & MCSPI_SYSSTATUS_RESETDONE) {
		pr_info("%s: controller ready for setting\n",
			DRIVER_NAME);

		/*here set mcspi controller in slave mode and more setting*/
		mcspi_slave_disable(slave);
		mcspi_slave_set_slave_mode(slave);
		mcspi_slave_set_cs(slave);
		ret = mcspi_slave_set_irq(slave);
		mcspi_slave_setup_pio_transfer(slave);
		mcspi_slave_enable(slave);


		if (ret < 0)
			return ret;
	} else
		pr_info("%s: internal module reset is on-going\n",
			DRIVER_NAME);

	return ret;
}

 /* default platform value located in .h file*/
static struct omap2_mcspi_platform_config mcspi_slave_pdata = {
	.regs_offset	= OMAP4_MCSPI_REG_OFFSET,
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
	const struct omap2_mcspi_platform_config	*pdata;

	int						ret = 0;
	u32						regs_offset = 0;

	struct spi_slave				*slave;

	u32						fifo_depth;
	u32						bits_per_word;
	u32						cs_sensitive;
	u32						cs_polarity;
	unsigned int					pin_dir;
	unsigned int					irq;

	pr_info("%s: Entry probe\n", DRIVER_NAME);

	dev  = &pdev->dev;
	node = pdev->dev.of_node;

	slave = kzalloc(sizeof(struct spi_slave), GFP_KERNEL);

	if (slave == NULL)
		return -ENOMEM;

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
		fifo_depth = SPI_MCSPI_SLAVE_FIFO_DEPTH;
		bits_per_word = SPI_MCSPI_SLAVE_BITS_PER_WORD;

		of_property_read_u32(node, "fifo_depth", &fifo_depth);
		of_property_read_u32(node, "bits_per_word", &bits_per_word);

		if (of_get_property(node, "cs_polarity", &cs_polarity))
			cs_polarity = MCSPI_CS_POLARITY_ACTIVE_HIGH;
		else
			cs_polarity = MCSPI_CS_POLARITY_ACTIVE_LOW;

		if (of_get_property(node, "cs_sensitive", &cs_sensitive))
			cs_sensitive = MCSPI_CS_SENSITIVE_DISABLED;
		else
			cs_sensitive = MCSPI_CS_SENSITIVE_ENABLED;

		if (of_get_property(node, "pindir-D0-out-D1-in", &pin_dir))
			pin_dir = MCSPI_PIN_DIR_D0_OUT_D1_IN;
		else
			pin_dir = MCSPI_PIN_DIR_D0_IN_D1_OUT;

		irq = irq_of_parse_and_map(node, 0);

	} else {
		pdata = dev_get_platdata(&pdev->dev);
		pr_err("%s: failed to match, install DTS", DRIVER_NAME);
		return -EINVAL;
	}

	regs_offset = pdata->regs_offset;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	/*copy resources because base address is changed*/
	memcpy(&cp_res, res, sizeof(struct resource));

	if (res == NULL) {
		pr_err("%s: res not availablee\n", DRIVER_NAME);
		return -ENODEV;
	}

	/* driver is increment allways when omap2 driver is install
	 * base addres is not correct when install a driver more times
	 * but when resources is copied it's ok
	 */
	cp_res.start += regs_offset;
	cp_res.end   += regs_offset;

	slave->base = devm_ioremap_resource(&pdev->dev, &cp_res);

	if (IS_ERR(slave->base)) {
		pr_err("%s: base addres ioremap error!!", DRIVER_NAME);
		ret = PTR_ERR(slave->base);
		return -ENODEV;
	}

	slave->dev			= dev;
	slave->fifo_depth		= fifo_depth;
	slave->cs_polarity		= cs_polarity;
	slave->start			= cp_res.start;
	slave->end			= cp_res.end;
	slave->reg_offset		= regs_offset;
	slave->bits_per_word		= bits_per_word;
	slave->cs_sensitive		= cs_sensitive;
	slave->pin_dir			= pin_dir;
	slave->irq			= irq;

	platform_set_drvdata(pdev, slave);

	pr_info("%s: start:%x\n", DRIVER_NAME, slave->start);
	pr_info("%s: end:%x\n", DRIVER_NAME, slave->end);
	pr_info("%s: regs_offset=%x\n", DRIVER_NAME, slave->reg_offset);
	pr_info("%s: memory_depth=%d\n", DRIVER_NAME, slave->fifo_depth);
	pr_info("%s: bits_per_word=%d\n", DRIVER_NAME, slave->bits_per_word);
	pr_info("%s: cs_sensitive=%d\n", DRIVER_NAME, slave->cs_sensitive);
	pr_info("%s: cs_polarity=%d\n", DRIVER_NAME, slave->cs_polarity);
	pr_info("%s: pin_dir=%d\n", DRIVER_NAME, slave->pin_dir);
	pr_info("%s: interrupt:%d\n", DRIVER_NAME, slave->irq);

	pm_runtime_use_autosuspend(&pdev->dev);
	pm_runtime_set_autosuspend_delay(&pdev->dev, SPI_AUTOSUSPEND_TIMEOUT);
	pm_runtime_enable(&pdev->dev);

	ret = mcspi_slave_setup(slave);

	pm_runtime_dont_use_autosuspend(&pdev->dev);
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	ret = request_irq(slave->irq, (irq_handler_t)mcspi_slave_irq, 0,
			  DRIVER_NAME, slave);

	if (ret)
		pr_info("%s: unable to request irq:%d\n", DRIVER_NAME,
			slave->irq);

	return ret;
}

static int mcspi_slave_remove(struct platform_device *pdev)
{
	struct spi_slave	*slave;

	slave = platform_get_drvdata(pdev);

	free_irq(slave->irq, slave);
	kfree(slave);

	pr_info("%s: remove\n", DRIVER_NAME);
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

	pr_info("%s: init\n", DRIVER_NAME);
	ret = platform_driver_register(&mcspi_slave_driver);

	if (ret == 0)
		pr_info("%s: platform driver register ok\n",
			DRIVER_NAME);
	else
		pr_err("%s: platform driver register error\n",
		       DRIVER_NAME);

	return ret;
}

static void __exit mcspi_slave_exit(void)
{
	platform_driver_unregister(&mcspi_slave_driver);

	pr_info("%s: exit\n", DRIVER_NAME);
}

module_init(mcspi_slave_init);
module_exit(mcspi_slave_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Patryk Mezydlo");
MODULE_DESCRIPTION("SPI slave for McSPI controller.");
MODULE_VERSION("1.0");
