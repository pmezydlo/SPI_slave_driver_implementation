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
#include <linux/of.h>
#include <linux/fs.h>
#include <linux/ioctl.h>
#include <asm/uaccess.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/sched.h>

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

#define MCSPI_MODE_TRM				0
#define MCSPI_MODE_RM				1
#define MCSPI_MODE_TM				2

#define SPI_SLAVE_BUF_DEPTH			64
#define SPI_SLAVE_BITS_PER_WORD			8
#define SPII_SLAVE_CS_SENSITIVE			MCSPI_CS_SENSITIVE_ENABLED
#define SPI_SLAVE_CS_POLARITY			MCSPI_CS_POLARITY_ACTIVE_LOW
#define SPI_SLAVE_PIN_DIR			MCSPI_PIN_DIR_D0_IN_D1_OUT
#define SPI_SLAVE_MODE				MCSPI_MODE_TRM
#define SPI_SLAVE_COPY_LENGTH			1

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
#define	MCSPI_DAFRX				0xA0

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
#define MCSPI_CHSTAT_RXS			BIT(0)
#define MCSPI_CHSTAT_RXFFF			BIT(6)
#define MCSPI_CHSTAT_RXFFE			BIT(5)
#define MCSPI_CHSTAT_TXFFF			BIT(4)
#define MCSPI_CHSTAT_TXFFE			BIT(3)

static inline unsigned int mcspi_slave_read_reg(void __iomem *base, u32 idx)
{
	return ioread32(base + idx);
}

static inline void mcspi_slave_write_reg(void __iomem *base,
		u32 idx, u32 val)
{
	iowrite32(val, base + idx);
}

static inline int mcspi_slave_bytes_per_word(int word_len)
{
	if (word_len <= 8)
		return 1;
	else if (word_len <= 16)
		return 2;
	else
		return 4;
}

static int mcspi_slave_wait_for_bit(void __iomem *reg, u32 bit)
{
	unsigned long				timeout;

	timeout = jiffies + msecs_to_jiffies(1000);
	while (!(ioread32(reg) & bit)) {
		if (time_after(jiffies, timeout)) {
			if (!(ioread32(reg) & bit)) {
				pr_err("%s: mcspi timeout!!!\n", DRIVER_NAME);
				return -ETIMEDOUT;
			} else
				return 0;
		}
		cpu_relax();
	}
	return 0;
}

static void mcspi_slave_enable(struct spi_slave *slave)
{
	u32					l;

	pr_info("%s: spi is enabled\n", DRIVER_NAME);
	l = mcspi_slave_read_reg(slave->base, MCSPI_CH0CTRL);

	l |= MCSPI_CHCTRL_EN;
	pr_info("%s: MCSPI_CH0CTRL:0x%x\n", DRIVER_NAME, l);

	mcspi_slave_write_reg(slave->base, MCSPI_CH0CTRL, l);
}

static void mcspi_slave_disable(struct spi_slave *slave)
{
	u32					l;

	pr_info("%s: spi is disabled\n", DRIVER_NAME);
	l = mcspi_slave_read_reg(slave->base, MCSPI_CH0CTRL);

	l &= ~MCSPI_CHCTRL_EN;
	pr_info("%s: MCSPI_CH0CTRL:0x%x\n", DRIVER_NAME, l);

	mcspi_slave_write_reg(slave->base, MCSPI_CH0CTRL, l);
}

static void mcspi_slave_pio_rx_transfer(unsigned long data)
{
	struct spi_slave			*slave;
	unsigned int				c;
	void __iomem				*rx_reg;
	void __iomem				*chstat;

	slave = (struct spi_slave *) data;

	rx_reg = slave->base + MCSPI_RX0;
	chstat = slave->base + MCSPI_CH0STAT;

	c = slave->bytes_per_load;
	c /= mcspi_slave_bytes_per_word(slave->bits_per_word);

	if (slave->rx_offset >= slave->buf_depth) {
		pr_err("%s: end of rx buffer!!!", DRIVER_NAME);
		slave->rx_offset = 0;
		return;
	}

	if (mcspi_slave_bytes_per_word(slave->bits_per_word) == 1) {
	u8 *rx;

	rx = slave->rx + slave->rx_offset;
	slave->rx_offset += (sizeof(u8) * c);

		do {
			c -= 1;
			if (mcspi_slave_wait_for_bit(chstat, MCSPI_CHSTAT_RXS)
						     < 0)
				goto out;

			*rx++ = readl_relaxed(rx_reg);
		} while (c);
	}

	if (mcspi_slave_bytes_per_word(slave->bits_per_word) == 2) {
	u16 *rx;

	rx = slave->rx + slave->rx_offset;
	slave->rx_offset += (sizeof(u16) * c);

		do {
			c -= 1;
			if (mcspi_slave_wait_for_bit(chstat, MCSPI_CHSTAT_RXS)
						     < 0)
				goto out;

			*rx++ = readl_relaxed(rx_reg);
		} while (c);
	}

	if (mcspi_slave_bytes_per_word(slave->bits_per_word) == 4) {
	u32 *rx;

	rx = slave->rx + slave->rx_offset;
	slave->rx_offset += (sizeof(u32) * c);

		do {
			c -= 1;
			if (mcspi_slave_wait_for_bit(chstat, MCSPI_CHSTAT_RXS)
						     < 0)
				goto out;

			*rx++ = readl_relaxed(rx_reg);
		} while (c);
	}

	return;
out:
	pr_err("%s: timeout!!!", DRIVER_NAME);
}
DECLARE_TASKLET(pio_rx_tasklet, mcspi_slave_pio_rx_transfer, 0);

static void mcspi_slave_pio_tx_transfer(struct spi_slave *slave)
{
	unsigned int				c;
	void __iomem				*tx_reg;
	void __iomem				*chstat;

	tx_reg = slave->base + MCSPI_TX0;
	chstat = slave->base + MCSPI_CH0STAT;

	if (slave->mode == MCSPI_MODE_TM)
		c = MCSPI_MAX_FIFO_DEPTH;
	else
		c = MCSPI_MAX_FIFO_DEPTH / 2;

	c /= mcspi_slave_bytes_per_word(slave->bits_per_word);

	if (slave->tx_offset >= slave->buf_depth) {
		pr_err("%s: end of tx buffer!!!", DRIVER_NAME);
		slave->tx_offset = 0;
		return;
	}

	if (mcspi_slave_bytes_per_word(slave->bits_per_word) == 1) {
	const u8 *tx;

	tx = slave->tx + slave->tx_offset;
	slave->tx_offset += (sizeof(u8) * c);

		do {
			c -= 1;
			if (mcspi_slave_wait_for_bit(chstat, MCSPI_CHSTAT_TXS)
						     < 0)
				goto out;

			writel_relaxed(*tx++, tx_reg);
		} while (c);
	}

	if (mcspi_slave_bytes_per_word(slave->bits_per_word) == 2) {
	const u16 *tx;

	tx = slave->tx + slave->tx_offset;
	slave->tx_offset += (sizeof(u16) * c);
		do {
			c -= 1;
			if (mcspi_slave_wait_for_bit(chstat, MCSPI_CHSTAT_TXS)
						     < 0)
				goto out;

			writel_relaxed(*tx++, tx_reg);
		} while (c);
	}

	if (mcspi_slave_bytes_per_word(slave->bits_per_word) == 4) {
	const u32 *tx;

	tx = slave->tx + slave->tx_offset;
	slave->tx_offset += (sizeof(u32) * c);

		do {
			c -= 1;
			if (mcspi_slave_wait_for_bit(chstat, MCSPI_CHSTAT_TXS)
						     < 0)
				goto out;

			writel_relaxed(*tx++, tx_reg);
		} while (c);
	}

	return;
out:
	pr_err("%s: timeout!!!", DRIVER_NAME);
}

static irq_handler_t mcspi_slave_irq(unsigned int irq, void *dev_id)
{
	struct spi_slave			*slave = dev_id;
	u32					l;

	l = mcspi_slave_read_reg(slave->base, MCSPI_CH0STAT);

	if (l & MCSPI_CHSTAT_EOT) {
		pr_info("%s: end of transfer is set\n", DRIVER_NAME);
		wake_up_interruptible(&slave->wait);
		mcspi_slave_disable(slave);
	} else
		pr_info("%s: end of transfer is clr\n", DRIVER_NAME);

	l = mcspi_slave_read_reg(slave->base, MCSPI_IRQSTATUS);

	if (l & MCSPI_IRQ_RX_FULL) {
		l |= MCSPI_IRQ_RX_FULL;
		pio_rx_tasklet.data = (unsigned long)slave;
		tasklet_schedule(&pio_rx_tasklet);
	}

	mcspi_slave_write_reg(slave->base, MCSPI_IRQSTATUS, l);

	return (irq_handler_t) IRQ_HANDLED;
}

static int mcspi_slave_set_irq(struct spi_slave *slave)
{
	u32					l;
	int					ret = 0;

	pr_info("%s: set interrupt\n", DRIVER_NAME);

	l = mcspi_slave_read_reg(slave->base, MCSPI_IRQENABLE);

	l &= ~MCSPI_IRQ_RX_FULL;
	l &= ~MCSPI_IRQ_TX_EMPTY;


	l |= MCSPI_IRQ_RX_FULL;

	pr_info("%s: MCSPI_IRQENABLE:0x%x\n", DRIVER_NAME, l);

	mcspi_slave_write_reg(slave->base, MCSPI_IRQENABLE, l);

	ret = devm_request_irq(&slave->dev, slave->irq,
				(irq_handler_t)mcspi_slave_irq,
				IRQF_TRIGGER_NONE,
				DRIVER_NAME, slave);
	if (ret) {
		pr_err("%s: unable to request irq:%d\n", DRIVER_NAME,
			slave->irq);
		ret = -EINTR;
	}

	return ret;
}

static int mcspi_slave_setup_pio_transfer(struct spi_slave *slave)
{
	u32					l;
	int					ret = 0;

	pr_info("%s: pio transfer setup\n", DRIVER_NAME);

	pr_info("%s: mode:%d\n", DRIVER_NAME, slave->mode);
	pr_info("%s: bits_per_word:%x\n", DRIVER_NAME, slave->bits_per_word);
	pr_info("%s: bytes_per_load:%d\n", DRIVER_NAME, slave->bytes_per_load);
	pr_info("%s: buf_depth:%d\n", DRIVER_NAME, slave->buf_depth);

	if (slave->mode == MCSPI_MODE_TM || slave->mode == MCSPI_MODE_TRM) {
		slave->tx = kzalloc(slave->buf_depth, GFP_KERNEL);
		if (slave->tx == NULL)
			return -ENOMEM;
	}

	if (slave->mode == MCSPI_MODE_RM || slave->mode == MCSPI_MODE_TRM) {
		slave->rx = kzalloc(slave->buf_depth, GFP_KERNEL);
		if (slave->rx == NULL)
			return -ENOMEM;
	}

	l = mcspi_slave_read_reg(slave->base, MCSPI_XFERLEVEL);

	l &= ~MCSPI_XFER_AEL;
	l &= ~MCSPI_XFER_AFL;

	if (slave->mode == MCSPI_MODE_RM || slave->mode == MCSPI_MODE_TRM)
		l  |= (slave->bytes_per_load - 1) << 8;

	l &= ~MCSPI_XFER_WCNT;

	mcspi_slave_write_reg(slave->base, MCSPI_XFERLEVEL, l);

	pr_info("%s: MCSPI_XFERLEVEL:0x%x\n", DRIVER_NAME, l);

	l = mcspi_slave_read_reg(slave->base, MCSPI_CH0CONF);

	l &= ~MCSPI_CHCONF_TRM;

	if (slave->mode == MCSPI_MODE_RM)
		l |= MCSPI_CHCONF_RM;
	else if (slave->mode == MCSPI_MODE_TM)
		l |= MCSPI_CHCONF_TM;

	l &= ~MCSPI_CHCONF_WL;
	l |= (slave->bits_per_word - 1) << 7;

	l &= ~MCSPI_CHCONF_FFER;
	l &= ~MCSPI_CHCONF_FFEW;

	if (slave->mode == MCSPI_MODE_RM || slave->mode == MCSPI_MODE_TRM)
		l |= MCSPI_CHCONF_FFER;

	if (slave->mode == MCSPI_MODE_TM || slave->mode == MCSPI_MODE_TRM)
		l |= MCSPI_CHCONF_FFEW;

	mcspi_slave_write_reg(slave->base, MCSPI_CH0CONF, l);
	pr_info("%s: MCSPI_CH0CONF:0x%x\n", DRIVER_NAME, l);

	l = mcspi_slave_read_reg(slave->base, MCSPI_MODULCTRL);

	l &= ~MCSPI_MODULCTRL_FDAA;

	mcspi_slave_write_reg(slave->base, MCSPI_MODULCTRL, l);
	pr_info("%s: MCSPI_MODULCTRL:0x%x\n", DRIVER_NAME, l);

	return ret;
}

static int mcspi_slave_clr_pio_transfer(struct spi_slave *slave)
{
	int					ret = 0;

	pr_info("%s: clear transfer", DRIVER_NAME);

	if (slave->tx != NULL)
		kfree(slave->tx);

	if (slave->rx != NULL)
		kfree(slave->rx);

	mcspi_slave_disable(slave);

	return ret;
}

static void mcspi_slave_set_slave_mode(struct spi_slave *slave)
{
	u32					l;

	pr_info("%s: set slave mode\n", DRIVER_NAME);

	l = mcspi_slave_read_reg(slave->base, MCSPI_MODULCTRL);

	l |= MCSPI_MODULCTRL_MS;

	pr_info("%s: MCSPI_MODULCTRL:0x%x\n", DRIVER_NAME, l);

	mcspi_slave_write_reg(slave->base, MCSPI_MODULCTRL, l);

	l = mcspi_slave_read_reg(slave->base, MCSPI_CH0CONF);

	l &= ~MCSPI_CHCONF_PHA;
	l &= ~MCSPI_CHCONF_POL;

	if (slave->pin_dir == MCSPI_PIN_DIR_D0_IN_D1_OUT) {
		l &= ~MCSPI_CHCONF_IS;
		l &= ~MCSPI_CHCONF_DPE1;
		l |= MCSPI_CHCONF_DPE0;
	} else {
		l |= MCSPI_CHCONF_IS;
		l |= MCSPI_CHCONF_DPE1;
		l &= ~MCSPI_CHCONF_DPE0;
	}

	if (slave->pol == MCSPI_POL_HELD_HIGH)
		l &= ~MCSPI_CHCONF_POL;
	else
		l |= MCSPI_CHCONF_POL;

	if (slave->pha == MCSPI_PHA_ODD_NUMBERED_EDGES)
		l &= ~MCSPI_CHCONF_PHA;
	else
		l |= MCSPI_CHCONF_PHA;

	pr_info("%s: MCSPI_CH0CONF:0x%x\n", DRIVER_NAME, l);
	mcspi_slave_write_reg(slave->base, MCSPI_CH0CONF, l);
}

static void mcspi_slave_set_cs(struct spi_slave *slave)
{
	u32					l;

	pr_info("%s: set cs sensitive and polarity\n", DRIVER_NAME);

	l = mcspi_slave_read_reg(slave->base, MCSPI_CH0CONF);

	if (slave->cs_polarity == MCSPI_CS_POLARITY_ACTIVE_LOW)
		l |= MCSPI_CHCONF_EPOL;
	else
		l &= ~MCSPI_CHCONF_EPOL;

	pr_info("%s: MCSPI_CH0CONF:0x%x\n", DRIVER_NAME, l);
	mcspi_slave_write_reg(slave->base, MCSPI_CH0CONF, l);
	l = mcspi_slave_read_reg(slave->base, MCSPI_MODULCTRL);

	if (slave->cs_sensitive == MCSPI_CS_SENSITIVE_ENABLED)
		l &= ~MCSPI_MODULCTRL_PIN34;
	else
		l |= MCSPI_MODULCTRL_PIN34;

	pr_info("%s: MCSPI_MODULCTRL:0x%x\n", DRIVER_NAME, l);
	mcspi_slave_write_reg(slave->base, MCSPI_MODULCTRL, l);
}

static int mcspi_slave_setup(struct spi_slave *slave)
{
	int					ret = 0;
	u32					l;

	pr_info("%s: slave setup\n", DRIVER_NAME);

	l = mcspi_slave_read_reg(slave->base, MCSPI_SYSSTATUS);

	pr_info("%s: MCSPI_SYSSTATUS:0x%x\n", DRIVER_NAME, l);

	if (mcspi_slave_wait_for_bit(slave->base + MCSPI_SYSSTATUS,
					      MCSPI_SYSSTATUS_RESETDONE) == 0) {

		pr_info("%s: controller ready for setting\n",
			DRIVER_NAME);

		mcspi_slave_disable(slave);
		mcspi_slave_set_slave_mode(slave);
		mcspi_slave_set_cs(slave);
		ret = mcspi_slave_set_irq(slave);

		if (ret < 0)
			return ret;

	} else {
		pr_err("%s: internal module reset is on-going\n",
			DRIVER_NAME);
		ret = -EIO;
	}
	return ret;
}

static void mcspi_slave_clean_up(struct spi_slave *slave)
{
	pr_info("%s: clean up", DRIVER_NAME);

	tasklet_kill(&pio_rx_tasklet);

	if (slave->tx != NULL)
		kfree(slave->tx);

	if (slave->rx != NULL)
		kfree(slave->rx);

	kfree(slave);
}

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
	struct device_node				*node;

	struct resource					*res;
	struct resource					cp_res;
	const struct of_device_id			*match;
	const struct omap2_mcspi_platform_config	*pdata;

	int						ret = 0;
	u32						regs_offset = 0;

	struct spi_slave				*slave;

	u32						cs_sensitive;
	u32						cs_polarity;
	unsigned int					pin_dir;
	unsigned int					irq;
	static int					bus_num;
	unsigned int					pha;
	unsigned int					pol;

	pr_info("%s: Entry probe\n", DRIVER_NAME);

	node = pdev->dev.of_node;


	slave = kzalloc(sizeof(struct spi_slave), GFP_KERNEL);

	if (slave == NULL)
		return -ENOMEM;

	match = of_match_device(mcspi_slave_of_match, &pdev->dev);

	if (match) {
		pdata = match->data;

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

		if (of_get_property(node, "pha", &pha))
			pha = MCSPI_PHA_EVEN_NUMBERED_EDGES;
		else
			pha = MCSPI_PHA_ODD_NUMBERED_EDGES;

		if (of_get_property(node, "pol", &pol))
			pol = MCSPI_POL_HELD_LOW;
		else
			pol = MCSPI_POL_HELD_HIGH;


		irq = irq_of_parse_and_map(node, 0);

		slave->bus_num = bus_num++;

	} else {
		pdata = dev_get_platdata(&pdev->dev);
		pr_err("%s: failed to match, install DTS", DRIVER_NAME);
		ret = -EINVAL;
		goto free_slave;
	}

	regs_offset = pdata->regs_offset;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	memcpy(&cp_res, res, sizeof(struct resource));

	if (res == NULL) {
		pr_err("%s: res not availablee\n", DRIVER_NAME);
		ret = -ENODEV;
		goto free_slave;
	}

	cp_res.start += regs_offset;
	cp_res.end   += regs_offset;

	slave->base = devm_ioremap_resource(&pdev->dev, &cp_res);

	if (IS_ERR(slave->base)) {
		pr_err("%s: base addres ioremap error!!", DRIVER_NAME);
		ret = PTR_ERR(slave->base);
		goto free_slave;
	}

	slave->dev.of_node		= node;
	slave->cs_polarity		= cs_polarity;
	slave->start			= cp_res.start;
	slave->end			= cp_res.end;
	slave->reg_offset		= regs_offset;
	slave->cs_sensitive		= cs_sensitive;
	slave->pin_dir			= pin_dir;
	slave->irq			= irq;
	slave->pol			= pol;
	slave->pha			= pha;
	slave->mode			= SPI_SLAVE_MODE;
	slave->buf_depth		= SPI_SLAVE_BUF_DEPTH;
	slave->bytes_per_load		= SPI_SLAVE_COPY_LENGTH;
	slave->bits_per_word		= SPI_SLAVE_BITS_PER_WORD;

	slave->enable = mcspi_slave_enable;
	slave->disable = mcspi_slave_disable;
	slave->set_transfer = mcspi_slave_setup_pio_transfer;
	slave->clr_transfer = mcspi_slave_clr_pio_transfer;
	slave->transfer = mcspi_slave_pio_tx_transfer;

	platform_set_drvdata(pdev, slave);

	pr_info("%s: start:%x\n", DRIVER_NAME, slave->start);
	pr_info("%s: end:%x\n", DRIVER_NAME, slave->end);
	pr_info("%s: bus_num:%d\n", DRIVER_NAME, slave->bus_num);
	pr_info("%s: regs_offset=%x\n", DRIVER_NAME, slave->reg_offset);
	pr_info("%s: cs_sensitive=%d\n", DRIVER_NAME, slave->cs_sensitive);
	pr_info("%s: cs_polarity=%d\n", DRIVER_NAME, slave->cs_polarity);
	pr_info("%s: pin_dir=%d\n", DRIVER_NAME, slave->pin_dir);
	pr_info("%s: interrupt:%d\n", DRIVER_NAME, slave->irq);
	pr_info("%s: pol=%d\n", DRIVER_NAME, slave->pol);
	pr_info("%s: pha=%d\n", DRIVER_NAME, slave->pha);

	pm_runtime_use_autosuspend(&pdev->dev);
	pm_runtime_set_autosuspend_delay(&pdev->dev, SPI_AUTOSUSPEND_TIMEOUT);
	pm_runtime_enable(&pdev->dev);

	ret = pm_runtime_get_sync(&pdev->dev);

	if (ret < 0)
		goto disable_pm;

	sprintf(slave->name, "%s%d", DRIVER_NAME, slave->bus_num);
	ret = devm_spislave_register_device(&pdev->dev, slave->name, slave);

	if (ret) {
		pr_err("%s: register device error\n", DRIVER_NAME);
		goto disable_pm;
	}

	ret = mcspi_slave_setup(slave);
	if (ret < 0)
		goto disable_pm;

	return ret;

disable_pm:
	pm_runtime_dont_use_autosuspend(&pdev->dev);
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

free_slave:
	if (slave != NULL) {
		put_device(&slave->dev);
		mcspi_slave_clean_up(slave);
	}

	return ret;
}

static int mcspi_slave_remove(struct platform_device *pdev)
{
	struct spi_slave			*slave;

	slave = platform_get_drvdata(pdev);

	spislave_unregister_device(slave);

	mcspi_slave_clean_up(slave);

	pm_runtime_dont_use_autosuspend(&pdev->dev);
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	pr_info("%s: remove\n", DRIVER_NAME);
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
