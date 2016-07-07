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

#include "spi-slave-dev.h"

#define MCSPI_PIN_DIR_D0_IN_D1_OUT		0
#define MCSPI_PIN_DIR_D0_OUT_D1_IN		1
#define MCSPI_CS_POLARITY_ACTIVE_HIGH		1
#define MCSPI_CS_POLARITY_ACTIVE_LOW		0
#define MCSPI_CS_SENSITIVE_ENABLED		1
#define MCSPI_CS_SENSITIVE_DISABLED		0
#define MCSPI_MAX_FIFO_DEPTH			64

#define MCSPI_MODE_TRM				0
#define MCSPI_MODE_RM				1
#define MCSPI_MODE_TM				2

#define SPI_MCSPI_SLAVE_BUF_DEPTH		64
#define SPI_MCSPI_SLAVE_BITS_PER_WORD		8
#define SPI_MCSPI_SLAVE_CS_SENSITIVE		MCSPI_CS_SENSITIVE_ENABLED
#define SPI_MCSPI_SLAVE_CS_POLARITY		MCSPI_CS_POLARITY_ACTIVE_LOW
#define SPI_MCSPI_SLAVE_PIN_DIR			MCSPI_PIN_DIR_D0_IN_D1_OUT
#define SPI_MCSPI_SLAVE_MODE			MCSPI_MODE_TRM
#define SPI_MCSPI_SLAVE_COPY_LENGTH		1
#define SPI_MCSPI_SLAVE_LENGTH_OF_TRANSFER	8

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
#define MCSPI_CH1CONF				0x40
#define MCSPI_CH1STAT				0x44
#define MCSPI_CH1CTRL				0x48
#define MCSPI_TX1				0x4C
#define MCSPI_RX1				0x50
#define MCSPI_CH2CONF				0x54
#define MCSPI_CH2STAT				0x58
#define MCSPI_CH2CTRL				0x5C
#define MCSPI_TX2				0x60
#define MCSPI_RX2				0x64
#define MCSPI_CH3CONF				0x68
#define MCSPI_CH3STAT				0x6C
#define MCSPI_CH3CTRL				0x70
#define MCSPI_TX3				0x74
#define MCSPI_RX3				0x78
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
#define MCSPI_CHSTAT_RXS			BIT(0)
#define MCSPI_CHSTAT_RXFFF			BIT(6)
#define MCSPI_CHSTAT_RXFFE			BIT(5)
#define MCSPI_CHSTAT_TXFFF			BIT(4)
#define MCSPI_CHSTAT_TXFFE			BIT(3)

#define SPISLAVE_MAJOR				154
#define N_SPI_MINORS				32

static					DECLARE_BITMAP(minors, N_SPI_MINORS);
static					LIST_HEAD(device_list);
static struct class			*spislave_class;

struct spi_slave {
	/*var defining device parameters*/
	struct	device			*dev;
	void	__iomem			*base;
	u32				start;
	u32				end;
	unsigned int			reg_offset;
	s16				bus_num;

	/*var defining cs and pin direct parameters*/
	unsigned int			pin_dir;
	u32				cs_sensitive;
	u32				cs_polarity;

	/*var defining interrupt*/
	unsigned int			irq;

	/*var defining msg */
	u32				tx_offset;
	u32				rx_offset;
	void  __iomem			*tx;
	void  __iomem			*rx;

	/*var defining the char driver parameters*/
	char				modalias[SPI_NAME_SIZE];
	dev_t				devt;
	struct	list_head		device_entry;
	unsigned int			users;
	wait_queue_head_t		wait;

	/*var defining the transfer parameters*/
	u32				mode;
	u32				length_of_transfer;
	u32				bytes_per_load;
	u32				bits_per_word;
	u32				buf_depth;
};

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
	unsigned long timeout;

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

	pr_info("%s: spi is disabled\n", DRIVER_NAME);
	l = mcspi_slave_read_reg(slave->base, MCSPI_CH0CTRL);

	/*clr bit(0) in ch0ctrl, spi is enabled*/
	l &= ~MCSPI_CHCTRL_EN;
	pr_info("%s: MCSPI_CH0CTRL:0x%x\n", DRIVER_NAME, l);

	mcspi_slave_write_reg(slave->base, MCSPI_CH0CTRL, l);
}

static void mcspi_slave_pio_rx_transfer(unsigned long data)
{
	struct spi_slave		*slave = (struct spi_slave *) data;
	unsigned int			c;
	void __iomem			*rx_reg;
	void __iomem			*chstat;

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
			pr_info("%s: read 8:0x%02x\n", DRIVER_NAME, *(rx-1));

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
			pr_info("%s: read 16:0x%04x\n", DRIVER_NAME, *(rx-1));

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
			pr_info("%s: read 32:0x%04x\n", DRIVER_NAME, *(rx-1));

		} while (c);
	}

	return;
out:
	pr_err("%s: timeout!!!", DRIVER_NAME);
}
DECLARE_TASKLET(pio_rx_tasklet, mcspi_slave_pio_rx_transfer, 0);

static void mcspi_slave_pio_tx_transfer(unsigned long data)
{
	struct spi_slave		*slave = (struct spi_slave *) data;
	unsigned int			c;
	void __iomem			*tx_reg;
	void __iomem			*chstat;

	tx_reg = slave->base + MCSPI_TX0;
	chstat = slave->base + MCSPI_CH0STAT;

	c = slave->bytes_per_load;
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

			pr_info("%s: write 8:0x%02x\n", DRIVER_NAME, *tx);
			writel_relaxed(*tx++, tx_reg);
		} while (c);
	}

	if (mcspi_slave_bytes_per_word(slave->bits_per_word) == 2) {
	const u16 *tx;

	tx = slave->tx + slave->tx_offset;
	slave->tx_offset += (sizeof(u8) * c);
		do {
			c -= 1;
			if (mcspi_slave_wait_for_bit(chstat, MCSPI_CHSTAT_TXS)
						     < 0)
				goto out;

			pr_info("%s: write 16:0x%04x\n", DRIVER_NAME, *tx);
			writel_relaxed(*tx++, tx_reg);
		} while (c);
	}

	if (mcspi_slave_bytes_per_word(slave->bits_per_word) == 4) {
	const u32 *tx;

	tx = slave->tx + slave->tx_offset;
	slave->tx_offset += (sizeof(u8) * c);

		do {
			c -= 1;
			if (mcspi_slave_wait_for_bit(chstat, MCSPI_CHSTAT_TXS)
						     < 0)
				goto out;

			pr_info("%s: write 32:0x%04x\n", DRIVER_NAME, *tx);
			writel_relaxed(*tx++, tx_reg);
		} while (c);
	}

	return;
out:
	pr_err("%s: timeout!!!", DRIVER_NAME);
}
DECLARE_TASKLET(pio_tx_tasklet, mcspi_slave_pio_tx_transfer, 0);

static void mcspi_slave_end_of_word_tasklet(unsigned long data)
{
	struct spi_slave		*slave = (struct spi_slave *) data;

	pr_info("%s: end of transaction!!", DRIVER_NAME);
}
DECLARE_TASKLET(end_of_word_tasklet, mcspi_slave_end_of_word_tasklet, 1);

static irq_handler_t mcspi_slave_irq(unsigned int irq, void *dev_id)
{
	struct spi_slave	*slave = dev_id;
	u32			l;

	l = mcspi_slave_read_reg(slave->base, MCSPI_IRQSTATUS);

	if (l & MCSPI_IRQ_RX_FULL) {
		l |= MCSPI_IRQ_RX_FULL;
		pio_rx_tasklet.data = (unsigned long)slave;
		tasklet_schedule(&pio_rx_tasklet);
	}

	if (l & MCSPI_IRQ_TX_EMPTY) {
		l |= MCSPI_IRQ_TX_EMPTY;
		pio_tx_tasklet.data = (unsigned long)slave;
		tasklet_schedule(&pio_tx_tasklet);
	}

	/*clear IRQSTATUS register*/
	mcspi_slave_write_reg(slave->base, MCSPI_IRQSTATUS, l);

	return (irq_handler_t) IRQ_HANDLED;
}

static int mcspi_slave_set_irq(struct spi_slave *slave)
{
	u32		l;
	int		ret = 0;

	pr_info("%s: set interrupt\n", DRIVER_NAME);

	l = mcspi_slave_read_reg(slave->base, MCSPI_IRQENABLE);

	l &= ~MCSPI_IRQ_RX_FULL;
	l &= ~MCSPI_IRQ_TX_EMPTY;

	if (slave->mode == MCSPI_MODE_TM || slave->mode == MCSPI_MODE_TRM)
		l |= MCSPI_IRQ_RX_FULL;


	if (slave->mode == MCSPI_MODE_RM || slave->mode == MCSPI_MODE_TRM)
		l |= MCSPI_IRQ_TX_EMPTY;

	l &= ~MCSPI_IRQ_EOW;

	pr_info("%s: MCSPI_IRQENABLE:0x%x\n", DRIVER_NAME, l);

	mcspi_slave_write_reg(slave->base, MCSPI_IRQENABLE, l);

	ret = devm_request_irq(slave->dev, slave->irq,
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
	u32				l;
	int				ret = 0;

	pr_info("%s: pio transfer setup\n", DRIVER_NAME);

	pr_info("%s: mode:%d\n", DRIVER_NAME, slave->mode);
	pr_info("%s: bits_per_word:%x\n", DRIVER_NAME, slave->bits_per_word);
	pr_info("%s: bytes_per_load:%d\n", DRIVER_NAME, slave->bytes_per_load);
	pr_info("%s: length_of_transfer:%d\n", DRIVER_NAME,
		slave->length_of_transfer);
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

	/*
	 * set maximum receive and transmit byte
	 * when mcspi generating interrupt
	 */
	if (slave->mode == MCSPI_MODE_RM || slave->mode == MCSPI_MODE_TRM)
		l  |= (slave->bytes_per_load - 1) << 8;

	if (slave->mode == MCSPI_MODE_TM || slave->mode == MCSPI_MODE_TRM)
		l  |= (slave->bytes_per_load - 1);

	/*enable word counter*/
	l &= ~MCSPI_XFER_WCNT;
	/*l |= (slave->length_of_transfer - 1) << 16;*/

	mcspi_slave_write_reg(slave->base, MCSPI_XFERLEVEL, l);

	pr_info("%s: MCSPI_XFERLEVEL:0x%x\n", DRIVER_NAME, l);

	l = mcspi_slave_read_reg(slave->base, MCSPI_CH0CONF);

	/*
	 * clr bit(13 and 12) in chconf,
	 * spi is set in transmit and receive mode
	 */
	l &= ~MCSPI_CHCONF_TRM;

	if (slave->mode == MCSPI_MODE_RM)
		l |= MCSPI_CHCONF_RM;
	else if (slave->mode == MCSPI_MODE_TM)
		l |= MCSPI_CHCONF_TM;

	/*
	 * available is only form 4 bits to 32 bits per word
	 * before setting clear all WL bits
	 */
	l &= ~MCSPI_CHCONF_WL;
	l |= (slave->bits_per_word - 1) << 7;

	l &= ~MCSPI_CHCONF_FFER;
	l &= ~MCSPI_CHCONF_FFEW;

	/*enable fifo*/
	l |= MCSPI_CHCONF_FFER;
	l |= MCSPI_CHCONF_FFEW;

	mcspi_slave_write_reg(slave->base, MCSPI_CH0CONF, l);
	pr_info("%s: MCSPI_CH0CONF:0x%x\n", DRIVER_NAME, l);

	l = mcspi_slave_read_reg(slave->base, MCSPI_MODULCTRL);

	l &= ~MCSPI_MODULCTRL_FDAA;

	/*multiple word ocp access*/
	/*l |= MCSPI_MODULCTRL_MOA;*/

	mcspi_slave_write_reg(slave->base, MCSPI_MODULCTRL, l);
	pr_info("%s: MCSPI_MODULCTRL:0x%x\n", DRIVER_NAME, l);

	return ret;
}

static int mcspi_slave_clr_pio_transfer(struct spi_slave *slave)
{
	int		ret = 0;

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
	u32		l;

	pr_info("%s: set slave mode\n", DRIVER_NAME);

	l = mcspi_slave_read_reg(slave->base, MCSPI_MODULCTRL);

	/*set bit(2) in modulctrl, spi is set in slave mode*/
	l |= MCSPI_MODULCTRL_MS;

	pr_info("%s: MCSPI_MODULCTRL:0x%x\n", DRIVER_NAME, l);

	mcspi_slave_write_reg(slave->base, MCSPI_MODULCTRL, l);

	l = mcspi_slave_read_reg(slave->base, MCSPI_CH0CONF);

	l &= ~MCSPI_CHCONF_PHA;
	l &= ~MCSPI_CHCONF_POL;

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

static void mcspi_slave_setup_system(struct spi_slave *slave)
{

}

static int mcspi_slave_setup(struct spi_slave *slave)
{
	int		ret = 0;
	u32		l;

	pr_info("%s: slave setup\n", DRIVER_NAME);

	/*verification status bit(0) in MCSPI system status register*/
	l = mcspi_slave_read_reg(slave->base, MCSPI_SYSSTATUS);

	pr_info("%s: MCSPI_SYSSTATUS:0x%x\n", DRIVER_NAME, l);

	if (mcspi_slave_wait_for_bit(slave->base + MCSPI_SYSSTATUS,
					      MCSPI_SYSSTATUS_RESETDONE) == 0) {

		pr_info("%s: controller ready for setting\n",
			DRIVER_NAME);

		/*here set mcspi controller in slave mode and more setting*/
		mcspi_slave_disable(slave);
		mcspi_slave_setup_system(slave);
		mcspi_slave_set_slave_mode(slave);
		mcspi_slave_set_cs(slave);
		ret = mcspi_slave_set_irq(slave);
	       /*
		*mcspi_slave_setup_pio_transfer(slave);
		*mcspi_slave_enable(slave);
		*/
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
	tasklet_kill(&pio_tx_tasklet);

	if (slave->tx != NULL)
		kfree(slave->tx);

	if (slave->rx != NULL)
		kfree(slave->rx);

	kfree(slave);
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

	u32						cs_sensitive;
	u32						cs_polarity;
	unsigned int					pin_dir;
	unsigned int					irq;
	static int					bus_num;

	unsigned long					minor;

	pr_info("%s: Entry probe\n", DRIVER_NAME);

	dev  = &pdev->dev;
	node = dev->of_node;

	slave = kzalloc(sizeof(struct spi_slave), GFP_KERNEL);

	if (slave == NULL)
		return -ENOMEM;

	match = of_match_device(mcspi_slave_of_match, dev);

	if (match) {/* user setting from dts*/
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

	/*copy resources because base address is changed*/
	memcpy(&cp_res, res, sizeof(struct resource));

	if (res == NULL) {
		pr_err("%s: res not availablee\n", DRIVER_NAME);
		ret = -ENODEV;
		goto free_slave;
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
		goto free_slave;
	}

	slave->dev			= dev;
	slave->cs_polarity		= cs_polarity;
	slave->start			= cp_res.start;
	slave->end			= cp_res.end;
	slave->reg_offset		= regs_offset;
	slave->cs_sensitive		= cs_sensitive;
	slave->pin_dir			= pin_dir;
	slave->irq			= irq;

	/*default setting when user dosn't get your setting*/
	slave->mode			= SPI_MCSPI_SLAVE_MODE;
	slave->buf_depth		= SPI_MCSPI_SLAVE_BUF_DEPTH;
	slave->bytes_per_load		= SPI_MCSPI_SLAVE_COPY_LENGTH;
	slave->bits_per_word		= SPI_MCSPI_SLAVE_BITS_PER_WORD;
	slave->length_of_transfer	= SPI_MCSPI_SLAVE_LENGTH_OF_TRANSFER;

	platform_set_drvdata(pdev, slave);

	pr_info("%s: start:%x\n", DRIVER_NAME, slave->start);
	pr_info("%s: end:%x\n", DRIVER_NAME, slave->end);
	pr_info("%s: bus_num:%d\n", DRIVER_NAME, slave->bus_num);
	pr_info("%s: regs_offset=%x\n", DRIVER_NAME, slave->reg_offset);
	pr_info("%s: cs_sensitive=%d\n", DRIVER_NAME, slave->cs_sensitive);
	pr_info("%s: cs_polarity=%d\n", DRIVER_NAME, slave->cs_polarity);
	pr_info("%s: pin_dir=%d\n", DRIVER_NAME, slave->pin_dir);
	pr_info("%s: interrupt:%d\n", DRIVER_NAME, slave->irq);

	pm_runtime_use_autosuspend(slave->dev);
	pm_runtime_set_autosuspend_delay(slave->dev, SPI_AUTOSUSPEND_TIMEOUT);
	pm_runtime_enable(slave->dev);

	ret = pm_runtime_get_sync(slave->dev);
	if (ret < 0)
		goto disable_pm;

	ret = mcspi_slave_setup(slave);
	if (ret < 0)
		goto disable_pm;

	minor = find_first_zero_bit(minors, N_SPI_MINORS);

	if (minor < N_SPI_MINORS) {
		struct device *dev;

		slave->devt = MKDEV(SPISLAVE_MAJOR, minor);
		dev = device_create(spislave_class, slave->dev,
				    slave->devt, slave, "spislave%d",
				    slave->bus_num);

		ret = PTR_ERR_OR_ZERO(dev);
	} else {
		pr_err("%s: no minor number available!!\n",
		       DRIVER_NAME);
		ret = -ENODEV;
	}

	if (ret == 0) {
		set_bit(minor, minors);
		list_add(&slave->device_entry, &device_list);
	}

	init_waitqueue_head(&slave->wait);

	if (wait_event_interruptible(slave->wait, 1))
		return ret;

	return ret;

	pr_info("%s: register device\n", DRIVER_NAME);

disable_pm:
	pm_runtime_dont_use_autosuspend(slave->dev);
	pm_runtime_put_sync(slave->dev);
	pm_runtime_disable(slave->dev);

free_slave:
	if (slave != NULL) {
		put_device(slave->dev);
		mcspi_slave_clean_up(slave);
	}

	return ret;
}

static int mcspi_slave_remove(struct platform_device *pdev)
{
	struct spi_slave	*slave;

	slave = platform_get_drvdata(pdev);

	mcspi_slave_clean_up(slave);

	list_del(&slave->device_entry);
	device_destroy(spislave_class, slave->devt);

	pm_runtime_dont_use_autosuspend(&pdev->dev);
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

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

static ssize_t spislave_read(struct file *flip, char __user *buf, size_t count,
			     loff_t *f_pos)
{
	struct spi_slave	*slave;
	int			error_count = 0;

	slave = flip->private_data;
	pr_info("%s: read begin\n", DRIVER_NAME);

	if (slave->rx == NULL) {
		pr_err("%s: slave->rx pointer is NULL\n", DRIVER_NAME);
		return -ENOMEM;
	}

	error_count = copy_to_user(buf, slave->rx, slave->rx_offset);

	pr_info("%s: read end count:%d rx_offset:%d\n", DRIVER_NAME,
		error_count, slave->rx_offset);

	/*after read clear receive buffer*/
	slave->rx_offset = 0;
	memset(slave->rx, 0, slave->buf_depth);

	if (error_count == 0)
		return 0;
	else
		return -EFAULT;
}

static ssize_t spislave_write(struct file *flip, const char __user *buf,
			      size_t count, loff_t *f_pos)
{
	ssize_t			ret = 0;
	struct spi_slave	*slave;
	unsigned long		missing;

	slave = flip->private_data;

	if (slave->tx == NULL) {
		pr_err("%s: slave->tx pointer is NULL\n", DRIVER_NAME);
		return -ENOMEM;
	}

	memset(slave->tx, 0, slave->buf_depth);

	if (count > MCSPI_MAX_FIFO_DEPTH/2) {
		pr_err("%s: message is too long!!!\n", DRIVER_NAME);
		return -EFAULT;
	}

	missing = copy_from_user(slave->tx, buf, count);

	if (missing == 0)
		ret = count;
	else
		return -EFAULT;

	/*mcspi_slave_pio_tx_load(slave, count);*/
	slave->tx_offset = 0;

	pr_info("%s: write\n", DRIVER_NAME);
	return ret;
}

static int spislave_release(struct inode *inode, struct file *filp)
{
	int			ret = 0;
	struct spi_slave	*slave;

	slave = filp->private_data;
	filp->private_data = NULL;

	slave->users--;

	pr_info("%s: release\n", DRIVER_NAME);
	return ret;
}

static int spislave_open(struct inode *inode, struct file *filp)
{
	int			ret = -ENXIO;
	struct spi_slave	*slave;

	list_for_each_entry(slave, &device_list, device_entry) {
		if (slave->devt == inode->i_rdev) {
			ret = 0;
			pr_info("%s: for each ret = 0\n", DRIVER_NAME);
			break;
		}
	}

	slave->users++;
	filp->private_data = slave;
	nonseekable_open(inode, filp);

	pr_info("%s: open\n", DRIVER_NAME);
	return ret;
}

static long spislave_ioctl(struct file *filp, unsigned int cmd,
			   unsigned long arg)
{
	int			ret = 0;
	int			err = 0;
	struct spi_slave	*slave;

	if (_IOC_TYPE(cmd) != SPISLAVE_IOC_MAGIC)
		return -ENOTTY;

	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,
				 (void __user *)arg, _IOC_SIZE(cmd));

	if (err == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_WRITE,
				 (void __user *)arg, _IOC_SIZE(cmd));

	if (err)
		return -EFAULT;

	slave = filp->private_data;

	switch (cmd) {
	case SPISLAVE_RD_TX_OFFSET:
		ret = __put_user(slave->tx_offset, (__u32 __user *)arg);
		break;

	case SPISLAVE_RD_RX_OFFSET:
		ret = __put_user(slave->rx_offset, (__u32 __user *)arg);
		break;

	case SPISLAVE_RD_BITS_PER_WORD:
		ret = __put_user(slave->bits_per_word, (__u32 __user *)arg);
		break;

	case SPISLAVE_RD_BYTES_PER_LOAD:
		ret = __put_user(slave->bytes_per_load, (__u32 __user *)arg);
		break;

	case SPISLAVE_RD_MODE:
		ret = __put_user(slave->mode, (__u32 __user *)arg);
		break;

	case SPISLAVE_RD_BUF_DEPTH:
		ret = __put_user(slave->buf_depth, (__u32 __user *)arg);
		break;

	case SPISLAVE_RD_LENGTH_OF_TRANSFER:
		ret = __put_user(slave->length_of_transfer,
				 (__u32 __user *)arg);
		break;

	case SPISLAVE_ENABLED:
		pr_info("%s: IOCTL mcspi set enabled\n", DRIVER_NAME);
		mcspi_slave_enable(slave);
		break;

	case SPISLAVE_DISABLED:
		pr_info("%s: IOCTL mcspi set disaabled\n", DRIVER_NAME);
		mcspi_slave_disable(slave);
		break;

	case SPISLAVE_SET_TRANSFER:
		pr_info("%s: IOCTL mcspi set transfer\n", DRIVER_NAME);
		mcspi_slave_setup_pio_transfer(slave);
		break;

	case SPISLAVE_CLR_TRANSFER:
		pr_info("%s: IOCTL mcspi clt transfer", DRIVER_NAME);
		mcspi_slave_clr_pio_transfer(slave);
		break;

	case SPISLAVE_WR_LENGTH_OF_TRANSFER:
		ret = __get_user(slave->length_of_transfer,
				 (__u32 __user *)arg);
		pr_info("%s: IOCTL length_of_transfer:%d\n", DRIVER_NAME,
			slave->length_of_transfer);
		break;

	case SPISLAVE_WR_BITS_PER_WORD:
		ret = __get_user(slave->bits_per_word, (__u32 __user *)arg);
		pr_info("%s: IOCTL bits_per_word:%d\n", DRIVER_NAME,
			slave->bits_per_word);
		break;

	case SPISLAVE_WR_MODE:
		ret = __get_user(slave->mode, (__u32 __user *)arg);
		pr_info("%s: IOCTL mode:%d\n", DRIVER_NAME, slave->mode);
		break;

	case SPISLAVE_WR_BUF_DEPTH:
		ret = __get_user(slave->buf_depth, (__u32 __user *)arg);
		pr_info("%s: IOCTL buf_depth:%d\n", DRIVER_NAME,
			slave->buf_depth);
		break;

	case SPISLAVE_WR_BYTES_PER_LOAD:
		ret = __get_user(slave->bytes_per_load, (__u32 __user *)arg);
		pr_info("%s: IOCTL bytes_per_load:%d\n", DRIVER_NAME,
			slave->bytes_per_load);
		break;

	default:

		break;
	}
	return ret;
}

static unsigned int spislave_event_poll(struct file *filp,
					struct poll_table_struct *wait)
{
	struct spi_slave *slave;
	unsigned int events = 0;

	slave = filp->private_data;

	if (slave == NULL) {
		pr_err("%s: slave pointer is NULL!!\n", DRIVER_NAME);
		return -EFAULT;
	}

	pr_info("%s: POLL method!!\n", DRIVER_NAME);
	poll_wait(filp, &slave->wait, wait);
	pr_info("%s: poll end of wait!!\n", DRIVER_NAME);
	if (slave->rx_offset != 0)
		events = POLLIN | POLLRDNORM;

	pr_info("%s: POLL method end!!\n", DRIVER_NAME);

	return events;
}

static const struct file_operations spislave_fops = {
	.owner		= THIS_MODULE,
	.open		= spislave_open,
	.read		= spislave_read,
	.write		= spislave_write,
	.release	= spislave_release,
	.unlocked_ioctl = spislave_ioctl,
	.poll		= spislave_event_poll,
};

static int __init mcspi_slave_init(void)
{
	int		ret = 0;

	pr_info("%s: init\n", DRIVER_NAME);

	BUILD_BUG_ON(N_SPI_MINORS > 256);

	ret = register_chrdev(SPISLAVE_MAJOR, "spi", &spislave_fops);
	if (ret < 0)
		return ret;

	spislave_class = class_create(THIS_MODULE, DRIVER_NAME);
	if (IS_ERR(spislave_class)) {
		unregister_chrdev(SPISLAVE_MAJOR, DRIVER_NAME);
		return PTR_ERR(spislave_class);
	}

	ret = platform_driver_register(&mcspi_slave_driver);
	if (ret < 0)
		pr_err("%s: platform driver error\n", DRIVER_NAME);

	return ret;
}

static void __exit mcspi_slave_exit(void)
{
	platform_driver_unregister(&mcspi_slave_driver);
	class_unregister(spislave_class);
	class_destroy(spislave_class);
	unregister_chrdev(SPISLAVE_MAJOR, DRIVER_NAME);

	pr_info("%s: exit\n", DRIVER_NAME);
}

module_init(mcspi_slave_init);
module_exit(mcspi_slave_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Patryk Mezydlo, <mezydlo.p@gmail.com>");
MODULE_DESCRIPTION("SPI slave for McSPI controller.");
MODULE_VERSION("1.0");
