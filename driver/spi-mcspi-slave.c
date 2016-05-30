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

/*
 * default platform value located in .h file
 */
static struct mcspi_slave_platform_config mcspi_slave_pdata = {
	.regs_offset	= OMAP4_MCSPI_SLAVE_REG_OFFSET,
	.memory_depth	= SPI_MCSPI_SLAVE_MEMORY_DEPTH,
	.num_cs		= SPI_MCSPI_SLAVE_NUM_CS,
};

static const struct of_device_id mcspi_slave_of_match[] = {
	{
            .compatible = "ti,omap4-mcspi",
            .data = &mcspi_slave_pdata,
        },
	{},
};
MODULE_DEVICE_TABLE(of, mcspi_slave_of_match);

static int mcspi_slave_probe(struct platform_device *pdev){
	printk(KERN_INFO "mcspi_slave: Entry probe\n");

	struct device					*dev;
	struct device_node				*node;

	struct resource					*res;
	struct resource					cp_res;
	const struct of_device_id			*match;
	const struct mcspi_slave_platform_config	*pdata;

	struct omap2_mcspi				*mcspi;

	int						ret = 0;
	u32						regs_offset;

	struct spi_slave				*slave;

	dev  = &pdev->dev;
	node = pdev->dev.of_node;

	slave = kzalloc(sizeof(struct spi_slave),GFP_KERNEL);

	if (slave == NULL){
		printk(KERN_INFO "slave allocation failed \n");
	}

	/*
	 * here allocate memory for slave structure
	 * I have to write the structure of the slave device
	 * I don't know what to put in it
	 *
	 * and here I have to fill this structure
	 */


	//platform_set_drvdata(pdev,slave);
	//mcspi = dev_get_drvdata(&slave->dev);

	/*
	 *  tell if an of_device structure has a metching
	 */

	match = of_match_device(mcspi_slave_of_match, dev);

	unsigned int	memory_depth;
	unsigned int	num_cs;

	if (match){// user setting from dts
		pdata = match->data;

		//default value num_cs and memory_depth
		//when this value is not define in dts
		num_cs = 1;
		memory_depth = 32;

		of_property_read_u32(node, "memory_depth", &memory_depth);

		of_property_read_u32(node, "ti,spi-num-cs", &num_cs);

	}else{//default setting from pdata
		pdata = dev_get_platdata(&pdev->dev);
		memory_depth = pdata->memory_depth;
		num_cs = pdata->num_cs;

	}

	regs_offset = pdata->regs_offset;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	//copy resources because base address is changed
	memcpy(&cp_res,res,sizeof(struct resource));

	if (res == NULL){
		printk(KERN_INFO "res not availablee \n");
	}


	//driver is increment allways when omap2 driver is install
	//base addres is not correct when install a driver more times
	//but when resources is copied it's ok
	cp_res.start += regs_offset;
	cp_res.end   += regs_offset;

	void __iomem *base = devm_ioremap_resource(&pdev->dev, &cp_res);

	if (IS_ERR(&base)){
		printk(KERN_INFO "base addres ioremap error!!");
	}

	slave->base		= base;
	slave->dev		= dev;
	slave->fifo_depth	= memory_depth;
	slave->num_cs		= num_cs;
	slave->start		= cp_res.start;
	slave->end		= cp_res.end;
	slave->reg_offset	= regs_offset;

	platform_set_drvdata(pdev,slave);

	return ret;
}

static int mcspi_slave_remove(struct platform_device *pdev){

	struct spi_slave *slave;

	slave = platform_get_drvdata(pdev);

	printk(KERN_INFO "start:%x \n",		slave->start);
	printk(KERN_INFO "end:%x \n",		slave->end);
	printk(KERN_INFO "regs_offset=%d \n",	slave->reg_offset);
	printk(KERN_INFO "memory_depth=%d \n",	slave->fifo_depth);
	printk(KERN_INFO "num_cs=%d \n",	slave->num_cs);

	printk(KERN_INFO "mcspi_slave: remove\n");
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

static int __init mcspi_slave_init(void){
        int ret;
        printk(KERN_INFO "mcspi_slave: init \n");

	ret = platform_driver_register(&mcspi_slave_driver);

        if (ret != 0)
		printk(KERN_INFO "mcspi_slave: platform driver register ok \n");

	return ret;
}

static void __exit mcspi_slave_exit(void){
	platform_driver_unregister(&mcspi_slave_driver);

	printk(KERN_INFO "mcspi_slave: exit \n");
	return;
}

module_init(mcspi_slave_init);
module_exit(mcspi_slave_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Patryk Mezydlo");
MODULE_DESCRIPTION("SPI slave for McSPI controller.");
MODULE_VERSION("1.0");
