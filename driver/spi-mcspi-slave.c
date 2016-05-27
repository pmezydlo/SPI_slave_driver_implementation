#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/device.h>

#define DRIVER_NAME "spi-mcspi-slave"

#include "spi-mcspi-slave.h"

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
	const struct of_device_id			*match;
	const struct mcspi_slave_platform_config	*pdata;

	struct omap2_mcspi				*mcspi;

	int						ret = 0;
	unsigned int					regs_offset;

	dev  = &pdev->dev;
	node = pdev->dev.of_node;

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

//	printk(KERN_INFO "id=%d \n", id);
	printk(KERN_INFO "memory_depth=%d \n", memory_depth);
	printk(KERN_INFO "num_cs=%d \n", num_cs);
	printk(KERN_INFO "regs_offset=%d \n", regs_offset);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	if (res == NULL){
		printk(KERN_INFO "res not availablee \n");
	}

	return ret;
}

static int mcspi_slave_remove(struct platform_device *pdev){
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
