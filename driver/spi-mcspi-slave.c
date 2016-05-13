#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>

#define DRIVER_NAME "spi-mcspi-slave"
#define DEBUG

#include <linux/device.h>

#include "spi-mcspi-slave.h"
 
/* 
 * define omap4 register offset in platform data
 */
static struct mcspi_slave_platform_config mcspi_slave_pdata = {
    .regs_offset = OMAP4_MCSPI_REG_OFFSET,
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

    printk(KERN_INFO "mcspi_slave probe\n");

    const struct of_device_id *of_id = of_match_device(mcspi_slave_of_match, &pdev->dev);
    const struct mcspi_slave_platform_config *pdata; 
    unsigned int regs_offset = 0;

    if (of_id) {
        pdata = of_id->data;
    }
    else
    {
        pdata = dev_get_platdata(&pdev->dev);
    }

    regs_offset = pdata->regs_offset;
        
    struct resource *res1;

    res1 = platform_get_resource(pdev, IORESOURCE_MEM, 0);

    //dev_dbg(&pdev->dev, "Start:%x,  End:%x Size:%d  Offset:%x \n ", (unsigned long)res1->start, (unsigned long)res1->end, resource_size(res1),(unsigned int)regs_);
    
    printk(KERN_INFO "Start:%x,  End:%x Size:%d  Offset:%x \n ", (unsigned long)res1->start, 
                                                                 (unsigned long)res1->end, 
                                                                 resource_size(res1),
                                                                 (unsigned int)regs_offset);

    return 0;
}

static int mcspi_slave_remove(struct platform_device *pdev){
//	dev_dbg(&pdev->dev, "driver remove\n");
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
        printk(KERN_INFO "mcspi_slave init \n");
        ret = platform_driver_register(&mcspi_slave_driver);
        
        if (ret != 0) {

	}
	else{
        
	}

	return 0;
}

static void __exit mcspi_slave_exit(void){
	platform_driver_unregister(&mcspi_slave_driver);
        printk(KERN_INFO "mcspi_slave exit \n");
	return;
}

module_init(mcspi_slave_init);
module_exit(mcspi_slave_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Patryk Mezydlo");
MODULE_DESCRIPTION("SPI slave for McSPI controller.");
MODULE_VERSION("1.0");
