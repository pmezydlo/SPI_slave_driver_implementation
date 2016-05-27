#ifndef _SLAVE_MCSPI_H
#define _SLAVE_MCSPI_H

#define OMAP4_MCSPI_SLAVE_REG_OFFSET	0x100
#define SPI_MCSPI_SLAVE_MEMORY_DEPTH	32
#define SPI_MCSPI_SLAVE_NUM_CS		1

struct mcspi_slave_platform_config {
	unsigned int	num_cs;
	unsigned int	regs_offset;
	unsigned int	memory_depth;
};

#endif
