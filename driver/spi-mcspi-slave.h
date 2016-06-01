#ifndef _SLAVE_MCSPI_H
#define _SLAVE_MCSPI_H

#define OMAP4_MCSPI_SLAVE_REG_OFFSET	0x100
#define SPI_MCSPI_SLAVE_FIFO_DEPTH	32
#define SPI_MCSPI_SLAVE_NUM_CS		1
#define SPI_MCSPI_SLAVE_BITS_PER_WORD	8

struct mcspi_slave_platform_config {
	u32		num_cs;
	u32		regs_offset;
	u32		fifo_depth;
	u32		bits_per_word;
};

#endif
