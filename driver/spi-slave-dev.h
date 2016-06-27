#ifndef SPISLAVE_H
#define SPISLAVE_H

#include <linux/types.h>


struct spislave_data {

	void __iomem	*tx_buf;
	void __iomem	*rx_buf;
	unsigned int	tx_offset;
	unsigned int	rx_offset;
};


#endif
