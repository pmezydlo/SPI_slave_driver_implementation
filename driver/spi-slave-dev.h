#ifndef SPISLAVE_H
#define SPISLAVE_H

#include <linux/types.h>


struct spislave_data {

	void		*tx_buf;
	void		*rx_buf;
	unsigned int	tx_offset;
	unsigned int	rx_offset;
};

#define SPISLAVE_IOC_MAGIC		'k'

#define	SPISLAVE_RD_BITS_PER_WORD	_IOR(SPISLAVE_IOC_MAGIC, 1, __u32)
#define SPISLAVE_WR_BITS_PER_WOED	_IOW(SPISLAVE_IOC_MAGIC, 1, __u32)

#define	SPISLAVE_RD_TX_OFFSET		_IOR(SPISLAVE_IOC_MAGIC, 2, __u32)

#define	SPISLAVE_RD_RX_OFFSET		_IOR(SPISLAVE_IOC_MAGIC, 3, __u32)

#endif
