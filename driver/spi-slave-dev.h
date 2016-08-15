/*
 * IOCTL commands for SPI userspace interface in slave mode.
 *
 * Copyright (C) 2016 Patryk Mężydło <mezydlo.p@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef SPISLAVE_H
#define SPISLAVE_H

#include <linux/types.h>

#define SPISLAVE_IOC_MAGIC		'k'

/*Read / Write SPI device word length (4..32)*/
#define SPISLAVE_RD_BITS_PER_WORD	_IOR(SPISLAVE_IOC_MAGIC, 1, __u32)
#define SPISLAVE_WR_BITS_PER_WORD	_IOW(SPISLAVE_IOC_MAGIC, 1, __u32)

/*Read / Write of SPI sub-mode
 * 0 - reveive and transmit mode
 * 1 - only receive mode
 * 2 - only transmit mode
 */
#define SPISLAVE_RD_MODE		_IOR(SPISLAVE_IOC_MAGIC, 2, __u32)
#define SPISLAVE_WR_MODE		_IOW(SPISLAVE_IOC_MAGIC, 2, __u32)

/*Read / Write data bytes in biggest supported SPI slave message*/
#define SPISLAVE_RD_BUF_DEPTH		_IOR(SPISLAVE_IOC_MAGIC, 3, __u32)
#define SPISLAVE_WR_BUF_DEPTH		_IOW(SPISLAVE_IOC_MAGIC, 3, __u32)

/*Read / Write used to optimization of the amount of interrupts,
 *this number must be a multiple number of received bytes (1..8)
 */
#define SPISLAVE_RD_BYTES_PER_LOAD	_IOR(SPISLAVE_IOC_MAGIC, 4, __u32)
#define SPISLAVE_WR_BYTES_PER_LOAD	_IOW(SPISLAVE_IOC_MAGIC, 4, __u32)

/*Read the amount of data in the buffers*/
#define SPISLAVE_RD_TX_OFFSET		_IOR(SPISLAVE_IOC_MAGIC, 5, __u32)
#define SPISLAVE_RD_RX_OFFSET		_IOR(SPISLAVE_IOC_MAGIC, 6, __u32)

/*activates transfer*/
#define SPISLAVE_ENABLED		_IO(SPISLAVE_IOC_MAGIC, 7)

/*deactivated transfer*/
#define SPISLAVE_DISABLED		_IO(SPISLAVE_IOC_MAGIC, 8)

/*sets transfer and prepares controller */
#define SPISLAVE_SET_TRANSFER		_IO(SPISLAVE_IOC_MAGIC, 9)

/*removes received transfer and cleans cache*/
#define SPISLAVE_CLR_TRANSFER		_IO(SPISLAVE_IOC_MAGIC, 10)

#endif
