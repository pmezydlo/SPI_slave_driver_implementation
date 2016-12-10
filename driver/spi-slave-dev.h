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

#define SPISLAVE_IOCTL_MAGIC		'k'

#define SPISLAVE_SLAVE			0x01
#define SPISLAVE_CPHA			0x02
#define SPISLAVE_CPOL			0x04
#define SPISLAVE_NO_CS			0x08
#define SPISLAVE_CS_HIGH		0x10
#define SPISLAVE_LSB_FIRST		0x20
#define SPISLAVE_RM			0x40
#define SPISLAVE_TM			0x80
#define SPISLAVE_TRM			0xC0

struct spislave_ioctl_transfer {
	__u64	tx_buf;
	__u64	rx_buf;

	__u32	tx_actual_length;
	__u32	rx_actual_length;

	__u32	mode;
	__u32	max_speed;
	__u8	bits_per_word;
};

#define SPISLAVE_MSGSIZE(N) \
	((((N) * (sizeof(struct spislave_ioctl_transfer))) \
	< (1 << _IOC_SIZEBITS)) \
	? ((N) * (sizeof(struct spislave_ioctl_transfer))) : 0)

#define SPISLAVE_MESSAGE(N)		__IOC(SPISLAVE_IOCTL_MAGIC, 0, \
						char[SPISLAVE_MSGSIZE(N)])

#define SPISLAVE_RD_BITS_PER_WORD	_IOR(SPISLAVE_IOCTL_MAGIC, 1, __u8)
#define SPISLAVE_WR_BITS_PER_WORD	_IOW(SPISLAVE_IOCTL_MAGIC, 1, __u8)

#define SPISLAVE_RD_MODE		_IOR(SPISLAVE_IOCTL_MAGIC, 2, __u32)
#define SPISLAVE_WR_MODE		_IOW(SPISLAVE_IOCTL_MAGIC, 2, __u32)

#define SPISLAVE_RD_MAX_SPEED		_IOR(SPISLAVE_IOCTL_MAGIC, 3, __u32)
#define SPISLAVE_WR_MAX_SPEED		_IOW(SPISLAVE_IOCTL_MAGIC, 3, __u32)

#define SPISLAVE_RD_TX_ACTUAL_LENGTH	_IOR(SPISLAVE_IOCTL_MAGIC, 4, __u32)
#define SPISLAVE_RD_RX_ACTUAL_LENGTH	_IOR(SPISLAVE_IOCTL_MAGIC, 5, __u32)

#endif
