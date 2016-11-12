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

#define SPISLAVE_RD_BITS_PER_WORD	_IOR(SPISLAVE_IOC_MAGIC, 1, __u8)
#define SPISLAVE_WR_BITS_PER_WORD	_IOW(SPISLAVE_IOC_MAGIC, 1, __u8)

#define SPISLAVE_RD_MODE		_IOR(SPISLAVE_IOC_MAGIC, 2, __u8)
#define SPISLAVE_WR_MODE		_IOW(SPISLAVE_IOC_MAGIC, 2, __u8)

#define SPISLAVE_RD_BUF_DEPTH		_IOR(SPISLAVE_IOC_MAGIC, 3, __u32)
#define SPISLAVE_WR_BUF_DEPTH		_IOW(SPISLAVE_IOC_MAGIC, 3, __u32)

#define SPISLAVE_RD_LSB_FIRST		_IOR(SPISLAVE_IOC_MAGIC, 4, __u8)
#define SPISLAVE_WR_LSB_FIRST		_IOW(SPISLAVE_IOC_MAGIC, 4, __u8)

#define SPISLAVE_RD_MAX_SPEED		_IOR(SPISLAVE_IOC_MAGIC, 5, __u32)
#define SPISLAVE_WR_MAX_SPEED		_IOW(SPISLAVE_IOC_MAGIC, 5, __u32)

#define SPISLAVE_RD_TX_ACTUAL_LENGTH	_IOR(SPISLAVE_IOC_MAGIC, 6, __u32)
#define SPISLAVE_RD_RX_ACTUAL_LENGTH	_IOR(SPISLAVE_IOC_MAGIC, 7, __u32)

#endif
