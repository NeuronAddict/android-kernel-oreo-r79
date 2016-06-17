/*
 * Copyright (c) 2014--2016 Intel Corporation.
 *
 * Author: Archana Vohra <archana.vohra@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/io.h>
#include "mipicsi_util.h"

#ifdef MNH_EMULATION
#define BASE_ADDR_CSI_TX0  0xFFFFFFC064010000
#define BASE_ADDR_CSI_TX1  0xFFFFFFC064011000
#define BASE_ADDR_CSI_RX0  0xFFFFFFC064012000
#define BASE_ADDR_CSI_RX1  0xFFFFFFC064013000
#define BASE_ADDR_CSI_RX2  0xFFFFFFC064014000
#define BASE_ADDR_MIPI_PHY 0xFFFFFFC064015000
#define BASE_ADDR_MIPI_TOP BASE_ADDR_MIPI_PHY
#else
#define BASE_ADDR_CSI_TX0  0xFFFFFFC004010000
#define BASE_ADDR_CSI_TX1  0xFFFFFFC004011000
#define BASE_ADDR_CSI_RX0  0xFFFFFFC004012000
#define BASE_ADDR_CSI_RX1  0xFFFFFFC004013000
#define BASE_ADDR_CSI_RX2  0xFFFFFFC004014000
#define BASE_ADDR_MIPI_PHY 0xFFFFFFC004015000
#define BASE_ADDR_MIPI_TOP BASE_ADDR_MIPI_PHY
#endif

uint64_t dev_addr_map[MIPI_MAX] = {
	[MIPI_TX0] = BASE_ADDR_CSI_TX0,
	[MIPI_TX1] = BASE_ADDR_CSI_TX1,
	[MIPI_RX0] = BASE_ADDR_CSI_RX0,
	[MIPI_RX1] = BASE_ADDR_CSI_RX1,
	[MIPI_RX2] = BASE_ADDR_CSI_RX2,
	[MIPI_TOP] = BASE_ADDR_MIPI_TOP
};

void mipicsi_util_save_virt_addr(enum mipicsi_top_dev dev, void *base_addr)
{
	dev_addr_map[dev] = base_addr;
}

uint32_t mipicsi_read(enum mipicsi_top_dev dev, uint64_t offset)
{
	uint32_t data;

	data = ioread32((uint64_t *)(dev_addr_map[dev] + offset));
	return data;
}

void mipicsi_write(enum mipicsi_top_dev dev, uint64_t offset, uint32_t data)
{
	iowrite32(data, (uint64_t *)(dev_addr_map[dev] + offset));
}

void mipicsi_write_part(enum mipicsi_top_dev dev, uint32_t offset,
			uint32_t data, uint8_t shift, uint8_t width)
{
	uint32_t mask = (1 << width) - 1;
	uint32_t temp = mipicsi_read(dev, offset);

	temp &= ~(mask << shift);
	temp |= (data & mask) << shift;
	mipicsi_write(dev, offset, temp);
}
