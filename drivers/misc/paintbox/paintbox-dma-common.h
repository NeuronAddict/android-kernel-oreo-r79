/*
 * DMA common support for the Paintbox programmable IPU
 *
 * Copyright (C) 2016 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __PAINTBOX_DMA_COMMON_H__
#define __PAINTBOX_DMA_COMMON_H__

#include <linux/types.h>

#include "paintbox-common.h"

/* The caller to these functions must hold pb->lock */
void set_dma_channel_mode(struct paintbox_dma_transfer *transfer,
		uint32_t src_type, uint32_t dst_type, bool gather);

int set_dma_image_parameters(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer,
		struct dma_image_config *config);

int set_dma_transfer_region_parameters(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer,
		struct dma_transfer_config *config);

#endif  /* __PAINTBOX_DMA_COMMON_H__ */
