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
#include "paintbox-regs.h"

/* The caller to these functions must hold pb->lock */
int set_dma_image_parameters(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer,
		struct dma_image_config *config);

int set_dma_transfer_region_parameters(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer,
		struct dma_transfer_config *config);

static inline void paintbox_dma_set_channel_mode(
		struct paintbox_dma_transfer *transfer, uint64_t src_type,
		uint64_t dst_type, bool gather)
{
	transfer->chan_mode = src_type << DMA_CHAN_MODE_SRC_SHIFT;
	transfer->chan_mode |= dst_type << DMA_CHAN_MODE_DST_SHIFT;
	transfer->chan_mode |= DMA_CHAN_MODE_CHAN_ENA_MASK;

	if (gather)
		transfer->chan_mode |= DMA_CHAN_MODE_GATHER_MASK;
}

static inline void paintbox_dma_set_lb_start(
		struct paintbox_dma_transfer *transfer, uint64_t x, uint64_t y)
{
	transfer->chan_img_pos |= (x << DMA_CHAN_IMG_POS_LB_START_X_SHIFT) &
			DMA_CHAN_IMG_POS_LB_START_X_MASK;
	transfer->chan_img_pos |= (y << DMA_CHAN_IMG_POS_LB_START_Y_SHIFT) &
			DMA_CHAN_IMG_POS_LB_START_Y_MASK;
}

static inline void paintbox_dma_set_img_start(
		struct paintbox_dma_transfer *transfer, uint64_t x, uint64_t y)
{
	transfer->chan_img_pos |= (x << DMA_CHAN_IMG_POS_START_X_SHIFT) &
			DMA_CHAN_IMG_POS_START_X_MASK;
	transfer->chan_img_pos |= (y << DMA_CHAN_IMG_POS_START_Y_SHIFT) &
			DMA_CHAN_IMG_POS_START_Y_MASK;
}

#endif  /* __PAINTBOX_DMA_COMMON_H__ */
