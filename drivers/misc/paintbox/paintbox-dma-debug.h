/*
 * DMA debug support for the Paintbox programmable IPU
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

#ifndef __PAINTBOX_DMA_DEBUG_H__
#define __PAINTBOX_DMA_DEBUG_H__

#include <linux/types.h>

#include "paintbox-common.h"


#define DMA_DEBUG_BUFFER_SIZE (DMA_NUM_REGS * REG_DEBUG_BUFFER_SIZE)

#ifdef VERBOSE_DEBUG
void log_dma_registers(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel, const char *msg);

#define LOG_DMA_REGISTERS(pb, channel)		\
	log_dma_registers(pb, channel, __func__)

#else
#define LOG_DMA_REGISTERS(pb, channel)		\
do { } while (0)
#endif

int dump_dma_registers(struct paintbox_debug *debug, char *buf,
		size_t len);
int dump_dma_channel_registers(struct paintbox_debug *debug, char *buf,
		size_t len);

void paintbox_dma_debug_init(struct paintbox_data *pb);
void paintbox_dma_channel_debug_init(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel);

#endif  /* __PAINTBOX_DMA_DEBUG_H__ */
