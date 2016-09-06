/*
 * Core driver for the Paintbox programmable IPU
 *
 * Copyright (C) 2015 Google, Inc.
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

#ifndef __PAINTBOX_DMA_H__
#define __PAINTBOX_DMA_H__

#include <linux/interrupt.h>
#include <linux/types.h>

#include "paintbox-common.h"


/* The caller to this function must hold pb->lock */
void dma_select_channel(struct paintbox_data *pb, uint32_t channel_id);

/* DMA Ioctl Handlers */
int allocate_dma_channel_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);

int release_dma_channel_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);

int bind_dma_interrupt_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);

int unbind_dma_interrupt_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);

int setup_dma_transfer_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);

int read_dma_transfer_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);

int start_dma_transfer_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);

int get_completed_transfer_count_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);

int paintbox_dma_init(struct paintbox_data *pb);

irqreturn_t paintbox_dma_interrupt(struct paintbox_data *pb,
		uint32_t channel_mask);

/* The caller to these functions must hold pb->lock */
int validate_dma_channel(struct paintbox_data *pb,
		struct paintbox_session *session, uint8_t channel_id);

struct paintbox_dma_channel *get_dma_channel(struct paintbox_data *pb,
		struct paintbox_session *session, uint8_t channel_id, int *err);

int release_dma_channel(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_dma_channel *dma);

int dma_start_transfer(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel);

int dma_stop_transfer(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel);

#endif  /* __PAINTBOX_DMA_H__ */
