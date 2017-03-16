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
#include <linux/ktime.h>
#include <linux/types.h>

#include "paintbox-common.h"

/* MAX_ACTIVE_TRANSFERS is number of transfers that can be queued in hardware.
 */
#define MAX_ACTIVE_TRANSFERS 2

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

int stop_dma_transfer_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);

int get_completed_transfer_count_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);

int flush_dma_transfers_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);

int paintbox_dma_init(struct paintbox_data *pb);

/* This function must be called in an interrupt context */
irqreturn_t paintbox_dma_interrupt(struct paintbox_data *pb,
		uint32_t channel_mask, ktime_t timestamp);

/* This function must be called in an interrupt context */
void dma_report_channel_error(struct paintbox_data *pb,
		unsigned int channel_id, int err);

/* This function must be called in an interrupt context */
void dma_report_error_all_channels(struct paintbox_data *pb, int err);

/* This function must be called in an interrupt context */
void dma_set_mipi_error(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel, int err);

/* This function must be called in an interrupt context */
void dma_report_mipi_output_completed(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel);

/* The caller to this function must hold pb->lock */
int validate_dma_channel(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned int channel_id);

/* The caller to this function must hold pb->lock */
struct paintbox_dma_channel *get_dma_channel(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned int channel_id,
		int *err);

/* The caller to this function must hold pb->lock */
void release_dma_channel(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_dma_channel *dma);

/* The caller to this function must hold pb->lock */
void dma_reset_channel(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel);

/* The caller to this function must hold pb->lock */
int dma_start_transfer(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel);

/* The caller to this function must hold pb->lock */
void dma_stop_transfer(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel);

/* The caller to this function must hold pb->lock */
struct paintbox_dma_channel *dma_handle_mipi_stream_allocated(
		struct paintbox_data *pb, struct paintbox_session *session,
		struct paintbox_mipi_stream *stream, unsigned int channel_id,
		int *ret);

/* The caller to this function must hold pb->lock */
void dma_handle_mipi_stream_released(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel);

#ifdef CONFIG_PAINTBOX_TEST_SUPPORT
int dma_test_reset_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);

int dma_test_channel_reset_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);
#endif

#endif  /* __PAINTBOX_DMA_H__ */
