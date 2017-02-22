/*
 * MIPI Support for Paintbox programmable IPU
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

#ifndef __PAINTBOX_MIPI_H__
#define __PAINTBOX_MIPI_H__

#include <linux/interrupt.h>
#include <linux/types.h>

#include "paintbox-common.h"

int allocate_mipi_input_stream_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);
int allocate_mipi_output_stream_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);
int release_mipi_stream_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg,
		bool is_input);
int setup_mipi_stream_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg,
		bool is_input);
int enable_mipi_stream_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg,
		bool is_input);
int disable_mipi_stream_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg,
		bool is_input);
int get_mipi_frame_number_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);
int cleanup_mipi_stream_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg,
		bool is_input);
int wait_for_mipi_interrupt_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg,
		bool is_input);
int bind_mipi_interrupt_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg,
		bool is_input);
int unbind_mipi_interrupt_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg,
		bool is_input);

/* The caller to this function must hold pb->lock */
void release_mipi_stream(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_mipi_stream *stream);

/* The caller to this function must hold pb->lock */
struct paintbox_mipi_stream *mipi_handle_dma_channel_allocated(
		struct paintbox_data *pb, struct paintbox_session *session,
		struct paintbox_dma_channel *channel);

/* The caller to this function must hold pb->lock */
void mipi_handle_dma_channel_released(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream);

/* The caller to this function must hold pb->lock */
void cleanup_mipi_stream(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream);

/* The caller to this function must hold pb->lock. */
void mipi_request_cleanup(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream);

/* The caller to this function must hold pb->lock. */
int verify_cleanup_completion(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream);

/* This function must be called from an interrupt context */
void mipi_input_handle_dma_completed(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream);

/* This function must be called from an interrupt context. */
irqreturn_t paintbox_mipi_input_interrupt(struct paintbox_data *pb,
		uint32_t stream_mask, ktime_t timestamp);

/* This function must be called from an interrupt context. */
irqreturn_t paintbox_mipi_output_interrupt(struct paintbox_data *pb,
		uint32_t stream_mask, ktime_t timestamp);

int paintbox_mipi_init(struct paintbox_data *pb);

#ifdef CONFIG_PAINTBOX_TEST_SUPPORT
int mipi_test_stream_reset_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg,
		bool is_input);
#endif

#endif  /* __PAINTBOX_MIPI_H__ */
