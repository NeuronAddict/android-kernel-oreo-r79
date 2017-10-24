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
#include "paintbox-regs.h"

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

int paintbox_mipi_enable_multiple_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg,
		bool is_input);
int paintbox_mipi_disable_multiple_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg,
		bool is_input);

int paintbox_mipi_input_wait_for_quiescence_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);

/* The caller to these functions must hold pb->lock.*/
void paintbox_mipi_enable_multiple_input(struct paintbox_data *pb,
		struct paintbox_session *session, uint32_t stream_id_mask,
		struct mipi_stream_enable_multiple *req);
void paintbox_mipi_enable_multiple_output(struct paintbox_data *pb,
		struct paintbox_session *session, uint32_t stream_id_mask,
		struct mipi_stream_enable_multiple *req);
void paintbox_mipi_disable_multiple_input(struct paintbox_data *pb,
		struct paintbox_session *session, uint32_t stream_id_mask,
		struct mipi_stream_disable_multiple *req);
void paintbox_mipi_disable_multiple_output(struct paintbox_data *pb,
		struct paintbox_session *session, uint32_t stream_id_mask,
		struct mipi_stream_disable_multiple *req);

/* The caller to this function must hold pb->lock and pb->io_ipu.mipi_lock. */
void paintbox_mipi_update_stream_count(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream, bool free_running,
		int32_t frame_count);

/* The caller to this function must hold pb->lock and pb->io_ipu.mipi_lock. */
void paintbox_mipi_enable_input_stream_common(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream, bool free_running,
		int32_t frame_count, bool disable_on_error);

/* The caller to this function must hold pb->lock and pb->io_ipu.mipi_lock. */
void paintbox_mipi_disable_stream_common(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream);

/* The caller to this function must hold pb->io_ipu.mipi_lock. */
void enable_mipi_interface(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream);

/* The caller to this function must hold pb->lock */
void release_mipi_stream(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_mipi_stream *stream);

/* The caller to this function must hold pb->lock */
int paintbox_mipi_enable_input_stream(struct paintbox_data *pb,
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

#if CONFIG_PAINTBOX_VERSION_MAJOR >= 1
/* This functions must be called from an interrupt context. */
irqreturn_t paintbox_mipi_input_error_interrupt(struct paintbox_data *pb,
		ktime_t timestamp);
#endif

/* This function must be called from an interrupt context. */
irqreturn_t paintbox_mipi_output_interrupt(struct paintbox_data *pb,
		uint32_t stream_mask, ktime_t timestamp);

#ifdef CONFIG_PAINTBOX_TEST_SUPPORT
int mipi_test_stream_reset_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg,
		bool is_input);
#endif

/* The caller to this function must hold pb->lock and pb->io_ipu.mipi_lock. */
int enable_mipi_output_stream(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream, bool free_running,
		int32_t frame_count, bool enable_row_sync);

/* The caller to this function must hold pb->io_ipu.mipi_lock. */
static inline void paintbox_mipi_select_input_stream(struct paintbox_data *pb,
		unsigned int stream_id)
{
	if (pb->io_ipu.selected_input_stream_id == stream_id)
		return;

	pb->io_ipu.selected_input_stream_id = stream_id;

	writel(stream_id, pb->io_ipu.ipu_base + MPI_STRM_SEL);
}

/* The caller to this function must hold pb->io_ipu.mipi_lock. */
static inline void paintbox_mipi_select_output_stream(struct paintbox_data *pb,
		unsigned int stream_id)
{
	if (pb->io_ipu.selected_output_stream_id == stream_id)
		return;

	pb->io_ipu.selected_output_stream_id = stream_id;

	writel(stream_id, pb->io_ipu.ipu_base + MPO_STRM_SEL);
}

/* The caller to this function must hold pb->io_ipu.mipi_lock. */
static inline void paintbox_mipi_select_stream(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream)
{
	if (stream->is_input)
		paintbox_mipi_select_input_stream(pb, stream->stream_id);
	else
		paintbox_mipi_select_output_stream(pb, stream->stream_id);

}

void paintbox_mipi_post_ipu_reset(struct paintbox_data *pb);

int paintbox_mipi_init(struct paintbox_data *pb);

/* The caller to this function must hold pb->lock */
void paintbox_mipi_release(struct paintbox_data *pb,
		struct paintbox_session *session);

/* All sessions must be released before remove can be called. */
void paintbox_mipi_remove(struct paintbox_data *pb);

#endif  /* __PAINTBOX_MIPI_H__ */
