/*
 * Paintbox V1 specific MIPI Support for Paintbox programmable IPU
 *
 * Copyright (C) 2017 Google, Inc.
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

#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <uapi/paintbox.h>

#include "paintbox-dma.h"
#include "paintbox-io.h"
#include "paintbox-mipi.h"
#include "paintbox-mipi-debug.h"
#include "paintbox-mipi-v1.h"
#include "paintbox-regs.h"

/* TODO(ahampson):  This function will enable several streams simultaneously but
 * it does not verify that all the streams are ready to be enabled or are in the
 * same state.  It also does not attempt to synchronize the stream update with
 * with any frame boundaries if the stream is already running.
 */
void paintbox_mipi_enable_multiple_input(struct paintbox_data *pb,
		struct paintbox_session *session, uint32_t stream_id_mask,
		struct mipi_stream_enable_multiple *req)
{
	struct paintbox_mipi_stream *stream, *stream_next;
	unsigned long irq_flags;
	uint64_t ctrl_set_mask = 0;
	uint32_t imr_set_mask = 0, imr_err_set_mask = 0;

	spin_lock_irqsave(&pb->io_ipu.mipi_lock, irq_flags);

	list_for_each_entry_safe(stream, stream_next,
			&session->mipi_input_list, session_entry) {
		if (!req->enable_all &&
				!(stream_id_mask & (1 << stream->stream_id)))
			continue;

		if (stream->enabled) {
			paintbox_mipi_update_stream_count(pb, stream,
					req->free_running, req->frame_count);
			continue;
		}

		paintbox_mipi_enable_input_stream_common(pb, stream,
				req->free_running, req->frame_count,
				req->input.disable_on_error);

		imr_set_mask |= 1 << (stream->stream_id + MPI_IMR_SOF0_SHIFT);
		imr_err_set_mask |= 1 << (stream->stream_id +
				MPI_ERR_IMR_OVF0_SHIFT);
		ctrl_set_mask |= MPI_CTRL_STRM_ENA_SET0_MASK <<
				stream->stream_id;

		/* TODO(ahampson, showarth):  Implement support for single shot
		 * mode.  In the interim, continuous mode will always be used
		 * with the stream disabled explicitly from the userspace or
		 * when the requested number of frames has been met.
		 */
		ctrl_set_mask |= MPI_CTRL_STRM_CONTINUOUS_SET0_MASK <<
				stream->stream_id;
	}

	mipi_input_set_imr(pb, imr_set_mask);
	mipi_input_set_err_imr(pb, imr_err_set_mask);

	writeq(ctrl_set_mask, pb->io_ipu.ipu_base + MPI_CTRL);

	spin_unlock_irqrestore(&pb->io_ipu.mipi_lock, irq_flags);
}

/* TODO(ahampson):  This function will enable several streams simultaneously but
 * it does not verify that all the streams are ready to be enabled or are in the
 * same state.  It also does not attempt to synchronize the stream update with
 * with any frame boundaries if the stream is already running.
 */
void paintbox_mipi_enable_multiple_output(struct paintbox_data *pb,
		struct paintbox_session *session, uint32_t stream_id_mask,
		struct mipi_stream_enable_multiple *req)
{
	struct paintbox_mipi_stream *stream, *stream_next;
	unsigned long irq_flags;
	uint64_t ctrl_set_mask = 0, ctrl_clear_mask = 0;
	uint32_t imr_set_mask = 0;

	spin_lock_irqsave(&pb->io_ipu.mipi_lock, irq_flags);

	list_for_each_entry_safe(stream, stream_next,
			&session->mipi_output_list, session_entry) {
		if (!req->enable_all &&
				!(stream_id_mask & (1 << stream->stream_id)))
			continue;

		if (stream->enabled) {
			paintbox_mipi_update_stream_count(pb, stream,
					req->free_running, req->frame_count);
			continue;
		}

		/* frame count is zero when in free running mode. */
		stream->free_running = req->free_running;

		/* set the frame count to the value passed in minus one
		 * since the stream will start the output frame on
		 * enable.
		 */
		stream->frame_count = req->free_running ? 0 :
				req->frame_count - 1;
		stream->is_clean = false;
		stream->enabled = true;

		enable_mipi_interface(pb, stream);

		/* TODO(showarth): wait for last frame. b/38357562 */

		paintbox_mipi_select_output_stream(pb, stream->stream_id);

		/* {set,clear} RSYNC_EN in case previously {cleared,set} */
		mipi_output_clear_and_set_control(pb, stream,
				req->output.enable_row_sync ? 0 :
				MPO_STRM_CTRL_RSYNC_EN_MASK,
				req->output.enable_row_sync ?
				MPO_STRM_CTRL_RSYNC_EN_MASK : 0);

		imr_set_mask |= 1 << (stream->stream_id + MPO_IMR_EOF0_SHIFT);

		ctrl_set_mask |= MPO_CTRL_STRM_ENA_SET0_MASK <<
				stream->stream_id;

		/* TODO(ahampson, showarth):  Implement support for single shot
		 * mode.  In the interim, continuous mode will always be used
		 * with the stream disabled explicitly from the userspace or
		 * when the requested number of frames has been met.
		 */
		ctrl_set_mask |= MPO_CTRL_STRM_CONTINUOUS_SET0_MASK <<
				stream->stream_id;

		stream->last_frame = !req->free_running &&
				req->frame_count == 0;

		/* TODO(ahampson, showarth):  Clear continuous mode for any
		 * streams that are on their last frame.  This should not be
		 * necessary once single shot mode is fully supported.
		 */
		if (stream->last_frame)
			ctrl_clear_mask |= MPO_CTRL_STRM_CONTINUOUS_CLR0_MASK;
	}

	mipi_output_set_imr(pb, imr_set_mask);

	writeq(ctrl_set_mask, pb->io_ipu.ipu_base + MPO_CTRL);

	if (!ctrl_clear_mask)
		writeq(ctrl_clear_mask, pb->io_ipu.ipu_base + MPO_CTRL);

	spin_unlock_irqrestore(&pb->io_ipu.mipi_lock, irq_flags);
}

void paintbox_mipi_disable_multiple_input(struct paintbox_data *pb,
		struct paintbox_session *session, uint32_t stream_id_mask,
		struct mipi_stream_disable_multiple *req)
{
	struct paintbox_mipi_stream *stream, *stream_next;
	uint64_t ctrl_clear_mask = 0;
	uint32_t imr_clear_mask = 0;
	unsigned long irq_flags;

	spin_lock_irqsave(&pb->io_ipu.mipi_lock, irq_flags);

	list_for_each_entry_safe(stream, stream_next, &session->mipi_input_list,
			session_entry) {
		if (!req->disable_all &&
				!(stream_id_mask & (1 << stream->stream_id)))
			continue;

		paintbox_mipi_disable_stream_common(pb, stream);

		/* Do not disable the overflow interrupt on disable.  This
		 * interrupt reports on the state of the current frame and needs
		 * to be left enabled.
		 */
		imr_clear_mask |= 1 << (stream->stream_id + MPI_IMR_SOF0_SHIFT);

		ctrl_clear_mask |= MPI_CTRL_STRM_CONTINUOUS_CLR0_MASK <<
				stream->stream_id;
	}

	mipi_input_clear_imr(pb, imr_clear_mask);

	writeq(ctrl_clear_mask, pb->io_ipu.ipu_base + MPI_CTRL);

	spin_unlock_irqrestore(&pb->io_ipu.mipi_lock, irq_flags);
}

void paintbox_mipi_disable_multiple_output(struct paintbox_data *pb,
		struct paintbox_session *session, uint32_t stream_id_mask,
		struct mipi_stream_disable_multiple *req)
{
	struct paintbox_mipi_stream *stream, *stream_next;
	uint64_t ctrl_clear_mask = 0;
	unsigned long irq_flags;

	spin_lock_irqsave(&pb->io_ipu.mipi_lock, irq_flags);

	list_for_each_entry_safe(stream, stream_next,
			&session->mipi_output_list, session_entry) {
		if (!req->disable_all &&
				!(stream_id_mask & (1 << stream->stream_id)))
			continue;

		paintbox_mipi_disable_stream_common(pb, stream);

		/* Do not disable the EOF interrupt on disable. This interrupt
		 * reports on the state of the current frame and needs to be
		 * left enabled.
		 */
		ctrl_clear_mask |= MPO_CTRL_STRM_CONTINUOUS_CLR0_MASK <<
				stream->stream_id;
	}

	writeq(ctrl_clear_mask, pb->io_ipu.ipu_base + MPO_CTRL);

	spin_unlock_irqrestore(&pb->io_ipu.mipi_lock, irq_flags);
}
