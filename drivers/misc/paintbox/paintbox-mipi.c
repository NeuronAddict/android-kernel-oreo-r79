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

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/paintbox.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>

#include "paintbox-dma.h"
#include "paintbox-debug.h"
#include "paintbox-io.h"
#include "paintbox-irq.h"
#include "paintbox-mipi.h"
#include "paintbox-mipi-debug.h"
#include "paintbox-regs.h"

/* TODO(ahampson):  The mapping here below applies to Easel but may not be the
 * same in future versions of the hardware.  This mapping should be moved into
 * the device tree.  b/32283059
 *
 * MIPI input streams 0..11 have a fixed mapping to DMA channels 0..11.
 * MIPI output streams 0..1 have a fixed mapping to DMA channels 14..15.
 */
#define MIPI_INPUT_DMA_CHANNEL_ID_START  0
#define MIPI_INPUT_DMA_CHANNEL_ID_END    11
#define MIPI_OUTPUT_DMA_CHANNEL_ID_START 14
#define MIPI_OUTPUT_DMA_CHANNEL_ID_END   15

static inline unsigned int mipi_stream_to_dma_channel_id(
		struct paintbox_mipi_stream *stream)
{
	return stream->is_input ? stream->stream_id +
			MIPI_INPUT_DMA_CHANNEL_ID_START:
			stream->stream_id + MIPI_OUTPUT_DMA_CHANNEL_ID_START;
}

/* The caller to this function must hold pb->lock */
static int validate_mipi_input_stream(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned int stream_id)
{
	if (stream_id >= pb->io_ipu.num_mipi_input_streams) {
		dev_err(&pb->pdev->dev, "%s: invalid stream id %u\n", __func__,
				stream_id);
		return -EINVAL;
	}

	if (pb->io_ipu.mipi_input_streams[stream_id].session != session) {
		dev_err(&pb->pdev->dev, "%s: access error, stream id %d\n",
				__func__, stream_id);
		return -EACCES;
	}

	return 0;
}

/* The caller to this function must hold pb->lock */
static int validate_mipi_output_stream(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned int stream_id)
{
	if (stream_id >= pb->io_ipu.num_mipi_output_streams) {
		dev_err(&pb->pdev->dev, "%s: invalid stream id %u\n", __func__,
				stream_id);
		return -EINVAL;
	}

	if (pb->io_ipu.mipi_output_streams[stream_id].session != session) {
		dev_err(&pb->pdev->dev, "%s: access error, stream id %d\n",
				__func__, stream_id);
		return -EACCES;
	}

	return 0;
}

/* The caller to this function must hold pb->lock */
struct paintbox_mipi_stream *get_mipi_stream(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned int stream_id,
		bool is_input, int *err)
{
	struct paintbox_mipi_stream *stream;
	int ret;

	if (is_input) {
		ret = validate_mipi_input_stream(pb, session, stream_id);
		stream = &pb->io_ipu.mipi_input_streams[stream_id];
	} else {
		ret = validate_mipi_output_stream(pb, session, stream_id);
		stream = &pb->io_ipu.mipi_output_streams[stream_id];
	}
	if (ret < 0) {
		*err = ret;
		return NULL;
	}

	*err = 0;
	return stream;
}

int allocate_mipi_input_stream_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	unsigned int stream_id = (unsigned int)arg;
	struct paintbox_mipi_stream *stream;
	struct paintbox_dma_channel *channel;
	unsigned long irq_flags;
	int ret = 0;

	if (stream_id >= pb->io_ipu.num_mipi_input_streams) {
		dev_err(&pb->pdev->dev, "%s: invalid stream_id %d\n", __func__,
				stream_id);
		return -EINVAL;
	}

	mutex_lock(&pb->lock);
	stream = &pb->io_ipu.mipi_input_streams[stream_id];
	if (stream->session) {
		dev_err(&pb->pdev->dev, "%s: access error stream_id %d\n",
				__func__, stream_id);
		mutex_unlock(&pb->lock);
		return -EACCES;
	}

	/* MIPI streams and DMA channels have a fixed mapping, i.e. a specific
	 * DMA channel must be used for transfers from a particular MIPI stream.
	 * Notify the DMA code that this MIPI stream has been allocated to the
	 * session.
	 */
	channel = dma_handle_mipi_stream_allocated(pb, session, stream,
			mipi_stream_to_dma_channel_id(stream), &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	stream->session = session;
	list_add_tail(&stream->session_entry, &session->mipi_input_list);

	spin_lock_irqsave(&pb->io_ipu.mipi_lock, irq_flags);

	stream->dma_channel = channel;

	spin_unlock_irqrestore(&pb->io_ipu.mipi_lock, irq_flags);

	mutex_unlock(&pb->lock);

	return 0;
}

int allocate_mipi_output_stream_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	unsigned int stream_id = (unsigned int)arg;
	struct paintbox_mipi_stream *stream;
	struct paintbox_dma_channel *channel;
	unsigned long irq_flags;
	int ret;

	if (stream_id >= pb->io_ipu.num_mipi_output_streams) {
		dev_err(&pb->pdev->dev, "%s: invalid stream_id %d\n", __func__,
				stream_id);
		return -EINVAL;
	}

	mutex_lock(&pb->lock);
	stream = &pb->io_ipu.mipi_output_streams[stream_id];
	if (stream->session) {
		dev_err(&pb->pdev->dev, "%s: access error stream_id %d\n",
				__func__, stream_id);
		mutex_unlock(&pb->lock);
		return -EACCES;
	}

	/* MIPI streams and DMA channels have a fixed mapping, i.e. a specific
	 * DMA channel must be used for transfers from a particular MIPI stream.
	 * Notify the DMA code that this MIPI stream has been allocated to the
	 * session.
	 */
	channel = dma_handle_mipi_stream_allocated(pb, session, stream,
			mipi_stream_to_dma_channel_id(stream), &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	stream->session = session;
	list_add_tail(&stream->session_entry, &session->mipi_output_list);

	spin_lock_irqsave(&pb->io_ipu.mipi_lock, irq_flags);

	stream->dma_channel = channel;

	spin_unlock_irqrestore(&pb->io_ipu.mipi_lock, irq_flags);

	mutex_unlock(&pb->lock);

	return 0;
}

/* The caller to this function must hold pb->lock. */
void release_mipi_stream(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_mipi_stream *stream)
{
	unsigned long irq_flags;

	/* Disable the MIPI stream and the stream interrupts. */
	spin_lock_irqsave(&pb->io_ipu.mipi_lock, irq_flags);

	writel(stream->stream_id, pb->io_ipu.ipu_base + stream->select_offset);
	writel(0, pb->io_ipu.ipu_base + stream->ctrl_offset);

	stream->enabled = false;

	spin_unlock_irqrestore(&pb->io_ipu.mipi_lock, irq_flags);

	/* Stop the DMA channel associated with this MIPI stream. If there is no
	 * dma_channel then there is no need to go through the stop procedures
	 * because it was already done when the DMA channel was released.
	 */
	if (stream->dma_channel) {
		dma_stop_transfer(pb, stream->dma_channel);

		/* Notify the DMA channel associated with this MIPI stream that
		 * the stream has been released.
		 */
		dma_handle_mipi_stream_released(pb, stream->dma_channel);

		spin_lock_irqsave(&pb->io_ipu.mipi_lock, irq_flags);

		stream->dma_channel = NULL;

		spin_unlock_irqrestore(&pb->io_ipu.mipi_lock, irq_flags);
	}

	/* Remove the MIPI stream from the session. */
	list_del(&stream->session_entry);
	stream->session = NULL;
}

/* The caller to this function must hold pb->lock */
void mipi_handle_dma_channel_released(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream)
{
	unsigned long irq_flags;

	/* Disable the MIPI stream and the stream interrupts. */
	spin_lock_irqsave(&pb->io_ipu.mipi_lock, irq_flags);

	writel(stream->stream_id, pb->io_ipu.ipu_base + stream->select_offset);
	writel(0, pb->io_ipu.ipu_base + stream->ctrl_offset);
	stream->enabled = false;

	stream->dma_channel = NULL;

	spin_unlock_irqrestore(&pb->io_ipu.mipi_lock, irq_flags);
}

/* The caller to this function must hold pb->lock */
struct paintbox_mipi_stream * mipi_handle_dma_channel_allocated(
		struct paintbox_data *pb, struct paintbox_session *session,
		struct paintbox_dma_channel *channel)
{
	struct paintbox_mipi_stream *stream;
	unsigned long irq_flags;
	unsigned int stream_id;

	if (channel->channel_id <= MIPI_INPUT_DMA_CHANNEL_ID_END) {
		stream_id = channel->channel_id;
		stream = &pb->io_ipu.mipi_input_streams[stream_id];
	} else if (channel->channel_id >= MIPI_OUTPUT_DMA_CHANNEL_ID_START &&
			channel->channel_id <= MIPI_OUTPUT_DMA_CHANNEL_ID_END) {
		stream_id = channel->channel_id -
				MIPI_OUTPUT_DMA_CHANNEL_ID_START;
		stream = &pb->io_ipu.mipi_output_streams[stream_id];
	} else {
		/* There is no MIPI stream associated with this DMA channel,
		 * nothing to do.  Note, this is not an error since this
		 * function will be called for all DMA channels.
		 */
		return NULL;
	}

	/* If the stream has not been allocated to a session then return with no
	 * error.  This is not an error if the DMA channel is allocated before
	 * the MIPI stream.  Cross-linking the DMA channel with the MIPI stream
	 * will be handled in dma_mipi_stream_allocated() once the MIPI stream
	 * is allocated.
	 */
	if (stream->session == NULL)
		return NULL;

	/* If the MIPI stream is owned by a different session then it can not be
	 * associated with this DMA channel.  This is only a problem if the user
	 * intends to use the DMA channel for MIPI transfers.  The DMA channel
	 * may be used legitimately for non-MIPI transfers so no error will be
	 * returned.
	 */
	if (stream->session != session)
		return NULL;

	/* There should not be a DMA channel object already associated with this
	 * stream.  If there is then there is a bug in cleanup.
	 */
	WARN_ON(stream->dma_channel);

	spin_lock_irqsave(&pb->io_ipu.mipi_lock, irq_flags);

	stream->dma_channel = channel;

	spin_unlock_irqrestore(&pb->io_ipu.mipi_lock, irq_flags);

	return stream;
}

int release_mipi_stream_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg,
		bool is_input)
{
	unsigned int stream_id = (unsigned int)arg;
	struct paintbox_mipi_stream *stream;
	int ret = 0;

	mutex_lock(&pb->lock);

	stream = get_mipi_stream(pb, session, stream_id, is_input, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	unbind_mipi_interrupt(pb, session, stream);

	release_mipi_stream(pb, session, stream);

	mutex_unlock(&pb->lock);

	return 0;
}

/* The caller to this function must hold pb->lock and pb->io_ipu.mipi_lock. */
static inline void strm_ctrl_toggle(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream, uint32_t toggle_value)
{
	uint32_t val;

	val = readl(pb->io_ipu.ipu_base + stream->ctrl_offset);
	val |= toggle_value;
	writel(val, pb->io_ipu.ipu_base + stream->ctrl_offset);
	val &= ~toggle_value;
	writel(val, pb->io_ipu.ipu_base + stream->ctrl_offset);
}

/* The caller to this function must hold pb->lock and pb->io_ipu.mipi_lock. */
static inline void strm_ctrl_set(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream, uint32_t new_val)
{
	uint32_t val;

	val = readl(pb->io_ipu.ipu_base + stream->ctrl_offset);
	val |= new_val;
	writel(val, pb->io_ipu.ipu_base + stream->ctrl_offset);
}

/* The caller to this function must hold pb->lock and pb->io_ipu.mipi_lock. */
static inline void strm_ctrl_clr(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream, uint32_t new_val)
{
	uint32_t val;

	val = readl(pb->io_ipu.ipu_base + stream->ctrl_offset);
	val &= ~new_val;
	writel(val, pb->io_ipu.ipu_base + stream->ctrl_offset);
}

/* The caller to this function must hold pb->io_ipu.mipi_lock. */
static void enable_mipi_interface_interrupt(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream)
{
	struct paintbox_mipi_interface *interface = stream->interface;
	if (interface->active_stream_mask == 0) {
		if (stream->is_input)
			io_enable_mipi_input_interface_interrupt(pb,
					interface->interface_id);
		else
			io_enable_mipi_output_interface_interrupt(pb,
					interface->interface_id);
	}

	interface->active_stream_mask |= 1 << stream->stream_id;
}

/* The caller to this function must hold pb->io_ipu.mipi_lock. */
static void disable_mipi_interface_interrupt(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream)
{
	struct paintbox_mipi_interface *interface = stream->interface;

	interface->active_stream_mask &= ~(1 << stream->stream_id);

	if (interface->active_stream_mask == 0) {
		if (stream->is_input)
			io_disable_mipi_input_interface_interrupt(pb,
					interface->interface_id);
		else
			io_disable_mipi_output_interface_interrupt(pb,
					interface->interface_id);
	}
}

/* The caller to this function must hold pb->lock and pb->io_ipu.mipi_lock.
 * The caller must also set the MPI/MPO stream select register to the correct
 * stream.
 */
static void enable_mipi_stream(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream, bool free_running,
		int32_t frame_count, bool enable_row_sync)
{
	uint32_t val;

	val = readl(pb->io_ipu.ipu_base + stream->ctrl_offset);

	val |= stream->is_input ?
			MPI_STRM_EN | MPI_STRM_SOF_IMR | MPI_STRM_OVF_IMR :
			MPO_STRM_EN | MPO_STRM_EOF_IMR;

	if (!stream->is_input && enable_row_sync)
		val |= MPO_STRM_RSYNC_EN;

	writel(val, pb->io_ipu.ipu_base + stream->ctrl_offset);

	enable_mipi_interface_interrupt(pb, stream);

	stream->enabled = true;
	stream->free_running = free_running;
	stream->is_clean = false;

	/* frame count is zero when in free running mode. */
	stream->frame_count = free_running ? 0 : frame_count;
}

/* The caller to this function must hold pb->lock and pb->io_ipu.mipi_lock.
 * The caller must also set the MPI/MPO stream select register to the correct
 * stream.
 */
static void disable_mipi_stream(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream)
{
	strm_ctrl_clr(pb, stream, stream->is_input ?
			MPI_STRM_EN | MPI_STRM_SOF_IMR | MPI_STRM_OVF_IMR :
			MPO_STRM_EN | MPO_STRM_RSYNC_EN | MPO_STRM_EOF_IMR);

	disable_mipi_interface_interrupt(pb, stream);

	stream->enabled = true;
	stream->free_running = false;
	stream->frame_count = 0;
}

int enable_mipi_stream_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg,
		bool is_input)
{
	struct mipi_stream_enable __user *user_req;
	struct mipi_stream_enable req;
	struct paintbox_mipi_stream *stream;
	unsigned long irq_flags;
	int ret = 0;

	user_req = (struct mipi_stream_enable __user *)arg;
	if (copy_from_user(&req, user_req, sizeof(req)))
		return -EFAULT;

	if (!req.free_running && req.frame_count <= 0) {
		dev_err(&pb->pdev->dev,
				"%s: mipi %u: stream is not free running and "
				"frame count is invalid, %d <= 0\n", __func__,
				req.stream_id, req.frame_count);
		return -EINVAL;
	}

	mutex_lock(&pb->lock);

	stream = get_mipi_stream(pb, session, req.stream_id, is_input, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	/* A stream can not be enabled if the corresponding DMA channel is not
	 * part of this session.
	 */
	if (!stream->dma_channel) {
		dev_err(&pb->pdev->dev,
				"%s: mipi %s stream%u: no corresponding DMA "
				"channel\n", __func__, is_input ? "input" :
				"output", stream->stream_id);
		mutex_unlock(&pb->lock);
		return -EINVAL;
	}

	spin_lock_irqsave(&pb->io_ipu.mipi_lock, irq_flags);

	writel(req.stream_id, pb->io_ipu.ipu_base + stream->select_offset);

	enable_mipi_stream(pb, stream, req.free_running, req.frame_count,
			is_input ? false : req.output.enable_row_sync);

	spin_unlock_irqrestore(&pb->io_ipu.mipi_lock, irq_flags);

	dev_dbg(&pb->pdev->dev, "%s: mipi %s stream%u: enable stream free "
			"running %d frame_count %u\n", __func__,
			is_input ? "input" : "output", req.stream_id,
			stream->free_running, stream->frame_count);

	mutex_unlock(&pb->lock);

	return 0;
}

int disable_mipi_stream_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg,
		bool is_input)
{
	struct paintbox_mipi_stream *stream;
	unsigned int stream_id = (unsigned int)arg;
	unsigned long irq_flags;
	int ret = 0;

	mutex_lock(&pb->lock);

	stream = get_mipi_stream(pb, session, stream_id, is_input, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	spin_lock_irqsave(&pb->io_ipu.mipi_lock, irq_flags);

	writel(stream_id, pb->io_ipu.ipu_base + stream->select_offset);

	disable_mipi_stream(pb, stream);

	spin_unlock_irqrestore(&pb->io_ipu.mipi_lock, irq_flags);

	dev_dbg(&pb->pdev->dev, "%s: mipi %s stream%u: disable stream\n",
			__func__, is_input ? "input" : "output", stream_id);

	mutex_unlock(&pb->lock);

	return 0;
}

int get_mipi_frame_number_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct paintbox_mipi_stream *stream;
	unsigned int stream_id = (unsigned int)arg;
	unsigned long irq_flags;
	uint32_t val;
	int ret = 0;

	mutex_lock(&pb->lock);

	stream = get_mipi_stream(pb, session, stream_id, true, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	spin_lock_irqsave(&pb->io_ipu.mipi_lock, irq_flags);

	writel(stream_id, pb->io_ipu.ipu_base + stream->select_offset);

	val = readl(pb->io_ipu.ipu_base + stream->ctrl_offset);

	ret = (int)((val & MPI_STRM_NUM_FRAME_MASK) >>
			MPI_STRM_NUM_FRAME_SHIFT);

	spin_unlock_irqrestore(&pb->io_ipu.mipi_lock, irq_flags);

	dev_dbg(&pb->pdev->dev, "%s: mipi input stream%u: frame number %d\n",
			__func__, stream_id, ret);

	mutex_unlock(&pb->lock);

	return ret;
}

static int validate_mipi_stream_setup(struct paintbox_data *pb,
		struct mipi_stream_setup *setup)
{
	if (setup->virtual_channel > MPI_VC_MAX) {
		dev_err(&pb->pdev->dev,
				"%s: mipi %u: virtual channel setting too "
				"large, %u > %u\n", __func__, setup->stream_id,
				setup->virtual_channel, MPI_VC_MAX);
		return -EINVAL;
	}

	if (setup->data_type > MPI_DT_IN_MAX) {
		dev_err(&pb->pdev->dev,
				"%s: mipi %u: data type invalid, %u\n",
				__func__, setup->stream_id, setup->data_type);
		return -EINVAL;
	}

	if (setup->unpacked_data_type > MPI_DT_PROC_MAX) {
		dev_err(&pb->pdev->dev,
				"%s: mipi %u: data proc invalid, %u\n",
				__func__, setup->stream_id,
				setup->unpacked_data_type);
		return -EINVAL;
	}

	if (setup->img_width > MPI_IMG_WIDTH_MAX) {
		dev_err(&pb->pdev->dev,
				"%s: mipi %u: invalid width %u > %u\n",
				__func__, setup->stream_id, setup->img_width,
				MPI_IMG_WIDTH_MAX);
		return -EINVAL;
	}

	if (setup->img_height > MPI_IMG_HEIGHT_MAX) {
		dev_err(&pb->pdev->dev,
				"%s: mipi %u: invalid height %u > %u\n",
				__func__, setup->stream_id, setup->img_height,
				MPI_IMG_HEIGHT_MAX);
		return -EINVAL;
	}

	if (setup->stripe_height > MPI_STRP_HEIGHT_MAX) {
		dev_err(&pb->pdev->dev,
				"%s: mipi %u: stripe height too large, %u > "
				"%u\n", __func__, setup->stream_id,
				setup->stripe_height, MPI_STRP_HEIGHT_MAX);
		return -EINVAL;
	}

	if (setup->enable_on_setup && !setup->free_running &&
			setup->frame_count <= 0) {
		dev_err(&pb->pdev->dev,
				"%s: mipi %u: stream is not free running and "
				"frame count is invalid, %d <= 0\n", __func__,
				setup->stream_id, setup->frame_count);
		return -EINVAL;
	}

	dev_dbg(&pb->pdev->dev,
			"\tvirtual channels: %u data type: %u data proc: %u\n",
			setup->virtual_channel, setup->data_type,
			setup->unpacked_data_type);
	dev_dbg(&pb->pdev->dev, "\twidth: %u height: %u stripe height: %u\n",
			setup->img_width, setup->img_height,
			setup->stripe_height);
	dev_dbg(&pb->pdev->dev, "\tfree running: %u frame count: %d\n",
			setup->free_running, setup->frame_count);

	return 0;
}

static int validate_mipi_input_stream_setup(struct paintbox_data *pb,
		struct mipi_stream_setup *setup)
{
	int ret;

	dev_dbg(&pb->pdev->dev, "mipi input stream%u\n", setup->stream_id);

	ret = validate_mipi_stream_setup(pb, setup);
	if (ret < 0)
		return ret;

	if (setup->input.seg_start > MPI_SEG_START_MAX) {
		dev_err(&pb->pdev->dev,
				"%s: mipi in%u: seg start too large, %u > %u\n",
				__func__, setup->stream_id,
				setup->input.seg_start, MPI_SEG_START_MAX);
		return -EINVAL;
	}

	if (setup->input.seg_end > MPI_SEG_END_MAX) {
		dev_err(&pb->pdev->dev,
				"%s: mipi in%u: seg end too large, %u > %u\n",
				__func__, setup->stream_id,
				setup->input.seg_end, MPI_SEG_END_MAX);
		return -EINVAL;
	}

	if (setup->input.seg_words_per_row > MPI_SEG_WORDS_PER_ROW_MAX) {
		dev_err(&pb->pdev->dev,
				"%s: mipi in%u: seg words per row too large, %u"
				"> %u\n", __func__, setup->stream_id,
				setup->input.seg_words_per_row,
				MPI_SEG_WORDS_PER_ROW_MAX);
		return -EINVAL;
	}

	if (setup->input.segs_per_row > MPI_SEGS_PER_ROW_MAX) {
		dev_err(&pb->pdev->dev,
				"%s: mipi in%u: segs per row too large, %u > %u"
				"\n", __func__, setup->stream_id,
				setup->input.segs_per_row,
				MPI_SEGS_PER_ROW_MAX);
		return -EINVAL;
	}

	dev_dbg(&pb->pdev->dev, "\tseg start %u, seg end %u, seg words per row "
			"%u segs per row %u\n",
			setup->input.seg_start, setup->input.seg_end,
			setup->input.seg_words_per_row,
			setup->input.segs_per_row);

	return 0;
}

static int validate_mipi_output_stream_setup(struct paintbox_data *pb,
		struct mipi_stream_setup *setup)
{
	int ret;

	dev_dbg(&pb->pdev->dev, "mipi output stream%u\n", setup->stream_id);

	ret = validate_mipi_stream_setup(pb, setup);
	if (ret < 0)
		return ret;

	if (setup->output.seg_end > MPO_SEG_END_MAX) {
		dev_err(&pb->pdev->dev,
				"%s: mipi out%u: seg end too large, %u > %u\n",
				__func__, setup->stream_id,
				setup->output.seg_end, MPO_SEG_END_MAX);
		return -EINVAL;
	}

	if (setup->output.segs_per_row > MPO_SEGS_PER_ROW_MAX) {
		dev_err(&pb->pdev->dev,
				"%s: mipi out%u: segs per row too large, %u > "
				"%u\n", __func__, setup->stream_id,
				setup->output.segs_per_row,
				MPO_SEGS_PER_ROW_MAX);
		return -EINVAL;
	}

	dev_dbg(&pb->pdev->dev, "\tsegs per row %u\n",
			setup->output.segs_per_row);

	return 0;
}

/* The caller to this function must hold pb->lock and pb->io_ipu.mipi_lock. */
int setup_mipi_input_stream(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream,
		struct mipi_stream_setup *setup)
{
	uint32_t val;
	int ret;

	ret = validate_mipi_input_stream_setup(pb, setup);
	if (ret < 0)
		return ret;

	val = setup->virtual_channel & MPI_VC_MASK;
	val |= (setup->data_type & MPI_DT_IN_M) << MPI_DT_IN_SHIFT;
	val |= (setup->unpacked_data_type & MPI_DT_PROC_M) << MPI_DT_PROC_SHIFT;
	val |= (setup->stripe_height & MPI_STRP_HEIGHT_M) <<
			MPI_STRP_HEIGHT_SHIFT;
	writel(val, pb->io_ipu.ipu_base + MPI_STRM_CNFG0_L);

	val = setup->img_width & MPI_IMG_WIDTH_MASK;
	val |= (setup->img_height & MPI_IMG_HEIGHT_M) << MPI_IMG_HEIGHT_SHIFT;
	writel(val, pb->io_ipu.ipu_base + MPI_STRM_CNFG0_H);

	val = setup->input.seg_start & MPI_SEG_START_MASK;
	val |= (setup->input.seg_end & MPI_SEG_END_M) <<  MPI_SEG_END_SHIFT;
	writel(val, pb->io_ipu.ipu_base + MPI_STRM_CNFG1_L);

	val = setup->input.segs_per_row & MPI_SEGS_PER_ROW_MASK;
	val |= (setup->input.seg_words_per_row & MPI_SEG_WORDS_PER_ROW_M) <<
			MPI_SEG_WORDS_PER_ROW_SHIFT;
	writel(val, pb->io_ipu.ipu_base + MPI_STRM_CNFG1_H);

	LOG_MIPI_REGISTERS(pb, stream);

	return 0;
}

/* The caller to this function must hold pb->lock and pb->io_ipu.mipi_lock. */
int setup_mipi_output_stream(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream,
		struct mipi_stream_setup *setup)
{
	uint32_t val;
	int ret;

	ret = validate_mipi_output_stream_setup(pb, setup);
	if (ret < 0)
		return ret;

	val = setup->virtual_channel & MPO_VC_MASK;
	val |= (setup->data_type & MPO_DT_OUT_M) << MPO_DT_OUT_SHIFT;
	val |= (setup->unpacked_data_type & MPO_DT_PROC_M) << MPO_DT_PROC_SHIFT;
	val |= (setup->stripe_height & MPO_STRP_HEIGHT_M) <<
			MPO_STRP_HEIGHT_SHIFT;
	writel(val, pb->io_ipu.ipu_base + MPO_STRM_CNFG0_L);

	val = setup->img_width & MPO_IMG_WIDTH_MASK;
	val |= (setup->img_height & MPO_IMG_HEIGHT_M) << MPO_IMG_HEIGHT_SHIFT;
	writel(val, pb->io_ipu.ipu_base + MPO_STRM_CNFG0_H);

	val = setup->output.seg_end & MPO_SEG_END_MASK;
	val |= (setup->output.segs_per_row & MPO_SEGS_PER_ROW_M) <<
			MPO_SEGS_PER_ROW_SHIFT;
	writel(val, pb->io_ipu.ipu_base + MPO_STRM_CNFG1);

	LOG_MIPI_REGISTERS(pb, stream);

	return 0;
}

int setup_mipi_stream_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg,
		bool is_input)
{
	struct mipi_stream_setup __user *user_setup;
	struct mipi_stream_setup setup;
	struct paintbox_mipi_stream *stream;
	unsigned long irq_flags;
	int ret = 0;

	user_setup = (struct mipi_stream_setup __user *)arg;
	if (copy_from_user(&setup, user_setup, sizeof(setup)))
		return -EFAULT;

	mutex_lock(&pb->lock);

	stream = get_mipi_stream(pb, session, setup.stream_id, is_input, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	spin_lock_irqsave(&pb->io_ipu.mipi_lock, irq_flags);

	writel(stream->stream_id, pb->io_ipu.ipu_base + stream->select_offset);

	/* Disable the stream while updating the stream configuration.  This is
	 * to guarantee that the update is atomic if the update occurs over a
	 * frame boundary.
	 */
	strm_ctrl_clr(pb, stream, MPI_STRM_EN);

	if (is_input)
		ret = setup_mipi_input_stream(pb, stream, &setup);
	else
		ret = setup_mipi_output_stream(pb, stream, &setup);
	if (ret < 0) {
		spin_unlock_irqrestore(&pb->io_ipu.mipi_lock, irq_flags);
		mutex_unlock(&pb->lock);
		return ret;
	}

	if (setup.enable_on_setup || stream->enabled)
		enable_mipi_stream(pb, stream, setup.free_running,
				setup.frame_count, stream->is_input ? false :
				setup.output.enable_row_sync);

	spin_unlock_irqrestore(&pb->io_ipu.mipi_lock, irq_flags);

	mutex_unlock(&pb->lock);

	return 0;
}

/* The caller to this function must hold pb->lock and pb->io_ipu.mipi_lock */
static void reset_mipi_stream(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream)
{
	writel(stream->stream_id, pb->io_ipu.ipu_base + stream->select_offset);
	writel(stream->is_input ? MPI_STRM_RST : MPO_STRM_RST,
			pb->io_ipu.ipu_base + stream->ctrl_offset);
	writel(0, pb->io_ipu.ipu_base + stream->ctrl_offset);

	stream->enabled = false;

	dev_dbg(&pb->pdev->dev, "%s: mipi %s stream%u: reset\n", __func__,
			stream->is_input ? "input" : "output",
			stream->stream_id);
}

/* The caller to this function must hold pb->io_ipu.mipi_lock. */
void mipi_report_completion(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream, int err)
{
	signal_waiters(pb, stream->irq, err);
}

/* The caller to this function must hold pb->lock. */
void mipi_request_cleanup(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream)
{
	unsigned long irq_flags;

	spin_lock_irqsave(&pb->io_ipu.mipi_lock, irq_flags);

	/* If the stream has already been cleaned then there is nothing to do
	 * here.
	 */
	if (stream->is_clean) {
		spin_unlock_irqrestore(&pb->io_ipu.mipi_lock, irq_flags);
		dev_dbg(&pb->pdev->dev, "%s: mipi %s stream%u: already clean\n",
			__func__, stream->is_input ? "input" : "output",
			stream->stream_id);
		return;
	}

	/* If a cleanup operation has not been started then start one now. */
	if (!stream->cleanup_in_progress) {
		stream->cleanup_in_progress = true;
		writel(stream->stream_id, pb->io_ipu.ipu_base +
				stream->select_offset);

		strm_ctrl_set(pb, stream, stream->is_input ?
				MPI_STRM_CLEANUP : MPO_STRM_CLEANUP);
	}

	mipi_report_completion(pb, stream, -ECANCELED);

	spin_unlock_irqrestore(&pb->io_ipu.mipi_lock, irq_flags);

	/* If the overflow interrupt started the cleanup work queue then cancel
	 * it.  Since software is requesting the cleanup we will handle the
	 * cleanup verification in the calling thread and do not need to handle
	 * it asynchronously.
	 */
	cancel_delayed_work_sync(&stream->cleanup_work);

	dev_dbg(&pb->pdev->dev, "%s: mipi %s stream%u: cleanup\n", __func__,
			stream->is_input ? "input" : "output",
			stream->stream_id);
}

/* The caller to this function must hold pb->lock. */
int verify_cleanup_completion(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream)
{
	unsigned long irq_flags;
#ifndef CONFIG_PAINTBOX_SIMULATOR_SUPPORT
	uint32_t status;
#endif
	int ret = 0;

	spin_lock_irqsave(&pb->io_ipu.mipi_lock, irq_flags);

	/* If a cleanup process has not been started then exit now. */
	if (!stream->cleanup_in_progress) {
		spin_unlock_irqrestore(&pb->io_ipu.mipi_lock, irq_flags);
		return 0;
	}

	stream->cleanup_in_progress = false;
	stream->is_clean = true;

	/* TODO(ahampson):  This needs to be disabled for QEMU/Simulator for the
	 * now.  The RTL test bench can not currently handle the wait for 200us
	 * before reading the CLEANUP bit requirement. This will be fixed in
	 * QEMU and the RTL testbench.  b/32338758
	 */
#ifndef CONFIG_PAINTBOX_SIMULATOR_SUPPORT
	writel(stream->stream_id, pb->io_ipu.ipu_base + stream->select_offset);

	status = readl(pb->io_ipu.ipu_base + stream->ctrl_offset);

	/* Verify that the stream cleanup completed, if it hasn't then reset
	 * the stream.
	 */
	if (status & (stream->is_input ? MPI_STRM_CLEANUP :
			MPO_STRM_CLEANUP)) {
		reset_mipi_stream(pb, stream);
		ret = -EIO;
	}
#endif

	spin_unlock_irqrestore(&pb->io_ipu.mipi_lock, irq_flags);

	if (ret < 0)
		dev_err(&pb->pdev->dev,
				"%s: mipi %s stream%u: cleanup failed, stream "
				"reset\n", __func__, stream->is_input ?
				"input" : "output", stream->stream_id);

	return ret;
}

static void paintbox_cleanup_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct paintbox_mipi_stream *stream =
			container_of(dwork, struct paintbox_mipi_stream,
			cleanup_work);
	struct paintbox_data *pb = stream->pb;
	int ret;

	mutex_lock(&pb->lock);

	ret = verify_cleanup_completion(pb, stream);
	if (ret < 0)
		dma_reset_channel(pb, stream->dma_channel);

	mutex_unlock(&pb->lock);
}

#ifdef CONFIG_PAINTBOX_TEST_SUPPORT
int mipi_test_stream_reset_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg,
		bool is_input)
{
	struct paintbox_mipi_stream *stream;
	unsigned int stream_id = (unsigned int)arg;
	int ret = 0;

	mutex_lock(&pb->lock);

	stream = get_mipi_stream(pb, session, stream_id, is_input, &ret);
	if (ret >= 0) {
		unsigned long irq_flags;

		spin_lock_irqsave(&pb->io_ipu.mipi_lock, irq_flags);

		reset_mipi_stream(pb, stream);

		spin_unlock_irqrestore(&pb->io_ipu.mipi_lock, irq_flags);
	}

	mutex_unlock(&pb->lock);

	return ret;
}
#endif

int cleanup_mipi_stream_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg,
		bool is_input)
{
	struct paintbox_mipi_stream *stream;
	unsigned int stream_id = (unsigned int)arg;
	int ret = 0;

	mutex_lock(&pb->lock);

	stream = get_mipi_stream(pb, session, stream_id, is_input, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	/* A stream can not be cleaned up if the corresponding DMA channel is
	 * not part of this session.
	 */
	if (!stream->dma_channel) {
		dev_err(&pb->pdev->dev,
				"%s: mipi %s stream%u: no corresponding DMA "
				"channel\n", __func__, is_input ? "input" :
				"output", stream_id);
		mutex_unlock(&pb->lock);
		return -EINVAL;
	}

	/* Stop any active transfer on the corresponding DMA channel for this MIPI
	 * stream.  This will also initiate a MIPI cleanup operation.
	 */
	dma_stop_transfer(pb, stream->dma_channel);

	dev_dbg(&pb->pdev->dev, "%s: mipi %s stream%u: cleanup\n", __func__,
			is_input ? "input" : "output", stream_id);

	mutex_unlock(&pb->lock);

	return 0;
}

int bind_mipi_interrupt_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg,
		bool is_input)
{
	struct mipi_interrupt_config __user *user_req;
	struct mipi_interrupt_config req;
	struct paintbox_mipi_stream *stream;
	int ret;

	user_req = (struct mipi_interrupt_config __user *)arg;
	if (copy_from_user(&req, user_req, sizeof(req)))
		return -EFAULT;

	mutex_lock(&pb->lock);
	stream = get_mipi_stream(pb, session, req.stream_id, is_input, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	ret = bind_mipi_interrupt(pb, session, stream, req.interrupt_id,
			is_input);

	init_waiters(pb, stream->irq);

	mutex_unlock(&pb->lock);

	return ret;
}

int unbind_mipi_interrupt_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg,
		bool is_input)
{
	unsigned int stream_id = (unsigned int)arg;
	struct paintbox_mipi_stream *stream;
	int ret = 0;

	mutex_lock(&pb->lock);
	stream = get_mipi_stream(pb, session, stream_id, is_input, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	ret = unbind_mipi_interrupt(pb, session, stream);
	if (ret < 0) {
		dev_err(&pb->pdev->dev,
				"%s: mipi stream%u: unable to unbind interrupt,"
				" %d\n", __func__, stream_id, ret);
		mutex_unlock(&pb->lock);
		return ret;
	}

	dev_dbg(&pb->pdev->dev, "%s: mipi stream%u: unbind interrupt\n",
			__func__, stream_id);

	mutex_unlock(&pb->lock);

	return 0;
}

static void paintbox_mipi_input_stream_interrupt(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream)
{
	uint32_t status;

	spin_lock(&pb->io_ipu.mipi_lock);

	writel(stream->stream_id, pb->io_ipu.ipu_base + MPI_STRM_SEL);

	status = readl(pb->io_ipu.ipu_base + MPI_STRM_CTRL);

	/* If there are no MIPI interrupts then return immediately. */
	if ((status & (MPI_STRM_SOF_ISR | MPI_STRM_OVF_ISR)) == 0) {
		spin_unlock(&pb->io_ipu.mipi_lock);
		return;
	}

	if (status & MPI_STRM_SOF_ISR) {
		writel(status & ~MPI_STRM_SOF_ISR, pb->io_ipu.ipu_base +
				MPI_STRM_CTRL);

		stream->stats.input.sof_interrupts++;

		/* If the stream is not free running and we have transferred the
		 * the requested number of frames then disable the stream.
		 */
		if (!stream->free_running) {
			if (stream->frame_count > 0)
				stream->frame_count--;
			else
				disable_mipi_stream(pb, stream);
		}

		mipi_report_completion(pb, stream, 0);
	}

	if (status & MPI_STRM_OVF_ISR) {
		writel(status & ~MPI_STRM_OVF_ISR, pb->io_ipu.ipu_base +
				MPI_STRM_CTRL);

		stream->stats.input.ovf_interrupts++;

		if (status & MPI_STRM_CLEANUP) {
			stream->cleanup_in_progress = true;

			/* The hardware will also initiate a cleanup operation
			 * on an overflow.  We don't want to wait for the
			 * cleanup operation to complete in the interrupt
			 * handler so we will spin off a workqueue verify that
			 * it completed successfully.  This is unlikely to fail
			 * but if it does then the MIPI stream will need to be
			 * reset.
			 */
			queue_delayed_work(system_wq, &stream->cleanup_work,
					usecs_to_jiffies(
					MIPI_CLEANUP_TIMEOUT_US));
		} else {
			/* If the cleanup bit is not set then the cleanup
			 * operation has already concluded or it might not have
			 * been initiated.  The former indicates that there may
			 * significant interrupt latency, the latter may
			 * indicate a hardware bug.
			 */
			dev_alert(&pb->pdev->dev,
				"%s: mipi input stream%u: overflow interrupt "
				"without cleanup\n", __func__,
				stream->stream_id);
		}

		mipi_report_completion(pb, stream, -EIO);

		/* Set the MIPI error for the active transfer so the error can
		 * be reported on the DMA EOF interrupt.
		 */
		dma_set_mipi_error(pb, stream->dma_channel, -EIO);
	}

	spin_unlock(&pb->io_ipu.mipi_lock);
}

irqreturn_t paintbox_mipi_input_interrupt(struct paintbox_data *pb,
		uint32_t interface_mask)
{
	unsigned int interface_id, stream_index;

	for (interface_id = 0;
			interface_id < pb->io_ipu.num_mipi_input_interfaces &&
			interface_mask; interface_id++, interface_mask >>= 1) {
		struct paintbox_mipi_interface *interface;

		if (!(interface_mask & 0x01))
			continue;

		interface = &pb->io_ipu.mipi_input_interfaces[interface_id];

		for (stream_index = 0; stream_index < interface->num_streams;
				stream_index++)
			paintbox_mipi_input_stream_interrupt(pb,
					interface->streams[stream_index]);
	}

	return IRQ_HANDLED;
}

static void paintbox_mipi_output_stream_interrupt(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream)
{
	uint32_t status;

	spin_lock(&pb->io_ipu.mipi_lock);

	writel(stream->stream_id, pb->io_ipu.ipu_base + MPO_STRM_SEL);

	status = readl(pb->io_ipu.ipu_base + MPO_STRM_CTRL);
	if (status & MPO_STRM_EOF_ISR) {
		writel(status & ~MPO_STRM_EOF_ISR, pb->io_ipu.ipu_base +
				MPO_STRM_CTRL);

		stream->stats.output.eof_interrupts++;

		/* If the stream is not free running and we have transferred the
		 * the requested number of frames then disable the stream.
		 */
		if (!stream->free_running && --stream->frame_count == 0)
			disable_mipi_stream(pb, stream);

		mipi_report_completion(pb, stream, 0);
	}

	spin_unlock(&pb->io_ipu.mipi_lock);
}

irqreturn_t paintbox_mipi_output_interrupt(struct paintbox_data *pb,
		uint32_t interface_mask)
{
	unsigned int interface_id, stream_index;

	for (interface_id = 0;
			interface_id < pb->io_ipu.num_mipi_output_interfaces &&
			interface_mask; interface_id++, interface_mask >>= 1) {
		struct paintbox_mipi_interface *interface;

		if (!(interface_mask & 0x01))
			continue;

		interface = &pb->io_ipu.mipi_output_interfaces[interface_id];

		for (stream_index = 0; stream_index < interface->num_streams;
				stream_index++)
			paintbox_mipi_output_stream_interrupt(pb,
					interface->streams[stream_index]);
	}

	return IRQ_HANDLED;
}

static int paintbox_mipi_input_init(struct paintbox_data *pb)
{
	struct paintbox_io_ipu *ipu = &pb->io_ipu;
	unsigned int stream_id, inf_id;
	unsigned int streams_per_interface = ipu->num_mipi_input_streams /
			ipu->num_mipi_input_interfaces;
	int ret;

	ipu->mipi_input_interfaces = kzalloc(sizeof(
			struct paintbox_mipi_interface) *
			ipu->num_mipi_input_interfaces, GFP_KERNEL);
	if (!ipu->mipi_input_interfaces)
		return -ENOMEM;

	ipu->mipi_input_streams = kzalloc(sizeof(struct paintbox_mipi_stream) *
			ipu->num_mipi_input_streams, GFP_KERNEL);
	if (!ipu->mipi_input_streams) {
		ret = -ENOMEM;
		goto err_exit;
	}

	for (inf_id = 0; inf_id < ipu->num_mipi_input_interfaces; inf_id++) {
		struct paintbox_mipi_interface *interface =
				&ipu->mipi_input_interfaces[inf_id];
		interface->interface_id = inf_id;
		interface->num_streams = streams_per_interface;

		interface->streams = kcalloc(ipu->num_mipi_input_streams,
				sizeof(struct paintbox_mipi_stream *),
				GFP_KERNEL);
		if (!interface->streams) {
			ret = -ENOMEM;
			goto err_exit;
		}
	}

	for (inf_id = 0, stream_id = 0;
			inf_id < ipu->num_mipi_input_interfaces; inf_id++) {
		unsigned int i;

		for (i = 0; i < streams_per_interface; i++, stream_id++) {
			struct paintbox_mipi_stream *stream =
					&ipu->mipi_input_streams[stream_id];
			struct paintbox_mipi_interface *interface =
					&ipu->mipi_input_interfaces[inf_id];
			stream->stream_id = stream_id;
			stream->interface = interface;
			stream->ctrl_offset = MPI_STRM_CTRL;
			stream->select_offset = MPI_STRM_SEL;
			stream->is_input = true;
			stream->is_clean = true;
			stream->pb = pb;

			INIT_DELAYED_WORK(&stream->cleanup_work,
					paintbox_cleanup_work);

			interface->streams[i] = stream;

			paintbox_mipi_input_stream_debug_init(pb, stream);
		}
	}

	return 0;

err_exit:
	kfree(ipu->mipi_input_streams);
	kfree(ipu->mipi_input_interfaces);

	ipu->mipi_input_streams = NULL;
	ipu->mipi_input_interfaces = NULL;

	return ret;
}

static int paintbox_mipi_output_init(struct paintbox_data *pb)
{
	struct paintbox_io_ipu *ipu = &pb->io_ipu;
	unsigned int stream_id, inf_id;
	unsigned int streams_per_interface = ipu->num_mipi_output_streams /
			ipu->num_mipi_output_interfaces;
	int ret;

	ipu->mipi_output_interfaces = kzalloc(sizeof(
			struct paintbox_mipi_interface) *
			ipu->num_mipi_output_interfaces, GFP_KERNEL);
	if (!ipu->mipi_output_interfaces)
		return -ENOMEM;

	ipu->mipi_output_streams = kzalloc(sizeof(
			struct paintbox_mipi_stream) *
			ipu->num_mipi_output_streams, GFP_KERNEL);
	if (!ipu->mipi_output_streams) {
		ret = -ENOMEM;
		goto err_exit;
	}

	for (inf_id = 0; inf_id < ipu->num_mipi_output_interfaces; inf_id++) {
		struct paintbox_mipi_interface *interface =
				&ipu->mipi_output_interfaces[inf_id];
		interface->interface_id = inf_id;
		interface->num_streams = streams_per_interface;

		interface->streams = kcalloc(ipu->num_mipi_output_streams,
				sizeof(struct paintbox_mipi_stream *),
				GFP_KERNEL);
		if (!interface->streams) {
			ret = -ENOMEM;
			goto err_exit;
		}
	}

	/* Store stream id with object as a convenience to avoid doing a lookup
	 * later on.
	 */
	for (inf_id = 0, stream_id = 0;
			inf_id < ipu->num_mipi_output_interfaces; inf_id++) {
		unsigned int i;

		for (i = 0; i < streams_per_interface; i++, stream_id++) {
			struct paintbox_mipi_stream *stream =
					&ipu->mipi_output_streams[stream_id];
			struct paintbox_mipi_interface *interface =
					&ipu->mipi_output_interfaces[inf_id];
			stream->stream_id = stream_id;
			stream->interface =
					&ipu->mipi_output_interfaces[inf_id];
			stream->ctrl_offset = MPO_STRM_CTRL;
			stream->select_offset = MPO_STRM_SEL;
			stream->is_input = false;
			stream->is_clean = true;
			stream->pb = pb;

			INIT_DELAYED_WORK(&stream->cleanup_work,
					paintbox_cleanup_work);

			interface->streams[i] = stream;

			paintbox_mipi_output_stream_debug_init(pb, stream);
		}
	}

	return 0;

err_exit:
	kfree(ipu->mipi_output_streams);
	kfree(ipu->mipi_output_interfaces);

	ipu->mipi_output_streams = NULL;
	ipu->mipi_output_interfaces = NULL;

	return ret;
}

int paintbox_mipi_init(struct paintbox_data *pb)
{
	uint32_t val;
	int ret;

	pb->io_ipu.ipu_base = pb->reg_base + IPU_IO_IPU_OFFSET;

	spin_lock_init(&pb->io_ipu.mipi_lock);

	paintbox_mipi_debug_init(pb);

	val = readl(pb->io_ipu.ipu_base + MPI_CAP);
	pb->io_ipu.num_mipi_input_streams = (val & MPI_MAX_STRM_MASK) >>
			MPI_MAX_STRM_SHIFT;
	pb->io_ipu.num_mipi_input_interfaces = val & MPI_MAX_IFC_MASK;

	if (pb->io_ipu.num_mipi_input_streams > 0) {
		ret = paintbox_mipi_input_init(pb);
		if (ret < 0)
			return ret;
	}

	val = readl(pb->io_ipu.ipu_base + MPO_CAP);
	pb->io_ipu.num_mipi_output_streams = (val & MPO_MAX_STRM_MASK) >>
			MPO_MAX_STRM_SHIFT;
		pb->io_ipu.num_mipi_output_interfaces = val & MPO_MAX_IFC_MASK;

	if (pb->io_ipu.num_mipi_output_streams > 0) {
		ret = paintbox_mipi_output_init(pb);
		if (ret < 0) {
			kfree(pb->io_ipu.mipi_input_streams);
			return ret;
		}
	}

	return 0;
}
