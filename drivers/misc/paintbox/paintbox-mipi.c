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
#include <linux/ktime.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/timekeeping.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <uapi/paintbox.h>

#include "paintbox-dma.h"
#include "paintbox-debug.h"
#include "paintbox-io.h"
#include "paintbox-irq.h"
#include "paintbox-mipi.h"
#include "paintbox-mipi-debug.h"
#if CONFIG_PAINTBOX_VERSION_MAJOR >= 1
#include "paintbox-mipi-v1.h"
#else
#include "paintbox-mipi-v0.h"
#endif
#include "paintbox-power.h"
#include "paintbox-regs.h"

/* The caller to this function must hold pb->io_ipu.mipi_lock. */
void enable_mipi_interface(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream)
{
	struct paintbox_mipi_interface *interface = stream->interface;

	if (interface->active_stream_mask == 0) {
		if (stream->is_input) {
			paintbox_pm_enable_mipi_input_interface(pb,
					interface->interface_id);
			paintbox_enable_mipi_input_interface_interrupt(pb,
					interface->interface_id);
#if CONFIG_PAINTBOX_VERSION_MAJOR >= 1
			paintbox_enable_mipi_input_interface_error_interrupt(pb,
					interface->interface_id);
#endif
		}
		else {
			paintbox_pm_enable_mipi_output_interface(pb,
					interface->interface_id);
			paintbox_enable_mipi_output_interface_interrupt(pb,
					interface->interface_id);
		}
	}

	interface->active_stream_mask |= 1 << stream->stream_id;
}

/* The caller to this function must hold pb->io_ipu.mipi_lock. */
static void disable_mipi_interface(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream)
{
	struct paintbox_mipi_interface *interface = stream->interface;

	/* If there are no active streams already then the interface interrupt
	 * is already disabled.
	 */
	if (interface->active_stream_mask == 0)
		return;

	interface->active_stream_mask &= ~(1 << stream->stream_id);

	if (interface->active_stream_mask == 0) {
		if (stream->is_input) {
			paintbox_disable_mipi_input_interface_interrupt(pb,
					interface->interface_id);
#if CONFIG_PAINTBOX_VERSION_MAJOR >= 1
			paintbox_disable_mipi_input_interface_error_interrupt(
					pb, interface->interface_id);
#endif
			paintbox_pm_disable_mipi_input_interface(pb,
					interface->interface_id);
		}
		else {
			paintbox_disable_mipi_output_interface_interrupt(pb,
					interface->interface_id);
			paintbox_pm_disable_mipi_output_interface(pb,
					interface->interface_id);
		}
	}
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

static int paintbox_validate_mipi_stream_mask(struct paintbox_data *pb,
		struct paintbox_session *session, uint32_t stream_id_mask,
		uint32_t *validated_mask, bool is_input)
{
	struct paintbox_mipi_stream *stream, *stream_next;
	struct list_head *stream_list;

	stream_list = is_input ? &session->mipi_input_list :
			&session->mipi_output_list;

	list_for_each_entry_safe(stream, stream_next, stream_list,
			session_entry) {
		unsigned int stream_id_bit = 1 << stream->stream_id;

		if (!(stream_id_mask & stream_id_bit))
			continue;

		/* A stream can not be enabled if the corresponding DMA channel
		 * is not part of this session.
		 */
		if (!stream->dma_channel) {
			dev_err(&pb->pdev->dev,
					"%s: mipi %s stream%u no corresponding dma channel\n",
					__func__, is_input ? "input" : "output",
					stream->stream_id);
			return -EINVAL;
		}

		*validated_mask |= stream_id_bit;
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
	stream->allocated = true;

	spin_unlock_irqrestore(&pb->io_ipu.mipi_lock, irq_flags);

	dev_dbg(&pb->pdev->dev, "mipi input stream%u allocated\n", stream_id);

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
	stream->allocated = true;

	spin_unlock_irqrestore(&pb->io_ipu.mipi_lock, irq_flags);

	dev_dbg(&pb->pdev->dev, "mipi output stream%u allocated\n", stream_id);

	mutex_unlock(&pb->lock);

	return 0;
}

/* The caller to this function must hold pb->lock. */
void release_mipi_stream(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_mipi_stream *stream)
{
	unsigned long irq_flags;

	dev_dbg(&pb->pdev->dev, "mipi %s stream%u released\n",
			stream->is_input ? "input" : "output",
			stream->stream_id);

	/* Disable the MIPI stream and the stream interrupts. */
	spin_lock_irqsave(&pb->io_ipu.mipi_lock, irq_flags);

	paintbox_mipi_select_stream(pb, stream);
	writel(0, pb->io_ipu.ipu_base + stream->ctrl_offset);

	disable_mipi_interface(pb, stream);

	stream->free_running = false;
	stream->frame_count = 0;
	stream->last_frame = false;
	stream->enabled = false;
	stream->allocated = false;

	if (stream->is_input) {
		stream->input.missed_sof_interrupt = false;
		stream->input.missed_ovf_interrupt = false;
	} else {
		stream->output.missed_eof_interrupt = false;
	}

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

	spin_lock_irqsave(&pb->io_ipu.mipi_lock, irq_flags);

	/* Disable the MIPI stream and the stream interrupts.  We don't care if
	 * an interrupt is pending since the stream is being released.
	 */
	paintbox_mipi_select_stream(pb, stream);
	writel(0, pb->io_ipu.ipu_base + stream->ctrl_offset);

	disable_mipi_interface(pb, stream);

	stream->free_running = false;
	stream->frame_count = 0;
	stream->last_frame = false;
	stream->enabled = false;
	stream->dma_channel = NULL;

	if (stream->is_input) {
		stream->input.missed_sof_interrupt = false;
		stream->input.missed_ovf_interrupt = false;
	} else {
		stream->output.missed_eof_interrupt = false;
	}

	spin_unlock_irqrestore(&pb->io_ipu.mipi_lock, irq_flags);
}

/* The caller to this function must hold pb->lock */
struct paintbox_mipi_stream *mipi_handle_dma_channel_allocated(
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
void paintbox_mipi_update_stream_count(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream, bool free_running,
		int32_t frame_count)
{
	/* frame count is zero when in free running mode. */
	stream->frame_count = free_running ? 0 : frame_count;
	stream->free_running = free_running;

	dev_dbg(&pb->pdev->dev,
			"mipi %s stream%u updating stream running period, free running %d frame_count %u\n",
			stream->is_input ? "input" : "output",
			stream->stream_id, free_running, frame_count);
}

/* The caller to this function must hold pb->lock and pb->io_ipu.mipi_lock. */
void paintbox_mipi_enable_input_stream_common(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream, bool free_running,
		int32_t frame_count, bool disable_on_error)
{
	/* frame count is zero when in free running mode. */
	stream->frame_count = free_running ? 0 : frame_count;
	stream->free_running = free_running;
	stream->is_clean = false;
	stream->last_frame = false;
	stream->enabled = true;
	stream->disable_on_error = disable_on_error;

	enable_mipi_interface(pb, stream);
}

/* The caller to this function must hold pb->lock and pb->io_ipu.mipi_lock. */
void paintbox_mipi_disable_stream_common(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream)
{
	stream->free_running = false;
	stream->frame_count = 0;
	stream->last_frame = true;
	stream->enabled = false;
}

/* The caller to this function must hold pb->lock and pb->io_ipu.mipi_lock. */
int enable_mipi_output_stream(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream, bool free_running,
		int32_t frame_count, bool enable_row_sync)
{
	int ret = 0;

	/* frame count is zero when in free running mode. Otherwise set the
	 * frame count to the value passed in minus one since the stream will
	 * start the output frame on enable.
	 */
	stream->frame_count = free_running ? 0 : frame_count - 1;
	stream->free_running = free_running;
	stream->is_clean = false;
	stream->enabled = true;

	enable_mipi_interface(pb, stream);

	/* Enable the stream.  This will copy the stream's pending configuration
	 * registers to the active set.
	 */
	/* TODO(showarth): wait for last frame. b/38357562 */
	ret = mipi_output_enable_irqs_and_stream(pb, stream,
			MIPI_OUTPUT_EOF_IMR, enable_row_sync);

	/* If the stream is not in free running mode and there are no frames yet
	 * to be sent then disable the stream.
	 */
	if (!free_running && stream->frame_count == 0) {
		stream->last_frame = true;

	/* TODO(ahampson):  Remove Simulator check once MIPI double buffering is
	 * implemented in the Simulator and QEMU.  b/29508438, b/32769802
	 *
	 * With the current Simulator, the driver can not disable the stream at
	 * this point because the Simulator MIPI model has not started the
	 * output frame.
	 *
	 * In the interim, the driver will set stream->last_frame to true and
	 * stream->frame_count to 0 and use the EOF interrupt handler to disable
	 * the stream at the end of the frame.
	 */
#ifndef CONFIG_PAINTBOX_SIMULATOR_SUPPORT
		stream->enabled = false;
		ret |= mipi_output_disable_stream(pb, stream);
#endif
	} else {
		stream->last_frame = false;
	}

	return ret;
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
				"%s: mipi stream%u stream is not free running and frame count is invalid, %d <= 0\n",
				__func__, req.stream_id, req.frame_count);
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
				"%s: mipi %s stream%u no corresponding dma channel\n",
				__func__, is_input ? "input" : "output",
				stream->stream_id);
		mutex_unlock(&pb->lock);
		return -EINVAL;
	}

	spin_lock_irqsave(&pb->io_ipu.mipi_lock, irq_flags);

	/* If the stream is already enabled then update the frame count and
	 * free running fields in the stream structure and return.
	 */
	if (stream->enabled) {
		/* frame count is zero when in free running mode. */
		stream->frame_count = req.free_running ? 0 : req.frame_count;
		stream->free_running = req.free_running;

		spin_unlock_irqrestore(&pb->io_ipu.mipi_lock, irq_flags);
		mutex_unlock(&pb->lock);

		dev_dbg(&pb->pdev->dev,
				"mipi %s stream%u updating stream running period, free running %d frame_count %u\n",
				is_input ? "input" : "output", req.stream_id,
				stream->free_running, stream->frame_count);
		return 0;
	}

	if (is_input) {
		paintbox_mipi_enable_input_stream_common(pb, stream,
				req.free_running, req.frame_count,
				req.input.disable_on_error);
		ret = mipi_input_enable_irqs_and_stream(pb, stream,
				MIPI_INPUT_SOF_IMR | MIPI_INPUT_OVF_IMR);
	} else {
		ret = enable_mipi_output_stream(pb, stream, req.free_running,
				req.frame_count, req.output.enable_row_sync);
	}

	spin_unlock_irqrestore(&pb->io_ipu.mipi_lock, irq_flags);

	mutex_unlock(&pb->lock);

	dev_dbg(&pb->pdev->dev,
			"mipi %s stream%u enable stream free running %d frame_count %u\n",
			is_input ? "input" : "output", req.stream_id,
			stream->free_running, stream->frame_count);

	/* Log a missed interrupt if it occurred but don't propagate the error
	 * up.
	 */
	if (ret == -EINTR) {
		dev_warn(&pb->pdev->dev, "%s: mipi %s stream%u may have missed an interrupt\n",
				__func__, is_input ? "input" : "output",
				stream->stream_id);
		ret = 0;
	}

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

	dev_dbg(&pb->pdev->dev, "mipi %s stream%u disable\n", is_input ?
			"input" : "output", stream->stream_id);

	spin_lock_irqsave(&pb->io_ipu.mipi_lock, irq_flags);

	stream->free_running = false;
	stream->frame_count = 0;
	stream->last_frame = true;
	stream->enabled = false;

	if (is_input) {
		/* Do not disable the overflow interrupt on disable. This
		 * interrupt reports on the state of the current frame and needs
		 * to be left enabled.
		 */
		ret = mipi_input_disable_irqs_and_stream(pb, stream,
				MIPI_INPUT_SOF_IMR);
	} else {
		/* Do not disable the EOF interrupt on disable. This interrupt
		 * reports on the state of the current frame and needs to be
		 * left enabled.
		 */
		ret = mipi_output_disable_stream(pb, stream);
	}

	spin_unlock_irqrestore(&pb->io_ipu.mipi_lock, irq_flags);

	mutex_unlock(&pb->lock);

	/* Log a missed interrupt if it occurred but don't propagate the error
	 * up.
	 */
	if (ret == -EINTR) {
		dev_warn(&pb->pdev->dev,
				"%s: mipi %s stream%u may have missed an interrupt\n",
				__func__, is_input ? "input" : "output",
				stream_id);
		ret = 0;
	}

	return 0;
}

int paintbox_mipi_enable_multiple_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg,
		bool is_input)
{
	struct mipi_stream_enable_multiple __user *user_req;
	struct mipi_stream_enable_multiple req;
	uint32_t stream_id_mask = 0;
	int ret;

	user_req = (struct mipi_stream_enable_multiple __user *)arg;
	if (copy_from_user(&req, user_req, sizeof(req)))
		return -EFAULT;

	mutex_lock(&pb->lock);

	if (!req.enable_all) {
		ret = paintbox_validate_mipi_stream_mask(pb, session,
				req.stream_id_mask, &stream_id_mask, is_input);
		if (ret < 0) {
			mutex_unlock(&pb->lock);
			return ret;
		}
	}

	if (is_input)
		paintbox_mipi_enable_multiple_input(pb, session, stream_id_mask,
				&req);
	else
		paintbox_mipi_enable_multiple_output(pb, session,
				stream_id_mask, &req);

	mutex_unlock(&pb->lock);

	return 0;
}

int paintbox_mipi_disable_multiple_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg,
		bool is_input)
{
	struct mipi_stream_disable_multiple __user *user_req;
	struct mipi_stream_disable_multiple req;
	uint32_t stream_id_mask = 0;
	int ret;

	user_req = (struct mipi_stream_disable_multiple __user *)arg;
	if (copy_from_user(&req, user_req, sizeof(req)))
		return -EFAULT;

	mutex_lock(&pb->lock);

	if (!req.disable_all) {
		ret = paintbox_validate_mipi_stream_mask(pb, session,
				req.stream_id_mask, &stream_id_mask, is_input);
		if (ret < 0) {
			mutex_unlock(&pb->lock);
			return ret;
		}
	}

	if (is_input)
		paintbox_mipi_disable_multiple_input(pb, session,
				stream_id_mask, &req);
	else
		paintbox_mipi_disable_multiple_output(pb, session,
				stream_id_mask, &req);

	mutex_unlock(&pb->lock);

	return 0;
}

int get_mipi_frame_number_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct paintbox_mipi_stream *stream;
	unsigned int stream_id = (unsigned int)arg;
	unsigned long irq_flags;
	int ret = 0;

	mutex_lock(&pb->lock);

	stream = get_mipi_stream(pb, session, stream_id, true, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	spin_lock_irqsave(&pb->io_ipu.mipi_lock, irq_flags);

	ret = stream->input.last_frame_number != MIPI_INVALID_FRAME_NUMBER ?
			stream->input.last_frame_number : -ENODATA;

	spin_unlock_irqrestore(&pb->io_ipu.mipi_lock, irq_flags);

	mutex_unlock(&pb->lock);

	dev_dbg(&pb->pdev->dev, "mipi input stream%u frame number %d\n",
			stream_id, ret);

	return ret;
}

static int validate_mipi_stream_setup(struct paintbox_data *pb,
		struct mipi_stream_setup *setup, bool is_input)
{
	/* Virtual channel range is the same for input and output streams */
	if (setup->virtual_channel > MPI_STRM_CNFG0_VC_MAX) {
		dev_err(&pb->pdev->dev,
				"%s: mipi %s stream%u virtual channel %u is not supported, max virtual channel is %llu\n",
				__func__, is_input ? "input" : "output",
				setup->stream_id, setup->virtual_channel,
				MPI_STRM_CNFG0_VC_MAX);
		return -EINVAL;
	}

	/* Supported packed data types are the same for input and output
	 * streams.
	 */
	if (setup->data_type > MPI_STRM_CNFG0_DT_IN_MAX) {
		dev_err(&pb->pdev->dev,
				"%s: mipi %s stream%u packed data type %u is invalid, valid range 0..%llu\n",
				__func__, is_input ? "input" : "output",
				setup->stream_id, setup->data_type,
				MPI_STRM_CNFG0_DT_IN_MAX);
		return -EINVAL;
	}

	/* Supported unpacked data types are the same for input and output
	 * streams.
	 */
	if (setup->unpacked_data_type > MPI_STRM_CNFG0_DT_PROC_MAX) {
		dev_err(&pb->pdev->dev,
				"%s: mipi %s stream%u unpacked data type %u is invalid, valid range 0..%llu\n",
				__func__, is_input ? "input" : "output",
				setup->stream_id, setup->data_type,
				MPI_STRM_CNFG0_DT_PROC_MAX);
		return -EINVAL;
	}

	/* Image dimension restrictions are the same for input and output
	 * streams.
	 */
	if (setup->img_width > MPI_STRM_CNFG0_IMG_WIDTH_MAX) {
		dev_err(&pb->pdev->dev,
				"%s: mipi %s stream%u invalid width %u max %llu\n",
				__func__, is_input ? "input" : "output",
				setup->stream_id, setup->img_width,
				MPI_STRM_CNFG0_IMG_WIDTH_MAX);
		return -EINVAL;
	}

	if (setup->img_height > MPI_STRM_CNFG0_IMG_HEIGHT_MAX) {
		dev_err(&pb->pdev->dev,
				"%s: mipi %s stream%u invalid height %u max %llu\n",
				__func__, is_input ? "input" : "output",
				setup->stream_id, setup->img_height,
				MPI_STRM_CNFG0_IMG_HEIGHT_MAX);
		return -EINVAL;
	}

	/* Stripe height alignment requirements are the same for input and
	 * output streams.
	 */
	if (setup->stripe_height % MPI_STRM_CNFG0_STRP_HEIGHT_ROW_ALIGN != 0) {
		dev_err(&pb->pdev->dev,
				"%s: mipi %s stream%u stripe height needs to be aligned to %u rows, requested stripe height was %u\n",
				__func__, is_input ? "input" : "output",
				setup->stream_id,
				MPI_STRM_CNFG0_STRP_HEIGHT_ROW_ALIGN,
				setup->stripe_height);
		return -EINVAL;
	}

	/* Segment end valid range is the same for input and output streams. */
	if (setup->seg_end < MPI_STRM_CNFG1_SEG_END_MIN ||
			setup->seg_end > MPI_STRM_CNFG1_SEG_END_MAX) {
		dev_err(&pb->pdev->dev,
				"%s: mipi %s stream%u segment end %u is out of range min %u max %u\n",
				__func__, is_input ? "input" : "output",
				setup->stream_id, setup->seg_end,
				MPI_STRM_CNFG1_SEG_END_MIN,
				MPI_STRM_CNFG1_SEG_END_MAX);
		return -EINVAL;
	}

	/* Segments row valid range is the same for input and output streams. */
	if (setup->segs_per_row < MPI_STRM_CNFG1_SEGS_PER_ROW_MIN ||
			setup->segs_per_row > MPI_STRM_CNFG1_SEGS_PER_ROW_MAX) {
		dev_err(&pb->pdev->dev,
				"%s: mipi %s stream%u requested segments per row %u is out of range min %u max %u\n",
				__func__, is_input ? "input" : "output",
				setup->stream_id, setup->segs_per_row,
				MPI_STRM_CNFG1_SEGS_PER_ROW_MIN,
				MPI_STRM_CNFG1_SEGS_PER_ROW_MAX);
		return -EINVAL;
	}

	if (setup->enable_on_setup && !setup->free_running &&
			setup->frame_count <= 0) {
		dev_err(&pb->pdev->dev,
				"%s: mipi %s stream%u: stream is not free running and frame count is invalid, %d <= 0\n",
				__func__, is_input ? "input" : "output",
				setup->stream_id, setup->frame_count);
		return -EINVAL;
	}

	return 0;
}

static int validate_mipi_input_stream_setup(struct paintbox_data *pb,
		struct mipi_stream_setup *setup)
{
	int ret;

	ret = validate_mipi_stream_setup(pb, setup, true /* is_input */);
	if (ret < 0)
		return ret;

	if (setup->input.seg_start > MPI_STRM_CNFG1_SEG_START_MAX) {
		dev_err(&pb->pdev->dev,
				"%s: mipi input stream%u segment start %u is too large, max %llu\n",
				__func__,
				setup->stream_id, setup->input.seg_start,
				MPI_STRM_CNFG1_SEG_START_MAX);
		return -EINVAL;
	}

	if (setup->input.seg_words_per_row <
			MPI_STRM_CNFG1_SEG_WORDS_PER_ROW_MIN ||
			setup->input.seg_words_per_row >
			MPI_STRM_CNFG1_SEG_WORDS_PER_ROW_MAX) {
		dev_err(&pb->pdev->dev,
				"%s: mipi input stream%u requested segment words per row %u is out of range min %u max %u\n",
				__func__, setup->stream_id,
				setup->input.seg_words_per_row,
				MPI_STRM_CNFG1_SEG_WORDS_PER_ROW_MIN,
				MPI_STRM_CNFG1_SEG_WORDS_PER_ROW_MAX);
		return -EINVAL;
	}

	if (setup->stripe_height < MPI_STRM_CNFG0_STRP_HEIGHT_MIN ||
			setup->stripe_height > MPI_STRM_CNFG0_STRP_HEIGHT_MAX) {
		dev_err(&pb->pdev->dev,
				"%s: mipi input stream%u requested stripe height %u is out of range min %u max %u\n",
				__func__, setup->stream_id,
				setup->stripe_height,
				MPI_STRM_CNFG0_STRP_HEIGHT_MIN,
				MPI_STRM_CNFG0_STRP_HEIGHT_MAX);
		return -EINVAL;
	}

	return 0;
}

static int validate_mipi_output_stream_setup(struct paintbox_data *pb,
		struct mipi_stream_setup *setup)
{
	int ret;

	ret = validate_mipi_stream_setup(pb, setup, false /* is_input */);
	if (ret < 0)
		return ret;

	if (setup->stripe_height < MPO_STRM_CNFG0_STRP_HEIGHT_MIN ||
			setup->stripe_height > MPO_STRM_CNFG0_STRP_HEIGHT_MAX) {
		dev_err(&pb->pdev->dev,
				"%s: mipi output stream%u requested stripe height %u out of range min %u max %u\n",
				__func__, setup->stream_id,
				setup->stripe_height,
				MPO_STRM_CNFG0_STRP_HEIGHT_MIN,
				MPO_STRM_CNFG0_STRP_HEIGHT_MAX);
		return -EINVAL;
	}

	return 0;
}

/* The caller to this function must hold pb->lock */
int setup_mipi_input_stream(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream,
			struct mipi_stream_setup *setup)
{
	unsigned long irq_flags;
	uint64_t val;
	int ret = 0;

	LOG_MIPI_INPUT_SETUP(pb, setup);

	spin_lock_irqsave(&pb->io_ipu.mipi_lock, irq_flags);

#if CONFIG_PAINTBOX_VERSION_MAJOR >= 1
	/* TODO(showarth): wait for ENA == 0 */
#else
	/* Disable the stream while updating the stream configuration.  This is
	 * to guarantee that the update is atomic if the update occurs over a
	 * frame boundary.
	 */
	ret = mipi_input_disable_stream(pb, stream);
#endif

	paintbox_mipi_select_input_stream(pb, stream->stream_id);

	val = setup->virtual_channel & MPI_STRM_CNFG0_VC_MASK;
	val |= (setup->data_type & MPI_STRM_CNFG0_DT_IN_M) <<
			MPI_STRM_CNFG0_DT_IN_SHIFT;
	val |= (setup->unpacked_data_type & MPI_STRM_CNFG0_DT_PROC_M) <<
			MPI_STRM_CNFG0_DT_PROC_SHIFT;

	/* The STRP_HEIGHT value in the MPI_STRM_CNFG0 register is subtracted by
	 * one in the register field.
	 */
	val |= ((setup->stripe_height - 1) & MPI_STRM_CNFG0_STRP_HEIGHT_M) <<
			MPI_STRM_CNFG0_STRP_HEIGHT_SHIFT;
	val |= ((uint64_t)setup->img_width & MPI_STRM_CNFG0_IMG_WIDTH_M) <<
			MPI_STRM_CNFG0_IMG_WIDTH_SHIFT;
	val |= ((uint64_t)setup->img_height & MPI_STRM_CNFG0_IMG_HEIGHT_M) <<
			MPI_STRM_CNFG0_IMG_HEIGHT_SHIFT;
	writeq(val, pb->io_ipu.ipu_base + MPI_STRM_CNFG0);

	val = setup->input.seg_start & MPI_STRM_CNFG1_SEG_START_MASK;

	/* The MPI_SEG_END value in the MPI_STRM_CNFG1 register is subtracted
	 * by one in the register field.
	 */
	val |= ((setup->seg_end - 1) & MPI_STRM_CNFG1_SEG_END_M) <<
			MPI_STRM_CNFG1_SEG_END_SHIFT;

	/* The MPI_SEGS_PER_ROW value in the MPI_STRM_CNFG1 register is
	 * subtracted by one in the register field.
	 */
	val |= (((uint64_t)setup->segs_per_row - 1) &
			MPI_STRM_CNFG1_SEGS_PER_ROW_M) <<
			MPI_STRM_CNFG1_SEGS_PER_ROW_SHIFT;

	/* The MPI_SEG_WORDS_PER_ROW value in the MPI_STRM_CNFG1 register is
	 * subtracted by one in the register field.
	 */
	val |= (((uint64_t)setup->input.seg_words_per_row - 1) &
			MPI_STRM_CNFG1_SEG_WORDS_PER_ROW_M) <<
			MPI_STRM_CNFG1_SEG_WORDS_PER_ROW_SHIFT;
	writeq(val, pb->io_ipu.ipu_base + MPI_STRM_CNFG1);

	/* Re-enable the stream if it was previously enabled.  Otherwise if the
	 * stream was requested to be enabled as part of the setup then do so
	 * now.
	 */
	if (setup->enable_on_setup) {
		paintbox_mipi_enable_input_stream_common(pb, stream,
				setup->free_running, setup->frame_count,
				setup->input.disable_on_error);

		ret = mipi_input_enable_irqs_and_stream(pb, stream,
				MIPI_INPUT_SOF_IMR | MIPI_INPUT_OVF_IMR);
	} else if (stream->enabled)
		ret |= mipi_input_enable_stream(pb, stream);

	LOG_MIPI_REGISTERS(pb, stream);

	spin_unlock_irqrestore(&pb->io_ipu.mipi_lock, irq_flags);

	/* Log a missed interrupt if it occurred but don't propagate the error
	 * up.
	 */
	if (ret == -EINTR)
		dev_warn(&pb->pdev->dev,
				"%s: mipi input stream%u possible missed interrupt\n",
				__func__, stream->stream_id);

	return 0;
}

/* The caller to this function must hold pb->lock */
int setup_mipi_output_stream(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream,
		struct mipi_stream_setup *setup)
{
	unsigned long irq_flags;
	uint64_t cnfg0_val;
	uint32_t cnfg1_val;
	int ret = 0;

	LOG_MIPI_OUTPUT_SETUP(pb, setup);

	spin_lock_irqsave(&pb->io_ipu.mipi_lock, irq_flags);

#if CONFIG_PAINTBOX_VERSION_MAJOR >= 1
	/* TODO(showarth): wait for ENA == 0 */
#else
	/* Disable the stream while updating the stream configuration.  This is
	 * to guarantee that the update is atomic if the update occurs over a
	 * frame boundary.
	 */
	ret = mipi_output_disable_stream(pb, stream);
#endif

	paintbox_mipi_select_output_stream(pb, stream->stream_id);

	cnfg0_val = setup->virtual_channel & MPO_STRM_CNFG0_VC_MASK;
	cnfg0_val |= (setup->data_type & MPO_STRM_CNFG0_DT_OUT_M) <<
			MPO_STRM_CNFG0_DT_OUT_SHIFT;
	cnfg0_val |= (setup->unpacked_data_type & MPO_STRM_CNFG0_DT_PROC_M) <<
			MPO_STRM_CNFG0_DT_PROC_SHIFT;

	/* The STRP_HEIGHT value in the MPO_STRM_CNFG0 register is subtraced by
	 * one in the register field.
	 */
	cnfg0_val |= ((setup->stripe_height - 1) &
			MPO_STRM_CNFG0_STRP_HEIGHT_M) <<
			MPO_STRM_CNFG0_STRP_HEIGHT_SHIFT;
	cnfg0_val |= ((uint64_t)setup->img_width &
			MPO_STRM_CNFG0_IMG_WIDTH_M) <<
			MPO_STRM_CNFG0_IMG_WIDTH_SHIFT;
	cnfg0_val |= ((uint64_t)setup->img_height &
			MPO_STRM_CNFG0_IMG_HEIGHT_M) <<
			MPO_STRM_CNFG0_IMG_HEIGHT_SHIFT;
	writeq(cnfg0_val, pb->io_ipu.ipu_base + MPO_STRM_CNFG0);

#if CONFIG_PAINTBOX_VERSION_MAJOR >= 1
	/* TODO(showarth): support multiple virtual channels (b/36103832). */
	cnfg1_val = 0;
#else
	/* The MPO_SEG_END value in the MPO_STRM_CNFG1 register is subtracted
	 * by one in the register field.
	 */
	cnfg1_val = (setup->seg_end - 1) & MPO_STRM_CNFG1_SEG_END_MASK;
#endif

	/* The MPO_SEGS_PER_ROW value in the MPO_STRM_CNFG1 register is
	 * subtracted by one in the register field.
	 */
	cnfg1_val |= ((setup->segs_per_row - 1) &
			MPO_STRM_CNFG1_SEGS_PER_ROW_M) <<
			MPO_STRM_CNFG1_SEGS_PER_ROW_SHIFT;
	writel(cnfg1_val, pb->io_ipu.ipu_base + MPO_STRM_CNFG1);

	/* Re-enable the stream if it was previously enabled.  Otherwise if the
	 * stream was requested to be enabled as part of the setup then do so
	 * now.
	 */
	if (setup->enable_on_setup)
		ret |= enable_mipi_output_stream(pb, stream,
				setup->free_running, setup->frame_count,
				setup->output.enable_row_sync);
	else if (stream->enabled)
		ret |= mipi_output_enable_stream(pb, stream);

	LOG_MIPI_REGISTERS(pb, stream);

	spin_unlock_irqrestore(&pb->io_ipu.mipi_lock, irq_flags);

	/* Log a missed interrupt if it occurred but don't propagate the error
	 * up.
	 */
	if (ret == -EINTR)
		dev_warn(&pb->pdev->dev,
				"%s: mipi output stream%u possible missed interrupt\n",
				__func__, stream->stream_id);

	return 0;
}

int setup_mipi_stream_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg,
		bool is_input)
{
	struct mipi_stream_setup __user *user_setup;
	struct mipi_stream_setup setup;
	struct paintbox_mipi_stream *stream;
	int ret;

	user_setup = (struct mipi_stream_setup __user *)arg;
	if (copy_from_user(&setup, user_setup, sizeof(setup)))
		return -EFAULT;

	ret = is_input ? validate_mipi_input_stream_setup(pb, &setup) :
			validate_mipi_output_stream_setup(pb, &setup);
	if (ret < 0)
		return ret;

	mutex_lock(&pb->lock);

	stream = get_mipi_stream(pb, session, setup.stream_id, is_input, &ret);
	if (!ret)
		ret = is_input ? setup_mipi_input_stream(pb, stream, &setup) :
				setup_mipi_output_stream(pb, stream, &setup);

	mutex_unlock(&pb->lock);

	return ret;
}

/* The caller to this function must hold pb->lock and pb->io_ipu.mipi_lock */
static void reset_mipi_stream(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream)
{
	/* Toggle the reset stream bit.  We don't care if an interrupt occurs
	 * during the reset.  The stream will be left in the disable state after
	 * the reset completes.
	 */
	paintbox_mipi_select_stream(pb, stream);
	writel(stream->is_input ? MPI_STRM_CTRL_RST_MASK :
			MPO_STRM_CTRL_RST_MASK,
			pb->io_ipu.ipu_base + stream->ctrl_offset);
	writel(0, pb->io_ipu.ipu_base + stream->ctrl_offset);

	dev_dbg(&pb->pdev->dev, "mipi %s stream%u reset\n", stream->is_input ?
			"input" : "output", stream->stream_id);
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
		return;
	}

	dev_dbg(&pb->pdev->dev, "mipi %s stream%u cleanup requested\n",
			stream->is_input ? "input" : "output",
			stream->stream_id);

	/* If a cleanup operation has not been started then start one now. */
	if (!stream->cleanup_in_progress) {
		uint32_t ctrl;

		stream->cleanup_in_progress = true;
		paintbox_mipi_select_stream(pb, stream);

		/* It is possible that an interrupt could be missed during the
		 * read, modify, write of the control register.  This should be
		 * ok since the client is initiating the cleanup and the
		 * completion will be signalled below.
		 */
		ctrl = readl(pb->io_ipu.ipu_base + stream->ctrl_offset);
		ctrl |= stream->is_input ?
				MPI_STRM_CTRL_CLEANUP_MASK :
				MPO_STRM_CTRL_CLEANUP_MASK;
		writel(ctrl, pb->io_ipu.ipu_base + stream->ctrl_offset);
	}

	paintbox_irq_waiter_signal(pb, stream->irq, ktime_get_boottime(), 0,
			-ECANCELED);

	spin_unlock_irqrestore(&pb->io_ipu.mipi_lock, irq_flags);
}

/* The caller to this function must hold pb->lock. */
int verify_cleanup_completion(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream)
{
	unsigned long irq_flags;
#ifndef CONFIG_PAINTBOX_SIMULATOR_SUPPORT
	uint32_t ctrl;
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
	paintbox_mipi_select_stream(pb, stream);

	/* Verify that the stream cleanup completed, if it hasn't then reset
	 * the stream.
	 */
	ctrl = readl(pb->io_ipu.ipu_base + stream->ctrl_offset);
	if (ctrl & (stream->is_input ? MPI_STRM_CTRL_CLEANUP_MASK :
			MPO_STRM_CTRL_CLEANUP_MASK)) {
		reset_mipi_stream(pb, stream);
		ret = -EIO;
	}
#endif

	spin_unlock_irqrestore(&pb->io_ipu.mipi_lock, irq_flags);

	if (ret < 0) {
		dev_err(&pb->pdev->dev,
				"%s: mipi %s stream%u cleanup failed, stream reset\n",
				__func__, stream->is_input ? "input" : "output",
				stream->stream_id);
	} else {
		dev_dbg(&pb->pdev->dev, "mipi %s stream%u overflow cleanup complete\n",
				stream->is_input ? "input" : "output",
				stream->stream_id);
	}

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
				"%s: mipi %s stream%u no corresponding DMA channel\n",
				__func__, is_input ? "input" : "output",
				stream_id);
		mutex_unlock(&pb->lock);
		return -EINVAL;
	}

	/* Stop any active transfer on the corresponding DMA channel for this
	 * MIPI stream.  This will also initiate a MIPI cleanup operation.
	 */
	dma_stop_transfer(pb, stream->dma_channel);

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

	dev_dbg(&pb->pdev->dev, "mipi %s stream%u bind irq%u\n",
			stream->is_input ? "input" : "output", req.stream_id,
			req.interrupt_id);

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
				"%s: mipi stream%u: unable to unbind irq, err %d\n",
				__func__, stream_id, ret);
		mutex_unlock(&pb->lock);
		return ret;
	}

	dev_dbg(&pb->pdev->dev, "mipi %s stream%u unbind irq\n",
			stream->is_input ? "input" : "output", stream_id);

	mutex_unlock(&pb->lock);

	return 0;
}

/* This function must be called in an interrupt context */
void mipi_input_handle_dma_completed(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream)
{
	if (!WARN_ON(stream == NULL)) {
		spin_lock(&pb->io_ipu.mipi_lock);

		stream->input.frame_in_progress = false;

		spin_unlock(&pb->io_ipu.mipi_lock);
	}
}

/* This function must be called in an interrupt context */
static void paintbox_mipi_input_sof_interrupt(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream, ktime_t timestamp)
{
	stream->input.frame_in_progress = true;

	dev_dbg(&pb->pdev->dev, "mipi input stream%u SOF interrupt\n",
			stream->stream_id);

	paintbox_irq_waiter_signal(pb, stream->irq, timestamp,
			stream->input.last_frame_number, 0 /* error */);

	/* If the stream is free running then exit now and skip the frame count
	 * and disable logic.
	 */
	if (stream->free_running)
		return;

	if (stream->frame_count > 0)
		stream->frame_count--;

	/* If the stream is enabled and the frame count has gone to zero then
	 * disable the stream.
	 */
	if (stream->enabled && stream->frame_count == 0) {
		stream->last_frame = true;
		stream->enabled = false;

		mipi_input_disable_irqs_and_stream(pb, stream,
				MIPI_INPUT_SOF_IMR);

		dev_dbg(&pb->pdev->dev,
				"mipi input stream%u last frame, disabled\n",
				stream->stream_id);
	}
}

/* This function must be called in an interrupt context */
static void paintbox_mipi_output_eof_interrupt(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream, ktime_t timestamp)
{
	paintbox_irq_waiter_signal(pb, stream->irq, timestamp, 0 /* payload */,
			0 /* error */);

	/* If the stream is free running then exit now and skip the frame count
	 * and disable logic.
	 */
	if (stream->free_running)
		return;

	if (stream->frame_count > 0)
		stream->frame_count--;

	/* If this was the last frame for this enable period then let DMA know
	 * that the MIPI output has completed so it can power down the channel
	 * if necessary.
	 */
	if (stream->last_frame) {
		dma_report_mipi_output_completed(pb, stream->dma_channel);
		mipi_output_disable_irqs(pb, stream, MIPI_OUTPUT_EOF_IMR);
	}

	/* If the stream is enabled and the frame count has gone to zero then
	 * disable the stream.
	 */
	if (stream->enabled && stream->frame_count == 0) {
		stream->last_frame = true;
		stream->enabled = false;
		mipi_output_disable_stream(pb, stream);
	}

	dev_dbg(&pb->pdev->dev,
			"mipi output stream%u EOF interrupt enabled %u\n",
			stream->stream_id, stream->enabled);
}

/* This function must be called in an interrupt context */
static void paintbox_mipi_input_ovf_interrupt(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream, ktime_t timestamp)
{
	uint32_t ctrl = readl(pb->io_ipu.ipu_base + MPI_STRM_CTRL);

	if (ctrl & MPI_STRM_CTRL_CLEANUP_MASK) {
		stream->cleanup_in_progress = true;

		dev_dbg(&pb->pdev->dev,
				"mipi input stream%u OVF interrupt (cleanup started)\n",
				stream->stream_id);

		/* The hardware will also initiate a cleanup operation
		 * on an overflow.  We don't want to wait for the
		 * cleanup operation to complete in the interrupt
		 * handler so we will spin off a workqueue verify that
		 * it completed successfully.  This is unlikely to fail
		 * but if it does then the MIPI stream will need to be
		 * reset.
		 */
		queue_delayed_work(system_wq, &stream->cleanup_work,
				usecs_to_jiffies(MIPI_CLEANUP_TIMEOUT_US));
	} else {
		/* If the cleanup bit is not set then the cleanup operation has
		 * already concluded.
		 */
		stream->is_clean = true;

		dev_dbg(&pb->pdev->dev,
				"mipi input stream%u OVF interrupt (cleanup completed)\n",
				stream->stream_id);
	}

	paintbox_irq_waiter_signal(pb, stream->irq, timestamp,
			stream->input.last_frame_number, -EIO);

#if CONFIG_PAINTBOX_VERSION_MAJOR == 0
	/* Set the MIPI error for the active transfer so the error can
	 * be reported on the DMA EOF interrupt.
	 */
	dma_set_mipi_error(pb, stream->dma_channel, -EIO);
#endif

	if (stream->disable_on_error) {
		mipi_input_disable_irqs_and_stream(pb, stream,
				MIPI_INPUT_SOF_IMR);
		stream->free_running = false;
		stream->frame_count = 0;
		stream->enabled = false;
	}
}

#if CONFIG_PAINTBOX_VERSION_MAJOR >= 1
/* The caller to this function must hold pb->io_ipu.mipi_lock. */
static void paintbox_mpi_process_sof_interrupts(struct paintbox_data *pb,
		uint32_t status, ktime_t timestamp)
{
	uint32_t ctrl;
	unsigned int stream_id;

	for (stream_id = 0; stream_id < pb->io_ipu.num_mipi_input_streams;
			stream_id++, status >>= 1) {
		struct paintbox_mipi_stream *stream;

		if (!(status & 0x1))
			continue;

		stream = &pb->io_ipu.mipi_input_streams[stream_id];
		paintbox_mipi_select_input_stream(pb, stream->stream_id);

		ctrl = readl(pb->io_ipu.ipu_base + MPI_STRM_CTRL);

		stream->input.last_frame_number = (int32_t)((ctrl &
				MPI_STRM_CTRL_NUM_FRAME_MASK) >>
				MPI_STRM_CTRL_NUM_FRAME_SHIFT);

		stream->input.stats.sof_interrupts++;

		paintbox_mipi_input_sof_interrupt(pb, stream, timestamp);
	}
}

/* This function must be called in an interrupt context */
irqreturn_t paintbox_mipi_input_interrupt(struct paintbox_data *pb,
		uint32_t interface_mask, ktime_t timestamp)
{
	uint32_t status;

	spin_lock(&pb->io_ipu.mipi_lock);

	status = readl(pb->io_ipu.ipu_base + MPI_ISR);

	do {
		if (status) {
			writel(status, pb->io_ipu.ipu_base + MPI_ISR);
			paintbox_mpi_process_sof_interrupts(pb, status,
					timestamp);
		}

		status = readl(pb->io_ipu.ipu_base + MPI_ISR_OVF);
		if (status) {
			writel(status, pb->io_ipu.ipu_base + MPI_ISR_OVF);
			paintbox_mpi_process_sof_interrupts(pb, status,
					timestamp);
		}

		status = readl(pb->io_ipu.ipu_base + MPI_ISR);
	} while (status);

	spin_unlock(&pb->io_ipu.mipi_lock);

	return IRQ_HANDLED;
}

/* The caller to this function must hold pb->io_ipu.mipi_lock. */
static void paintbox_mpi_process_ovf_interrupts(struct paintbox_data *pb,
		uint32_t status, ktime_t timestamp)
{
	unsigned int stream_id;

	for (stream_id = 0; stream_id < pb->io_ipu.num_mipi_input_streams;
			stream_id++, status >>= 1) {
		struct paintbox_mipi_stream *stream;

		if (!(status & 0x1))
			continue;

		stream = &pb->io_ipu.mipi_input_streams[stream_id];
		stream->input.stats.ovf_interrupts++;
		paintbox_mipi_input_ovf_interrupt(pb, stream, timestamp);
	}
}

/* This function must be called in an interrupt context */
irqreturn_t paintbox_mipi_input_error_interrupt(struct paintbox_data *pb,
		ktime_t timestamp)
{
	uint32_t status;

	spin_lock(&pb->io_ipu.mipi_lock);

	status = readl(pb->io_ipu.ipu_base + MPI_ERR_ISR);

	do {
		if (status) {
			writel(status, pb->io_ipu.ipu_base + MPI_ERR_ISR);
			paintbox_mpi_process_ovf_interrupts(pb, status,
					timestamp);
		}

		status = readl(pb->io_ipu.ipu_base + MPI_ERR_ISR_OVF);
		if (status) {
			writel(status, pb->io_ipu.ipu_base + MPI_ERR_ISR_OVF);
			paintbox_mpi_process_ovf_interrupts(pb, status,
					timestamp);
		}

		status = readl(pb->io_ipu.ipu_base + MPI_ERR_ISR);
	} while (status);

	spin_unlock(&pb->io_ipu.mipi_lock);

	return IRQ_HANDLED;
}

/* The caller to this function must hold pb->io_ipu.mipi_lock. */
static void paintbox_mpo_process_eof_interrupts(struct paintbox_data *pb,
		uint32_t status, ktime_t timestamp)
{
	unsigned int stream_id;

	for (stream_id = 0; stream_id < pb->io_ipu.num_mipi_output_streams;
			stream_id++, status >>= 1) {
		struct paintbox_mipi_stream *stream;

		if (!(status & 0x1))
			continue;

		stream = &pb->io_ipu.mipi_output_streams[stream_id];
		stream->output.stats.eof_interrupts++;
		paintbox_mipi_output_eof_interrupt(pb, stream, timestamp);
	}
}

/* This function must be called in an interrupt context */
irqreturn_t paintbox_mipi_output_interrupt(struct paintbox_data *pb,
		uint32_t interface_mask, ktime_t timestamp)
{
	uint32_t status;

	spin_lock(&pb->io_ipu.mipi_lock);

	status = readl(pb->io_ipu.ipu_base + MPO_ISR);

	do {
		if (status) {
			writel(status, pb->io_ipu.ipu_base + MPO_ISR);
			paintbox_mpo_process_eof_interrupts(pb, status,
					timestamp);
		}

		status = readl(pb->io_ipu.ipu_base + MPO_ISR_OVF);
		if (status) {
			writel(status, pb->io_ipu.ipu_base + MPO_ISR_OVF);
			paintbox_mpo_process_eof_interrupts(pb, status,
					timestamp);
		}

		status = readl(pb->io_ipu.ipu_base + MPO_ISR);
	} while (status);

	spin_unlock(&pb->io_ipu.mipi_lock);

	return IRQ_HANDLED;
}
#else
/* This function must be called in an interrupt context */
static void paintbox_mipi_input_stream_interrupt(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream, ktime_t timestamp)
{
	uint32_t ctrl;

	spin_lock(&pb->io_ipu.mipi_lock);

	paintbox_mipi_select_input_stream(pb, stream->stream_id);

	ctrl = readl(pb->io_ipu.ipu_base + MPI_STRM_CTRL);

	/* If there are no MIPI interrupts then return immediately. */
	if ((ctrl & (MPI_STRM_CTRL_SOF_ISR_MASK |
			MPI_STRM_CTRL_OVF_ISR_MASK)) == 0) {
		spin_unlock(&pb->io_ipu.mipi_lock);
		return;
	}

	if (ctrl & MPI_STRM_CTRL_SOF_ISR_MASK) {
		stream->input.last_frame_number = (int32_t)((ctrl &
				MPI_STRM_CTRL_NUM_FRAME_MASK) >>
				MPI_STRM_CTRL_NUM_FRAME_SHIFT);

		mipi_input_clear_control(pb, stream,
				MPI_STRM_CTRL_SOF_ISR_MASK);

		stream->input.stats.sof_interrupts++;

		paintbox_mipi_input_sof_interrupt(pb, stream, timestamp);
	}

	if (stream->input.missed_sof_interrupt) {
		stream->input.missed_sof_interrupt = false;
		stream->input.stats.missed_sof_interrupts++;

		/* If we missed the SOF then we can not guarantee that the
		 * frame number is valid.
		 */
		stream->input.last_frame_number = MIPI_INVALID_FRAME_NUMBER;

		paintbox_mipi_input_sof_interrupt(pb, stream, timestamp);
	}

	if (ctrl & MPI_STRM_CTRL_OVF_ISR_MASK) {
		mipi_input_clear_control(pb, stream,
				MPI_STRM_CTRL_OVF_ISR_MASK);

		stream->input.stats.ovf_interrupts++;

		paintbox_mipi_input_ovf_interrupt(pb, stream, timestamp);
	}

	if (stream->input.missed_ovf_interrupt) {
		stream->input.missed_ovf_interrupt = false;
		stream->input.stats.missed_ovf_interrupts++;

		paintbox_mipi_input_ovf_interrupt(pb, stream, timestamp);
	}

	spin_unlock(&pb->io_ipu.mipi_lock);
}

/* This function must be called in an interrupt context */
irqreturn_t paintbox_mipi_input_interrupt(struct paintbox_data *pb,
		uint32_t interface_mask, ktime_t timestamp)
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
				stream_index++) {
			struct paintbox_mipi_stream *stream;

			stream = interface->streams[stream_index];

			/* If the stream is not allocated then skip it. */
			if (!stream->allocated)
				continue;

			paintbox_mipi_input_stream_interrupt(pb, stream,
					timestamp);
		}
	}

	return IRQ_HANDLED;
}

/* This function must be called in an interrupt context */
static void paintbox_mipi_output_stream_interrupt(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream, ktime_t timestamp)
{
	uint32_t ctrl;

	spin_lock(&pb->io_ipu.mipi_lock);

	paintbox_mipi_select_output_stream(pb, stream->stream_id);

	ctrl = readl(pb->io_ipu.ipu_base + MPO_STRM_CTRL);
	if (ctrl & MPO_STRM_CTRL_EOF_ISR_MASK) {
		mipi_output_clear_control(pb, stream,
				MPO_STRM_CTRL_EOF_ISR_MASK);

		stream->output.stats.eof_interrupts++;

		/* The interrupt is acknowledged in the EOF interrupt handler */
		paintbox_mipi_output_eof_interrupt(pb, stream, timestamp);
	}

	if (stream->output.missed_eof_interrupt) {
		stream->output.missed_eof_interrupt = false;
		stream->output.stats.missed_eof_interrupts++;

		paintbox_mipi_output_eof_interrupt(pb, stream, timestamp);
	}

	spin_unlock(&pb->io_ipu.mipi_lock);
}

/* This function must be called in an interrupt context */
irqreturn_t paintbox_mipi_output_interrupt(struct paintbox_data *pb,
		uint32_t interface_mask, ktime_t timestamp)
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
				stream_index++) {
			struct paintbox_mipi_stream *stream;

			stream = interface->streams[stream_index];

			/* If the stream is not allocated then skip it. */
			if (!stream->allocated)
				continue;

			paintbox_mipi_output_stream_interrupt(pb, stream,
					timestamp);
		}
	}

	return IRQ_HANDLED;
}
#endif

static int paintbox_mipi_input_init(struct paintbox_data *pb)
{
	struct paintbox_io_ipu *ipu = &pb->io_ipu;
	unsigned int stream_id, inf_id;
	unsigned int streams_per_interface = ipu->num_mipi_input_streams /
			ipu->num_mipi_input_interfaces;
	int ret;

#if CONFIG_PAINTBOX_VERSION_MAJOR >= 1
	/* For MIPI we want to disable the interrupt at the IER to prevent
	 * false positive interrupts on unused streams in the ISR register.
	 * This can occur when virtual channel 0 is used.
	 */
	writel(0, pb->io_ipu.ipu_base + MPI_IER);
	writel(0, pb->io_ipu.ipu_base + MPI_ERR_IER);
#endif

	pb->io_ipu.selected_input_stream_id = MPI_STRM_SEL_DEF &
			MPI_STRM_SEL_MPI_STRM_SEL_M;

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
			stream->input.last_frame_number =
					MIPI_INVALID_FRAME_NUMBER;
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

#if CONFIG_PAINTBOX_VERSION_MAJOR >= 1
	/* For MIPI we want to disable the interrupt at the IER to prevent
	 * false positive interrupts on unused streams in the ISR register.
	 * This can occur when virtual channel 0 is used.
	 */
	writel(0, pb->io_ipu.ipu_base + MPO_IER);
#endif

	pb->io_ipu.selected_output_stream_id = MPO_STRM_SEL_DEF &
			MPO_STRM_SEL_MPO_STRM_SEL_M;

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
	int ret;

	spin_lock_init(&pb->io_ipu.mipi_lock);

	paintbox_mipi_debug_init(pb);

	if (pb->io_ipu.num_mipi_input_streams > 0) {
		ret = paintbox_mipi_input_init(pb);
		if (ret < 0)
			return ret;
	}

	if (pb->io_ipu.num_mipi_output_streams > 0) {
		ret = paintbox_mipi_output_init(pb);
		if (ret < 0) {
			kfree(pb->io_ipu.mipi_input_streams);
			return ret;
		}
	}

	return 0;
}
