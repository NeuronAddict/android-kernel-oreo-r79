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

#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/paintbox.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include "paintbox-debug.h"
#include "paintbox-io.h"
#include "paintbox-irq.h"
#include "paintbox-mipi.h"
#include "paintbox-mipi-debug.h"
#include "paintbox-regs.h"


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

static struct paintbox_mipi_stream *mipi_stream_lock(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned int stream_id,
		bool is_input, int *err)
{
	struct paintbox_mipi_stream *stream;

	mutex_lock(&pb->lock);

	stream = get_mipi_stream(pb, session, stream_id, is_input, err);
	if (!stream) {
		mutex_unlock(&pb->lock);
		return NULL;
	}

	io_disable_mipi_interrupts(pb);

	writel(stream_id, pb->io_ipu.ipu_base + stream->select_offset);

	*err = 0;

	return stream;
}

static void mipi_stream_unlock(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream)
{
	io_enable_mipi_interrupts(pb);

	mutex_unlock(&pb->lock);
}

int allocate_mipi_input_stream_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	unsigned int stream_id = (unsigned int)arg;
	struct paintbox_mipi_stream *stream;

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

	stream->session = session;
	list_add_tail(&stream->session_entry, &session->mipi_input_list);

	/* TODO(ahampson):  Determine allocation actions */

	mutex_unlock(&pb->lock);

	return 0;
}

int allocate_mipi_output_stream_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	unsigned int stream_id = (unsigned int)arg;
	struct paintbox_mipi_stream *stream;

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

	stream->session = session;
	list_add_tail(&stream->session_entry, &session->mipi_output_list);

	/* TODO(ahampson):  Determine allocation actions */

	mutex_unlock(&pb->lock);

	return 0;
}

/* The caller to this function must hold pb->lock and with mipi interrupts
 * disabled.
 */
void release_mipi_stream(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_mipi_stream *stream)
{
	writel(0, pb->io_ipu.ipu_base + stream->ctrl_offset);

	/* Remove the MIPI stream from the session. */
	list_del(&stream->session_entry);
	stream->session = NULL;
}

int release_mipi_stream_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg,
		bool is_input)
{
	unsigned int stream_id = (unsigned int)arg;
	struct paintbox_mipi_stream *stream;
	int ret;

	stream = mipi_stream_lock(pb, session, stream_id, is_input, &ret);
	if (ret < 0)
		return ret;

	unbind_mipi_interrupt(pb, session, stream);

	release_mipi_stream(pb, session, stream);

	mipi_stream_unlock(pb, stream);

	return 0;
}

/* The caller to this function must hold pb->lock and with mipi interrupts
 * disabled.
 */
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

/* The caller to this function must hold pb->lock and with mipi interrupts
 * disabled.
 */
static inline void strm_ctrl_set(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream, uint32_t new_val)
{
	uint32_t val;

	val = readl(pb->io_ipu.ipu_base + stream->ctrl_offset);
	val |= new_val;
	writel(val, pb->io_ipu.ipu_base + stream->ctrl_offset);
}

/* The caller to this function must hold pb->lock and with mipi interrupts
 * disabled.
 */
static inline void strm_ctrl_clr(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream, uint32_t new_val)
{
	uint32_t val;

	val = readl(pb->io_ipu.ipu_base + stream->ctrl_offset);
	val &= ~new_val;
	writel(val, pb->io_ipu.ipu_base + stream->ctrl_offset);
}

int enable_mipi_stream_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg,
		bool is_input)
{
	struct paintbox_mipi_interface *interface;
	struct paintbox_mipi_stream *stream;
	unsigned int stream_id = (unsigned int)arg;
	int ret = 0;

	stream = mipi_stream_lock(pb, session, stream_id, is_input, &ret);
	if (ret < 0)
		return ret;

	interface = stream->interface;

	strm_ctrl_set(pb, stream, MPI_STRM_EN);

	stream->enabled = true;

	mipi_stream_unlock(pb, stream);

	dev_dbg(&pb->pdev->dev, "%s: mipi %s stream%u: enable stream\n",
			__func__, is_input ? "input" : "output", stream_id);

	return 0;
}

int disable_mipi_stream_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg,
		bool is_input)
{
	struct paintbox_mipi_stream *stream;
	unsigned int stream_id = (unsigned int)arg;
	int ret = 0;

	stream = mipi_stream_lock(pb, session, stream_id, is_input, &ret);
	if (ret < 0)
		return ret;

	strm_ctrl_clr(pb, stream, MPI_STRM_EN);

	stream->enabled = false;

	mipi_stream_unlock(pb, stream);

	dev_dbg(&pb->pdev->dev, "%s: mipi %s stream%u: disable stream\n",
			__func__, is_input ? "input" : "output", stream_id);

	return 0;
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

	dev_dbg(&pb->pdev->dev,
			"\tvirtual channels: %u data type: %u data proc: %u\n",
			setup->virtual_channel, setup->data_type,
			setup->unpacked_data_type);
	dev_dbg(&pb->pdev->dev, "\twidth: %u height: %u stripe height: %u\n",
			setup->img_width, setup->img_height,
			setup->stripe_height);

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

/* mipi_stream_lock must be called before this function is called. */
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

/* mipi_stream_lock must be called before this function is called. */
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
	int ret;

	user_setup = (struct mipi_stream_setup __user *)arg;
	if (copy_from_user(&setup, user_setup, sizeof(setup)))
		return -EFAULT;

	stream = mipi_stream_lock(pb, session, setup.stream_id, is_input, &ret);
	if (ret < 0)
		return ret;

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
		mipi_stream_unlock(pb, stream);
		return ret;
	}

	/* TODO(ahampson):  Add support for row sync b/30276467 */

	if (setup.enable_on_setup || stream->enabled) {
		stream->enabled = true;
		strm_ctrl_set(pb, stream, MPI_STRM_EN);
	}

	mipi_stream_unlock(pb, stream);

	return 0;
}

int reset_mipi_stream_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg,
		bool is_input)
{
	struct paintbox_mipi_stream *stream;
	unsigned int stream_id = (unsigned int)arg;
	int ret = 0;

	stream = mipi_stream_lock(pb, session, stream_id, is_input, &ret);
	if (ret < 0)
		return ret;

	strm_ctrl_toggle(pb, stream, is_input ? MPI_STRM_RST : MPO_STRM_RST);

	stream->enabled = false;

	mipi_stream_unlock(pb, stream);

	dev_dbg(&pb->pdev->dev, "%s: mipi %s stream%u: reset\n", __func__,
			is_input ? "input" : "output", stream_id);

	return 0;
}

int cleanup_mipi_stream_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg,
		bool is_input)
{
	struct paintbox_mipi_stream *stream;
	unsigned int stream_id = (unsigned int)arg;
	int ret = 0;

	stream = mipi_stream_lock(pb, session, stream_id, is_input, &ret);
	if (ret < 0)
		return ret;

	strm_ctrl_toggle(pb, stream, is_input ? MPI_STRM_CLEANUP :
			MPO_STRM_CLEANUP);

	stream->enabled = false;

	mipi_stream_unlock(pb, stream);

	dev_dbg(&pb->pdev->dev, "%s: mipi %s stream%u: cleanup\n", __func__,
			is_input ? "input" : "output", stream_id);

	return 0;
}

int enable_mipi_interrupt_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg,
		bool is_input)
{
	struct paintbox_mipi_stream *stream;
	unsigned int stream_id = (unsigned int)arg;
	int ret = 0;

	stream = mipi_stream_lock(pb, session, stream_id, is_input, &ret);
	if (ret < 0)
		return ret;

	if (is_input) {
		strm_ctrl_set(pb, stream, MPI_STRM_SOF_IMR | MPI_STRM_OVF_IMR);
		io_enable_mipi_input_interface_interrupt(pb,
				stream->interface->interface_id);
	} else {
		strm_ctrl_set(pb, stream, MPO_STRM_EOF_IMR);
		io_enable_mipi_output_interface_interrupt(pb,
				stream->interface->interface_id);
	}

	mipi_stream_unlock(pb, stream);

	dev_dbg(&pb->pdev->dev, "%s: mipi %s stream%u: enable interrupt\n",
			__func__, is_input ? "input" : "output", stream_id);

	return 0;
}

int disable_mipi_interrupt_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg,
		bool is_input)
{
	struct paintbox_mipi_stream *stream;
	unsigned int stream_id = (unsigned int)arg;
	int ret = 0;

	stream = mipi_stream_lock(pb, session, stream_id, is_input, &ret);
	if (ret < 0)
		return ret;

	if (is_input) {
		strm_ctrl_clr(pb, stream, MPI_STRM_SOF_IMR | MPI_STRM_OVF_IMR);
		io_disable_mipi_input_interface_interrupt(pb,
				stream->interface->interface_id);
	} else {
		strm_ctrl_clr(pb, stream, MPO_STRM_EOF_IMR);
		io_disable_mipi_output_interface_interrupt(pb,
				stream->interface->interface_id);
	}

	dev_dbg(&pb->pdev->dev, "%s: mipi %s stream%u: disable interrupt\n",
			__func__, is_input ? "input" : "output", stream_id);

	mipi_stream_unlock(pb, stream);

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

/* This function is called from an interrupt context. */
void mipi_report_completion(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream, int err)
{
	signal_waiters(pb, stream->irq, err);
}

irqreturn_t paintbox_mipi_input_interrupt(struct paintbox_data *pb,
		uint32_t interface_mask)
{
	unsigned int interface_id, stream_index;

	for (interface_id = 0;
			interface_id < pb->io_ipu.num_mipi_input_interfaces &&
			interface_mask; interface_id++, interface_mask >>= 1) {
		struct paintbox_mipi_interface *interface;
		struct paintbox_mipi_stream *stream;

		if (!(interface_mask & 0x01))
			continue;

		interface = &pb->io_ipu.mipi_input_interfaces[interface_id];

		for (stream_index = 0; stream_index < interface->num_streams;
				stream_index++) {
			uint32_t status;

			stream = interface->streams[stream_index];

			writel(stream->stream_id, pb->io_ipu.ipu_base +
					MPI_STRM_SEL);

			status = readl(pb->io_ipu.ipu_base + MPI_STRM_CTRL);
			writel(status & ~(MPI_STRM_SOF_ISR | MPI_STRM_OVF_ISR),
					pb->io_ipu.ipu_base + MPI_STRM_CTRL);

			if (status & MPI_STRM_OVF_ISR)
				mipi_report_completion(pb, stream, -EIO);
			else if (status & MPI_STRM_SOF_ISR)
				mipi_report_completion(pb, stream, 0);
		}
	}

	return IRQ_HANDLED;
}

irqreturn_t paintbox_mipi_output_interrupt(struct paintbox_data *pb,
		uint32_t interface_mask)
{
	unsigned int interface_id, stream_index;

	for (interface_id = 0;
			interface_id < pb->io_ipu.num_mipi_output_interfaces &&
			interface_mask; interface_id++, interface_mask >>= 1) {
		struct paintbox_mipi_interface *interface;
		struct paintbox_mipi_stream *stream;

		if (!(interface_mask & 0x01))
			continue;

		interface = &pb->io_ipu.mipi_output_interfaces[interface_id];

		for (stream_index = 0; stream_index < interface->num_streams;
				stream_index++) {
			uint32_t status;

			stream = interface->streams[stream_index];

			writel(stream->stream_id, pb->io_ipu.ipu_base +
					MPO_STRM_SEL);

			status = readl(pb->io_ipu.ipu_base + MPO_STRM_CTRL);
			if (status & MPO_STRM_EOF_ISR) {
				writel(status & ~MPO_STRM_EOF_ISR,
						pb->io_ipu.ipu_base +
						MPO_STRM_CTRL);
				mipi_report_completion(pb, stream, 0);
			}
		}
	}

	return IRQ_HANDLED;
}

static int paintbox_mipi_input_init(struct paintbox_data *pb)
{
	struct paintbox_io_ipu *ipu = &pb->io_ipu;
	unsigned int stream_id, interface_id;
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

	for (interface_id = 0; interface_id < ipu->num_mipi_input_interfaces;
			interface_id++) {
		struct paintbox_mipi_interface *interface =
				&ipu->mipi_input_interfaces[interface_id];
		interface->interface_id = interface_id;
		interface->num_streams = streams_per_interface;

		interface->streams = kcalloc(ipu->num_mipi_input_streams,
				sizeof(struct paintbox_mipi_stream *),
				GFP_KERNEL);
		if (!interface->streams) {
			ret = -ENOMEM;
			goto err_exit;
		}
	}

	for (stream_id = 0, interface_id = 0;
			stream_id < ipu->num_mipi_input_streams; stream_id++) {
		struct paintbox_mipi_stream *stream =
				&ipu->mipi_input_streams[stream_id];
		struct paintbox_mipi_interface *interface =
				&ipu->mipi_input_interfaces[interface_id];

		stream->stream_id = stream_id;
		stream->interface = interface;
		stream->ctrl_offset = MPI_STRM_CTRL;
		stream->select_offset = MPI_STRM_SEL;
		stream->is_input = true;

		interface->streams[stream_id % streams_per_interface] = stream;

		paintbox_mipi_input_stream_debug_init(pb, stream);
		interface_id = stream_id / streams_per_interface;
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
	unsigned int stream_id, interface_id;
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

	for (interface_id = 0; interface_id < ipu->num_mipi_output_interfaces;
			interface_id++) {
		struct paintbox_mipi_interface *interface =
				&ipu->mipi_output_interfaces[interface_id];
		interface->interface_id = interface_id;
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
	for (stream_id = 0, interface_id = 0;
			stream_id < ipu->num_mipi_output_streams; stream_id++) {
		struct paintbox_mipi_stream *stream =
				&ipu->mipi_output_streams[stream_id];
		struct paintbox_mipi_interface *interface =
				&ipu->mipi_output_interfaces[interface_id];

		stream->stream_id = stream_id;
		stream->interface = &ipu->mipi_output_interfaces[interface_id];
		stream->ctrl_offset = MPO_STRM_CTRL;
		stream->select_offset = MPO_STRM_SEL;
		stream->is_input = false;

		interface->streams[stream_id % streams_per_interface] = stream;

		paintbox_mipi_output_stream_debug_init(pb, stream);
		interface_id = stream_id / streams_per_interface;
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
