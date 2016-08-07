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

#include <linux/completion.h>
#include <linux/debugfs.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/paintbox.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/version.h>

#include "paintbox-debug.h"
#include "paintbox-io.h"
#include "paintbox-mipi.h"
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

static struct paintbox_mipi_stream *mipi_stream_lock(struct paintbox_data *pb,
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

	mutex_lock(&pb->lock);

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
	list_add_tail(&stream->entry, &session->mipi_input_list);

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
	list_add_tail(&stream->entry, &session->mipi_output_list);

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

	if (stream->wait_count > 0) {
		complete_all(&stream->completion);
		stream->wait_count = 0;
	}

	list_del(&stream->entry);
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

	release_mipi_stream(pb, session, stream);

	mipi_stream_unlock(pb, stream);

	return 0;
}

#ifdef CONFIG_DEBUG_FS
static uint32_t mipi_reg_entry_read(struct paintbox_debug_reg_entry *reg_entry)
{
	struct paintbox_debug *debug = reg_entry->debug;
	struct paintbox_mipi_stream *stream = container_of(debug,
			struct paintbox_mipi_stream, debug);
	struct paintbox_data *pb = debug->pb;
	uint32_t val;

	mutex_lock(&pb->lock);

	if (stream->is_input) {
		writel(stream->stream_id, pb->io_ipu.ipu_base + MPI_STRM_SEL);
		val = readl(pb->io_ipu.ipu_base + reg_entry->reg_offset);
	} else {
		writel(stream->stream_id, pb->io_ipu.ipu_base + MPO_STRM_SEL);
		val = readl(pb->io_ipu.ipu_base + MPO_COMMON_BLOCK_START +
				reg_entry->reg_offset);
	}

	mutex_unlock(&pb->lock);

	return val;
}

static void mipi_reg_entry_write(struct paintbox_debug_reg_entry *reg_entry,
		uint32_t val)
{
	struct paintbox_debug *debug = reg_entry->debug;
	struct paintbox_mipi_stream *stream = container_of(debug,
			struct paintbox_mipi_stream, debug);
	struct paintbox_data *pb = debug->pb;

	mutex_lock(&pb->lock);

	if (stream->is_input) {
		writel(stream->stream_id, pb->io_ipu.ipu_base + MPI_STRM_SEL);
		writel(val, pb->io_ipu.ipu_base + reg_entry->reg_offset);
	} else {
		writel(stream->stream_id, pb->io_ipu.ipu_base + MPO_STRM_SEL);
		writel(val, pb->io_ipu.ipu_base + MPO_COMMON_BLOCK_START +
				reg_entry->reg_offset);
	}

	mutex_unlock(&pb->lock);
}

#endif

#if defined(CONFIG_DEBUG_FS) || defined(VERBOSE_DEBUG)

static const char *io_ipu_reg_names[IO_IPU_NUM_REGS] = {
	REG_NAME_ENTRY(MPI_CAP),
	REG_NAME_ENTRY(MPI_STRM_SEL),
	REG_NAME_ENTRY(MPI_STRM_CTRL),
	REG_NAME_ENTRY(MPI_STRM_CNFG0_L),
	REG_NAME_ENTRY(MPI_STRM_CNFG0_H),
	REG_NAME_ENTRY(MPI_STRM_CNFG1_L),
	REG_NAME_ENTRY(MPI_STRM_CNFG1_H),
	REG_NAME_ENTRY(MPI_STRM_CNFG0_L_RO),
	REG_NAME_ENTRY(MPI_STRM_CNFG0_H_RO),
	REG_NAME_ENTRY(MPI_STRM_CNFG1_L_RO),
	REG_NAME_ENTRY(MPI_STRM_CNFG1_H_RO),
	REG_NAME_ENTRY(MPO_CAP),
	REG_NAME_ENTRY(MPO_STRM_SEL),
	REG_NAME_ENTRY(MPO_STRM_CTRL),
	REG_NAME_ENTRY(MPO_STRM_CNFG0_L),
	REG_NAME_ENTRY(MPO_STRM_CNFG0_H),
	REG_NAME_ENTRY(MPO_STRM_CNFG1),
	REG_NAME_ENTRY(MPO_STRM_CNFG0_L_RO),
	REG_NAME_ENTRY(MPO_STRM_CNFG0_H_RO),
	REG_NAME_ENTRY(MPO_STRM_CNFG1_RO)
};

static inline int dump_io_ipu_reg(struct paintbox_data *pb, uint32_t reg_offset,
		char *buf, int *written, size_t len)
{
	const char *reg_name = io_ipu_reg_names[REG_INDEX(reg_offset)];
	return dump_ipu_register(pb, pb->io_ipu.ipu_base, reg_offset, reg_name,
			buf, written, len);
}

static int dump_io_ipu_reg_verbose(struct paintbox_data *pb,
		uint32_t reg_offset, char *buf, int *written, size_t len,
		const char *format, ...)
{
	va_list args;
	int ret;

	ret = dump_io_ipu_reg(pb, reg_offset, buf, written, len);
	if (ret < 0)
		return ret;

	va_start(args, format);

	ret = dump_ipu_vprintf(pb, buf, written, len, format, args);

	va_end(args);

	return ret;
}

static int dump_io_ipu_strm_ctrl(struct paintbox_data *pb, uint32_t offset,
		char *buf, int *written, size_t len)
{
	uint32_t val;

	val = readl(pb->io_ipu.ipu_base + offset);
	return dump_io_ipu_reg_verbose(pb, offset, buf, written, len,
			"\tIMR %u IRQ %u RST %u CLEANUP %u EN %u\n",
			!!(val & MPI_STRM_IMR), !!(val & MPI_STRM_IRQ),
			!!(val & MPI_STRM_RST), !!(val & MPI_STRM_CLEANUP),
			!!(val & MPI_STRM_EN));
}

static int dump_io_ipu_strm_cnfg0_high(struct paintbox_data *pb,
		uint32_t offset, char *buf, int *written, size_t len)
{
	uint32_t val;

	val = readl(pb->io_ipu.ipu_base + offset);
	return dump_io_ipu_reg_verbose(pb, offset, buf, written, len,
			"\tIMG_WIDTH %u IMG_HEIGHT %u\n",
			val & MPI_IMG_WIDTH_MASK,
			(val & MPI_IMG_HEIGHT_MASK) >> MPI_IMG_HEIGHT_SHIFT);
}

int dump_io_ipu_registers(struct paintbox_debug *debug, char *buf, size_t len)
{
	struct paintbox_data *pb = debug->pb;
	int ret, written = 0;

	ret = dump_io_ipu_reg_verbose(pb, MPI_CAP, buf, &written,
			len, "\tMAX_STRM %u\n",
			readl(pb->io_ipu.ipu_base + MPI_CAP) &
			MPI_MAX_STRM_MASK);
	if (ret < 0)
		return ret;

	ret = dump_io_ipu_reg(pb, MPI_STRM_SEL, buf, &written, len);
	if (ret < 0)
		return ret;

	ret = dump_io_ipu_reg_verbose(pb, MPO_CAP, buf, &written,
			len, "\tMAX_STRM %u\n",
			readl(pb->io_ipu.ipu_base + MPO_CAP) &
			MPO_MAX_STRM_MASK);
	if (ret < 0)
		return ret;

	ret = dump_io_ipu_reg(pb, MPO_STRM_SEL, buf, &written, len);
	if (ret < 0)
		return ret;

	return written;
}

int dump_mipi_input_stream_registers(struct paintbox_debug *debug, char *buf,
		size_t len)
{
	struct paintbox_mipi_stream *stream = container_of(debug,
			struct paintbox_mipi_stream, debug);
	struct paintbox_data *pb = debug->pb;
	int ret, written = 0;
	uint32_t val;

	writel(stream->stream_id, pb->io_ipu.ipu_base + MPI_STRM_SEL);

	ret = dump_io_ipu_strm_ctrl(pb, MPI_STRM_CTRL, buf, &written, len);
	if (ret < 0)
		return ret;

	val = readl(pb->io_ipu.ipu_base + MPI_STRM_CNFG0_L);
	ret = dump_io_ipu_reg_verbose(pb, MPI_STRM_CNFG0_L, buf, &written, len,
			"\tVC %u DT_IN %u DT_PROC %u STRP_HEIGHT %u\n",
			val & MPI_VC_MASK,
			(val & MPI_DT_IN_MASK) >> MPI_DT_IN_SHIFT,
			(val & MPI_DT_PROC_MASK) >> MPI_DT_PROC_SHIFT,
			(val & MPI_STRP_HEIGHT_MASK) >> MPI_STRP_HEIGHT_SHIFT);
	if (ret < 0)
		return ret;

	ret = dump_io_ipu_strm_cnfg0_high(pb, MPI_STRM_CNFG0_H, buf, &written,
			len);
	if (ret < 0)
		return ret;

	val = readl(pb->io_ipu.ipu_base + MPI_STRM_CNFG1_L);
	ret = dump_io_ipu_reg_verbose(pb, MPI_STRM_CNFG1_L, buf, &written, len,
			"\tSEG_START %u SEG_END %u\n", val & MPI_SEG_START_MASK,
			(val & MPI_SEG_END_MASK) >> MPI_SEG_END_SHIFT);
	if (ret < 0)
		return ret;

	val = readl(pb->io_ipu.ipu_base + MPI_STRM_CNFG1_H);
	ret = dump_io_ipu_reg_verbose(pb, MPI_STRM_CNFG1_H, buf, &written, len,
			"\tSEGS_PER_ROW %u SEG_WORDS_PER_ROW %u\n",
			val & MPI_SEGS_PER_ROW_MASK, (val &
			MPI_SEG_WORDS_PER_ROW_MASK) >>
			MPI_SEG_WORDS_PER_ROW_SHIFT );
	if (ret < 0)
		return ret;

	val = readl(pb->io_ipu.ipu_base + MPI_STRM_CNFG0_L_RO);
	ret = dump_io_ipu_reg_verbose(pb, MPI_STRM_CNFG0_L_RO, buf, &written,
			len, "\tVC %u DT_IN %u DT_PROC %u\n", val & MPI_VC_MASK,
			(val & MPI_DT_IN_MASK) >> MPI_DT_IN_SHIFT,
			(val & MPI_DT_PROC_MASK) >> MPI_DT_PROC_SHIFT);
	if (ret < 0)
		return ret;

	ret = dump_io_ipu_strm_cnfg0_high(pb, MPI_STRM_CNFG0_H_RO, buf,
			&written, len);
	if (ret < 0)
		return ret;

	val = readl(pb->io_ipu.ipu_base + MPI_STRM_CNFG1_L_RO);
	ret = dump_io_ipu_reg_verbose(pb, MPI_STRM_CNFG1_L_RO, buf, &written,
			len, "\tSEG_START %u SEG_END %u\n",
			val & MPI_SEG_START_MASK,
			(val & MPI_SEG_END_MASK) >> MPI_SEG_END_SHIFT);
	if (ret < 0)
		return ret;

	val = readl(pb->io_ipu.ipu_base + MPI_STRM_CNFG1_H_RO);
	ret = dump_io_ipu_reg_verbose(pb, MPI_STRM_CNFG1_H_RO, buf, &written,
			len, "\tSEGS_PER_ROW %u SEG_WORDS_PER_ROW %u\n",
			val & MPI_SEGS_PER_ROW_MASK, (val &
			MPI_SEG_WORDS_PER_ROW_MASK) >>
			MPI_SEG_WORDS_PER_ROW_SHIFT );
	if (ret < 0)
		return ret;

	return written;
}

int dump_mipi_output_stream_registers(struct paintbox_debug *debug, char *buf,
		size_t len)
{
	struct paintbox_mipi_stream *stream = container_of(debug,
			struct paintbox_mipi_stream, debug);
	struct paintbox_data *pb = debug->pb;
	int ret, written = 0;
	uint32_t val;

	writel(stream->stream_id, pb->io_ipu.ipu_base + MPO_STRM_SEL);

	ret = dump_io_ipu_strm_ctrl(pb, MPO_STRM_CTRL, buf, &written, len);
	if (ret < 0)
		return ret;

	val = readl(pb->io_ipu.ipu_base + MPO_STRM_CNFG0_L);
	ret = dump_io_ipu_reg_verbose(pb, MPO_STRM_CNFG0_L, buf, &written, len,
			"\tVC %u DT_OUT %u DT_PROC %u STRP_HEIGHT %u\n",
			val & MPO_VC_MASK,
			(val & MPO_DT_OUT_MASK) >> MPO_DT_OUT_SHIFT,
			(val & MPO_DT_PROC_MASK) >> MPO_DT_PROC_SHIFT,
			(val & MPO_STRP_HEIGHT_MASK) >> MPO_STRP_HEIGHT_SHIFT);
	if (ret < 0)
		return ret;

	ret = dump_io_ipu_strm_cnfg0_high(pb, MPO_STRM_CNFG0_H, buf, &written,
			len);
	if (ret < 0)
		return ret;

	val = readl(pb->io_ipu.ipu_base + MPO_STRM_CNFG1);
	ret = dump_io_ipu_reg_verbose(pb, MPO_STRM_CNFG1, buf, &written, len,
			"\tSEG_END %u SEGS_PER_ROW %u\n",
			val & MPO_SEG_END_MASK, (val & MPO_SEGS_PER_ROW_MASK) >>
			MPO_SEGS_PER_ROW_SHIFT);
	if (ret < 0)
		return ret;

	val = readl(pb->io_ipu.ipu_base + MPO_STRM_CNFG0_L_RO);
	ret = dump_io_ipu_reg_verbose(pb, MPO_STRM_CNFG0_L_RO, buf, &written,
			len, "\tVC %u DT_OUT %u DT_PROC %u STRP_HEIGHT %u\n",
			val & MPO_VC_MASK,
			(val & MPO_DT_OUT_MASK) >> MPO_DT_OUT_SHIFT,
			(val & MPO_DT_PROC_MASK) >> MPO_DT_PROC_SHIFT,
			(val & MPO_STRP_HEIGHT_MASK) >> MPO_STRP_HEIGHT_SHIFT);

	ret = dump_io_ipu_strm_cnfg0_high(pb, MPO_STRM_CNFG0_H, buf, &written,
			len);
	if (ret < 0)
		return ret;

	val = readl(pb->io_ipu.ipu_base + MPO_STRM_CNFG1_RO);
	ret = dump_io_ipu_reg_verbose(pb, MPO_STRM_CNFG1_RO, buf, &written, len,
			"\tSEG_END %u SEGS_PER_ROW %u\n",
			val & MPO_SEG_END_MASK, (val & MPO_SEGS_PER_ROW_MASK) >>
			MPO_SEGS_PER_ROW_SHIFT);
	if (ret < 0)
		return ret;

	return written;
}

#endif

#ifdef VERBOSE_DEBUG
static void log_mipi_registers(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream, const char *msg)
{
	int ret, written;

	ret = snprintf(pb->vdbg_log, pb->vdbg_log_len, "mipi stream%u:\n",
			stream->stream_id);
	if (ret < 0)
		goto err_exit;

	written = ret;

	if (stream->is_input)
		ret = dump_mipi_input_stream_registers(
				&pb->io_ipu.mipi_input_streams[
						stream->stream_id],
				pb->vdbg_log + written,
				pb->vdbg_log_len - written);
	else
		ret = dump_mipi_output_stream_registers(
				&pb->io_ipu.mipi_output_streams[
						stream->stream_id],
				pb->vdbg_log + written,
				pb->vdbg_log_len - written);
	if (ret < 0)
		goto err_exit;

	dev_vdbg(&pb->pdev->dev, "%s\n%s", msg, pb->vdbg_log);

err_exit:
	dev_err(&pb->pdev->dev, "%s: register log error, err = %d", __func__,
			ret);
}

#define LOG_MIPI_REGISTERS(pb, stream)		\
	log_mipi_output_registers(pb, stream, __func__)

#else
#define LOG_MIPI_REGISTERS(pb, stream)		\
do { } while (0)
#endif

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
	struct paintbox_mipi_stream *stream;
	unsigned int stream_id = (unsigned int)arg;
	int ret = 0;

	stream = mipi_stream_lock(pb, session, stream_id, is_input, &ret);
	if (ret < 0)
		return ret;

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

	dev_dbg(&pb->pdev->dev,
			"\tvirtual channels: %u data type: %u data proc: %u\n",
			setup->virtual_channel, setup->data_type,
			setup->unpacked_data_type);
	dev_dbg(&pb->pdev->dev, "\twidth: %u height: %u\n", setup->img_width,
			setup->img_height);

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

	/* TODO(ahampson): Determine bounds checking for SEGS_PER_ROW and
	 * and STRP_HEIGHT if these are to be passed in.
	 */

	dev_dbg(&pb->pdev->dev, "\tseg start %u, seg end %u, seg words per row "
			"%u\n", setup->input.seg_start, setup->input.seg_end,
			setup->input.seg_words_per_row);

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
	writel(val, pb->io_ipu.ipu_base + MPI_STRM_CNFG0_L);

	val = setup->img_width & MPI_IMG_WIDTH_MASK;
	val |= (setup->img_height & MPI_IMG_HEIGHT_M) << MPI_IMG_HEIGHT_SHIFT;
	writel(val, pb->io_ipu.ipu_base + MPI_STRM_CNFG0_H);

	/* TODO(ahampson): Determine if SEGS_PER_ROW and STRP_HEIGHT should be
	 * passed in or computed in the driver.
	 */

	val = setup->input.seg_start & MPI_SEG_START_MASK;
	val |= (setup->input.seg_end & MPI_SEG_END_M) <<  MPI_SEG_END_SHIFT;
	writel(val, pb->io_ipu.ipu_base + MPI_STRM_CNFG1_L);

	writel(setup->input.seg_words_per_row & MPI_SEG_WORDS_PER_ROW_MASK,
			pb->io_ipu.ipu_base + MPI_STRM_CNFG1_H);

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
	writel(val, pb->io_ipu.ipu_base + MPO_STRM_CNFG0_L);

	val = setup->img_width & MPO_IMG_WIDTH_MASK;
	val |= (setup->img_height & MPO_IMG_HEIGHT_M) << MPO_IMG_HEIGHT_SHIFT;
	writel(val, pb->io_ipu.ipu_base + MPO_STRM_CNFG0_H);

	writel(setup->output.segs_per_row & MPO_SEGS_PER_ROW_MASK,
			pb->io_ipu.ipu_base + MPO_STRM_CNFG1);

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

	strm_ctrl_toggle(pb, stream, MPI_STRM_RST);

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

	strm_ctrl_toggle(pb, stream, MPI_STRM_CLEANUP);

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

	/* TODO(ahampson): This should be cleaned up when the QEMU kernel is
	 * updated to 3.13 or greater.
	 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 13, 0)
	INIT_COMPLETION(stream->completion);
#else
	reinit_completion(&stream->completion);
#endif

	strm_ctrl_set(pb, stream, MPI_STRM_IMR);

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

	strm_ctrl_clr(pb, stream, MPI_STRM_IMR);

	dev_dbg(&pb->pdev->dev, "%s: mipi %s stream%u: disable interrupt\n",
			__func__, is_input ? "input" : "output", stream_id);

	mipi_stream_unlock(pb, stream);

	return 0;
}

int wait_for_mipi_interrupt_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg,
		bool is_input)
{
	struct mipi_interrupt_wait __user *user_wait;
	struct mipi_interrupt_wait wait;
	struct paintbox_mipi_stream *stream;
	long err;
	int wait_ret = 0, ret = 0;

	user_wait = (struct mipi_interrupt_wait __user *)arg;
	if (copy_from_user(&wait, user_wait, sizeof(wait)))
		return -EFAULT;

	stream = mipi_stream_lock(pb, session, wait.stream_id, is_input, &ret);
	if (ret < 0)
		return ret;

	stream->wait_count++;

	mipi_stream_unlock(pb, stream);

	if (wait.timeout_ns != INT_MAX) {
		err = wait_for_completion_interruptible_timeout(
				&stream->completion,
				nsecs_to_jiffies64(wait.timeout_ns));
		if (err == 0) {
			dev_err(&pb->pdev->dev,
					"%s: mipi %s stream%u: wait for "
					"interrupt timeout\n", __func__,
					is_input ? "input" : "output",
					wait.stream_id);
			wait_ret = -ETIMEDOUT;
		}
	} else {
		wait_ret = wait_for_completion_interruptible(
				&stream->completion);
	}

	stream = mipi_stream_lock(pb, session, wait.stream_id, is_input, &ret);
	if (ret < 0)
		return ret;

	stream->wait_count--;

	if (wait_ret >= 0)
		wait_ret = stream->error;

	mipi_stream_unlock(pb, stream);

	return wait_ret;
}

/* This function is called from an interrupt context.  */
void mipi_report_completion(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream, int err)
{
	stream->error = err;
	complete_all(&stream->completion);
}

irqreturn_t paintbox_mipi_input_interrupt(struct paintbox_data *pb,
		uint32_t stream_mask)
{
	unsigned int stream_id;

	for (stream_id = 0; stream_id < pb->io_ipu.num_mipi_input_streams &&
			stream_mask; stream_id++, stream_mask >>= 1) {
		struct paintbox_mipi_stream *stream;

		if (!(stream_mask & 0x01))
			continue;

		stream = &pb->io_ipu.mipi_input_streams[stream_id];

		writel(stream_id, pb->io_ipu.ipu_base + MPI_STRM_SEL);

		strm_ctrl_clr(pb, stream, MPI_STRM_IRQ);

		stream->enabled = false;

		mipi_report_completion(pb, stream, -EIO);
	}

	return IRQ_HANDLED;
}

irqreturn_t paintbox_mipi_output_interrupt(struct paintbox_data *pb,
		uint32_t stream_mask)
{
	unsigned int stream_id;

	for (stream_id = 0; stream_id < pb->io_ipu.num_mipi_output_streams &&
			stream_mask; stream_id++, stream_mask >>= 1) {
		struct paintbox_mipi_stream *stream;

		if (!(stream_mask & 0x01))
			continue;

		stream = &pb->io_ipu.mipi_output_streams[stream_id];

		writel(stream_id, pb->io_ipu.ipu_base + MPI_STRM_SEL);

		strm_ctrl_clr(pb, stream, MPI_STRM_IRQ);

		mipi_report_completion(pb, stream, 0);
	}

	return IRQ_HANDLED;
}

static int paintbox_mipi_input_init(struct paintbox_data *pb)
{
	unsigned int stream_id;

	pb->io_ipu.mipi_input_streams = kzalloc(
			sizeof(struct paintbox_mipi_stream) *
			pb->io_ipu.num_mipi_input_streams, GFP_KERNEL);
	if (!pb->io_ipu.mipi_input_streams)
		return -ENOMEM;

	/* Store stream id with object as a convenience to avoid doing a lookup
	 * later on.
	 */
	for (stream_id = 0; stream_id < pb->io_ipu.num_mipi_input_streams;
			stream_id++) {
		struct paintbox_mipi_stream *stream =
				&pb->io_ipu.mipi_input_streams[stream_id];
		stream->stream_id = stream_id;
		stream->ctrl_offset = MPI_STRM_CTRL;
		stream->select_offset = MPI_STRM_SEL;
		stream->is_input = true;
		init_completion(&stream->completion);

#ifdef CONFIG_DEBUG_FS
		paintbox_debug_create_entry(pb, &stream->debug,
				pb->io_ipu.debug.debug_dir, "in",
				stream_id, dump_mipi_input_stream_registers,
				stream);

		paintbox_debug_create_reg_entries(pb, &stream->debug,
				io_ipu_reg_names,
				MPI_COMMON_NUM_REGS + MPI_STRM_NUM_REGS,
				mipi_reg_entry_write, mipi_reg_entry_read);
#endif
	}

	return 0;
}

static int paintbox_mipi_output_init(struct paintbox_data *pb)
{
	unsigned int stream_id;

	pb->io_ipu.mipi_output_streams = kzalloc(sizeof(
			struct paintbox_mipi_stream) *
			pb->io_ipu.num_mipi_output_streams, GFP_KERNEL);
	if (!pb->io_ipu.mipi_output_streams)
		return -ENOMEM;

	/* Store stream id with object as a convenience to avoid doing a lookup
	 * later on.
	 */
	for (stream_id = 0; stream_id < pb->io_ipu.num_mipi_output_streams;
			stream_id++) {
		struct paintbox_mipi_stream *stream =
				&pb->io_ipu.mipi_output_streams[stream_id];
		stream->stream_id = stream_id;
		stream->ctrl_offset = MPO_STRM_CTRL;
		stream->select_offset = MPO_STRM_SEL;
		stream->is_input = false;
		init_completion(&stream->completion);

#ifdef CONFIG_DEBUG_FS
		paintbox_debug_create_entry(pb, &stream->debug,
				pb->io_ipu.debug.debug_dir, "out",
				stream_id, dump_mipi_output_stream_registers,
				stream);

		paintbox_debug_create_reg_entries(pb, &stream->debug,
				&io_ipu_reg_names[REG_INDEX(
						MPO_COMMON_BLOCK_START)],
				MPO_COMMON_NUM_REGS + MPO_STRM_NUM_REGS,
				mipi_reg_entry_write, mipi_reg_entry_read);
#endif
	}

	return 0;
}

int paintbox_mipi_init(struct paintbox_data *pb)
{
	int ret;

	pb->io_ipu.ipu_base = pb->reg_base + IPU_IO_IPU_OFFSET;

#ifdef CONFIG_DEBUG_FS
	paintbox_debug_create_entry(pb, &pb->io_ipu.debug, pb->debug_root,
			"mipi", -1, dump_io_ipu_registers, &pb->io_ipu);
#endif

#ifdef VERBOSE_DEBUG
	paintbox_alloc_debug_buffer(pb, MIPI_DEBUG_BUFFER_SIZE);
#endif

	pb->io_ipu.num_mipi_input_streams = readl(pb->io_ipu.ipu_base +
			MPI_CAP);

	if (pb->io_ipu.num_mipi_input_streams > 0) {
		ret = paintbox_mipi_input_init(pb);
		if (ret < 0)
			return ret;
	}

	pb->io_ipu.num_mipi_output_streams = readl(pb->io_ipu.ipu_base +
			MPO_CAP);

	if (pb->io_ipu.num_mipi_output_streams > 0) {
		ret = paintbox_mipi_output_init(pb);
		if (ret < 0) {
			kfree(pb->io_ipu.mipi_input_streams);
			return ret;
		}
	}

	return 0;
}
