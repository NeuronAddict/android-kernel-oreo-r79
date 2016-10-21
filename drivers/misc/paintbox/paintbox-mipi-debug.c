/*
 * MIPI Debug Support for Paintbox programmable IPU
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

#include <linux/debugfs.h>
#include <linux/kernel.h>
#include <linux/paintbox.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/types.h>

#include "paintbox-debug.h"
#include "paintbox-mipi.h"
#include "paintbox-mipi-debug.h"
#include "paintbox-regs.h"


static uint64_t mipi_reg_entry_read(struct paintbox_debug_reg_entry *reg_entry)
{
	struct paintbox_debug *debug = reg_entry->debug;
	struct paintbox_mipi_stream *stream = container_of(debug,
			struct paintbox_mipi_stream, debug);
	struct paintbox_data *pb = debug->pb;
	uint64_t val;

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
		uint64_t val)
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

static int dump_io_ipu_mpi_strm_ctrl(struct paintbox_data *pb, char *buf,
			int *written, size_t len)
{
	uint32_t val;

	val = readl(pb->io_ipu.ipu_base + MPI_STRM_CTRL);
	return dump_io_ipu_reg_verbose(pb, MPI_STRM_CTRL, buf, written, len,
			"\tOVF IMR %u OVF ISR %u SOF IMR %u SOF ISR %u RST %u "
			"CLEANUP %u EN %u\n", !!(val & MPI_STRM_OVF_IMR),
			!!(val & MPI_STRM_OVF_ISR), !!(val & MPI_STRM_SOF_IMR),
			!!(val & MPI_STRM_SOF_ISR), !!(val & MPI_STRM_RST),
			!!(val & MPI_STRM_CLEANUP), !!(val & MPI_STRM_EN));
}

static int dump_io_ipu_mpo_strm_ctrl(struct paintbox_data *pb, char *buf,
		int *written, size_t len)
{
	uint32_t val;

	val = readl(pb->io_ipu.ipu_base + MPO_STRM_CTRL);
	return dump_io_ipu_reg_verbose(pb, MPO_STRM_CTRL, buf, written, len,
			"\tEOF IMR %u EOF ISR %u RST %u CLEANUP %u EN %u\n",
			!!(val & MPO_STRM_EOF_IMR), !!(val & MPO_STRM_EOF_ISR),
			!!(val & MPO_STRM_RST), !!(val & MPO_STRM_CLEANUP),
			!!(val & MPO_STRM_EN));
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
	uint32_t val;

	val = readl(pb->io_ipu.ipu_base + MPI_CAP);
	ret = dump_io_ipu_reg_verbose(pb, MPI_CAP, buf, &written,
			len, "\tMAX_STRM %u MAX_IFC %u\n",
			(val & MPI_MAX_STRM_MASK) >> MPI_MAX_STRM_SHIFT,
			val & MPI_MAX_IFC_MASK);
	if (ret < 0)
		return ret;

	ret = dump_io_ipu_reg(pb, MPI_STRM_SEL, buf, &written, len);
	if (ret < 0)
		return ret;

	val = readl(pb->io_ipu.ipu_base + MPO_CAP);
	ret = dump_io_ipu_reg_verbose(pb, MPO_CAP, buf, &written,
			len, "\tMAX_STRM %u MAX_IFC %u\n",
			(val & MPO_MAX_STRM_MASK) >> MPO_MAX_STRM_SHIFT,
			val & MPO_MAX_IFC_MASK);
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

	ret = dump_io_ipu_mpi_strm_ctrl(pb, buf, &written, len);
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

	ret = dump_io_ipu_mpo_strm_ctrl(pb, buf, &written, len);
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

void log_mipi_registers(struct paintbox_data *pb,
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
						stream->stream_id].debug,
				pb->vdbg_log + written,
				pb->vdbg_log_len - written);
	else
		ret = dump_mipi_output_stream_registers(
				&pb->io_ipu.mipi_output_streams[
						stream->stream_id].debug,
				pb->vdbg_log + written,
				pb->vdbg_log_len - written);
	if (ret < 0)
		goto err_exit;

	dev_vdbg(&pb->pdev->dev, "%s\n%s", msg, pb->vdbg_log);

	return;

err_exit:
	dev_err(&pb->pdev->dev, "%s: register log error, err = %d", __func__,
			ret);
}

void paintbox_mipi_input_stream_debug_init(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream)
{
	paintbox_debug_create_entry(pb, &stream->debug,
			pb->io_ipu.debug.debug_dir, "in", stream->stream_id,
			dump_mipi_input_stream_registers, stream);

	paintbox_debug_create_reg_entries(pb, &stream->debug,
			io_ipu_reg_names, MPI_COMMON_NUM_REGS +
			MPI_STRM_NUM_REGS, mipi_reg_entry_write,
			mipi_reg_entry_read);
}

void paintbox_mipi_output_stream_debug_init(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream)
{
	paintbox_debug_create_entry(pb, &stream->debug,
			pb->io_ipu.debug.debug_dir, "out", stream->stream_id,
			dump_mipi_output_stream_registers, stream);

	paintbox_debug_create_reg_entries(pb, &stream->debug,
			&io_ipu_reg_names[REG_INDEX(MPO_COMMON_BLOCK_START)],
			MPO_COMMON_NUM_REGS + MPO_STRM_NUM_REGS,
			mipi_reg_entry_write, mipi_reg_entry_read);
}

void paintbox_mipi_debug_init(struct paintbox_data *pb)
{
	paintbox_debug_create_entry(pb, &pb->io_ipu.debug, pb->debug_root,
			"mipi", -1, dump_io_ipu_registers, &pb->io_ipu);

	paintbox_alloc_debug_buffer(pb, MIPI_DEBUG_BUFFER_SIZE);
}
