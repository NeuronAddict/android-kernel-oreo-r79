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
#include <linux/spinlock.h>
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
	unsigned long irq_flags;
	uint64_t val;

	mutex_lock(&pb->lock);

	spin_lock_irqsave(&pb->io_ipu.mipi_lock, irq_flags);

	if (stream->is_input) {
		writel(stream->stream_id, pb->io_ipu.ipu_base + MPI_STRM_SEL);
		val = readl(pb->io_ipu.ipu_base + reg_entry->reg_offset);
	} else {
		writel(stream->stream_id, pb->io_ipu.ipu_base + MPO_STRM_SEL);
		val = readl(pb->io_ipu.ipu_base + MPO_COMMON_BLOCK_START +
				reg_entry->reg_offset);
	}

	spin_unlock_irqrestore(&pb->io_ipu.mipi_lock, irq_flags);

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
	unsigned long irq_flags;

	mutex_lock(&pb->lock);

	spin_lock_irqsave(&pb->io_ipu.mipi_lock, irq_flags);

	if (stream->is_input) {
		writel(stream->stream_id, pb->io_ipu.ipu_base + MPI_STRM_SEL);
		writel(val, pb->io_ipu.ipu_base + reg_entry->reg_offset);
	} else {
		writel(stream->stream_id, pb->io_ipu.ipu_base + MPO_STRM_SEL);
		writel(val, pb->io_ipu.ipu_base + MPO_COMMON_BLOCK_START +
				reg_entry->reg_offset);
	}

	spin_unlock_irqrestore(&pb->io_ipu.mipi_lock, irq_flags);

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
		uint32_t reg_value, char *buf, int *written, size_t len)
{
	const char *reg_name = io_ipu_reg_names[REG_INDEX(reg_offset)];
	return dump_ipu_register_with_value(pb, pb->io_ipu.ipu_base, reg_offset,
			reg_value, reg_name, buf, written, len);
}

static int dump_io_ipu_reg_verbose(struct paintbox_data *pb,
		uint32_t reg_offset, uint32_t reg_value, char *buf,
		int *written, size_t len, const char *format, ...)
{
	va_list args;
	int ret;

	ret = dump_io_ipu_reg(pb, reg_offset, reg_value, buf, written, len);
	if (ret < 0)
		return ret;

	va_start(args, format);

	ret = dump_ipu_vprintf(pb, buf, written, len, format, args);

	va_end(args);

	return ret;
}

static int dump_io_ipu_strm_cnfg0_high(struct paintbox_data *pb,
		uint32_t offset, uint32_t val, char *buf, int *written,
		size_t len)
{
	return dump_io_ipu_reg_verbose(pb, offset, val, buf, written, len,
			"\tIMG_WIDTH %u IMG_HEIGHT %u\n",
			val & MPI_IMG_WIDTH_MASK,
			(val & MPI_IMG_HEIGHT_MASK) >> MPI_IMG_HEIGHT_SHIFT);
}

int dump_io_ipu_registers(struct paintbox_debug *debug, char *buf, size_t len)
{
	uint32_t mipi_registers[MPI_COMMON_NUM_REGS];
	struct paintbox_data *pb = debug->pb;
	unsigned long irq_flags;
	unsigned int reg_offset;
	int ret, written = 0;
	uint32_t val;

	spin_lock_irqsave(&pb->io_ipu.mipi_lock, irq_flags);

	for (reg_offset = MPI_CAP; reg_offset < MPI_COMMON_BLOCK_END;
			reg_offset += IPU_REG_WIDTH) {
		if (!io_ipu_reg_names[REG_INDEX(reg_offset)])
			continue;

		mipi_registers[REG_INDEX(reg_offset)] =
				readl(pb->io_ipu.ipu_base + reg_offset);
	}

	spin_unlock_irqrestore(&pb->io_ipu.mipi_lock, irq_flags);

	val = mipi_registers[REG_INDEX(MPI_CAP)];
	ret = dump_io_ipu_reg_verbose(pb, MPI_CAP, val, buf, &written,
			len, "\tMAX_STRM %u MAX_IFC %u\n",
			(val & MPI_MAX_STRM_MASK) >> MPI_MAX_STRM_SHIFT,
			val & MPI_MAX_IFC_MASK);
	if (ret < 0)
		return ret;

	val = mipi_registers[REG_INDEX(MPI_CAP)];
	ret = dump_io_ipu_reg(pb, MPI_STRM_SEL, val, buf, &written, len);
	if (ret < 0)
		return ret;

	spin_lock_irqsave(&pb->io_ipu.mipi_lock, irq_flags);

	for (reg_offset = MPO_COMMON_BLOCK_START;
			reg_offset < MPO_COMMON_BLOCK_END;
			reg_offset += IPU_REG_WIDTH) {
		if (!io_ipu_reg_names[REG_INDEX(reg_offset)])
			continue;

		mipi_registers[REG_INDEX(reg_offset - MPO_COMMON_BLOCK_START)] =
				readl(pb->io_ipu.ipu_base + reg_offset);
	}

	spin_unlock_irqrestore(&pb->io_ipu.mipi_lock, irq_flags);

	val = mipi_registers[REG_INDEX(MPO_CAP - MPO_COMMON_BLOCK_START)];
	ret = dump_io_ipu_reg_verbose(pb, MPO_CAP, val, buf, &written,
			len, "\tMAX_STRM %u MAX_IFC %u\n",
			(val & MPO_MAX_STRM_MASK) >> MPO_MAX_STRM_SHIFT,
			val & MPO_MAX_IFC_MASK);
	if (ret < 0)
		return ret;

	val = mipi_registers[REG_INDEX(MPO_STRM_SEL - MPO_COMMON_BLOCK_START)];
	ret = dump_io_ipu_reg(pb, MPO_STRM_SEL, val, buf, &written, len);
	if (ret < 0)
		return ret;

	return written;
}

int dump_mipi_input_stream_registers(struct paintbox_debug *debug, char *buf,
		size_t len)
{
	uint32_t mipi_registers[MPI_STRM_NUM_REGS];
	struct paintbox_mipi_stream *stream = container_of(debug,
			struct paintbox_mipi_stream, debug);
	struct paintbox_data *pb = debug->pb;
	unsigned int reg_offset;
	unsigned long irq_flags;
	int ret, written = 0;
	uint32_t val;

	spin_lock_irqsave(&pb->io_ipu.mipi_lock, irq_flags);

	writel(stream->stream_id, pb->io_ipu.ipu_base + MPI_STRM_SEL);

	for (reg_offset = MPI_STRM_BLOCK_START; reg_offset < MPI_STRM_BLOCK_END;
			reg_offset += IPU_REG_WIDTH) {
		if (!io_ipu_reg_names[REG_INDEX(reg_offset)])
			continue;

		mipi_registers[REG_INDEX(reg_offset - MPI_STRM_BLOCK_START)] =
				readl(pb->io_ipu.ipu_base + reg_offset);
	}

	spin_unlock_irqrestore(&pb->io_ipu.mipi_lock, irq_flags);

	val = mipi_registers[REG_INDEX(MPI_STRM_CTRL - MPI_STRM_BLOCK_START)];
	ret = dump_io_ipu_reg_verbose(pb, MPI_STRM_CTRL, val, buf, &written,
			len,
			"\tOVF IMR %u OVF ISR %u SOF IMR %u SOF ISR %u RST %u "
			"CLEANUP %u EN %u\n", !!(val & MPI_STRM_OVF_IMR),
			!!(val & MPI_STRM_OVF_ISR), !!(val & MPI_STRM_SOF_IMR),
			!!(val & MPI_STRM_SOF_ISR), !!(val & MPI_STRM_RST),
			!!(val & MPI_STRM_CLEANUP), !!(val & MPI_STRM_EN));
	if (ret < 0)
		return ret;

	val = mipi_registers[REG_INDEX(MPI_STRM_CNFG0_L -
			MPI_STRM_BLOCK_START)];
	ret = dump_io_ipu_reg_verbose(pb, MPI_STRM_CNFG0_L, val, buf, &written,
			len, "\tVC %u DT_IN %u DT_PROC %u STRP_HEIGHT %u\n",
			val & MPI_VC_MASK,
			(val & MPI_DT_IN_MASK) >> MPI_DT_IN_SHIFT,
			(val & MPI_DT_PROC_MASK) >> MPI_DT_PROC_SHIFT,
			((val & MPI_STRP_HEIGHT_MASK) >>
			MPI_STRP_HEIGHT_SHIFT) + 1);
	if (ret < 0)
		return ret;

	val = mipi_registers[REG_INDEX(MPI_STRM_CNFG0_H -
			MPI_STRM_BLOCK_START)];
	ret = dump_io_ipu_strm_cnfg0_high(pb, MPI_STRM_CNFG0_H, val, buf,
			&written, len);
	if (ret < 0)
		return ret;

	val = mipi_registers[REG_INDEX(MPI_STRM_CNFG1_L -
			MPI_STRM_BLOCK_START)];
	ret = dump_io_ipu_reg_verbose(pb, MPI_STRM_CNFG1_L, val, buf, &written,
			len, "\tSEG_START %u SEG_END %u\n",
			val & MPI_SEG_START_MASK,
			((val & MPI_SEG_END_MASK) >> MPI_SEG_END_SHIFT) + 1);
	if (ret < 0)
		return ret;

	val = mipi_registers[REG_INDEX(MPI_STRM_CNFG1_H -
			MPI_STRM_BLOCK_START)];
	ret = dump_io_ipu_reg_verbose(pb, MPI_STRM_CNFG1_H, val, buf, &written,
			len, "\tSEGS_PER_ROW %u SEG_WORDS_PER_ROW %u\n",
			(val & MPI_SEGS_PER_ROW_MASK) + 1,
			((val & MPI_SEG_WORDS_PER_ROW_MASK) >>
			MPI_SEG_WORDS_PER_ROW_SHIFT) + 1);
	if (ret < 0)
		return ret;

	val = mipi_registers[REG_INDEX(MPI_STRM_CNFG0_L_RO -
			MPI_STRM_BLOCK_START)];
	ret = dump_io_ipu_reg_verbose(pb, MPI_STRM_CNFG0_L_RO, val, buf,
			&written, len, "\tVC %u DT_IN %u DT_PROC %u STRP_HEIGHT"
			"%u\n", val & MPI_VC_MASK,
			(val & MPI_DT_IN_MASK) >> MPI_DT_IN_SHIFT,
			(val & MPI_DT_PROC_MASK) >> MPI_DT_PROC_SHIFT,
			((val & MPI_STRP_HEIGHT_MASK) >>
			MPI_STRP_HEIGHT_SHIFT) + 1);
	if (ret < 0)
		return ret;

	val = mipi_registers[REG_INDEX(MPI_STRM_CNFG0_H_RO -
			MPI_STRM_BLOCK_START)];
	ret = dump_io_ipu_strm_cnfg0_high(pb, MPI_STRM_CNFG0_H_RO, val, buf,
			&written, len);
	if (ret < 0)
		return ret;

	val = mipi_registers[REG_INDEX(MPI_STRM_CNFG1_L_RO -
			MPI_STRM_BLOCK_START)];
	ret = dump_io_ipu_reg_verbose(pb, MPI_STRM_CNFG1_L_RO, val, buf,
			&written, len, "\tSEG_START %u SEG_END %u\n",
			val & MPI_SEG_START_MASK,
			((val & MPI_SEG_END_MASK) >> MPI_SEG_END_SHIFT) + 1);
	if (ret < 0)
		return ret;

	val = mipi_registers[REG_INDEX(MPI_STRM_CNFG1_H_RO -
			MPI_STRM_BLOCK_START)];
	ret = dump_io_ipu_reg_verbose(pb, MPI_STRM_CNFG1_H_RO, val, buf,
			&written, len,
			"\tSEGS_PER_ROW %u SEG_WORDS_PER_ROW %u\n",
			(val & MPI_SEGS_PER_ROW_MASK) + 1,
			((val & MPI_SEG_WORDS_PER_ROW_MASK) >>
			MPI_SEG_WORDS_PER_ROW_SHIFT) + 1);
	if (ret < 0)
		return ret;

	return written;
}

int dump_mipi_output_stream_registers(struct paintbox_debug *debug, char *buf,
		size_t len)
{
	uint32_t mipi_registers[MPO_STRM_NUM_REGS];
	struct paintbox_mipi_stream *stream = container_of(debug,
			struct paintbox_mipi_stream, debug);
	struct paintbox_data *pb = debug->pb;
	unsigned int reg_offset;
	unsigned long irq_flags;
	int ret, written = 0;
	uint32_t val;

	spin_lock_irqsave(&pb->io_ipu.mipi_lock, irq_flags);

	writel(stream->stream_id, pb->io_ipu.ipu_base + MPO_STRM_SEL);

	for (reg_offset = MPO_STRM_BLOCK_START; reg_offset < MPO_STRM_BLOCK_END;
			reg_offset += IPU_REG_WIDTH) {

		if (!io_ipu_reg_names[REG_INDEX(reg_offset)])
			continue;

		mipi_registers[REG_INDEX(reg_offset - MPO_STRM_BLOCK_START)] =
				readl(pb->io_ipu.ipu_base + reg_offset);
	}

	spin_unlock_irqrestore(&pb->io_ipu.mipi_lock, irq_flags);

	val = mipi_registers[REG_INDEX(MPO_STRM_CTRL - MPO_STRM_BLOCK_START)];
	ret = dump_io_ipu_reg_verbose(pb, MPO_STRM_CTRL, val, buf, &written,
			len,
			"\tEOF IMR %u EOF ISR %u RST %u CLEANUP %u EN %u\n",
			!!(val & MPO_STRM_EOF_IMR), !!(val & MPO_STRM_EOF_ISR),
			!!(val & MPO_STRM_RST), !!(val & MPO_STRM_CLEANUP),
			!!(val & MPO_STRM_EN));
	if (ret < 0)
		return ret;

	val = mipi_registers[REG_INDEX(MPO_STRM_CNFG0_L -
			MPO_STRM_BLOCK_START)];
	ret = dump_io_ipu_reg_verbose(pb, MPO_STRM_CNFG0_L, val, buf, &written,
			len,
			"\tVC %u DT_OUT %u DT_PROC %u STRP_HEIGHT %u\n",
			val & MPO_VC_MASK,
			(val & MPO_DT_OUT_MASK) >> MPO_DT_OUT_SHIFT,
			(val & MPO_DT_PROC_MASK) >> MPO_DT_PROC_SHIFT,
			((val & MPO_STRP_HEIGHT_MASK) >>
			MPO_STRP_HEIGHT_SHIFT) + 1);
	if (ret < 0)
		return ret;

	val = mipi_registers[REG_INDEX(MPO_STRM_CNFG0_H -
			MPO_STRM_BLOCK_START)];
	ret = dump_io_ipu_strm_cnfg0_high(pb, MPO_STRM_CNFG0_H, val, buf,
			&written, len);
	if (ret < 0)
		return ret;

	val = mipi_registers[REG_INDEX(MPO_STRM_CNFG1 - MPO_STRM_BLOCK_START)];
	ret = dump_io_ipu_reg_verbose(pb, MPO_STRM_CNFG1, val, buf, &written,
			len, "\tSEG_END %u SEGS_PER_ROW %u\n",
			(val & MPO_SEG_END_MASK) + 1,
			((val & MPO_SEGS_PER_ROW_MASK) >>
			MPO_SEGS_PER_ROW_SHIFT) + 1);
	if (ret < 0)
		return ret;

	val = mipi_registers[REG_INDEX(MPO_STRM_CNFG0_L_RO -
			MPO_STRM_BLOCK_START)];
	ret = dump_io_ipu_reg_verbose(pb, MPO_STRM_CNFG0_L_RO, val, buf,
			&written, len,
			"\tVC %u DT_OUT %u DT_PROC %u STRP_HEIGHT %u\n",
			val & MPO_VC_MASK,
			(val & MPO_DT_OUT_MASK) >> MPO_DT_OUT_SHIFT,
			(val & MPO_DT_PROC_MASK) >> MPO_DT_PROC_SHIFT,
			((val & MPO_STRP_HEIGHT_MASK) >>
			MPO_STRP_HEIGHT_SHIFT) + 1);

	val = mipi_registers[REG_INDEX(MPO_STRM_CNFG0_H_RO -
			MPO_STRM_BLOCK_START)];
	ret = dump_io_ipu_strm_cnfg0_high(pb, MPO_STRM_CNFG0_H_RO, val, buf,
			&written, len);
	if (ret < 0)
		return ret;

	val = mipi_registers[REG_INDEX(MPO_STRM_CNFG1_RO -
			MPO_STRM_BLOCK_START)];
	ret = dump_io_ipu_reg_verbose(pb, MPO_STRM_CNFG1_RO, val, buf, &written,
			len, "\tSEG_END %u SEGS_PER_ROW %u\n",
			val & MPO_SEG_END_MASK, (val & MPO_SEGS_PER_ROW_MASK) >>
			MPO_SEGS_PER_ROW_SHIFT);
	if (ret < 0)
		return ret;

	return written;
}

/* The caller to this function must hold pb->lock */
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

int dump_mipi_input_stream_stats(struct paintbox_debug *debug, char *buf,
		size_t len)
{
	struct paintbox_mipi_stream *stream = container_of(debug,
			struct paintbox_mipi_stream, debug);
	struct paintbox_mipi_interface *interface = stream->interface;
	int ret, written;

	ret = snprintf(buf, len,
			"interrupts INF %u SOF %u OVF %u Missed SOF %u Missed "
			"OVF %u\n", interface->inf_interrupts,
			stream->input.stats.sof_interrupts,
			stream->input.stats.ovf_interrupts,
			stream->input.stats.missed_sof_interrupts,
			stream->input.stats.missed_ovf_interrupts);
	if (ret < 0)
		return ret;

	written = ret;

	ret = snprintf(buf + written, len - written,
			"\tenabled %d free running %d frames remaining %d last "
			"frame %d\n", stream->enabled, stream->free_running,
			stream->frame_count, stream->last_frame);
	if (ret < 0)
		return ret;

	return written + ret;
}

int dump_mipi_output_stream_stats(struct paintbox_debug *debug, char *buf,
		size_t len)
{
	struct paintbox_mipi_stream *stream = container_of(debug,
			struct paintbox_mipi_stream, debug);
	struct paintbox_mipi_interface *interface = stream->interface;
	int ret, written;

	ret = snprintf(buf, len, "interrupts INF %u EOF %u Missed EOF %u\n",
			interface->inf_interrupts,
			stream->output.stats.eof_interrupts,
			stream->output.stats.missed_eof_interrupts);
	if (ret < 0)
		return ret;

	written = ret;

	ret = snprintf(buf + written, len - written,
			"\tenabled %d free running %d frames remaining %d last "
			"frame %d\n", stream->enabled, stream->free_running,
			stream->frame_count, stream->last_frame);
	if (ret < 0)
		return ret;

	return written + ret;
}

void paintbox_mipi_input_stream_debug_init(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream)
{
	paintbox_debug_create_entry(pb, &stream->debug,
			pb->io_ipu.debug.debug_dir, "in", stream->stream_id,
			dump_mipi_input_stream_registers,
			dump_mipi_input_stream_stats, stream);

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
			dump_mipi_output_stream_registers,
			dump_mipi_output_stream_stats, stream);

	paintbox_debug_create_reg_entries(pb, &stream->debug,
			&io_ipu_reg_names[REG_INDEX(MPO_COMMON_BLOCK_START)],
			MPO_COMMON_NUM_REGS + MPO_STRM_NUM_REGS,
			mipi_reg_entry_write, mipi_reg_entry_read);
}

void paintbox_mipi_debug_init(struct paintbox_data *pb)
{
	paintbox_debug_create_entry(pb, &pb->io_ipu.debug, pb->debug_root,
			"mipi", -1, dump_io_ipu_registers, NULL, &pb->io_ipu);

	paintbox_alloc_debug_buffer(pb, MIPI_DEBUG_BUFFER_SIZE);
}
