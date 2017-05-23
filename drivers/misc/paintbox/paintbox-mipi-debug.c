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
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <uapi/paintbox.h>

#include "paintbox-debug.h"
#include "paintbox-mipi.h"
#include "paintbox-mipi-debug.h"
#include "paintbox-regs.h"

void paintbox_log_mipi_input_setup(struct paintbox_data *pb,
		struct mipi_stream_setup *setup)
{
	dev_info(&pb->pdev->dev, "mipi input stream%u setup\n",
			setup->stream_id);
	dev_info(&pb->pdev->dev,
			"\twidth %upx height %upx virtual channel %u data type %u data proc %u\n",
			setup->img_width, setup->img_height,
			setup->virtual_channel, setup->data_type,
			setup->unpacked_data_type);
	dev_info(&pb->pdev->dev,
			"\tseg start %u seg end %u segs per row %u seg words per row %u stripe_height %u\n",
			setup->input.seg_start, setup->seg_end,
			setup->segs_per_row, setup->input.seg_words_per_row,
			setup->stripe_height);
	dev_info(&pb->pdev->dev,
			"\tenable on setup %u free running %u frame count %d\n",
			setup->enable_on_setup, setup->free_running,
			setup->frame_count);
}

void paintbox_log_mipi_output_setup(struct paintbox_data *pb,
		struct mipi_stream_setup *setup)
{
	dev_info(&pb->pdev->dev, "mipi output stream%u setup\n",
			setup->stream_id);
	dev_info(&pb->pdev->dev,
			"\twidth %upx height %upx virtual channel %u data type %u data proc %u\n",
			setup->img_width, setup->img_height,
			setup->virtual_channel, setup->data_type,
			setup->unpacked_data_type);
	dev_info(&pb->pdev->dev,
			"\tseg end %u segs per row %u stripe height %u\n",
			setup->seg_end, setup->segs_per_row,
			setup->stripe_height);
	dev_info(&pb->pdev->dev,
			"\tenable on setup %u enable row sync %u free running %u frame count %d\n",
			setup->enable_on_setup, setup->output.enable_row_sync,
			setup->free_running, setup->frame_count);
}

#ifdef CONFIG_PAINTBOX_DEBUG
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
		paintbox_mipi_select_input_stream(pb, stream->stream_id);
		val = readq(pb->io_ipu.ipu_base + reg_entry->reg_offset);
	} else {
		paintbox_mipi_select_output_stream(pb, stream->stream_id);
		val = readq(pb->io_ipu.ipu_base + MPO_COMMON_BLOCK_START +
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
		paintbox_mipi_select_input_stream(pb, stream->stream_id);
		writeq(val, pb->io_ipu.ipu_base + reg_entry->reg_offset);
	} else {
		paintbox_mipi_select_output_stream(pb, stream->stream_id);
		writeq(val, pb->io_ipu.ipu_base + MPO_COMMON_BLOCK_START +
				reg_entry->reg_offset);
	}

	spin_unlock_irqrestore(&pb->io_ipu.mipi_lock, irq_flags);

	mutex_unlock(&pb->lock);
}
#endif

static const char *io_ipu_reg_names[IO_IPU_NUM_REGS] = {
	REG_NAME_ENTRY(MPI_CAP),
	REG_NAME_ENTRY(MPI_STRM_SEL),
	REG_NAME_ENTRY(MPI_STRM_CTRL),
	REG_NAME_ENTRY(MPI_STRM_CNFG0),
	REG_NAME_ENTRY(MPI_STRM_CNFG1),
	REG_NAME_ENTRY(MPI_STRM_CNFG0_RO),
	REG_NAME_ENTRY(MPI_STRM_CNFG1_RO),
	REG_NAME_ENTRY(MPO_CAP),
	REG_NAME_ENTRY(MPO_STRM_SEL),
	REG_NAME_ENTRY(MPO_STRM_CTRL),
	REG_NAME_ENTRY(MPO_STRM_CNFG0),
	REG_NAME_ENTRY(MPO_STRM_CNFG1),
	REG_NAME_ENTRY(MPO_STRM_CNFG0_RO),
	REG_NAME_ENTRY(MPO_STRM_CNFG1_RO),
#if CONFIG_PAINTBOX_VERSION_MAJOR >= 1
	REG_NAME_ENTRY(MPI_IER),
	REG_NAME_ENTRY(MPI_IMR),
	REG_NAME_ENTRY(MPI_ISR),
	REG_NAME_ENTRY(MPI_ISR_OVF),
	REG_NAME_ENTRY(MPI_ITR),
	REG_NAME_ENTRY(MPI_ERR_IER),
	REG_NAME_ENTRY(MPI_ERR_IMR),
	REG_NAME_ENTRY(MPI_ERR_ISR),
	REG_NAME_ENTRY(MPI_ERR_ISR_OVF),
	REG_NAME_ENTRY(MPI_ERR_ITR),
	REG_NAME_ENTRY(MPI_CTRL),
	REG_NAME_ENTRY(MPI_STATUS),
	REG_NAME_ENTRY(MPO_IER),
	REG_NAME_ENTRY(MPO_IMR),
	REG_NAME_ENTRY(MPO_ISR),
	REG_NAME_ENTRY(MPO_ISR_OVF),
	REG_NAME_ENTRY(MPO_ITR),
	REG_NAME_ENTRY(MPO_CTRL),
	REG_NAME_ENTRY(MPO_STATUS),
#endif
};

static inline int dump_io_ipu_reg(struct paintbox_data *pb, uint32_t reg_offset,
		uint64_t reg_value, char *buf, int *written, size_t len)
{
	const char *reg_name = io_ipu_reg_names[REG_INDEX(reg_offset)];

	return dump_ipu_register_with_value(pb, pb->io_ipu.ipu_base, reg_offset,
			reg_value, reg_name, buf, written, len);
}

static int dump_io_ipu_reg_verbose(struct paintbox_data *pb,
		uint32_t reg_offset, uint64_t reg_value, char *buf,
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

#if CONFIG_PAINTBOX_VERSION_MAJOR >= 1
static int dump_mipi_input_strm_ctrl(struct paintbox_data *pb, uint32_t offset,
		uint64_t val, char *buf, int *written, size_t len)
{
	return dump_io_ipu_reg_verbose(pb, offset, val, buf, written, len,
			"\tNUM_FRAME %u RST %u CLEANUP %u\n",
			(val & MPI_STRM_CTRL_NUM_FRAME_MASK) >>
			MPI_STRM_CTRL_NUM_FRAME_SHIFT,
			!!(val & MPI_STRM_CTRL_RST_MASK),
			!!(val & MPI_STRM_CTRL_CLEANUP_MASK));
}
#else
static int dump_mipi_input_strm_ctrl(struct paintbox_data *pb, uint32_t offset,
		uint64_t val, char *buf, int *written, size_t len)
{
	return dump_io_ipu_reg_verbose(pb, offset, val, buf, written, len,
			"\tNUM_FRAME %u OVF IMR %u OVF ISR %u SOF IMR %u SOF ISR %u RST %u CLEANUP %u EN %u\n",
			(val & MPI_STRM_CTRL_NUM_FRAME_MASK) >>
			MPI_STRM_CTRL_NUM_FRAME_SHIFT,
			!!(val & MPI_STRM_CTRL_OVF_IMR_MASK),
			!!(val & MPI_STRM_CTRL_OVF_ISR_MASK),
			!!(val & MPI_STRM_CTRL_SOF_IMR_MASK),
			!!(val & MPI_STRM_CTRL_SOF_ISR_MASK),
			!!(val & MPI_STRM_CTRL_RST_MASK),
			!!(val & MPI_STRM_CTRL_CLEANUP_MASK),
			!!(val & MPI_STRM_CTRL_EN_MASK));
}
#endif

static int dump_mipi_input_strm_cnfg0(struct paintbox_data *pb, uint32_t offset,
		uint64_t val, char *buf, int *written, size_t len)
{
	return dump_io_ipu_reg_verbose(pb, offset, val, buf, written, len,
			"\tIMG_HEIGHT %u IMG_WIDTH %u STRP_HEIGHT %u DT_PROC %u DT_IN %u VC %u\n",
			(val & MPI_STRM_CNFG0_IMG_HEIGHT_MASK) >>
			MPI_STRM_CNFG0_IMG_HEIGHT_SHIFT,
			(val & MPI_STRM_CNFG0_IMG_WIDTH_MASK) >>
			MPI_STRM_CNFG0_IMG_WIDTH_SHIFT,
			((val & MPI_STRM_CNFG0_STRP_HEIGHT_MASK) >>
			MPI_STRM_CNFG0_STRP_HEIGHT_SHIFT) + 1,
			(val & MPI_STRM_CNFG0_DT_PROC_MASK) >>
			MPI_STRM_CNFG0_DT_PROC_SHIFT,
			(val & MPI_STRM_CNFG0_DT_IN_MASK) >>
			MPI_STRM_CNFG0_DT_IN_SHIFT,
			val & MPI_STRM_CNFG0_VC_MASK);
}

static int dump_mipi_input_strm_cnfg1(struct paintbox_data *pb, uint32_t offset,
		uint64_t val, char *buf, int *written, size_t len)
{
	return dump_io_ipu_reg_verbose(pb, offset, val, buf, written, len,
			"\tSEG_WORDS_PER_ROW %u SEGS_PER_ROW %u SEG_END %u SEG_START %u\n",
			((val & MPI_STRM_CNFG1_SEG_WORDS_PER_ROW_MASK) >>
			MPI_STRM_CNFG1_SEG_WORDS_PER_ROW_SHIFT) + 1,
			((val & MPI_STRM_CNFG1_SEGS_PER_ROW_MASK) >>
			MPI_STRM_CNFG1_SEGS_PER_ROW_SHIFT) + 1,
			((val & MPI_STRM_CNFG1_SEG_END_MASK) >>
			MPI_STRM_CNFG1_SEG_END_SHIFT) + 1,
			val & MPI_STRM_CNFG1_SEG_START_MASK);
}

#if CONFIG_PAINTBOX_VERSION_MAJOR >= 1
static int dump_mipi_output_strm_ctrl(struct paintbox_data *pb, uint32_t offset,
		uint64_t val, char *buf, int *written, size_t len)
{
	return dump_io_ipu_reg_verbose(pb, offset, val, buf, written, len,
			"\tRST %u CLEANUP %u RSYNC_EN %u\n",
			!!(val & MPO_STRM_CTRL_RST_MASK),
			!!(val & MPO_STRM_CTRL_CLEANUP_MASK),
			!!(val & MPO_STRM_CTRL_RSYNC_EN_MASK));
}
#else
static int dump_mipi_output_strm_ctrl(struct paintbox_data *pb, uint32_t offset,
		uint64_t val, char *buf, int *written, size_t len)
{
	return dump_io_ipu_reg_verbose(pb, offset, val, buf, written, len,
			"\tEOF IMR %u EOF ISR %u RST %u CLEANUP %u RSYNC_EN %u EN %u\n",
			!!(val & MPO_STRM_CTRL_EOF_IMR_MASK),
			!!(val & MPO_STRM_CTRL_EOF_ISR_MASK),
			!!(val & MPO_STRM_CTRL_RST_MASK),
			!!(val & MPO_STRM_CTRL_CLEANUP_MASK),
			!!(val & MPO_STRM_CTRL_RSYNC_EN_MASK),
			!!(val & MPO_STRM_CTRL_EN_MASK));
}
#endif

static int dump_mipi_output_strm_cnfg0(struct paintbox_data *pb,
		uint32_t offset, uint64_t val, char *buf, int *written,
		size_t len)
{
	return dump_io_ipu_reg_verbose(pb, offset, val, buf, written, len,
			"\tIMG_HEIGHT %u IMG_WIDTH %u STRP_HEIGHT %u DT_PROC %u DT_OUT %u VC %u\n",
			(val & MPO_STRM_CNFG0_IMG_HEIGHT_MASK) >>
			MPO_STRM_CNFG0_IMG_HEIGHT_SHIFT,
			(val & MPO_STRM_CNFG0_IMG_WIDTH_MASK) >>
			MPO_STRM_CNFG0_IMG_WIDTH_SHIFT,
			((val & MPO_STRM_CNFG0_STRP_HEIGHT_MASK) >>
			MPO_STRM_CNFG0_STRP_HEIGHT_SHIFT) + 1,
			(val & MPO_STRM_CNFG0_DT_PROC_MASK) >>
			MPO_STRM_CNFG0_DT_PROC_SHIFT,
			(val & MPO_STRM_CNFG0_DT_OUT_MASK) >>
			MPO_STRM_CNFG0_DT_OUT_SHIFT,
			val & MPO_STRM_CNFG0_VC_MASK);
}

#if CONFIG_PAINTBOX_VERSION_MAJOR >= 1
static int dump_mipi_output_strm_cnfg1(struct paintbox_data *pb,
		uint32_t offset, uint64_t val, char *buf, int *written,
		size_t len)
{
	return dump_io_ipu_reg_verbose(pb, offset, val, buf, written, len,
			"\tSEGS_PER_ROW %u SEG_START %u\n",
			((val & MPO_STRM_CNFG1_SEGS_PER_ROW_MASK) >>
			MPO_STRM_CNFG1_SEGS_PER_ROW_SHIFT) + 1,
			(val & MPO_STRM_CNFG1_SEG_START_MASK) + 1);
}
#else
static int dump_mipi_output_strm_cnfg1(struct paintbox_data *pb,
		uint32_t offset, uint64_t val, char *buf, int *written,
		size_t len)
{
	return dump_io_ipu_reg_verbose(pb, offset, val, buf, written, len,
			"\tSEGS_PER_ROW %u SEG_END %u\n",
			((val & MPO_STRM_CNFG1_SEGS_PER_ROW_MASK) >>
			MPO_STRM_CNFG1_SEGS_PER_ROW_SHIFT) + 1,
			(val & MPO_STRM_CNFG1_SEG_END_MASK) + 1);
}
#endif

/* The caller to this function must hold pb->lock */
void paintbox_log_mipi_registers(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream, const char *msg)
{
	dev_info(&pb->pdev->dev, "%s\n", msg);

	if (stream->is_input) {
		dump_mipi_input_strm_ctrl(pb, MPI_STRM_CTRL,
				readq(pb->io_ipu.ipu_base + MPI_STRM_CTRL),
				NULL, NULL, 0);

		dump_mipi_input_strm_cnfg0(pb, MPI_STRM_CNFG0,
				readq(pb->io_ipu.ipu_base + MPI_STRM_CNFG0),
				NULL, NULL, 0);

		dump_mipi_input_strm_cnfg1(pb, MPI_STRM_CNFG1,
				readq(pb->io_ipu.ipu_base + MPI_STRM_CNFG1),
				NULL, NULL, 0);

		dump_mipi_input_strm_cnfg0(pb, MPI_STRM_CNFG0_RO,
				readq(pb->io_ipu.ipu_base + MPI_STRM_CNFG0_RO),
				NULL, NULL, 0);

		dump_mipi_input_strm_cnfg1(pb, MPI_STRM_CNFG1_RO,
				readq(pb->io_ipu.ipu_base + MPI_STRM_CNFG1_RO),
				NULL, NULL, 0);
	} else {
		dump_mipi_output_strm_ctrl(pb, MPO_STRM_CTRL,
				readq(pb->io_ipu.ipu_base + MPO_STRM_CTRL),
				NULL, NULL, 0);

		dump_mipi_output_strm_cnfg0(pb, MPO_STRM_CNFG0,
				readq(pb->io_ipu.ipu_base + MPO_STRM_CNFG0),
				NULL, NULL, 0);

		dump_mipi_output_strm_cnfg1(pb, MPO_STRM_CNFG1,
				readq(pb->io_ipu.ipu_base + MPO_STRM_CNFG1),
				NULL, NULL, 0);

		dump_mipi_output_strm_cnfg0(pb, MPO_STRM_CNFG0_RO,
				readq(pb->io_ipu.ipu_base + MPO_STRM_CNFG0_RO),
				NULL, NULL, 0);

		dump_mipi_output_strm_cnfg1(pb, MPO_STRM_CNFG1_RO,
				readq(pb->io_ipu.ipu_base + MPO_STRM_CNFG1_RO),
				NULL, NULL, 0);
	}
}

#ifdef CONFIG_PAINTBOX_DEBUG
int paintbox_dump_mipi_common_registers(struct paintbox_debug *debug, char *buf,
		size_t len)
{
	uint64_t mipi_registers[MPI_COMMON_NUM_REGS];
	struct paintbox_data *pb = debug->pb;
	unsigned long irq_flags;
	unsigned int reg_offset;
	int ret, written = 0;
	uint64_t val;

	spin_lock_irqsave(&pb->io_ipu.mipi_lock, irq_flags);

	for (reg_offset = MPI_CAP; reg_offset < MPI_COMMON_BLOCK_END;
			reg_offset += IPU_REG_WIDTH) {
		if (!io_ipu_reg_names[REG_INDEX(reg_offset)])
			continue;

		mipi_registers[REG_INDEX(reg_offset)] =
				readq(pb->io_ipu.ipu_base + reg_offset);
	}

	spin_unlock_irqrestore(&pb->io_ipu.mipi_lock, irq_flags);

	val = mipi_registers[REG_INDEX(MPI_CAP)];
	ret = dump_io_ipu_reg_verbose(pb, MPI_CAP, val, buf, &written,
			len, "\tMAX_STRM %u MAX_IFC %u\n",
			(val & MPI_CAP_MAX_STRM_MASK) >> MPI_CAP_MAX_STRM_SHIFT,
			val & MPI_CAP_MAX_IFC_MASK);
	if (ret < 0)
		return ret;

#if CONFIG_PAINTBOX_VERSION_MAJOR >= 1
	val = mipi_registers[REG_INDEX(MPI_IER)];
	ret = dump_io_ipu_reg(pb, MPI_IER, val, buf, &written, len);
	if (ret < 0)
		return ret;

	val = mipi_registers[REG_INDEX(MPI_IMR)];
	ret = dump_io_ipu_reg(pb, MPI_IMR, val, buf, &written, len);
	if (ret < 0)
		return ret;

	val = mipi_registers[REG_INDEX(MPI_ISR)];
	ret = dump_io_ipu_reg(pb, MPI_ISR, val, buf, &written, len);
	if (ret < 0)
		return ret;

	val = mipi_registers[REG_INDEX(MPI_ISR_OVF)];
	ret = dump_io_ipu_reg(pb, MPI_ISR_OVF, val, buf, &written, len);
	if (ret < 0)
		return ret;

	val = mipi_registers[REG_INDEX(MPI_ITR)];
	ret = dump_io_ipu_reg(pb, MPI_ITR, val, buf, &written, len);
	if (ret < 0)
		return ret;

	val = mipi_registers[REG_INDEX(MPI_ERR_IER)];
	ret = dump_io_ipu_reg(pb, MPI_ERR_IER, val, buf, &written, len);
	if (ret < 0)
		return ret;

	val = mipi_registers[REG_INDEX(MPI_ERR_IMR)];
	ret = dump_io_ipu_reg(pb, MPI_ERR_IMR, val, buf, &written, len);
	if (ret < 0)
		return ret;

	val = mipi_registers[REG_INDEX(MPI_ERR_ISR)];
	ret = dump_io_ipu_reg(pb, MPI_ERR_ISR, val, buf, &written, len);
	if (ret < 0)
		return ret;

	val = mipi_registers[REG_INDEX(MPI_ERR_ISR_OVF)];
	ret = dump_io_ipu_reg(pb, MPI_ERR_ISR_OVF, val, buf, &written, len);
	if (ret < 0)
		return ret;

	val = mipi_registers[REG_INDEX(MPI_ERR_ITR)];
	ret = dump_io_ipu_reg(pb, MPI_ERR_ITR, val, buf, &written, len);
	if (ret < 0)
		return ret;

	val = mipi_registers[REG_INDEX(MPI_CTRL)];
	ret = dump_io_ipu_reg_verbose(pb, MPI_CTRL, val, buf, &written,
			len, "\tCONTINUOUS %03x\n",
			(val & MPI_CTRL_STRM_CONTINUOUS_MASK) >>
					MPI_CTRL_STRM_CONTINUOUS_SHIFT);
	if (ret < 0)
		return ret;

	val = mipi_registers[REG_INDEX(MPI_STATUS)];
	ret = dump_io_ipu_reg_verbose(pb, MPI_STATUS, val, buf, &written,
			len, "\tIDLE %03x\n",
			(val & MPI_STATUS_STRM_IDLE_MASK) >>
					MPI_STATUS_STRM_IDLE_SHIFT);
	if (ret < 0)
		return ret;
#endif

	val = mipi_registers[REG_INDEX(MPI_STRM_SEL)];
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
				readq(pb->io_ipu.ipu_base + reg_offset);
	}

	spin_unlock_irqrestore(&pb->io_ipu.mipi_lock, irq_flags);

	val = mipi_registers[REG_INDEX(MPO_CAP - MPO_COMMON_BLOCK_START)];
	ret = dump_io_ipu_reg_verbose(pb, MPO_CAP, val, buf, &written,
			len, "\tMAX_STRM %u MAX_IFC %u\n",
			(val & MPO_CAP_MAX_STRM_MASK) >> MPO_CAP_MAX_STRM_SHIFT,
			val & MPO_CAP_MAX_IFC_MASK);
	if (ret < 0)
		return ret;

#if CONFIG_PAINTBOX_VERSION_MAJOR >= 1
	val = mipi_registers[REG_INDEX(MPO_IER - MPO_COMMON_BLOCK_START)];
	ret = dump_io_ipu_reg(pb, MPO_IER, val, buf, &written, len);
	if (ret < 0)
		return ret;

	val = mipi_registers[REG_INDEX(MPO_IMR - MPO_COMMON_BLOCK_START)];
	ret = dump_io_ipu_reg(pb, MPO_IMR, val, buf, &written, len);
	if (ret < 0)
		return ret;

	val = mipi_registers[REG_INDEX(MPO_ISR - MPO_COMMON_BLOCK_START)];
	ret = dump_io_ipu_reg(pb, MPO_ISR, val, buf, &written, len);
	if (ret < 0)
		return ret;

	val = mipi_registers[REG_INDEX(MPO_ISR_OVF - MPO_COMMON_BLOCK_START)];
	ret = dump_io_ipu_reg(pb, MPO_ISR_OVF, val, buf, &written, len);
	if (ret < 0)
		return ret;

	val = mipi_registers[REG_INDEX(MPO_ITR - MPO_COMMON_BLOCK_START)];
	ret = dump_io_ipu_reg(pb, MPO_ITR, val, buf, &written, len);
	if (ret < 0)
		return ret;

	val = mipi_registers[REG_INDEX(MPO_CTRL - MPO_COMMON_BLOCK_START)];
	ret = dump_io_ipu_reg_verbose(pb, MPO_CTRL, val, buf, &written,
			len, "\tCONTINUOUS %03x\n",
			(val & MPO_CTRL_STRM_CONTINUOUS_MASK) >>
					MPO_CTRL_STRM_CONTINUOUS_SHIFT);
	if (ret < 0)
		return ret;

	val = mipi_registers[REG_INDEX(MPO_STATUS)];
	ret = dump_io_ipu_reg_verbose(pb, MPO_STATUS, val, buf, &written,
			len, "\tIDLE %03x\n",
			(val & MPO_STATUS_STRM_IDLE_MASK) >>
					MPO_STATUS_STRM_IDLE_SHIFT);
	if (ret < 0)
		return ret;
#endif

	val = mipi_registers[REG_INDEX(MPO_STRM_SEL - MPO_COMMON_BLOCK_START)];
	ret = dump_io_ipu_reg(pb, MPO_STRM_SEL, val, buf, &written, len);
	if (ret < 0)
		return ret;

	return written;
}

static inline uint64_t get_mipi_input_stream_reg(uint64_t *reg_values,
		uint32_t reg_offset)
{
	return reg_values[REG_INDEX(reg_offset - MPI_STRM_BLOCK_START)];
}

int paintbox_dump_mipi_input_stream_registers(struct paintbox_debug *debug,
		char *buf, size_t len)
{
	uint64_t mipi_registers[MPI_STRM_NUM_REGS];
	struct paintbox_mipi_stream *stream = container_of(debug,
			struct paintbox_mipi_stream, debug);
	struct paintbox_data *pb = debug->pb;
	unsigned int reg_offset;
	unsigned long irq_flags;
	int ret, written = 0;
	uint64_t val;

	spin_lock_irqsave(&pb->io_ipu.mipi_lock, irq_flags);

	paintbox_mipi_select_input_stream(pb, stream->stream_id);

	for (reg_offset = MPI_STRM_BLOCK_START; reg_offset < MPI_STRM_BLOCK_END;
			reg_offset += IPU_REG_WIDTH) {
		if (!io_ipu_reg_names[REG_INDEX(reg_offset)])
			continue;

		mipi_registers[REG_INDEX(reg_offset - MPI_STRM_BLOCK_START)] =
				readq(pb->io_ipu.ipu_base + reg_offset);
	}

	spin_unlock_irqrestore(&pb->io_ipu.mipi_lock, irq_flags);

	val = get_mipi_input_stream_reg(mipi_registers, MPI_STRM_CTRL);
	ret = dump_mipi_input_strm_ctrl(pb, MPI_STRM_CTRL, val, buf, &written,
			len);
	if (ret < 0)
		return ret;

	val = get_mipi_input_stream_reg(mipi_registers, MPI_STRM_CNFG0);
	ret = dump_mipi_input_strm_cnfg0(pb, MPI_STRM_CNFG0, val, buf, &written,
			len);
	if (ret < 0)
		return ret;

	val = get_mipi_input_stream_reg(mipi_registers, MPI_STRM_CNFG1);
	ret = dump_mipi_input_strm_cnfg1(pb, MPI_STRM_CNFG1, val, buf, &written,
			len);
	if (ret < 0)
		return ret;

	val = get_mipi_input_stream_reg(mipi_registers, MPI_STRM_CNFG0_RO);
	ret = dump_mipi_input_strm_cnfg0(pb, MPI_STRM_CNFG0_RO, val, buf,
			&written, len);
	if (ret < 0)
		return ret;

	val = get_mipi_input_stream_reg(mipi_registers, MPI_STRM_CNFG1_RO);
	ret = dump_mipi_input_strm_cnfg1(pb, MPI_STRM_CNFG1_RO, val, buf,
			&written, len);
	if (ret < 0)
		return ret;

	return written;
}

static inline uint64_t get_mipi_output_stream_reg(uint64_t *reg_values,
		uint32_t reg_offset)
{
	return reg_values[REG_INDEX(reg_offset - MPO_STRM_BLOCK_START)];
}

int paintbox_dump_mipi_output_stream_registers(struct paintbox_debug *debug,
		char *buf, size_t len)
{
	uint64_t mipi_registers[MPO_STRM_NUM_REGS];
	struct paintbox_mipi_stream *stream = container_of(debug,
			struct paintbox_mipi_stream, debug);
	struct paintbox_data *pb = debug->pb;
	unsigned int reg_offset;
	unsigned long irq_flags;
	int ret, written = 0;
	uint64_t val;

	spin_lock_irqsave(&pb->io_ipu.mipi_lock, irq_flags);

	paintbox_mipi_select_output_stream(pb, stream->stream_id);

	for (reg_offset = MPO_STRM_BLOCK_START; reg_offset < MPO_STRM_BLOCK_END;
			reg_offset += IPU_REG_WIDTH) {

		if (!io_ipu_reg_names[REG_INDEX(reg_offset)])
			continue;

		mipi_registers[REG_INDEX(reg_offset - MPO_STRM_BLOCK_START)] =
				readq(pb->io_ipu.ipu_base + reg_offset);
	}

	spin_unlock_irqrestore(&pb->io_ipu.mipi_lock, irq_flags);

	val = get_mipi_output_stream_reg(mipi_registers, MPO_STRM_CTRL);
	ret = dump_mipi_output_strm_ctrl(pb, MPO_STRM_CTRL, val, buf, &written,
			len);
	if (ret < 0)
		return ret;

	val = get_mipi_output_stream_reg(mipi_registers, MPO_STRM_CNFG0);
	ret = dump_mipi_output_strm_cnfg0(pb, MPO_STRM_CNFG0, val, buf,
			&written, len);
	if (ret < 0)
		return ret;

	val = get_mipi_output_stream_reg(mipi_registers, MPO_STRM_CNFG1);
	ret = dump_mipi_output_strm_cnfg1(pb, MPO_STRM_CNFG1, val, buf,
			&written, len);
	if (ret < 0)
		return ret;

	val = get_mipi_output_stream_reg(mipi_registers, MPO_STRM_CNFG0_RO);
	ret = dump_mipi_output_strm_cnfg0(pb, MPO_STRM_CNFG0_RO, val, buf,
			&written, len);
	if (ret < 0)
		return ret;

	val = get_mipi_output_stream_reg(mipi_registers, MPO_STRM_CNFG1_RO);
	ret = dump_mipi_output_strm_cnfg1(pb, MPO_STRM_CNFG1_RO, val, buf,
			&written, len);
	if (ret < 0)
		return ret;

	return written;
}

static int paintbox_dump_mipi_input_stream_stats(struct paintbox_debug *debug,
		char *buf, size_t len)
{
	struct paintbox_mipi_stream *stream = container_of(debug,
			struct paintbox_mipi_stream, debug);
	struct paintbox_mipi_interface *interface = stream->interface;
	int ret, written;

	ret = snprintf(buf, len,
			"interrupts INF %u SOF %u OVF %u Missed SOF %u Missed OVF %u\n",
			interface->inf_interrupts,
			stream->input.stats.sof_interrupts,
			stream->input.stats.ovf_interrupts,
			stream->input.stats.missed_sof_interrupts,
			stream->input.stats.missed_ovf_interrupts);
	if (ret < 0)
		return ret;

	written = ret;

	ret = snprintf(buf + written, len - written,
			"\tenabled %d free running %d frames remaining %d last frame %d\n",
			stream->enabled, stream->free_running,
			stream->frame_count, stream->last_frame);
	if (ret < 0)
		return ret;

	return written + ret;
}

static int paintbox_dump_mipi_output_stream_stats(struct paintbox_debug *debug,
		char *buf, size_t len)
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
			"\tenabled %d free running %d frames remaining %d last frame %d\n",
			stream->enabled, stream->free_running,
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
			paintbox_dump_mipi_input_stream_registers,
			paintbox_dump_mipi_input_stream_stats, stream);

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
			paintbox_dump_mipi_output_stream_registers,
			paintbox_dump_mipi_output_stream_stats, stream);

	paintbox_debug_create_reg_entries(pb, &stream->debug,
			&io_ipu_reg_names[REG_INDEX(MPO_COMMON_BLOCK_START)],
			MPO_COMMON_NUM_REGS + MPO_STRM_NUM_REGS,
			mipi_reg_entry_write, mipi_reg_entry_read);
}

void paintbox_mipi_debug_init(struct paintbox_data *pb)
{
	paintbox_debug_create_entry(pb, &pb->io_ipu.debug, pb->debug_root,
			"mipi", -1, paintbox_dump_mipi_common_registers, NULL,
			&pb->io_ipu);
}
#endif
