/*
 * DMA debug support for the Paintbox programmable IPU
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
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/paintbox.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/types.h>

#include "paintbox-debug.h"
#include "paintbox-dma.h"
#include "paintbox-dma-debug.h"
#include "paintbox-regs.h"


static const char *dma_reg_names[DMA_NUM_REGS] = {
	REG_NAME_ENTRY(DMA_CTRL),
	REG_NAME_ENTRY(DMA_CHAN_CTRL_L),
	REG_NAME_ENTRY(DMA_CHAN_CTRL_H),
	REG_NAME_ENTRY(DMA_CAP0),
	REG_NAME_ENTRY(DMA_PMON_CFG),
	REG_NAME_ENTRY(DMA_PMON_CNT_0_CFG),
	REG_NAME_ENTRY(DMA_PMON_CNT_0),
	REG_NAME_ENTRY(DMA_PMON_CNT_0_STS_ACC),
	REG_NAME_ENTRY(DMA_PMON_CNT_0_STS),
	REG_NAME_ENTRY(DMA_PMON_CNT_1_CFG),
	REG_NAME_ENTRY(DMA_PMON_CNT_1),
	REG_NAME_ENTRY(DMA_PMON_CNT_1_STS_ACC),
	REG_NAME_ENTRY(DMA_PMON_CNT_1_STS),
	REG_NAME_ENTRY(DMA_PMON_CNT_2_CFG),
	REG_NAME_ENTRY(DMA_PMON_CNT_2),
	REG_NAME_ENTRY(DMA_PMON_CNT_2_STS_ACC),
	REG_NAME_ENTRY(DMA_PMON_CNT_2_STS),
	REG_NAME_ENTRY(DMA_PMON_CNT_3_CFG),
	REG_NAME_ENTRY(DMA_PMON_CNT_3),
	REG_NAME_ENTRY(DMA_PMON_CNT_3_STS_ACC),
	REG_NAME_ENTRY(DMA_PMON_CNT_3_STS),
	REG_NAME_ENTRY(DMA_CHAN_MODE),
	REG_NAME_ENTRY(DMA_CHAN_IMG_FORMAT),
	REG_NAME_ENTRY(DMA_CHAN_IMG_SIZE),
	REG_NAME_ENTRY(DMA_CHAN_IMG_POS_L),
	REG_NAME_ENTRY(DMA_CHAN_IMG_POS_H),
	REG_NAME_ENTRY(DMA_CHAN_IMG_LAYOUT_L),
	REG_NAME_ENTRY(DMA_CHAN_IMG_LAYOUT_H),
	REG_NAME_ENTRY(DMA_CHAN_BIF_XFER),
	REG_NAME_ENTRY(DMA_CHAN_VA_L),
	REG_NAME_ENTRY(DMA_CHAN_VA_H),
	REG_NAME_ENTRY(DMA_CHAN_VA_BDRY_L),
	REG_NAME_ENTRY(DMA_CHAN_VA_BDRY_H),
	REG_NAME_ENTRY(DMA_CHAN_NOC_XFER_L),
	REG_NAME_ENTRY(DMA_CHAN_NOC_XFER_H),
	REG_NAME_ENTRY(DMA_CHAN_NODE),
	REG_NAME_ENTRY(DMA_CHAN_IMR),
	REG_NAME_ENTRY(DMA_CHAN_ISR),
	REG_NAME_ENTRY(DMA_CHAN_ISR_OVF),
	REG_NAME_ENTRY(DMA_CHAN_MODE_RO),
	REG_NAME_ENTRY(DMA_CHAN_IMG_FORMAT_RO),
	REG_NAME_ENTRY(DMA_CHAN_IMG_SIZE_L_RO),
	REG_NAME_ENTRY(DMA_CHAN_IMG_SIZE_H_RO),
	REG_NAME_ENTRY(DMA_CHAN_IMG_POS_L_RO),
	REG_NAME_ENTRY(DMA_CHAN_IMG_POS_H_RO),
	REG_NAME_ENTRY(DMA_CHAN_IMG_LAYOUT_L_RO),
	REG_NAME_ENTRY(DMA_CHAN_IMG_LAYOUT_H_RO),
	REG_NAME_ENTRY(DMA_CHAN_BIF_XFER_RO),
	REG_NAME_ENTRY(DMA_CHAN_VA_L_RO),
	REG_NAME_ENTRY(DMA_CHAN_VA_H_RO),
	REG_NAME_ENTRY(DMA_CHAN_VA_BDRY_L_RO),
	REG_NAME_ENTRY(DMA_CHAN_VA_BDRY_H_RO),
	REG_NAME_ENTRY(DMA_CHAN_NOC_XFER_L_RO),
	REG_NAME_ENTRY(DMA_CHAN_NOC_XFER_H_RO),
	REG_NAME_ENTRY(DMA_CHAN_NODE_RO),
	REG_NAME_ENTRY(DMA_CHAN_DEPENDENCY),
	REG_NAME_ENTRY(DMA_STAT_CTRL),
	REG_NAME_ENTRY(DMA_STAT_STATE),
	REG_NAME_ENTRY(DMA_STAT_PTR_L),
	REG_NAME_ENTRY(DMA_STAT_PTR_H),
	REG_NAME_ENTRY(DMS_STAT_ADDR_L),
	REG_NAME_ENTRY(DMA_STAT_ADDR_H),
	REG_NAME_ENTRY(DMA_SPARE)
};

static uint32_t dma_reg_entry_read(struct paintbox_debug_reg_entry *reg_entry)
{
	struct paintbox_debug *debug = reg_entry->debug;
	struct paintbox_dma *dma = container_of(debug, struct paintbox_dma,
			debug);
	return readl(dma->dma_base + reg_entry->reg_offset);
}

static void dma_reg_entry_write(struct paintbox_debug_reg_entry *reg_entry,
		uint32_t val)
{
	struct paintbox_debug *debug = reg_entry->debug;
	struct paintbox_dma *dma = container_of(debug, struct paintbox_dma,
			debug);
	writel(val, dma->dma_base + reg_entry->reg_offset);
}

static uint32_t dma_channel_reg_entry_read(
		struct paintbox_debug_reg_entry *reg_entry)
{
	struct paintbox_debug *debug = reg_entry->debug;
	struct paintbox_dma_channel *channel = container_of(debug,
			struct paintbox_dma_channel, debug);
	struct paintbox_data *pb = debug->pb;
	uint32_t val;

	mutex_lock(&pb->lock);

	dma_select_channel(pb, channel->channel_id);
	val = readl(pb->dma.dma_base + DMA_CHAN_BLOCK_START +
		reg_entry->reg_offset);

	mutex_unlock(&pb->lock);

	return val;
}

static void dma_channel_reg_entry_write(
		struct paintbox_debug_reg_entry *reg_entry, uint32_t val)
{
	struct paintbox_debug *debug = reg_entry->debug;
	struct paintbox_dma_channel *channel = container_of(debug,
			struct paintbox_dma_channel, debug);
	struct paintbox_data *pb = debug->pb;

	mutex_lock(&pb->lock);

	dma_select_channel(pb, channel->channel_id);
	writel(val, pb->dma.dma_base + DMA_CHAN_BLOCK_START +
		reg_entry->reg_offset);

	mutex_unlock(&pb->lock);
}

static inline int dump_dma_reg(struct paintbox_data *pb, uint32_t reg_offset,
		char *buf, int *written, size_t len)
{
	const char *reg_name = dma_reg_names[REG_INDEX(reg_offset)];
	return dump_ipu_register(pb, pb->dma.dma_base, reg_offset, reg_name,
			buf, written, len);
}

static int dump_dma_reg_verbose(struct paintbox_data *pb, uint32_t reg_offset,
		char *buf, int *written, size_t len, const char *format, ...)
{
	va_list args;
	int ret;

	ret = dump_dma_reg(pb, reg_offset, buf, written, len);
	if (ret < 0)
		return ret;

	va_start(args, format);

	ret = dump_ipu_vprintf(pb, buf, written, len, format, args);

	va_end(args);

	return ret;
}

static inline const char *dma_swizzle_to_str(uint32_t val)
{
	switch ((val & DMA_AXI_SWIZZLE_MASK) >>
			DMA_AXI_SWIZZLE_SHIFT) {
	case DMA_AXI_SWIZZLE_NONE:
		return "NONE";
	case DMA_AXI_SWIZZLE_BIG_ENDIAN:
		return "BIG ENDIAN";
	case DMA_AXI_SWIZZLE_NEIGHBOR_BYTES:
		return "NEIGHBOR BYTES";
	default:
		return "UNKNOWN";
	};
}

int dump_dma_registers(struct paintbox_debug *debug, char *buf, size_t len)
{
	struct paintbox_data *pb = debug->pb;
	uint32_t val, axi_swizzle;
	unsigned int i;
	int ret, written = 0;

	val = readl(pb->dma.dma_base + DMA_CTRL);
	axi_swizzle = (val & DMA_AXI_SWIZZLE_MASK) >> DMA_AXI_SWIZZLE_SHIFT;
	ret = dump_dma_reg_verbose(pb, DMA_CTRL, buf, &written, len,
			"\tAXI_SWIZZLE %s DMA_CHAN_SEL 0x%02x RESET %d\n",
			dma_swizzle_to_str(axi_swizzle),
			(val & DMA_CHAN_SEL_MASK) >> DMA_CHAN_SEL_SHIFT,
			val & DMA_RESET);
	if (ret < 0)
		goto err_exit;

	val = readl(pb->dma.dma_base + DMA_CHAN_CTRL_L);
	ret = dump_dma_reg_verbose(pb, DMA_CHAN_CTRL_L, buf, &written, len,
			"\tDOUBLE_BUF 0x%04x CHAN_RESET 0x%04x\n",
			(val & DMA_CHAN_DOUBLE_BUF_MASK) >>
					DMA_CHAN_DOUBLE_BUF_SHIFT,
			val & DMA_CHAN_RESET_MASK);
	if (ret < 0)
		goto err_exit;

	val = readl(pb->dma.dma_base + DMA_CHAN_CTRL_H);
	ret = dump_dma_reg_verbose(pb, DMA_CHAN_CTRL_H, buf, &written, len,
			"\tSTOP 0x%04x\n", val & DMA_STOP_MASK);
	if (ret < 0)
		goto err_exit;

	val = readl(pb->dma.dma_base + DMA_CAP0);
	ret = dump_dma_reg_verbose(pb, DMA_CAP0, buf, &written, len,
			"\tMAX_DMA_CHAN %u\n", val & MAX_DMA_CHAN_MASK);
	if (ret < 0)
		goto err_exit;

	for (i = REG_INDEX(DMA_PMON_CFG); i <= REG_INDEX(DMA_PMON_CNT_3_STS);
			i++) {
		if (dma_reg_names[i] != NULL) {
			ret = dump_dma_reg(pb, i * IPU_REG_WIDTH, buf,
					&written, len);
			if (ret < 0)
				goto err_exit;
		}
	}

	for (i = REG_INDEX(DMA_STAT_BLOCK_START);
			i < REG_INDEX(DMA_STAT_BLOCK_END); i++) {
		if (dma_reg_names[i] != NULL) {
			ret = dump_dma_reg(pb, i * IPU_REG_WIDTH, buf,
					&written, len);
			if (ret < 0)
				goto err_exit;
		}
	}

	return written;

err_exit:
	dev_err(&pb->pdev->dev, "%s: register dump error, err = %d", __func__,
			ret);
	return ret;
}

static inline unsigned int get_bit_depth(uint32_t val)
{
	switch ((val & DMA_CHAN_BIT_DEPTH_MASK) >> DMA_CHAN_BIT_DEPTH_SHIFT) {
	case DMA_CHAN_BIT_DEPTH8:
		return 8;
	case DMA_CHAN_BIT_DEPTH10:
		return 10;
	case DMA_CHAN_BIT_DEPTH12:
		return 12;
	case DMA_CHAN_BIT_DEPTH14:
		return 14;
	case DMA_CHAN_BIT_DEPTH16:
		return 16;
	default:
		return 0;
	};
}

static inline const char *dma_src_to_str(uint32_t val)
{
	switch ((val & DMA_CHAN_SRC_MASK) >> DMA_CHAN_SRC_SHIFT) {
	case DMA_CHAN_SRC_DRAM:
		return "DRAM";
	case DMA_CHAN_SRC_LBP:
		return "LBP";
	case DMA_CHAN_SRC_STP:
		return "STP";
	case DMA_CHAN_SRC_MIPI_IN:
		return "MIPI IN";
	default:
		return "UNKNOWN";
	};
}

static inline const char *dma_dst_to_str(uint32_t val)
{
	switch ((val & DMA_CHAN_DST_MASK) >> DMA_CHAN_DST_SHIFT) {
	case DMA_CHAN_DST_DRAM:
		return "DRAM";
	case DMA_CHAN_DST_LBP:
		return "LBP";
	case DMA_CHAN_DST_STP:
		return "STP";
	case DMA_CHAN_DST_MIPI_OUT:
		return "MIPI OUT";
	default:
		return "UNKNOWN";
	};
}

static inline const char *dma_rgba_to_str(uint32_t val)
{
	switch ((val & DMA_CHAN_RGBA_FORMAT_MASK) >>
			DMA_CHAN_RGBA_FORMAT_SHIFT) {
	case DMA_CHAN_RGBA_FORMAT_DISABLED:
		return "DISABLED";
	case DMA_CHAN_RGBA_FORMAT_RGBA:
		return "RGBA";
	case DMA_CHAN_RGBA_FORMAT_ARGB:
		return "ARGB";
	default:
		return "UNKNOWN";
	};
}

int dump_dma_channel_registers(struct paintbox_debug *debug, char *buf,
			size_t len)
{
	struct paintbox_dma_channel *channel = container_of(debug,
			struct paintbox_dma_channel, debug);
	struct paintbox_data *pb = debug->pb;
	uint64_t va, va_bdry, plane_stride;
	uint32_t val, row_stride;
	unsigned int i;
	int ret, written = 0;

	dma_select_channel(pb, channel->channel_id);

	val = readl(pb->dma.dma_base + DMA_CHAN_MODE);
	ret = dump_dma_reg_verbose(pb, DMA_CHAN_MODE, buf, &written, len,
			"\tGATHER %u ADDR_MODE %s DST %s SRC %s ENA %u\n",
			!!(val & DMA_CHAN_GATHER),
			val & DMA_CHAN_ADDR_MODE_PHYSICAL ? "PHYSICAL" :
			"ABSTRACT",
			dma_dst_to_str(val),
			dma_src_to_str(val),
			!!(val & DMA_CHAN_ENA));
	if (ret < 0)
		goto err_exit;

	val = readl(pb->dma.dma_base + DMA_CHAN_IMG_FORMAT);
	ret = dump_dma_reg_verbose(pb, DMA_CHAN_IMG_FORMAT, buf, &written, len,
			"\tBLOCK_4x4 %u RGBA %s MIPI_RAW_FORMAT %d BIT DEPTH %u"
			" PLANES %u COMPONENTS %u\n",
			!!(val & DMA_CHAN_BLOCK_4X4),
			dma_rgba_to_str(val),
			!!(val & DMA_CHAN_MIPI_RAW_FORMAT),
			get_bit_depth(val),
			((val & DMA_CHAN_PLANES_MASK) >>
					DMA_CHAN_PLANES_SHIFT) + 1,
			(val & DMA_CHAN_COMPONENTS_MASK) + 1);
	if (ret < 0)
		goto err_exit;

	val = readl(pb->dma.dma_base + DMA_CHAN_IMG_SIZE);
	ret = dump_dma_reg_verbose(pb, DMA_CHAN_IMG_SIZE, buf, &written, len,
			"\tIMG_HEIGHT %u IMG_WIDTH %u\n",
			(val & DMA_CHAN_IMG_HEIGHT_MASK) >>
					DMA_CHAN_IMG_HEIGHT_SHIFT,
			val & DMA_CHAN_IMG_SIZE_MASK);
	if (ret < 0)
		goto err_exit;

	val = readl(pb->dma.dma_base + DMA_CHAN_IMG_POS_L);
	ret = dump_dma_reg_verbose(pb, DMA_CHAN_IMG_POS_L, buf, &written, len,
			"\tSTART_Y %u START_X %u\n",
			(val & DMA_CHAN_START_Y_MASK) >> DMA_CHAN_START_Y_SHIFT,
			val & DMA_CHAN_START_X_MASK);
	if (ret < 0)
		goto err_exit;

	val = readl(pb->dma.dma_base + DMA_CHAN_IMG_POS_H);
	ret = dump_dma_reg_verbose(pb, DMA_CHAN_IMG_POS_H, buf, &written, len,
			"\tLB_START_Y %d LB_START_X %d\n",
			(int16_t)((val & DMA_CHAN_LB_START_Y_MASK) >>
					DMA_CHAN_LB_START_Y_SHIFT),
			(int16_t)(val & DMA_CHAN_LB_START_X_MASK));
	if (ret < 0)
		goto err_exit;

	ret = dump_dma_reg(pb, DMA_CHAN_IMG_LAYOUT_L, buf, &written, len);
	if (ret < 0)
		goto err_exit;

	val = readl(pb->dma.dma_base + DMA_CHAN_IMG_LAYOUT_L);

	row_stride = val & DMA_CHAN_ROW_STRIDE_MASK;
	plane_stride = (val & DMA_CHAN_PLANE_STRIDE_LOW_MASK) >>
			DMA_CHAN_PLANE_STRIDE_LOW_SHIFT;

	val = readl(pb->dma.dma_base + DMA_CHAN_IMG_LAYOUT_H);

	plane_stride |= (uint64_t)(val & DMA_CHAN_PLANE_STRIDE_HIGH_MASK) <<
			DMA_CHAN_PLANE_STRIDE_LOW_WIDTH;
	ret = dump_dma_reg_verbose(pb, DMA_CHAN_IMG_LAYOUT_H, buf, &written,
			len, "\tROW_STRIDE %u PLANE_STRIDE %llu\n", row_stride,
			plane_stride);
	if (ret < 0)
		goto err_exit;

	val = readl(pb->dma.dma_base + DMA_CHAN_BIF_XFER);
	ret = dump_dma_reg_verbose(pb, DMA_CHAN_BIF_XFER, buf, &written, len,
			"\tOUTSTANDING %u STRIPE_HEIGHT %u\n",
			(val & DMA_CHAN_OUTSTANDING_MASK) >>
					DMA_CHAN_OUTSTANDING_SHIFT,
			val & DMA_CHAN_STRIPE_HEIGHT_MASK);
	if (ret < 0)
		goto err_exit;

	ret = dump_dma_reg(pb, DMA_CHAN_VA_L, buf, &written, len);
	if (ret < 0)
		goto err_exit;

	va = readl(pb->dma.dma_base + DMA_CHAN_VA_L);
	va |= ((uint64_t)readl(pb->dma.dma_base + DMA_CHAN_VA_H)) <<
			32;
	ret = dump_dma_reg_verbose(pb, DMA_CHAN_VA_H, buf, &written, len,
			"\tVA 0x%016llx\n", va);
	if (ret < 0)
		goto err_exit;

	ret = dump_dma_reg(pb, DMA_CHAN_VA_BDRY_L, buf, &written, len);
	if (ret < 0)
		goto err_exit;

	va_bdry = readl(pb->dma.dma_base + DMA_CHAN_VA_BDRY_L);
	va_bdry |= ((uint64_t)readl(pb->dma.dma_base + DMA_CHAN_VA_BDRY_H)) <<
			32;
	ret = dump_dma_reg_verbose(pb, DMA_CHAN_VA_BDRY_H, buf, &written, len,
			"\tVA BDRY %llu\n", va_bdry);
	if (ret < 0)
		goto err_exit;

	val = readl(pb->dma.dma_base + DMA_CHAN_NOC_XFER_L);
	ret = dump_dma_reg_verbose(pb, DMA_CHAN_NOC_XFER_L, buf, &written, len,
			"\tOUTSTANDING %u SHEET_HEIGHT %u SHEET_WIDTH %u\n",
			(val & DMA_CHAN_NOC_OUTSTANDING_MASK) >>
					DMA_CHAN_NOC_OUTSTANDING_SHIFT,
			(val & DMA_CHAN_SHEET_HEIGHT_MASK) >>
					DMA_CHAN_SHEET_HEIGHT_SHIFT,
			val & DMA_CHAN_SHEET_WIDTH_MASK);
	if (ret < 0)
		goto err_exit;

	val = readl(pb->dma.dma_base + DMA_CHAN_NOC_XFER_H);
	ret = dump_dma_reg_verbose(pb, DMA_CHAN_NOC_XFER_H, buf, &written, len,
			"\tRETRY_INTERVAL %u\n",
			val & DMA_CHAN_RETRY_INTERVAL_MASK);
	if (ret < 0)
		goto err_exit;

	val = readl(pb->dma.dma_base + DMA_CHAN_NODE);
	ret = dump_dma_reg_verbose(pb, DMA_CHAN_NODE, buf, &written, len,
			"\tCORE_ID %u LB_ID %u RPTR_ID %u\n",
			val & DMA_CHAN_CORE_ID_MASK,
			(val & DMA_CHAN_LB_ID_MASK) >> DMA_CHAN_LB_ID_SHIFT,
			(val & DMA_CHAN_RPTR_ID_MASK) >>
					DMA_CHAN_RPTR_ID_SHIFT);
	if (ret < 0)
		goto err_exit;

	for (i = REG_INDEX(DMA_CHAN_IMR); i <= REG_INDEX(DMA_CHAN_NODE_RO);
			i++) {
		if (dma_reg_names[i] != NULL) {
			ret = dump_dma_reg(pb, i * IPU_REG_WIDTH, buf,
					&written, len);
			if (ret < 0)
				goto err_exit;
		}
	}

	return written;

err_exit:
	dev_err(&pb->pdev->dev, "%s: register dump error, err = %d", __func__,
			ret);
	return ret;
}

void log_dma_registers(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel, const char *msg)
{
	int ret, written;

	ret = snprintf(pb->vdbg_log, pb->vdbg_log_len, "dma:\n");
	if (ret < 0)
		goto err_exit;

	written = ret;

	ret = dump_dma_registers(&pb->dma.debug, pb->vdbg_log + written,
			pb->vdbg_log_len - written);
	if (ret < 0)
		goto err_exit;

	dev_vdbg(&pb->pdev->dev, "%s\n%s", msg, pb->vdbg_log);

	ret = snprintf(pb->vdbg_log, pb->vdbg_log_len, "dma ch%u:\n",
			channel->channel_id);
	if (ret < 0)
		goto err_exit;

	written = ret;

	ret = dump_dma_channel_registers(
			&pb->dma.channels[channel->channel_id].debug,
			pb->vdbg_log + written, pb->vdbg_log_len - written);
	if (ret < 0)
		goto err_exit;

	dev_vdbg(&pb->pdev->dev, "%s\n%s", msg, pb->vdbg_log);

err_exit:
	dev_err(&pb->pdev->dev, "%s: register log error, err = %d", __func__,
			ret);
}

void paintbox_dma_channel_debug_init(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel)
{
	paintbox_debug_create_entry(pb, &channel->debug,
			pb->dma.debug.debug_dir, "channel", channel->channel_id,
			dump_dma_channel_registers, channel);

	paintbox_debug_create_reg_entries(pb, &channel->debug,
			&dma_reg_names[REG_INDEX(DMA_CHAN_BLOCK_START)],
			DMA_CHAN_NUM_REGS, dma_channel_reg_entry_write,
			dma_channel_reg_entry_read);
}

void paintbox_dma_debug_init(struct paintbox_data *pb)
{
	unsigned int i, reg_index;
	size_t reg_count = DMA_CTRL_NUM_REGS + DMA_STAT_NUM_REGS;

	int ret;

	paintbox_debug_create_entry(pb, &pb->dma.debug, pb->debug_root, "dma",
			-1, dump_dma_registers, &pb->dma);

	ret = paintbox_debug_alloc_reg_entries(pb, &pb->dma.debug, reg_count);

	for (i = 0, reg_index = 0; i < DMA_CTRL_NUM_REGS &&
			reg_index < REG_INDEX(DMA_CTRL_BLOCK_LEN);
			reg_index++, i++) {
		if (!dma_reg_names[i])
			continue;

		ret = paintbox_debug_create_reg_entry(pb, &pb->dma.debug, i,
				dma_reg_names[reg_index],
				reg_index * IPU_REG_WIDTH,
				dma_reg_entry_write, dma_reg_entry_read);
		if (ret < 0) {
			paintbox_debug_free_reg_entries(&pb->dma.debug);
			return;
		}
	}

	for (reg_index = REG_INDEX(DMA_STAT_BLOCK_START); i < reg_count &&
			reg_index < REG_INDEX(DMA_STAT_BLOCK_END); reg_index++,
			i++) {
		if (!dma_reg_names[reg_index])
			continue;

		ret = paintbox_debug_create_reg_entry(pb, &pb->dma.debug,
				i, dma_reg_names[reg_index],
				reg_index * IPU_REG_WIDTH,
				dma_reg_entry_write, dma_reg_entry_read);
		if (ret < 0) {
			paintbox_debug_free_reg_entries(&pb->dma.debug);
			return;
		}
	}

	paintbox_alloc_debug_buffer(pb, DMA_DEBUG_BUFFER_SIZE);
}
