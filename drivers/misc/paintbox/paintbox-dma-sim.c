/*
 * Simulator DMA support for the Paintbox programmable IPU
 *
 * Copyright (C) 2015 Google, Inc.
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
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/paintbox.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include "paintbox-debug.h"
#include "paintbox-dma.h"
#include "paintbox-io.h"
#include "paintbox-lbp.h"
#include "paintbox-regs.h"
#include "paintbox-sim-regs.h"


/* The caller to this function must hold pb->lock */
static inline void dma_select_channel(struct paintbox_data *pb,
		uint32_t channel_id)
{
	writel((channel_id << DMA_CHAN_SEL_SHIFT) & DMA_CHAN_SEL_MASK,
			pb->dma.dma_base + DMA_CTRL);
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

#ifdef CONFIG_DEBUG_FS
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
#endif

#if defined(CONFIG_DEBUG_FS) || defined(VERBOSE_DEBUG)

static const char *dma_reg_names[DMA_NUM_REGS] = {
	REG_NAME_ENTRY(DMA_CTRL),
	REG_NAME_ENTRY(DMA_CHAN_CTRL),
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
	REG_NAME_ENTRY(DMA_STS_CTRL),
	REG_NAME_ENTRY(DMA_STS_STATE),
	REG_NAME_ENTRY(DMA_STS_PTR_L),
	REG_NAME_ENTRY(DMA_STS_PTR_H),
	REG_NAME_ENTRY(DMS_STS_ADDR_L),
	REG_NAME_ENTRY(DMA_STS_ADDR_H),
	REG_NAME_ENTRY(DMA_SPARE)
};

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

	val = readl(pb->dma.dma_base + DMA_CHAN_CTRL);
	ret = dump_dma_reg_verbose(pb, DMA_CHAN_CTRL, buf, &written, len,
			"\tDOUBLE_BUF 0x%04x CHAN_RESET 0x%04x\n",
			(val & DMA_CHAN_DOUBLE_BUF_MASK) >>
					DMA_CHAN_DOUBLE_BUF_SHIFT,
			val & DMA_CHAN_RESET_MASK);
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

	for (i = REG_INDEX(DMA_STS_BLOCK_START);
			i < REG_INDEX(DMA_STS_BLOCK_END); i++) {
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
			"\tLB_START_Y %u LB_START_X %u\n",
			(val & DMA_CHAN_LB_START_Y_MASK) >>
					DMA_CHAN_LB_START_Y_SHIFT,
			val & DMA_CHAN_LB_START_X_MASK);
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
			!!(val & DMA_CHAN_NOC_OUTSTANDING),
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

#endif

#ifdef VERBOSE_DEBUG
static void log_dma_registers(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel, const char *msg)
{
	int ret, written;

	ret = snprintf(pb->vdbg_log, pb->vdbg_log_len, "dma:\n");
	if (ret < 0)
		goto err_exit;

	written = ret;

	ret = dump_dma_ctrl_registers(&pb->dma.debug, pb->vdbg_log + written,
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


	ret = dump_dma_status_registers(&pb->dma.debug, pb->vdbg_log + written,
			pb->vdbg_log_len - written);
	if (ret < 0)
		goto err_exit;

	dev_vdbg(&pb->pdev->dev, "%s\n%s", msg, pb->vdbg_log);

err_exit:
	dev_err(&pb->pdev->dev, "%s: register log error, err = %d", __func__,
			ret);
}

#define LOG_DMA_REGISTERS(pb, channel)		\
	log_dma_registers(pb, channel, __func__)

#else
#define LOG_DMA_REGISTERS(pb, channel)		\
do { } while (0)
#endif

/* The caller to this function must hold pb->lock */
static inline void set_dma_channel_mode(struct paintbox_dma_transfer *transfer,
		uint32_t src_type, uint32_t dst_type, bool gather)
{
	transfer->chan_mode |= src_type << DMA_CHAN_SRC_SHIFT;
	transfer->chan_mode |= dst_type << DMA_CHAN_DST_SHIFT;
	transfer->chan_mode |= DMA_CHAN_ENA;
}

/* The caller to this function must hold pb->lock */
static int set_dma_lbp_parameters(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer,
		struct dma_lbp_config *config)
{
	struct paintbox_lb *lb;
	int ret;

	lb = get_lb(pb, session, config->lbp_id, config->lb_id, &ret);
	if (ret < 0)
		return ret;

	if (config->read_ptr_id >= lb->num_read_ptrs) {
		dev_err(&pb->pdev->dev, "%s: dma%u invalid rptr id %u\n",
				__func__, channel->channel_id,
				config->read_ptr_id);
		return -EINVAL;
	}

	if (config->start_x_pixels > DMA_CHAN_LB_START_MAX ||
			config->start_x_pixels < DMA_CHAN_LB_START_MIN) {
		dev_err(&pb->pdev->dev,
				"%s: dma%u lb_start x out of bounds (%d <= %d "
				"<= %d\n", __func__, channel->channel_id,
				DMA_CHAN_LB_START_MIN, config->start_x_pixels,
				DMA_CHAN_LB_START_MAX);
		return -ERANGE;
	}

	if (config->start_y_pixels > DMA_CHAN_LB_START_MAX ||
			config->start_y_pixels < DMA_CHAN_LB_START_MIN) {
		dev_err(&pb->pdev->dev,
				"%s: dma%u lb_start y out of bounds (%d <= %d "
				"<= %d\n", __func__, channel->channel_id,
				DMA_CHAN_LB_START_MIN, config->start_y_pixels,
				DMA_CHAN_LB_START_MAX);
		return -ERANGE;
	}

	/* Set the LBP node configuration */
	transfer->chan_node = config->lbp_id;
	transfer->chan_node |= config->lb_id << DMA_CHAN_LB_ID_SHIFT;
	transfer->chan_node |= config->read_ptr_id << DMA_CHAN_RPTR_ID_SHIFT;

	/* Set the line buffer image position */
	transfer->chan_img_pos_high = config->start_x_pixels;
	transfer->chan_img_pos_high |= config->start_y_pixels <<
			DMA_CHAN_LB_START_Y_SHIFT;

	return 0;
}

/* The caller to this function must hold pb->lock */
static int validate_dma_image_parameters(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel,
		struct dma_image_config *config)
{
	/* Image Size */
	if (config->width_pixels > DMA_CHAN_IMG_SIZE_MAX) {
		dev_err(&pb->pdev->dev,
				"%s: dma%u image width %u > %u too large\n",
				__func__,
				channel->channel_id, config->width_pixels,
				DMA_CHAN_IMG_SIZE_MAX);
		return -ERANGE;
	}

	if (config->height_pixels > DMA_CHAN_IMG_SIZE_MAX) {
		dev_err(&pb->pdev->dev,
				"%s: dma%u image height %u > %u too large\n",
				__func__,
				channel->channel_id, config->height_pixels,
				DMA_CHAN_IMG_SIZE_MAX);
		return -ERANGE;
	}

	if (config->start_x_pixels > DMA_CHAN_START_MAX)
		pr_info("START_X OVERFLOW\n");

	if (config->start_x_pixels < DMA_CHAN_START_MIN)
		pr_info("START_X UNDERFLOW\n");

	if (config->start_y_pixels > DMA_CHAN_START_MAX)
		pr_info("START_Y OVERFLOW\n");

	if (config->start_y_pixels < DMA_CHAN_START_MIN)
		pr_info("START_Y UNDERFLOW\n");

	/* Image Position */
	if (config->start_x_pixels > DMA_CHAN_START_MAX ||
			config->start_x_pixels < DMA_CHAN_START_MIN) {
		dev_err(&pb->pdev->dev,
				"%s: dma%u start x out of bounds (%d <= %d <= "
				"%d\n", __func__, channel->channel_id,
				DMA_CHAN_START_MIN, config->start_x_pixels,
				DMA_CHAN_START_MAX);
		return -ERANGE;
	}

	if (config->start_y_pixels > DMA_CHAN_START_MAX ||
			config->start_y_pixels < DMA_CHAN_START_MIN) {
		dev_err(&pb->pdev->dev,
				"%s: dma%u start y out of bounds (%d <= %d <= "
				"%d\n", __func__, channel->channel_id,
				DMA_CHAN_START_MIN, config->start_y_pixels,
				DMA_CHAN_START_MAX);
		return -ERANGE;
	}

	/* Image Format */
	if (config->components < DMA_CHAN_MIN_COMPONENTS ||
			config->components > DMA_CHAN_MAX_COMPONENTS) {
		dev_err(&pb->pdev->dev,
				"%s: dma%u invalid number of components, %u\n",
				__func__, channel->channel_id,
				config->components);
		return -EINVAL;
	}

	if (config->planes < DMA_CHAN_MIN_PLANES ||
			config->planes > DMA_CHAN_MAX_PLANES) {
		dev_err(&pb->pdev->dev,
				"%s: dma%u invalid number of planes, %u\n",
				__func__, channel->channel_id, config->planes);
		return -EINVAL;
	}

	/* Image Layout */
	if (config->row_stride_bytes > DMA_CHAN_ROW_STRIDE_MAX) {
		dev_err(&pb->pdev->dev,
				"%s: dma%u invalid row stride, %u\n", __func__,
				channel->channel_id, config->row_stride_bytes);
		return -EINVAL;
	}

	if (config->plane_stride_bytes > DMA_CHAN_PLANE_STRIDE_MAX) {
		dev_err(&pb->pdev->dev,
				"%s: dma%u invalid plane stride, %llu\n",
				__func__, channel->channel_id,
				config->plane_stride_bytes);
		return -EINVAL;
	}

	return 0;
}

/* The caller to this function must hold pb->lock */
static int set_dma_image_parameters(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer,
		struct dma_image_config *config)
{
	int ret;

	ret = validate_dma_image_parameters(pb, channel, config);
	if (ret < 0)
		return ret;

	/* Image Start Position */
	transfer->chan_img_pos_low = config->start_x_pixels;
	transfer->chan_img_pos_low |= config->start_y_pixels <<
			DMA_CHAN_START_Y_SHIFT;

	/* Image Size */
	transfer->chan_img_size = config->width_pixels;
	transfer->chan_img_size |= config->height_pixels <<
			DMA_CHAN_IMG_HEIGHT_SHIFT;

	/* Image Format */
	transfer->chan_img_format = config->components - 1;
	transfer->chan_img_format |= (config->planes - 1) <<
			DMA_CHAN_PLANES_SHIFT;

	switch (config->bit_depth) {
	case 8:
		transfer->chan_img_format |= DMA_CHAN_BIT_DEPTH8 <<
				DMA_CHAN_BIT_DEPTH_SHIFT;
		break;
	case 10:
		transfer->chan_img_format |= DMA_CHAN_BIT_DEPTH10 <<
				DMA_CHAN_BIT_DEPTH_SHIFT;
		break;
	case 12:
		transfer->chan_img_format |= DMA_CHAN_BIT_DEPTH12 <<
				DMA_CHAN_BIT_DEPTH_SHIFT;
		break;
	case 14:
		transfer->chan_img_format |= DMA_CHAN_BIT_DEPTH14 <<
				DMA_CHAN_BIT_DEPTH_SHIFT;
		break;
	case 16:
		transfer->chan_img_format |= DMA_CHAN_BIT_DEPTH16 <<
				DMA_CHAN_BIT_DEPTH_SHIFT;
		break;
	default:
		dev_err(&pb->pdev->dev, "%s: dma%u: unsupported bit depth %u",
			__func__, channel->channel_id, config->bit_depth);
		return -EINVAL;
	};

	switch (config->rgba_format) {
	case RGBA_FORMAT_DISABLED:
		transfer->chan_img_format |=
				DMA_CHAN_RGBA_FORMAT_DISABLED <<
				DMA_CHAN_RGBA_FORMAT_SHIFT;
		break;
	case RGBA_FORMAT_RGBA:
		transfer->chan_img_format |= DMA_CHAN_RGBA_FORMAT_RGBA <<
				DMA_CHAN_RGBA_FORMAT_SHIFT;
		break;
	case RGBA_FORMAT_ARGB:
		transfer->chan_img_format |= DMA_CHAN_RGBA_FORMAT_ARGB <<
				DMA_CHAN_RGBA_FORMAT_SHIFT;
		break;
	default:
		dev_err(&pb->pdev->dev, "%s: dma%u: invalid RGBA format %u",
			__func__, channel->channel_id, config->rgba_format);
		return -EINVAL;
	};

	if (config->block4x4)
		transfer->chan_img_format |= DMA_CHAN_BLOCK_4X4;

	if (config->mipi_raw_format)
		transfer->chan_img_format |= DMA_CHAN_MIPI_RAW_FORMAT;

	/* Image Layout */
	transfer->chan_img_layout_low = config->row_stride_bytes;
	transfer->chan_img_layout_low |= (config->plane_stride_bytes &
			DMA_CHAN_PLANE_STRIDE_LOW_M) <<
			DMA_CHAN_PLANE_STRIDE_LOW_SHIFT;
	transfer->chan_img_layout_high |= config->plane_stride_bytes >>
			DMA_CHAN_PLANE_STRIDE_LOW_WIDTH;

	return 0;
}

/* The caller to this function must hold pb->lock */
static void set_dma_dram_parameters(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer)
{
	/* TODO(ahampson):  Only physical addresses to contiguous allocations
	 * are support right now.  Once the IOMMU is ready virtual addresses to
	 * non-contiguous memory will be supported.
	 */
#ifndef CONFIG_PAINTBOX_IOMMU
	transfer->chan_va_low = (uint32_t)transfer->buf_paddr;
	transfer->chan_va_high = (uint32_t)((uint64_t)transfer->buf_paddr >>
			32);
#endif

	/* VA_BDRY is the virtual address boundary for the DMA transfer.  The
	 * memory transferred by DMA is [VA, VA + VA_BDRY].
	 */
	transfer->chan_va_bdry_low = (uint32_t)transfer->len_bytes;
	transfer->chan_va_bdry_high = (uint32_t)(transfer->len_bytes >>
			32);
}

/* The caller to this function must hold pb->lock */
static int set_dma_transfer_region_parameters(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer,
		struct dma_transfer_config *config)
{
	if (config->stripe_height > DMA_CHAN_MAX_STRIPES) {
		dev_err(&pb->pdev->dev, "%s: dma%u: invalid stripe height %u",
			__func__, channel->channel_id, config->stripe_height);
		return -EINVAL;
	}

	if (config->sheet_width > DMA_CHAN_MAX_SHEET_WIDTH) {
		dev_err(&pb->pdev->dev, "%s: dma%u: invalid sheet width %u",
			__func__, channel->channel_id, config->sheet_width);
		return -EINVAL;
	}

	if (config->sheet_height > DMA_CHAN_MAX_SHEET_HEIGHT) {
		dev_err(&pb->pdev->dev, "%s: dma%u: invalid sheet height %u",
			__func__, channel->channel_id, config->sheet_height);
		return -EINVAL;
	}

	transfer->chan_bif_xfer = config->stripe_height;
	transfer->chan_bif_xfer |= DMA_CHAN_OUTSTANDING_DEF <<
			DMA_CHAN_OUTSTANDING_SHIFT;

	transfer->chan_noc_xfer_low = config->sheet_width;
	transfer->chan_noc_xfer_low |= config->sheet_height <<
			DMA_CHAN_SHEET_HEIGHT_SHIFT;
	transfer->chan_noc_xfer_high = DMA_CHAN_RETRY_INTERVAL_DEF;

	return 0;
}

/* The caller to this function must hold pb->lock */
int dma_setup_dram_to_lbp_transfer(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer,
		struct dma_transfer_config *config)
{
	int ret;

	dev_dbg(&pb->pdev->dev,
			"%s: dma%u: va %p pa %pa ->lbp%u lb%u %llu bytes\n",
			__func__, config->channel_id, transfer->buf_vaddr,
			&transfer->buf_paddr, config->dst.lbp.lbp_id,
			config->dst.lbp.lb_id, config->src.dram.len_bytes);

	set_dma_channel_mode(transfer, DMA_CHAN_SRC_DRAM, DMA_CHAN_DST_LBP,
			config->dst.lbp.gather);

	ret = set_dma_lbp_parameters(pb, session, channel, transfer,
			&config->dst.lbp);
	if (ret < 0)
		return ret;

	ret = set_dma_image_parameters(pb, channel, transfer, &config->img);
	if (ret < 0)
		return ret;

	ret = set_dma_transfer_region_parameters(pb, channel, transfer, config);
	if (ret < 0)
		return ret;

	set_dma_dram_parameters(pb, channel, transfer);

	return 0;
}

/* The caller to this function must hold pb->lock */
int dma_setup_dram_to_stp_transfer(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer,
		struct dma_transfer_config *config)
{
	int ret;

	set_dma_channel_mode(transfer, DMA_CHAN_SRC_DRAM, DMA_CHAN_DST_STP,
			false);

	/* TODO(ahampson):  Not supported in the simulator (b/28197242).  Once
	 * simulator support is added this code will need to be completed.
	 * Kernel support for this transfer mode is tracked in b/28341158.
	 */
	ret = set_dma_transfer_region_parameters(pb, channel, transfer, config);
	if (ret < 0)
		return ret;

	set_dma_dram_parameters(pb, channel, transfer);

	return 0;
}

/* The caller to this function must hold pb->lock */
int dma_setup_lbp_to_dram_transfer(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer,
		struct dma_transfer_config *config)
{
	int ret;

	dev_dbg(&pb->pdev->dev,
			"%s: dma%u: lbp%u lb%u -> va %p pa %pa %llu bytes\n",
			__func__, config->channel_id, config->src.lbp.lbp_id,
			config->src.lbp.lb_id, transfer->buf_vaddr,
			&transfer->buf_paddr, config->dst.dram.len_bytes);

	set_dma_channel_mode(transfer, DMA_CHAN_SRC_LBP, DMA_CHAN_DST_DRAM,
			config->src.lbp.gather);

	ret = set_dma_lbp_parameters(pb, session, channel, transfer,
			&config->src.lbp);
	if (ret < 0)
		return ret;

	ret = set_dma_image_parameters(pb, channel, transfer, &config->img);
	if (ret < 0)
		return ret;

	ret = set_dma_transfer_region_parameters(pb, channel, transfer, config);
	if (ret < 0)
		return ret;

	set_dma_dram_parameters(pb, channel, transfer);

	return 0;
}

/* The caller to this function must hold pb->lock */
int dma_setup_stp_to_dram_transfer(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer,
		struct dma_transfer_config *config)
{
	int ret;

	/* TODO(ahampson):  Not supported in the simulator (b/28197242).  Once
	 * simulator support is added this code will need to be completed.
	 * Kernel support for this transfer mode is tracked in b/28341158.
	 */

	set_dma_channel_mode(transfer, DMA_CHAN_SRC_STP, DMA_CHAN_DST_DRAM,
			false);

	ret = set_dma_transfer_region_parameters(pb, channel, transfer, config);
	if (ret < 0)
		return ret;

	set_dma_dram_parameters(pb, channel, transfer);

	return 0;
}

/* The caller to this function must hold pb->lock */
int dma_setup_mipi_to_lbp_transfer(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer,
		struct dma_transfer_config *config)
{
	int ret;

	if (config->dst.lbp.gather) {
		dev_err(&pb->pdev->dev,
				"%s: dma%u: gather mode not supported for MIPI "
				"transfers", __func__, channel->channel_id);
		return -EINVAL;
	}

	set_dma_channel_mode(transfer, DMA_CHAN_SRC_MIPI_IN, DMA_CHAN_DST_LBP,
			false);

	ret = set_dma_lbp_parameters(pb, session, channel, transfer,
			&config->dst.lbp);
	if (ret < 0)
		return ret;

	ret = set_dma_image_parameters(pb, channel, transfer, &config->img);
	if (ret < 0)
		return ret;

	ret = set_dma_transfer_region_parameters(pb, channel, transfer, config);
	if (ret < 0)
		return ret;

	return 0;
}

/* The caller to this function must hold pb->lock */
int dma_setup_lbp_to_mipi_transfer(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer,
		struct dma_transfer_config *config)
{
	int ret;

	if (config->src.lbp.gather) {
		dev_err(&pb->pdev->dev,
				"%s: dma%u: gather mode not supported for MIPI "
				"transfers", __func__, channel->channel_id);
		return -EINVAL;
	}

	set_dma_channel_mode(transfer, DMA_CHAN_SRC_LBP, DMA_CHAN_DST_MIPI_OUT,
			false);

	ret = set_dma_lbp_parameters(pb, session, channel, transfer,
			&config->src.lbp);
	if (ret < 0)
		return ret;

	ret = set_dma_image_parameters(pb, channel, transfer, &config->img);
	if (ret < 0)
		return ret;

	ret = set_dma_transfer_region_parameters(pb, channel, transfer, config);
	if (ret < 0)
		return ret;

	return 0;
}

static void commit_transfer_to_hardware(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel)
{
	struct paintbox_dma_transfer *transfer;
	struct paintbox_dma *dma = &pb->dma;

	io_disable_dma_interrupts(pb);

	/* TODO(ahampson):  There should be a transfer queue that we dequeue
	 * from here.  Currently only a single transfer is supported.
	 */
	transfer = &channel->transfer;

	dma_select_channel(pb, channel->channel_id);

	/* Load the transfer into the DMA channel registers */
	writel(transfer->chan_img_format, dma->dma_base + DMA_CHAN_IMG_FORMAT);
	writel(transfer->chan_img_size, dma->dma_base + DMA_CHAN_IMG_SIZE);
	writel(transfer->chan_img_pos_low, dma->dma_base + DMA_CHAN_IMG_POS_L);
	writel(transfer->chan_img_pos_high, dma->dma_base + DMA_CHAN_IMG_POS_H);
	writel(transfer->chan_img_layout_low, dma->dma_base +
			DMA_CHAN_IMG_LAYOUT_L);
	writel(transfer->chan_img_layout_high, dma->dma_base +
			DMA_CHAN_IMG_LAYOUT_H);
	writel(transfer->chan_bif_xfer, dma->dma_base + DMA_CHAN_BIF_XFER);
	writel(transfer->chan_va_low, dma->dma_base + DMA_CHAN_VA_L);
	writel(transfer->chan_va_high, dma->dma_base + DMA_CHAN_VA_H);
	writel(transfer->chan_va_bdry_low, dma->dma_base + DMA_CHAN_VA_BDRY_L);
	writel(transfer->chan_va_bdry_high, dma->dma_base + DMA_CHAN_VA_BDRY_H);
	writel(transfer->chan_noc_xfer_low, dma->dma_base +
			DMA_CHAN_NOC_XFER_L);
	writel(transfer->chan_noc_xfer_high, dma->dma_base +
			DMA_CHAN_NOC_XFER_H);
	writel(transfer->chan_node, dma->dma_base + DMA_CHAN_NODE);

	/* Enable interrupts for the channel */
	writel(DMA_CHAN_INT_EOF | DMA_CHAN_INT_MIF_ERR | DMA_CHAN_INT_VA_ERR,
			dma->dma_base + DMA_CHAN_IMR);

	/* Write the channel mode register last as this will enqueue the
	 * transfer into the hardware.
	 */
	writel(transfer->chan_mode, dma->dma_base + DMA_CHAN_MODE);

	io_enable_dma_channel_interrupt(pb, channel->channel_id);

	io_enable_dma_interrupts(pb);

	LOG_DMA_REGISTERS(pb, channel);
}

/* The caller to this function must hold pb->lock */
int dma_start_transfer(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel)
{
	/* TODO(ahampson):  Implement support for double buffering b/28316153 */
	commit_transfer_to_hardware(pb, channel);

	return 0;
}

/* The caller to this function must hold pb->lock */
int dma_stop_transfer(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel)
{
	uint32_t mode;

	io_disable_dma_interrupts(pb);

	dma_select_channel(pb, channel->channel_id);

	writel(0, pb->dma.dma_base + DMA_CHAN_IMR);

	mode = readl(pb->dma.dma_base + DMA_CHAN_MODE);
	mode &= ~DMA_CHAN_ENA;
	writel(mode, pb->dma.dma_base + DMA_CHAN_MODE);

	io_enable_dma_interrupts(pb);

	return 0;
}

irqreturn_t paintbox_dma_interrupt(struct paintbox_data *pb,
		uint32_t channel_mask)
{
	unsigned int channel_id;

	for (channel_id = 0; channel_id < pb->dma.num_channels && channel_mask;
			channel_id++, channel_mask >>= 1) {
		struct paintbox_dma_channel *channel;
		uint32_t status;

		if (!(channel_mask & 0x01))
			continue;

		channel = &pb->dma.channels[channel_id];

		dma_select_channel(pb, channel_id);

		status = readl(pb->dma.dma_base + DMA_CHAN_ISR);
		writel(status, pb->dma.dma_base + DMA_CHAN_ISR);

		if ((status & DMA_CHAN_INT_MIF_ERR) ||
				(status & DMA_CHAN_INT_VA_ERR))
			dma_report_completion(pb, channel, -EIO);
		else if (status & DMA_CHAN_INT_EOF)
			dma_report_completion(pb, channel, 0);
	}

	return IRQ_HANDLED;
}

#ifdef CONFIG_DEBUG_FS
void paintbox_dma_debug_init(struct paintbox_data *pb)
{
	unsigned int i, reg_index;
	size_t reg_count = DMA_CTRL_NUM_REGS + DMA_STS_NUM_REGS;

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

	for (reg_index = REG_INDEX(DMA_STS_BLOCK_START); i < reg_count &&
			reg_index < REG_INDEX(DMA_STS_BLOCK_END); reg_index++,
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
}
#endif

int paintbox_dma_init(struct paintbox_data *pb)
{
	unsigned int i;

	pb->dma.dma_base = pb->reg_base + IPU_DMA_OFFSET;

	pb->dma.num_channels = readl(pb->dma.dma_base + DMA_CAP0) &
			MAX_DMA_CHAN_MASK;

#ifdef CONFIG_DEBUG_FS
	paintbox_dma_debug_init(pb);
#endif

#ifdef VERBOSE_DEBUG
	paintbox_alloc_debug_buffer(pb, DMA_DEBUG_BUFFER_SIZE);
#endif

	/* TODO(ahampson):  refactor out the storage of the caps structure  */
	pb->caps.num_dma_channels = pb->dma.num_channels;

	pb->dma.channels = kzalloc(sizeof(struct paintbox_dma_channel) *
			pb->dma.num_channels, GFP_KERNEL);
	if (!pb->dma.channels)
		return -ENOMEM;

	/* Store channel id with object as a convenience to avoid doing a
	 * lookup later on.
	 */
	for (i = 0; i < pb->dma.num_channels; i++) {
		struct paintbox_dma_channel *channel = &pb->dma.channels[i];
		channel->channel_id = i;
		channel->interrupt_id = DMA_NO_INTERRUPT;
#ifdef CONFIG_DEBUG_FS
		paintbox_debug_create_entry(pb, &channel->debug,
				pb->dma.debug.debug_dir, "channel", i,
				dump_dma_channel_registers, channel);

		paintbox_debug_create_reg_entries(pb, &channel->debug,
				&dma_reg_names[REG_INDEX(DMA_CHAN_BLOCK_START)],
				DMA_CHAN_NUM_REGS, dma_channel_reg_entry_write,
				dma_channel_reg_entry_read);
#endif
	}

	dev_dbg(&pb->pdev->dev, "dma: base %p len %u dma channels %u\n",
			pb->dma.dma_base, DMA_BLOCK_LEN,
			pb->dma.num_channels);

	return 0;
}
