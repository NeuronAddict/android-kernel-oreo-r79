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

#include "paintbox-dma.h"
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
	REG_NAME_ENTRY(DMA_CHAN_DEPENDENCY)
};

static inline void dump_dma_reg(struct paintbox_data *pb, uint32_t reg,
		char *buf, int *written, size_t len)
{
	const char *reg_name = dma_reg_names[REG_INDEX(reg)];
	int ret;

	ret = snprintf(buf + *written, len - *written, "0x%04lx: %s\t0x%08x\n",
			pb->dma.dma_base - pb->reg_base + reg,
			reg_name ? reg_name : REG_UNUSED,
			readl(pb->dma.dma_base + reg));
	if (ret < 0) {
		dev_err(&pb->pdev->dev, "%s: register dump error, err = %d",
				__func__, ret);
		return;
	}

	*written += ret;
}

int dump_dma_registers(struct paintbox_data *pb, char *buf, size_t len)
{
	uint32_t val;
	int ret, written = 0;

	dump_dma_reg(pb, DMA_CTRL, buf, &written, len);

	val = readl(pb->dma.dma_base + DMA_CTRL);
	ret = snprintf(buf + written, len - written,
			"\tDMA_CHAN_SEL 0x%02x RESET %d\n",
			(val & DMA_CHAN_SEL_MASK) >> DMA_CHAN_SEL_SHIFT,
			val & DMA_RESET);
	if (ret < 0)
		goto err_exit;

	written += ret;

	dump_dma_reg(pb, DMA_CHAN_CTRL, buf, &written, len);

	val = readl(pb->dma.dma_base + DMA_CHAN_CTRL);
	ret = snprintf(buf + written, len - written,
			"\tDOUBLE_BUF 0x%04x CHAN_RESET 0x%04x\n",
			(val & DMA_CHAN_DOUBLE_BUF_MASK) >>
					DMA_CHAN_DOUBLE_BUF_SHIFT,
			val & DMA_CHAN_RESET_MASK);
	if (ret < 0)
		goto err_exit;

	written += ret;

	dump_dma_reg(pb, DMA_CAP0, buf, &written, len);

	val = readl(pb->dma.dma_base + DMA_CAP0);
	ret = snprintf(buf + written, len - written,
			"\tMAX_DMA_CHAN %u\n", val & MAX_DMA_CHAN_MASK);
	if (ret < 0)
		goto err_exit;

	written += ret;

	dump_dma_reg(pb, DMA_PMON_CFG, buf, &written, len);
	dump_dma_reg(pb, DMA_PMON_CNT_0_CFG, buf, &written, len);
	dump_dma_reg(pb, DMA_PMON_CNT_0, buf, &written, len);
	dump_dma_reg(pb, DMA_PMON_CNT_0_STS_ACC, buf, &written, len);
	dump_dma_reg(pb, DMA_PMON_CNT_0_STS, buf, &written, len);
	dump_dma_reg(pb, DMA_PMON_CNT_1_CFG, buf, &written, len);
	dump_dma_reg(pb, DMA_PMON_CNT_1, buf, &written, len);
	dump_dma_reg(pb, DMA_PMON_CNT_1_STS_ACC, buf, &written, len);
	dump_dma_reg(pb, DMA_PMON_CNT_1_STS, buf, &written, len);

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

static inline const char *dma_swizzle_to_str(uint32_t val)
{
	switch ((val & DMA_CHAN_SWIZZLE_MASK) >> DMA_CHAN_SWIZZLE_SHIFT) {
	case DMA_CHAN_SWIZZLE_NO_SWIZZLE:
		return "NONE";
	case DMA_CHAN_SWIZZLE_BIG_ENDIAN:
		return "BIG ENDIAN";
	case DMA_CHAN_SWIZZLE_NEIGHBOR:
		return "NEIGHBOR";
	default:
		return "UNKNOWN";
	};
}

static inline const char *dma_rgba_to_str(uint32_t val)
{
	switch ((val & DMA_CHAN_RGBA_MASK) >> DMA_CHAN_RGBA_SHIFT) {
	case DMA_CHAN_ALPHA_MODE_RGBA_DISABLED:
		return "DISABLED";
	case DMA_CHAN_ALPHA_MODE_RGBA:
		return "RGBA";
	case DMA_CHAN_ALPHA_MODE_ARGB:
		return "ARGB";
	default:
		return "UNKNOWN";
	};
}

int dump_dma_channel_registers(struct paintbox_data *pb,
		unsigned int channel_id, char *buf, size_t len)
{
	uint64_t va, va_bdry, plane_stride;
	uint32_t val, row_stride, retry_interval;
	int ret, written = 0;

	dma_select_channel(pb, channel_id);

	dump_dma_reg(pb, DMA_CHAN_MODE, buf, &written, len);

	val = readl(pb->dma.dma_base + DMA_CHAN_MODE);
	ret = snprintf(buf + written, len - written,
			"\tGATHER %u ADDR_MODE %s DST %s SRC %s ENA %u\n",
			!!(val & DMA_CHAN_GATHER),
			val & DMA_CHAN_ADDR_MODE_PHYSICAL ? "PHYSICAL" :
			"ABSTRACT",
			dma_dst_to_str(val),
			dma_src_to_str(val),
			!!(val & DMA_CHAN_ENA));
	if (ret < 0)
		goto err_exit;

	written += ret;

	dump_dma_reg(pb, DMA_CHAN_IMG_FORMAT, buf, &written, len);

	val = readl(pb->dma.dma_base + DMA_CHAN_IMG_FORMAT);
	ret = snprintf(buf + written, len - written,
			"\tBLOCK_4x4 %u RGBA %s RAW 0x%02x SWIZZLE %s BIT DEPTH"
			" %u PLANES %u COMPONENTS %u\n",
			!!(val & DMA_CHAN_BLOCK_4X4),
			dma_rgba_to_str(val),
			(val & DMA_CHAN_RAW_MASK) >> DMA_CHAN_RAW_SHIFT,
			dma_swizzle_to_str(val),
			get_bit_depth(val),
			((val & DMA_CHAN_PLANES_MASK) >>
					DMA_CHAN_PLANES_SHIFT) + 1,
			(val & DMA_CHAN_COMPONENTS_MASK) + 1);
	if (ret < 0)
		goto err_exit;

	written += ret;

	dump_dma_reg(pb, DMA_CHAN_IMG_SIZE, buf, &written, len);

	val = readl(pb->dma.dma_base + DMA_CHAN_IMG_SIZE);
	ret = snprintf(buf + written, len - written,
			"\tIMG_HEIGHT %u IMG_WIDTH %u\n",
			(val & DMA_CHAN_IMG_HEIGHT_MASK) >>
					DMA_CHAN_IMG_HEIGHT_SHIFT,
			val & DMA_CHAN_IMG_SIZE_MASK);
	if (ret < 0)
		goto err_exit;

	written += ret;

	dump_dma_reg(pb, DMA_CHAN_IMG_POS_L, buf, &written, len);

	val = readl(pb->dma.dma_base + DMA_CHAN_IMG_POS_L);
	written += snprintf(buf + written, len - written,
			"\tSTART_Y %u START_X %u\n",
			(val & DMA_CHAN_START_Y_MASK) >> DMA_CHAN_START_Y_SHIFT,
			val & DMA_CHAN_START_X_MASK);

	dump_dma_reg(pb, DMA_CHAN_IMG_POS_H, buf, &written, len);

	val = readl(pb->dma.dma_base + DMA_CHAN_IMG_POS_H);
	ret = snprintf(buf + written, len - written,
			"\tLB_START_Y %u LB_START_X %u\n",
			(val & DMA_CHAN_LB_START_Y_MASK) >>
					DMA_CHAN_LB_START_Y_SHIFT,
			val & DMA_CHAN_LB_START_X_MASK);
	if (ret < 0)
		goto err_exit;

	written += ret;

	dump_dma_reg(pb, DMA_CHAN_IMG_LAYOUT_L, buf, &written, len);
	dump_dma_reg(pb, DMA_CHAN_IMG_LAYOUT_H, buf, &written, len);

	val = readl(pb->dma.dma_base + DMA_CHAN_IMG_LAYOUT_L);

	row_stride = val & DMA_CHAN_ROW_STRIDE_MASK;
	plane_stride = (val & DMA_CHAN_PLANE_STRIDE_LOW_MASK) >>
			DMA_CHAN_PLANE_STRIDE_LOW_SHIFT;

	val = readl(pb->dma.dma_base + DMA_CHAN_IMG_LAYOUT_H);

	plane_stride |= (uint64_t)(val & DMA_CHAN_PLANE_STRIDE_HIGH_MASK) <<
			DMA_CHAN_PLANE_STRIDE_LOW_WIDTH;
	ret = snprintf(buf + written, len - written,
			"\tROW_STRIDE %u PLANE_STRIDE %llu\n", row_stride,
			plane_stride);
	if (ret < 0)
		goto err_exit;

	written += ret;

	dump_dma_reg(pb, DMA_CHAN_BIF_XFER, buf, &written, len);

	val = readl(pb->dma.dma_base + DMA_CHAN_BIF_XFER);
	ret = snprintf(buf + written, len - written,
			"\tOUTSTANDING %u STRIPE_HEIGHT %u\n",
			(val & DMA_CHAN_OUTSTANDING_MASK) >>
					DMA_CHAN_OUTSTANDING_SHIFT,
			val & DMA_CHAN_STRIPE_HEIGHT_MASK);
	if (ret < 0)
		goto err_exit;

	written += ret;

	dump_dma_reg(pb, DMA_CHAN_VA_L, buf, &written, len);
	dump_dma_reg(pb, DMA_CHAN_VA_H, buf, &written, len);

	va = readl(pb->dma.dma_base + DMA_CHAN_VA_L);
	va |= ((uint64_t)readl(pb->dma.dma_base + DMA_CHAN_VA_H)) <<
			32;
	ret = snprintf(buf + written, len - written,
			"\tVA 0x%016llx\n", va);
	if (ret < 0)
		goto err_exit;

	written += ret;

	dump_dma_reg(pb, DMA_CHAN_VA_BDRY_L, buf, &written, len);
	dump_dma_reg(pb, DMA_CHAN_VA_BDRY_H, buf, &written, len);

	va_bdry = readl(pb->dma.dma_base + DMA_CHAN_VA_BDRY_L);
	va_bdry |= ((uint64_t)readl(pb->dma.dma_base + DMA_CHAN_VA_BDRY_H)) <<
			32;
	ret = snprintf(buf + written, len - written,
			"\tVA BDRY %llu\n", va_bdry);
	if (ret < 0)
		goto err_exit;

	written += ret;

	dump_dma_reg(pb, DMA_CHAN_NOC_XFER_L, buf, &written, len);

	val = readl(pb->dma.dma_base + DMA_CHAN_NOC_XFER_L);

	ret = snprintf(buf + written, len - written,
			"\tOUTSTANDING %u SHEET_HEIGHT %u SHEET_WIDTH %u\n",
			!!(val & DMA_CHAN_NOC_OUTSTANDING),
			(val & DMA_CHAN_SHEET_HEIGHT_MASK) >>
					DMA_CHAN_SHEET_HEIGHT_SHIFT,
			val & DMA_CHAN_SHEET_WIDTH_MASK);
	if (ret < 0)
		goto err_exit;

	written += ret;

	dump_dma_reg(pb, DMA_CHAN_NOC_XFER_H, buf, &written, len);

	val = readl(pb->dma.dma_base + DMA_CHAN_NOC_XFER_H);

	ret = snprintf(buf + written, len - written, "\tRETRY_INTERVAL %u\n",
			val & DMA_CHAN_RETRY_INTERVAL_MASK);
	if (ret < 0)
		goto err_exit;

	written += ret;

	dump_dma_reg(pb, DMA_CHAN_NODE, buf, &written, len);

	val = readl(pb->dma.dma_base + DMA_CHAN_NODE);
	ret = snprintf(buf + written, len - written,
			"\tCORE_ID %u LB_ID %u RPTR_ID %u\n",
			val & DMA_CHAN_CORE_ID_MASK,
			(val & DMA_CHAN_LB_ID_MASK) >> DMA_CHAN_LB_ID_SHIFT,
			(val & DMA_CHAN_RPTR_ID_MASK) >>
					DMA_CHAN_RPTR_ID_SHIFT);
	if (ret < 0)
		goto err_exit;

	written += ret;

	dump_dma_reg(pb, DMA_CHAN_IMR, buf, &written, len);
	dump_dma_reg(pb, DMA_CHAN_ISR, buf, &written, len);
	dump_dma_reg(pb, DMA_CHAN_ISR_OVF, buf, &written, len);

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

	ret = dump_dma_registers(pb, pb->vdbg_log + written,
			pb->vdbg_log_len - written);
	if (ret < 0)
		goto err_exit;

	dev_vdbg(&pb->pdev->dev, "%s\n%s", msg, pb->vdbg_log);

	ret = snprintf(pb->vdbg_log, pb->vdbg_log_len, "dma ch%u:\n",
			channel->channel_id);
	if (ret < 0)
		goto err_exit;

	written = ret;

	ret = dump_dma_channel_registers(pb, channel->channel_id,
			pb->vdbg_log + written, pb->vdbg_log_len - written);
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

static void enable_channel_interrupts(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel)
{
	unsigned long irq_flags;
	uint32_t ipu_imr;

	spin_lock_irqsave(&pb->irq_lock, irq_flags);

	dma_select_channel(pb, channel->channel_id);

	/* Enable interrupts for the channel */
	writel(DMA_CHAN_INT_EOF | DMA_CHAN_INT_MIF_ERR | DMA_CHAN_INT_VA_ERR,
			pb->dma.dma_base + DMA_CHAN_IMR);

	ipu_imr = readl(pb->io.apb_base + IPU_IMR);
	ipu_imr |= 1 << channel->channel_id;
	writel(ipu_imr, pb->io.apb_base + IPU_IMR);

	spin_unlock_irqrestore(&pb->irq_lock, irq_flags);
}

static void disable_channel_interrupts(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel)
{
	unsigned long irq_flags;
	uint32_t ipu_imr;

	spin_lock_irqsave(&pb->irq_lock, irq_flags);

	dma_select_channel(pb, channel->channel_id);

	writel(0, pb->dma.dma_base + DMA_CHAN_IMR);

	ipu_imr = readl(pb->io.apb_base + IPU_IMR);
	ipu_imr &= ~(1 << channel->channel_id);
	writel(ipu_imr, pb->io.apb_base + IPU_IMR);

	spin_unlock_irqrestore(&pb->irq_lock, irq_flags);
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
			config->start_x_pixels > lb->width_pixels) {
		dev_err(&pb->pdev->dev,
				"%s: dma%u lb_start x out of bounds %u\n",
				__func__, channel->channel_id,
				config->start_x_pixels);
		return -ERANGE;
	}

	if (config->start_y_pixels > DMA_CHAN_LB_START_MAX ||
			config->start_y_pixels > lb->height_pixels) {
		dev_err(&pb->pdev->dev,
				"%s: dma%u lb start y out of bounds %u\n",
				__func__, channel->channel_id,
				config->start_y_pixels);
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

	/* Image Position */
	if (config->start_x_pixels > DMA_CHAN_START_MAX ||
			config->start_x_pixels > config->width_pixels) {
		dev_err(&pb->pdev->dev, "%s: dma%u start x out of bounds %u\n",
				__func__, channel->channel_id,
				config->start_x_pixels);
		return -ERANGE;
	}

	if (config->start_y_pixels > DMA_CHAN_START_MAX ||
			config->start_y_pixels > config->height_pixels) {
		dev_err(&pb->pdev->dev, "%s: dma%u start y out of bounds %u\n",
				__func__, channel->channel_id,
				config->start_y_pixels);
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

	switch (config->swizzle_mode) {
	case SWIZZLE_MODE_DISABLED:
		transfer->chan_img_format |= DMA_CHAN_SWIZZLE_NO_SWIZZLE <<
				DMA_CHAN_SWIZZLE_SHIFT;
		break;
	case SWIZZLE_MODE_BIG_ENDIAN:
		transfer->chan_img_format |= DMA_CHAN_SWIZZLE_BIG_ENDIAN <<
				DMA_CHAN_SWIZZLE_SHIFT;
		break;
	case SWIZZLE_MODE_NEIGHBOR:
		transfer->chan_img_format |= DMA_CHAN_SWIZZLE_NEIGHBOR <<
				DMA_CHAN_SWIZZLE_SHIFT;
		break;
	default:
		dev_err(&pb->pdev->dev, "%s: dma%u: invalid swizzle mode %u",
			__func__, channel->channel_id, config->swizzle_mode);
		return -EINVAL;
	};

	switch (config->alpha_mode) {
	case ALPHA_MODE_DISABLED:
		transfer->chan_img_format |=
				DMA_CHAN_ALPHA_MODE_RGBA_DISABLED <<
				DMA_CHAN_RGBA_SHIFT;
		break;
	case ALPHA_MODE_RGBA:
		transfer->chan_img_format |= DMA_CHAN_ALPHA_MODE_RGBA <<
				DMA_CHAN_RGBA_SHIFT;
		break;
	case ALPHA_MODE_ARGB:
		transfer->chan_img_format |= DMA_CHAN_ALPHA_MODE_ARGB <<
				DMA_CHAN_RGBA_SHIFT;
		break;
	default:
		dev_err(&pb->pdev->dev, "%s: dma%u: invalid alpha mode %u",
			__func__, channel->channel_id, config->alpha_mode);
		return -EINVAL;
	};

	if (config->block4x4)
		transfer->chan_img_format |= DMA_CHAN_BLOCK_4X4;

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

	enable_channel_interrupts(pb, channel);

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

	enable_channel_interrupts(pb, channel);

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

	enable_channel_interrupts(pb, channel);

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

	enable_channel_interrupts(pb, channel);

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

	if (config->src.lbp.gather) {
		dev_err(&pb->pdev->dev,
				"%s: dma%u: gather mode not supported for MIPI "
				"transfers", __func__, channel->channel_id);
		return -EINVAL;
	}

	/* TODO(ahampson):  Not supported in the simulator (b/28197903).  Once
	 * simulator support is added this code will need to be completed.
	 * Kernel support for this feature is tracked by b/28340987.
	 */

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

	enable_channel_interrupts(pb, channel);

	return 0;
}

static void commit_transfer_to_hardware(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel)
{
	struct paintbox_dma_transfer *transfer;
	struct paintbox_dma *dma = &pb->dma;
	unsigned long irq_flags;

	spin_lock_irqsave(&pb->irq_lock, irq_flags);

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

	/* Write the channel mode register last as this will enqueue the
	 * transfer into the hardware.
	 */
	writel(transfer->chan_mode, dma->dma_base + DMA_CHAN_MODE);

	spin_unlock_irqrestore(&pb->irq_lock, irq_flags);

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
	unsigned long irq_flags;
	uint32_t mode;

	disable_channel_interrupts(pb, channel);

	/* TODO(ahampson):  Determine proper channel shutdown procedures. */
	spin_lock_irqsave(&pb->irq_lock, irq_flags);

	dma_select_channel(pb, channel->channel_id);

	mode = readl(pb->dma.dma_base + DMA_CHAN_MODE);
	mode &= ~DMA_CHAN_ENA;
	writel(mode, pb->dma.dma_base + DMA_CHAN_MODE);

	spin_unlock_irqrestore(&pb->irq_lock, irq_flags);

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
static int dma_debug_regs_show(struct seq_file *s, void *unused)
{
	struct paintbox_dma *dma = s->private;
	struct paintbox_data *pb = container_of(dma, struct paintbox_data, dma);
	char *buf;
	size_t len;
	int ret, written;

	len = seq_get_buf(s, &buf);
	if (!buf)
		return -ENOMEM;

	mutex_lock(&pb->lock);

	ret = snprintf(buf, len, "dma:\n");
	if (ret < 0)
		goto err_exit;

	written = ret;

	ret = dump_dma_registers(pb, buf + written, len - written);
	if (ret < 0)
		goto err_exit;

	written += ret;

	mutex_unlock(&pb->lock);

	seq_commit(s, written);

	return 0;

err_exit:
	mutex_unlock(&pb->lock);
	dev_err(&pb->pdev->dev, "%s: register dump error, err = %d", __func__,
			ret);
	return ret;
}

static int dma_debug_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, dma_debug_regs_show, inode->i_private);
}

static const struct file_operations dma_debug_regs_fops = {
	.open = dma_debug_regs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static void paintbox_dma_debug_init(struct paintbox_data *pb,
		struct paintbox_dma *dma)
{
	dma->debug_dir = debugfs_create_dir("dma", pb->debug_root);
	if (IS_ERR(dma->debug_dir)) {
		dev_err(&pb->pdev->dev, "%s: err = %ld", __func__,
				PTR_ERR(dma->debug_dir));
		return;
	}

	dma->regs_dentry = debugfs_create_file("regs", S_IRUGO | S_IWUSR,
			dma->debug_dir, dma, &dma_debug_regs_fops);
	if (IS_ERR(dma->regs_dentry)) {
		dev_err(&pb->pdev->dev, "%s: err = %ld", __func__,
				PTR_ERR(dma->regs_dentry));
		return;
	}
}

static int dma_channel_debug_regs_show(struct seq_file *s, void *unused)
{
	struct paintbox_dma_channel *channel = s->private;
	struct paintbox_data *pb = channel->pb;
	char *buf;
	size_t len;
	int ret, written;

	len = seq_get_buf(s, &buf);
	if (!buf)
		return -ENOMEM;

	mutex_lock(&pb->lock);

	ret = snprintf(buf, len, "dma ch%u:\n", channel->channel_id);
	if (ret < 0)
		goto err_exit;

	written = ret;

	ret = dump_dma_channel_registers(pb, channel->channel_id, buf + written,
			len - written);
	if (ret < 0)
		goto err_exit;

	written += ret;

	mutex_unlock(&pb->lock);

	seq_commit(s, written);

	return 0;

err_exit:
	mutex_unlock(&pb->lock);
	dev_err(&pb->pdev->dev, "%s: register dump error, err = %d", __func__,
			ret);
	return ret;
}

static int dma_channel_debug_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, dma_channel_debug_regs_show, inode->i_private);
}

static const struct file_operations dma_channel_debug_regs_fops = {
	.open = dma_channel_debug_regs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static void paintbox_dma_channel_debug_init(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel)
{
	char channel_name[RESOURCE_NAME_LEN];

	snprintf(channel_name, RESOURCE_NAME_LEN, "ch%u", channel->channel_id);

	channel->debug_dir = debugfs_create_dir(channel_name,
			pb->dma.debug_dir);
	if (IS_ERR(channel->debug_dir)) {
		dev_err(&pb->pdev->dev, "%s: err = %ld", __func__,
				PTR_ERR(channel->debug_dir));
		return;
	}

	channel->regs_dentry = debugfs_create_file("regs", S_IRUGO | S_IWUSR,
			channel->debug_dir, channel,
			&dma_channel_debug_regs_fops);
	if (IS_ERR(channel->regs_dentry)) {
		dev_err(&pb->pdev->dev, "%s: err = %ld", __func__,
				PTR_ERR(channel->regs_dentry));
		return;
	}
}
#endif

int paintbox_dma_init(struct paintbox_data *pb)
{
	unsigned int i;

	pb->dma.dma_base = pb->reg_base + IPU_DMA_OFFSET;

	pb->dma.num_channels = readl(pb->dma.dma_base + DMA_CAP0) &
		MAX_DMA_CHAN_MASK;

	pb->io.dma_mask = (1 << pb->dma.num_channels) - 1;

#ifdef CONFIG_DEBUG_FS
	paintbox_dma_debug_init(pb, &pb->dma);
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
		channel->pb = pb;
#ifdef CONFIG_DEBUG_FS
		paintbox_dma_channel_debug_init(pb, channel);
#endif
	}

	dev_dbg(&pb->pdev->dev, "dma: base %p len %u dma channels %u\n",
			pb->dma.dma_base, DMA_BLOCK_LEN,
			pb->dma.num_channels);

	return 0;
}
