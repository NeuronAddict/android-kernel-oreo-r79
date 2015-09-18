/*
 * Line Buffer Pool Support for Paintbox IPU
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
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/stddef.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include "paintbox-common.h"
#include "paintbox-lbp.h"
#include "paintbox-regs.h"
#include "paintbox-stp.h"
#include "paintbox-utils.h"


/* The IPU uses a 4x4 block when transferring pixels */
#define LB_BLOCK_TRANSFER_HEIGHT 4
#define LB_BLOCK_TRANSFER_WIDTH  4

#if defined(CONFIG_DEBUG_FS) || defined(VERBOSE_DEBUG)

static const char *lbp_reg_names[LBP_NUM_REGS] = {
	REG_NAME_ENTRY(LBP_SEL),
	REG_NAME_ENTRY(LBP_CTRL_L),
	REG_NAME_ENTRY(LBP_CTRL_H),
	REG_NAME_ENTRY(LBP_STAT),
	REG_NAME_ENTRY(LBP_CAP0),
	REG_NAME_ENTRY(LBP_CAP1),
	REG_NAME_ENTRY(LBP_RAM_CTRL),
	REG_NAME_ENTRY(LBP_RAM_DATA0),
	REG_NAME_ENTRY(LBP_RAM_DATA0_H),
	REG_NAME_ENTRY(LBP_RAM_DATA1),
	REG_NAME_ENTRY(LBP_RAM_DATA1_H),
	REG_NAME_ENTRY(LBP_RAM_DATA2),
	REG_NAME_ENTRY(LBP_RAM_DATA2_H),
	REG_NAME_ENTRY(LBP_RAM_DATA3),
	REG_NAME_ENTRY(LBP_RAM_DATA3_H),
	REG_NAME_ENTRY(LB_CTRL0),
	REG_NAME_ENTRY(LB_OFFSET),
	REG_NAME_ENTRY(LB_BDRY),
	REG_NAME_ENTRY(LB_IMG_SIZE),
	REG_NAME_ENTRY(LB_SB_SIZE),
	REG_NAME_ENTRY(LB_BASE),
	REG_NAME_ENTRY(LB_STAT),
	REG_NAME_ENTRY(LB_L_PARAM)
};

static inline void dump_lbp_reg(struct paintbox_data *pb, uint32_t reg,
		char *buf, int *written, size_t len)
{
	const char *reg_name = lbp_reg_names[REG_INDEX(reg)];
	*written += snprintf(buf + *written, len - *written,
			"0x%04lx: %s\t0x%08x\n",
			pb->lbp_base - pb->reg_base + reg,
			reg_name ? reg_name : REG_UNUSED,
			readl(pb->lbp_base + reg));
}

int dump_lbp_registers(struct paintbox_data *pb, char *buf, size_t len)
{
	uint32_t val;
	int written = 0;

	dump_lbp_reg(pb, LBP_SEL, buf, &written, len);

	val = readl(pb->lbp_base + LBP_SEL);
	written += snprintf(buf + written, len - written,
			"\tLB_SEL 0x%02x LBP_SEL 0x%02x\n",
			(val & LBP_LB_SEL_MASK) >> LBP_LB_SEL_SHIFT,
			val & LBP_LBP_SEL_MASK);

	dump_lbp_reg(pb, LBP_CTRL_L, buf, &written, len);

	val = readl(pb->lbp_base + LBP_CTRL_L);
	written += snprintf(buf + written, len - written,
			"\tLB_RESET 0x%04x RESET %d LBP_ENA 0x%02x\n",
			(val & LBP_LB_RESET_MASK) >> LBP_LB_RESET_SHIFT,
			!!(val & LBP_CTRL_RESET), val & LBP_LB_ENA_MASK);

	dump_lbp_reg(pb, LBP_CTRL_H, buf, &written, len);

	val = readl(pb->lbp_base + LBP_CTRL_H);
	written += snprintf(buf + written, len - written, "\tLB_INIT 0x%02x\n",
			val & LBP_LB_INIT_MASK);

	dump_lbp_reg(pb, LBP_STAT, buf, &written, len);
	dump_lbp_reg(pb, LBP_CAP0, buf, &written, len);

	val = readl(pb->lbp_base + LBP_CAP0);
	written += snprintf(buf + written, len - written,
			"\tMAX_FB_ROWS %u MAX_CHAN %u MAX_RPTR %u MAX_LB %u\n",
			(val & LBP_MAX_FB_ROWS_MASK) >> LBP_MAX_FB_ROWS_SHIFT,
			(val & LBP_MAX_CHAN_MASK) >> LBP_MAX_CHAN_SHIFT,
			(val & LBP_MAX_RPTR_MASK) >> LBP_MAX_RPTR_SHIFT,
			val & LBP_MAX_LB_MASK);

	dump_lbp_reg(pb, LBP_CAP1, buf, &written, len);

	val = readl(pb->lbp_base + LBP_CAP1);
	written += snprintf(buf + written, len - written,
			"\tMEM_SIZE %u bytes\n", val);

	dump_lbp_reg(pb, LBP_RAM_CTRL, buf, &written, len);
	dump_lbp_reg(pb, LBP_RAM_DATA0, buf, &written, len);
	dump_lbp_reg(pb, LBP_RAM_DATA0_H, buf, &written, len);
	dump_lbp_reg(pb, LBP_RAM_DATA1, buf, &written, len);
	dump_lbp_reg(pb, LBP_RAM_DATA1_H, buf, &written, len);
	dump_lbp_reg(pb, LBP_RAM_DATA2, buf, &written, len);
	dump_lbp_reg(pb, LBP_RAM_DATA2_H, buf, &written, len);
	dump_lbp_reg(pb, LBP_RAM_DATA3, buf, &written, len);
	dump_lbp_reg(pb, LBP_RAM_DATA3_H, buf, &written, len);

	return written;
}

int dump_lb_registers(struct paintbox_data *pb, char *buf, size_t len)
{
	uint32_t val;
	int written = 0;

	dump_lbp_reg(pb, LB_CTRL0, buf, &written, len);

	val = readl(pb->lbp_base + LB_CTRL0);
	written += snprintf(buf + written, len - written,
			"\tFB_ROWS %u REUSE_ROWS %u RPTRs %u CHANs %u\n",
			(val & LB_FB_ROWS_MASK) >> LB_FB_ROWS_SHIFT,
			(val & LB_REUSE_ROWS_MASK) >> LB_REUSE_ROWS_SHIFT,
			(val & LB_NUM_RPTR_MASK) >> LB_NUM_RPTR_SHIFT,
			val & LB_NUM_CHAN_MASK);

	dump_lbp_reg(pb, LB_OFFSET, buf, &written, len);

	val = readl(pb->lbp_base + LB_OFFSET);
	written += snprintf(buf + written, len - written,
			"\tFB_OFFSET %u OFFSET_CHAN %u OFFSET_X %u OFFSET_Y %u"
			"\n", (val & LB_FB_OFFSET_MASK) >> LB_FB_OFFSET_SHIFT,
			(val & LB_OFFSET_CHAN_MASK) >> LB_OFFSET_CHAN_SHIFT,
			(val & LB_OFFSET_X_MASK) >> LB_OFFSET_X_SHIFT,
			val & LB_OFFSET_Y_MASK);

	dump_lbp_reg(pb, LB_BDRY, buf, &written, len);

	val = readl(pb->lbp_base + LB_BDRY);
	written += snprintf(buf + written, len - written,
			"\tBDRY_VAL 0x%04x BDRY 0x%x\n",
			(val & LB_BDRY_VAL_MASK) >> LB_BDRY_VAL_SHIFT,
			val & LB_BDRY_MASK);

	dump_lbp_reg(pb, LB_IMG_SIZE, buf, &written, len);

	val = readl(pb->lbp_base + LB_IMG_SIZE);
	written += snprintf(buf + written, len - written,
			"\tHEIGHT %upx WIDTH %upx\n",
			(val & LB_IMG_HEIGHT_MASK) >> LB_IMG_HEIGHT_SHIFT,
			val & LB_IMG_WIDTH_MASK);

	dump_lbp_reg(pb, LB_SB_SIZE, buf, &written, len);

	val = readl(pb->lbp_base + LB_SB_SIZE);
	written += snprintf(buf + written, len - written,
			"\tSB_ROWS %u SB_COLS %u\n",
			(val & LB_SB_ROWS_MASK) >> LB_SB_ROWS_SHIFT,
			val & LB_SB_COLS_MASK);

	dump_lbp_reg(pb, LB_BASE, buf, &written, len);

	val = readl(pb->lbp_base + LB_BASE);
	written += snprintf(buf + written, len - written,
			"\tSB_BASE_ADDR 0x%08x FB_BASE_ADDR 0x%08x\n",
			((val & LB_SB_BASE_ADDR_MASK) >>
			LB_SB_BASE_ADDR_SHIFT) << LB_ADDR_ALIGN_SHIFT,
			(val & LB_FB_BASE_ADDR_MASK) << LB_ADDR_ALIGN_SHIFT);

	dump_lbp_reg(pb, LB_STAT, buf, &written, len);

	val = readl(pb->lbp_base + LB_STAT);
	written += snprintf(buf + written, len - written,
			"\tRPTR2 EMPTY %d RPTR1 EMPTY %d RTPR0 EMPTY %d FULL %d\n",
			!!(val & LB_STAT_EMPTY2), !!(val & LB_STAT_EMPTY1),
			!!(val & LB_STAT_EMPTY0), !!(val & LB_STAT_FULL));

	dump_lbp_reg(pb, LB_L_PARAM, buf, &written, len);

	val = readl(pb->lbp_base + LB_L_PARAM);
	written += snprintf(buf + written, len - written,
			"\tL_WIDTH %u L_INC %u\n",
			(val & LB_L_WIDTH_MASK) >> LB_L_WIDTH_SHIFT,
			val & LB_L_INC_MASK);

	return written;
}

#endif

#ifdef VERBOSE_DEBUG
static void log_lbp_registers(struct paintbox_data *pb,
		struct paintbox_lbp *lbp, struct paintbox_lb *lb,
		const char *msg)
{
	int written;

	written = snprintf(pb->vdbg_log, pb->vdbg_log_len, "lbp%u:\n",
			lbp->pool_id);
	dump_lbp_registers(pb, pb->vdbg_log + written,
			pb->vdbg_log_len - written);
	dev_vdbg(&pb->pdev->dev, "%s\n%s", msg, pb->vdbg_log);

	written = snprintf(pb->vdbg_log, pb->vdbg_log_len, "lbp%u lb%u:\n",
			lb->lbp->pool_id, lb->lb_id);
	dump_lb_registers(pb, pb->vdbg_log + written,
			pb->vdbg_log_len - written);
	dev_vdbg(&pb->pdev->dev, pb->vdbg_log);
}

#define LOG_LBP_REGISTERS(pb, lbp, lb)		\
	log_lbp_registers(pb, lbp, lb, __func__)

#else

#define LOG_LBP_REGISTERS(pb, lbp, lb)		\
do { } while (0)
#endif

static int validate_lb_config(struct paintbox_data *pb,
		struct paintbox_lbp *lbp, struct line_buffer_config *lb_config)
{
	/* TODO(ahampson): Need to figure out how the broadcast id will be
	 * expressed.
	 */
	if (lb_config->lb_id < 0 || lb_config->lb_id >= lbp->max_lbs) {
		dev_err(&pb->pdev->dev, "%s: invalid line buffer id %d\n",
				__func__, lb_config->lb_id);
		return -EINVAL;
	}

	if (lb_config->num_reuse_rows > MAX_REUSE_ROWS) {
		dev_err(&pb->pdev->dev,
				"%s: lb%u.%u: invalid reuse rows, %u >= %u\n",
				__func__, lb_config->lb_pool_id,
				lb_config->lb_id, lb_config->num_reuse_rows,
				MAX_REUSE_ROWS);
		return -EINVAL;
	}

	if (lb_config->num_read_ptrs > lbp->max_rptrs) {
		dev_err(&pb->pdev->dev,
				"%s: lb%u.%u: invalid max read ptrs, %u > %u\n",
				__func__, lb_config->lb_pool_id,
				lb_config->lb_id, lb_config->num_read_ptrs,
				lbp->max_rptrs);
		return -EINVAL;
	}

	if (lb_config->fb_rows > lbp->max_fb_rows) {
		dev_err(&pb->pdev->dev,
				"%s: lb%u.%u: invalid fb_rows, %u > %u\n",
				__func__, lb_config->lb_pool_id,
				lb_config->lb_id, lb_config->fb_rows,
				lbp->max_fb_rows);
		return -EINVAL;
	}

	/* TODO(ahampson): this will need to be reevaluated when sliding buffer
	 * support is added.
	 */
	if (lb_config->fb_rows == 0) {
		dev_err(&pb->pdev->dev, "%s: lb%u.%u: invalid fb_rows, %u\n",
				__func__, lb_config->lb_pool_id,
				lb_config->lb_id, lb_config->fb_rows);
		return -EINVAL;
	}

	if (lb_config->chan_offset_pixels > MAX_CHAN_OFFSET) {
		dev_err(&pb->pdev->dev,
				"%s: lb%u.%u: invalid CHAN offset, %u > %u\n",
				__func__, lb_config->lb_pool_id,
				lb_config->lb_id, lb_config->chan_offset_pixels,
				MAX_CHAN_OFFSET);
		return -EINVAL;
	}

	if ((lb_config->ipu_fb_base_addr & LB_ADDR_ALIGN_MASK) != 0) {
		dev_err(&pb->pdev->dev,
				"%s: lb%u.%u: FB base alignment error, 0x%x\n",
				__func__, lb_config->lb_pool_id,
				lb_config->lb_id, lb_config->ipu_fb_base_addr);
		return -EINVAL;
	}

	if ((lb_config->ipu_sb_base_addr & LB_ADDR_ALIGN_MASK) != 0) {
		dev_err(&pb->pdev->dev,
				"%s: lb%u.%u: SB base alignment error, 0x%x\n",
				__func__, lb_config->lb_pool_id,
				lb_config->lb_id, lb_config->ipu_fb_base_addr);
		return -EINVAL;
	}

	dev_dbg(&pb->pdev->dev, "lbp%u.%u width: %u height: %u channels: %u\n",
			lb_config->lb_pool_id, lb_config->lb_id,
			lb_config->width_pixels, lb_config->height_pixels,
			lb_config->num_channels);
	dev_dbg(&pb->pdev->dev,
			"\tfb offset %u x offset: %u y offset: %u chan offset "
			"%u\n", lb_config->fb_offset_pixels,
			lb_config->x_offset_pixels, lb_config->y_offset_pixels,
			lb_config->chan_offset_pixels);
	dev_dbg(&pb->pdev->dev,
			"\rptrs: %u reuse rows: %u fb rows: %u sb rows: %u\n",
			lb_config->num_read_ptrs, lb_config->num_reuse_rows,
			lb_config->fb_rows, lb_config->sb_rows);
	dev_dbg(&pb->pdev->dev, "\fb base addr: 0x%08x sb base addr 0x%08x\n",
			lb_config->ipu_fb_base_addr,
			lb_config->ipu_sb_base_addr);
	dev_dbg(&pb->pdev->dev, "\tpadding method: %u value: %u\n",
			lb_config->padding.method,
			lb_config->padding.value_or_period);

	return 0;
}

/* The caller to this function must hold pb->lock */
int validate_lbp(struct paintbox_data *pb,
		struct paintbox_session *session, int pool_id)
{
	if (pool_id >= pb->caps.num_lbps) {
		dev_err(&pb->pdev->dev, "%s: invalid lb pool id %d\n", __func__,
				pool_id);
		return -EINVAL;
	}

	if (pb->lbps[pool_id].session != session) {
		dev_err(&pb->pdev->dev, "%s: access error, lb pool id %d\n",
				__func__, pool_id);
		return -EACCES;
	}

	return 0;
}

/* The caller to this function must hold pb->lock */
struct paintbox_lbp *get_lbp(struct paintbox_data *pb,
		struct paintbox_session *session, int pool_id, int *err)
{
	int ret = validate_lbp(pb, session, pool_id);
	if (ret < 0) {
		*err = ret;
		return NULL;
	}

	*err = 0;
	return &pb->lbps[pool_id];
}

/* The caller to this function must hold pb->lock */
struct paintbox_lb *get_lb(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned int lbp_id,
		unsigned int lb_id, int *err)
{
	struct paintbox_lbp *lbp;
	struct paintbox_lb *lb;
	int ret;

	lbp = get_lbp(pb, session, lbp_id, &ret);
	if (ret < 0) {
		*err = ret;
		return NULL;
	}

	if (lb_id >= lbp->max_lbs) {
		dev_err(&pb->pdev->dev, "%s: lbp%u invalid lb id %u\n",
				__func__, lbp_id, lb_id);
		*err = -EINVAL;
		return NULL;
	}

	lb = &lbp->lbs[lb_id];

	if (!lb->configured) {
		dev_err(&pb->pdev->dev, "%s: lbp%u lb%u not configured\n",
				__func__, lbp_id, lb_id);
		*err = -EINVAL;
		return NULL;
	}

	*err = 0;
	return lb;
}

/* The caller to this function must hold pb->lock */
void release_lbp(struct paintbox_data *pb,
		struct paintbox_session *session, struct paintbox_lbp *lbp)
{
	/* TODO(ahampson): Figure out LBP cleanup procedures */

	/* TODO(ahampson):  Clear configured bit. */

	list_del(&lbp->entry);
	lbp->session = NULL;
}

int allocate_lbp_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	unsigned int pool_id = (unsigned int)arg;
	struct paintbox_lbp *lbp;

	if (pool_id >= pb->caps.num_lbps) {
		dev_err(&pb->pdev->dev, "%s: invalid lb pool id %d\n", __func__,
				pool_id);
		return -EINVAL;
	}

	mutex_lock(&pb->lock);

	lbp = &pb->lbps[pool_id];
	if (lbp->session) {
		dev_err(&pb->pdev->dev, "%s: access error, lb pool id %d\n",
				__func__, pool_id);
		mutex_unlock(&pb->lock);
		return -EACCES;
	}

	lbp->session = session;
	list_add_tail(&lbp->entry, &session->lbp_list);

	/* Grant access to all STPs in this session. */
	enable_stp_access_to_lbp(pb, session, lbp);

	mutex_unlock(&pb->lock);

	return 0;
}

int release_lbp_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	unsigned int pool_id = (unsigned int)arg;
	struct paintbox_lbp *lbp;
	int ret = 0;

	mutex_lock(&pb->lock);
	lbp = get_lbp(pb, session, pool_id, &ret);
	if (!ret)
		release_lbp(pb, session, lbp);
	mutex_unlock(&pb->lock);

	return ret;
}

int setup_lb_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct line_buffer_config __user *user_lb_config;
	struct line_buffer_config lb_config;
	struct paintbox_lbp *lbp;
	struct paintbox_lb *lb;
	uint32_t val, reset_mask, lb_bdry;
	int ret;

	user_lb_config = (struct line_buffer_config __user *)arg;
	if (copy_from_user(&lb_config, user_lb_config, sizeof(lb_config)))
		return -EFAULT;

	mutex_lock(&pb->lock);
	lbp = get_lbp(pb, session, lb_config.lb_pool_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	lb = &lbp->lbs[lb_config.lb_id];

	ret = validate_lb_config(pb, lbp, &lb_config);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	switch (lb_config.padding.method) {
	case IPU_PADDING_DO_NOT_PAD:
		lb_bdry = LB_BDRY_CLAMP;
		break;
	case IPU_PADDING_CONSTANT:
		lb_bdry = (lb_config.padding.value_or_period <<
				LB_BDRY_VAL_SHIFT) | LB_BDRY_CLAMP;
		break;
	case IPU_PADDING_PERIODIC:
		lb_bdry = (lb_config.padding.value_or_period <<
				LB_BDRY_VAL_SHIFT) | LB_BDRY_REPEAT;
		break;
	case IPU_PADDING_SYMMETRIC:
		lb_bdry = (lb_config.padding.value_or_period <<
				LB_BDRY_VAL_SHIFT) | LB_BDRY_REFLECT;
		break;
	default:
		dev_err(&pb->pdev->dev,
				"%s: lbp%u lb%u: invalid padding method, %d\n",
				__func__, lb_config.lb_pool_id, lb_config.lb_id,
				lb_config.padding.method);
		mutex_unlock(&pb->lock);
		return ret;
	}

	writel(lb_config.lb_pool_id | lb_config.lb_id << LBP_LB_SEL_SHIFT,
			pb->lbp_base + LBP_SEL);

	reset_mask = 1 << lb_config.lb_id << LBP_LB_RESET_SHIFT;

	val = readl(pb->lbp_base + LBP_CTRL_L);
	val &= ~LBP_LB_ENA_MASK;
	val |= lb_config.lb_id + 1;
	val |= reset_mask;
	writel(val, pb->lbp_base + LBP_CTRL_L);

	/* This register is not self clearing, we need to clear the reset bit */
	val &= ~reset_mask;
	writel(val, pb->lbp_base + LBP_CTRL_L);

	writel(lb_config.fb_rows << LB_FB_ROWS_SHIFT |
			lb_config.num_reuse_rows << LB_REUSE_ROWS_SHIFT |
			lb_config.num_read_ptrs << LB_NUM_RPTR_SHIFT |
			lb_config.num_channels, pb->lbp_base + LB_CTRL0);

	lb->fb_rows = lb_config.fb_rows;
	lb->num_read_ptrs = lb_config.num_read_ptrs;
	lb->num_channels = lb_config.num_channels;

	writel(lb_config.fb_offset_pixels << LB_FB_OFFSET_SHIFT |
			lb_config.chan_offset_pixels << LB_OFFSET_CHAN_SHIFT |
			lb_config.x_offset_pixels << LB_OFFSET_X_SHIFT |
			lb_config.y_offset_pixels, pb->lbp_base + LB_OFFSET);

	writel(lb_bdry, pb->lbp_base + LB_BDRY);

	lb->width_pixels = lb_config.width_pixels;
	lb->height_pixels = lb_config.height_pixels;

	writel(lb_config.height_pixels << LB_IMG_HEIGHT_SHIFT |
			lb_config.width_pixels, pb->lbp_base + LB_IMG_SIZE);

	writel(lb_config.sb_rows << LB_SB_ROWS_SHIFT | lb_config.sb_cols,
		pb->lbp_base + LB_SB_SIZE);

	lb->sb_cols = lb_config.sb_cols;
	lb->sb_rows = lb_config.sb_rows;

	val = lb_config.ipu_fb_base_addr >> LB_ADDR_ALIGN_SHIFT;
	val |= (lb_config.ipu_sb_base_addr >> LB_ADDR_ALIGN_SHIFT) <<
			LB_SB_BASE_ADDR_SHIFT;
	writel(val, pb->lbp_base + LB_BASE);

	/* Compute the parameters for the LB_L_PARAM */
	val = (lb->fb_rows / LB_BLOCK_TRANSFER_HEIGHT) *
			(lb->width_pixels / LB_BLOCK_TRANSFER_WIDTH);
	val |= ((lb->fb_rows / LB_BLOCK_TRANSFER_HEIGHT) *
			(lb->width_pixels / LB_BLOCK_TRANSFER_WIDTH) +
			(lb->sb_cols / LB_BLOCK_TRANSFER_HEIGHT)) <<
			LB_L_WIDTH_SHIFT;
	writel(val, pb->lbp_base + LB_L_PARAM);

	/* Initialize the line buffer */
	val = readl(pb->lbp_base + LBP_CTRL_H);
	val |= 1 << lb_config.lb_id;
	writel(val, pb->lbp_base + LBP_CTRL_H);

	/* This register is not self clearing, we need to clear the init bit */
	val &= ~(1 << lb_config.lb_id);
	writel(val, pb->lbp_base + LBP_CTRL_H);

	lb->configured = true;

	LOG_LBP_REGISTERS(pb, lbp, lb);

	mutex_unlock(&pb->lock);

	return 0;
}

int write_lbp_memory_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct ipu_sram_write __user *user_memory_write;
	struct ipu_sram_write memory_write;
	struct paintbox_lbp *lbp;
	uint32_t ram_ctrl_mask = 0;
	size_t sram_transfer_width = LBP_DATA_REG_COUNT * sizeof(uint32_t);
	uint16_t sram_transfer_addr;
	int ret;

	user_memory_write = (struct ipu_sram_write __user *)arg;
	if (copy_from_user(&memory_write, user_memory_write,
			sizeof(memory_write)))
		return -EFAULT;

	mutex_lock(&pb->lock);
	lbp = get_lbp(pb, session, memory_write.id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	writel(memory_write.id | LBP_LBP_SEL_MASK, pb->lbp_base + LBP_SEL);

	/* Verify that the sram_address is word aligned */
	if (memory_write.sram_addr % sram_transfer_width != 0) {
		dev_err(&pb->pdev->dev,
				"%s: lbp%u: memory transfer to unaligned addr, "
				"0x%08x\n", __func__, memory_write.id,
				memory_write.sram_addr);
		mutex_unlock(&pb->lock);
		return -EINVAL;
	}

	sram_transfer_addr = (uint16_t)(memory_write.sram_addr /
			sram_transfer_width);

	if (memory_write.priority)
		ram_ctrl_mask |= LBP_RAM_PRI;

	ret = write_data_common(pb, memory_write.buf, memory_write.len_bytes,
			sram_transfer_addr, ram_ctrl_mask,
			pb->lbp_base + LBP_RAM_CTRL, LBP_DATA_REG_COUNT);
	if (ret < 0)
		dev_err(&pb->pdev->dev,
				"%s: lbp%u: write error addr: 0x%04x "
				"ram_ctrl 0x%08x err = %d\n",
				__func__, memory_write.id,
				memory_write.sram_addr,
				readl(pb->lbp_base + LBP_RAM_CTRL), ret);

	mutex_unlock(&pb->lock);

	return ret;
}

int read_lbp_memory_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct ipu_sram_read __user *user_memory_read;
	struct ipu_sram_read memory_read;
	struct paintbox_lbp *lbp;
	uint32_t ram_ctrl_mask = 0;
	size_t sram_transfer_width = LBP_DATA_REG_COUNT * sizeof(uint32_t);
	uint16_t sram_transfer_addr;
	int ret;

	user_memory_read = (struct ipu_sram_read __user *)arg;
	if (copy_from_user(&memory_read, user_memory_read, sizeof(memory_read)))
		return -EFAULT;

	mutex_lock(&pb->lock);
	lbp = get_lbp(pb, session, memory_read.id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	writel(memory_read.id | LBP_LBP_SEL_MASK, pb->lbp_base + LBP_SEL);

	/* Verify that the sram_address is word aligned */
	if (memory_read.sram_addr % sram_transfer_width != 0) {
		dev_err(&pb->pdev->dev,
				"%s: lbp%u: memory transfer to unaligned addr, "
				"0x%08x\n", __func__, memory_read.id,
				memory_read.sram_addr);
		mutex_unlock(&pb->lock);
		return -EINVAL;
	}

	sram_transfer_addr = (uint16_t)(memory_read.sram_addr /
			sram_transfer_width);

	if (memory_read.priority)
		ram_ctrl_mask |= LBP_RAM_PRI;

	ret = read_data_common(pb, memory_read.buf, memory_read.len_bytes,
			sram_transfer_addr, ram_ctrl_mask,
			pb->lbp_base + LBP_RAM_CTRL, LBP_DATA_REG_COUNT);
	if (ret < 0)
		dev_err(&pb->pdev->dev,
				"%s: lbp%u: read error addr: 0x%04x "
				"ram_ctrl 0x%08x err = %d\n",
				__func__, memory_read.id, memory_read.sram_addr,
				readl(pb->lbp_base + LBP_RAM_CTRL), ret);

	mutex_unlock(&pb->lock);

	return ret;
}

#ifdef CONFIG_DEBUG_FS
static int lb_debug_regs_show(struct seq_file *s, void *unused)
{
	struct paintbox_lb *lb = s->private;
	struct paintbox_lbp *lbp = lb->lbp;
	struct paintbox_data *pb = lbp->pb;
	char *buf;
	size_t len;
	int written;

	len = seq_get_buf(s, &buf);
	if (!buf)
		return -ENOMEM;

	mutex_lock(&pb->lock);

	written = snprintf(buf, len, "lbp%u lb%u:\n", lb->lbp->pool_id,
			lb->lb_id);

	writel(lbp->pool_id | (lb->lb_id << LBP_LB_SEL_SHIFT),
			pb->lbp_base + LBP_SEL);
	written += dump_lb_registers(pb, buf + written, len - written);

	mutex_unlock(&pb->lock);

	seq_commit(s, written);

	return 0;
}

static int lbp_debug_regs_show(struct seq_file *s, void *unused)
{
	struct paintbox_lbp *lbp = s->private;
	struct paintbox_data *pb = lbp->pb;
	char *buf;
	size_t len;
	int written;

	len = seq_get_buf(s, &buf);
	if (!buf)
		return -ENOMEM;

	mutex_lock(&pb->lock);

	written = snprintf(buf, len, "lbp%u:\n", lbp->pool_id);

	writel(lbp->pool_id, pb->lbp_base + LBP_SEL);
	written += dump_lbp_registers(pb, buf + written, len - written);

	mutex_unlock(&pb->lock);

	seq_commit(s, written);

	return 0;
}

static int lb_debug_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, lb_debug_regs_show, inode->i_private);
}

static int lbp_debug_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, lbp_debug_regs_show, inode->i_private);
}

static const struct file_operations lb_debug_regs_fops = {
	.open = lb_debug_regs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations lbp_debug_regs_fops = {
	.open = lbp_debug_regs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static void paintbox_lbp_debug_init(struct paintbox_data *pb,
		struct paintbox_lbp *lbp)
{
	char name[RESOURCE_NAME_LEN];
	unsigned int i;

	snprintf(name, RESOURCE_NAME_LEN, "lbp%u", lbp->pool_id);

	lbp->debug_dir = debugfs_create_dir(name, pb->debug_root);

	lbp->regs_dentry = debugfs_create_file("regs", S_IRUGO | S_IWUSR,
			lbp->debug_dir, lbp, &lbp_debug_regs_fops);

	for (i = 0; i < lbp->max_lbs; i++) {
		struct paintbox_lb *lb = &lbp->lbs[i];

		snprintf(name, RESOURCE_NAME_LEN, "lb%u", i);

		lb->debug_dir = debugfs_create_dir(name, lbp->debug_dir);

		lb->regs_dentry = debugfs_create_file("regs", S_IRUGO | S_IWUSR,
			lb->debug_dir, lb, &lb_debug_regs_fops);
	}
}
#endif

static int init_lbp(struct paintbox_data *pb, unsigned int lbp_index)
{
	struct paintbox_lbp *lbp;
	unsigned int i;
	uint32_t caps;

	lbp = &pb->lbps[lbp_index];

	lbp->pb = pb;

	/* Store pool id with object as a convenience to avoid doing a lookup
	 * later on.
	 */
	lbp->pool_id = lbp_index;

	/* Read LBP/LB specific capabilities */
	writel(lbp_index, pb->lbp_base + LBP_SEL);
	caps = readl(pb->lbp_base + LBP_CAP0);

	lbp->max_lbs = caps & LBP_MAX_LB_MASK;
	lbp->max_rptrs = (caps & LBP_MAX_RPTR_MASK) >> LBP_MAX_RPTR_SHIFT;
	lbp->max_channels = (caps & LBP_MAX_CHAN_MASK) >> LBP_MAX_CHAN_SHIFT;
	lbp->max_fb_rows = (caps & LBP_MAX_FB_ROWS_MASK) >>
			LBP_MAX_FB_ROWS_SHIFT;
	lbp->mem_size = readl(pb->lbp_base + LBP_CAP1);

	lbp->lbs = kzalloc(sizeof(struct paintbox_lb) * lbp->max_lbs,
			GFP_KERNEL);
	if (!lbp->lbs)
		return -ENOMEM;

	for (i = 0; i < lbp->max_lbs; i++) {
		lbp->lbs[i].lbp = lbp;
		lbp->lbs[i].lb_id = i;
	}

#ifdef CONFIG_DEBUG_FS
	paintbox_lbp_debug_init(pb, lbp);
#endif

#ifdef VERBOSE_DEBUG
	paintbox_alloc_debug_buffer(pb, max(LBP_DEBUG_BUFFER_SIZE,
			LB_DEBUG_BUFFER_SIZE));
#endif

	dev_dbg(&pb->pdev->dev, "lbp%u: base %p len %u max lbs%u\n", lbp_index,
			pb->lbp_base, LBP_BLOCK_LEN, lbp->max_lbs);
	dev_dbg(&pb->pdev->dev, "\trptrs %u ch %u fbrows %u size %u\n",
			pb->lbps[lbp_index].max_rptrs,
			pb->lbps[lbp_index].max_channels,
			pb->lbps[lbp_index].max_fb_rows,
			pb->lbps[lbp_index].mem_size);

	return 0;
}

int paintbox_lbp_init(struct paintbox_data *pb)
{
	unsigned int i;
	int ret;

	pb->lbp_base = pb->reg_base + IPU_LBP_OFFSET;

	pb->lbps = kzalloc(sizeof(struct paintbox_lbp) * pb->caps.num_lbps,
			GFP_KERNEL);
	if (!pb->lbps)
		return -ENOMEM;

	for (i = 0; i < pb->caps.num_lbps; i++) {
		ret = init_lbp(pb, i);
		if (ret < 0)
			return ret;
	}

	return 0;
}

void paintbox_lbp_deinit(struct paintbox_data *pb)
{
	unsigned int i;

	for (i = 0; i < pb->caps.num_lbps; i++)
		kfree(pb->lbps[i].lbs);

	kfree(pb->lbps);
}
