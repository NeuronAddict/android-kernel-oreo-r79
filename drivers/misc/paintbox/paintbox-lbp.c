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
#include <linux/delay.h>
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
#include "paintbox-debug.h"
#include "paintbox-lbp.h"
#include "paintbox-power.h"
#include "paintbox-regs.h"
#include "paintbox-regs-supplemental.h"
#include "paintbox-sram.h"
#include "paintbox-stp.h"


/* The IPU uses a 4x4 block when transferring pixels */
#define LB_BLOCK_TRANSFER_HEIGHT 4
#define LB_BLOCK_TRANSFER_WIDTH  4

#ifdef CONFIG_DEBUG_FS
static uint64_t lbp_reg_entry_read(struct paintbox_debug_reg_entry *reg_entry)
{
	struct paintbox_debug *debug = reg_entry->debug;
	struct paintbox_lbp *lbp = container_of(debug, struct paintbox_lbp,
			debug);
	struct paintbox_data *pb = debug->pb;
	uint64_t val;

	mutex_lock(&pb->lock);

	writel(lbp->pool_id, pb->lbp_base + LBP_SEL);
	val = readq(pb->lbp_base + reg_entry->reg_offset);

	mutex_unlock(&pb->lock);

	return val;
}

static void lbp_reg_entry_write(struct paintbox_debug_reg_entry *reg_entry,
		uint64_t val)
{
	struct paintbox_debug *debug = reg_entry->debug;
	struct paintbox_lbp *lbp = container_of(debug, struct paintbox_lbp,
			debug);
	struct paintbox_data *pb = debug->pb;

	mutex_lock(&pb->lock);

	writel(lbp->pool_id, pb->lbp_base + LBP_SEL);
	writeq(val, pb->lbp_base + reg_entry->reg_offset);

	mutex_unlock(&pb->lock);
}

static uint64_t lb_reg_entry_read(struct paintbox_debug_reg_entry *reg_entry)
{
	struct paintbox_debug *debug = reg_entry->debug;
	struct paintbox_lb *lb = container_of(debug, struct paintbox_lb, debug);
	struct paintbox_lbp *lbp = lb->lbp;
	struct paintbox_data *pb = debug->pb;
	uint64_t val;

	mutex_lock(&pb->lock);

	writel(lbp->pool_id | (lb->lb_id << LBP_SEL_LB_SEL_SHIFT),
			pb->lbp_base + LBP_SEL);
	val = readq(pb->lbp_base + LB_BLOCK_START + reg_entry->reg_offset);

	mutex_unlock(&pb->lock);

	return val;
}

static void lb_reg_entry_write(struct paintbox_debug_reg_entry *reg_entry,
		uint64_t val)
{
	struct paintbox_debug *debug = reg_entry->debug;
	struct paintbox_lb *lb = container_of(debug, struct paintbox_lb, debug);
	struct paintbox_lbp *lbp = lb->lbp;
	struct paintbox_data *pb = debug->pb;

	mutex_lock(&pb->lock);

	writel(lbp->pool_id | (lb->lb_id << LBP_SEL_LB_SEL_SHIFT),
			pb->lbp_base + LBP_SEL);
	writeq(val, pb->lbp_base + LB_BLOCK_START + reg_entry->reg_offset);

	mutex_unlock(&pb->lock);
}
#endif

#if defined(CONFIG_DEBUG_FS) || defined(VERBOSE_DEBUG)

static const char *lbp_reg_names[LBP_NUM_REGS] = {
	REG_NAME_ENTRY(LBP_SEL),
	REG_NAME_ENTRY(LBP_CTRL),
	REG_NAME_ENTRY(LBP_STAT),
	REG_NAME_ENTRY(LBP_CAP0),
	REG_NAME_ENTRY(LBP_CAP1),
	REG_NAME_ENTRY(LBP_RAM_CTRL),
	REG_NAME_ENTRY(LBP_RAM_DATA0),
	REG_NAME_ENTRY(LBP_RAM_DATA1),
	REG_NAME_ENTRY(LBP_RAM_DATA2),
	REG_NAME_ENTRY(LBP_RAM_DATA3),
	REG_NAME_ENTRY(LBP_PMON_CFG),
	REG_NAME_ENTRY(LBP_PMON_CNT_0_CFG),
	REG_NAME_ENTRY(LBP_PMON_CNT_0),
	REG_NAME_ENTRY(LBP_PMON_CNT_0_STS),
	REG_NAME_ENTRY(LBP_PMON_CNT_1_CFG),
	REG_NAME_ENTRY(LBP_PMON_CNT_1),
	REG_NAME_ENTRY(LBP_PMON_CNT_1_STS),
	REG_NAME_ENTRY(LB_CTRL0),
	REG_NAME_ENTRY(LB_OFFSET),
	REG_NAME_ENTRY(LB_BDRY),
	REG_NAME_ENTRY(LB_IMG_SIZE),
	REG_NAME_ENTRY(LB_SB_SIZE),
	REG_NAME_ENTRY(LB_BASE),
	REG_NAME_ENTRY(LB_STAT),
	REG_NAME_ENTRY(LB_L_PARAM)
};

static inline int dump_lbp_reg(struct paintbox_data *pb, uint32_t reg_offset,
		char *buf, int *written, size_t len)
{
	const char *reg_name = lbp_reg_names[REG_INDEX(reg_offset)];
	return dump_ipu_register64(pb, pb->lbp_base, reg_offset, reg_name, buf,
			written, len);
}

static int dump_lbp_reg_verbose(struct paintbox_data *pb, uint32_t reg_offset,
		char *buf, int *written, size_t len, const char *format, ...)
{
	va_list args;
	int ret;

	ret = dump_lbp_reg(pb, reg_offset, buf, written, len);
	if (ret < 0)
		return ret;

	va_start(args, format);

	ret = dump_ipu_vprintf(pb, buf, written, len, format, args);

	va_end(args);

	return ret;
}

int dump_lbp_registers(struct paintbox_debug *debug, char *buf, size_t len)
{
	struct paintbox_lbp *lbp = container_of(debug, struct paintbox_lbp,
			debug);
	struct paintbox_data *pb = debug->pb;
	unsigned int reg_offset;
	uint64_t val;
	int ret, written = 0;

	writel(lbp->pool_id, pb->lbp_base + LBP_SEL);

	val = readq(pb->lbp_base + LBP_SEL);
	ret = dump_lbp_reg_verbose(pb, LBP_SEL, buf, &written, len,
			"\tLB_SEL 0x%02x LBP_SEL 0x%02x\n",
			(val & LBP_SEL_LB_SEL_MASK) >> LBP_SEL_LB_SEL_SHIFT,
			val & LBP_SEL_LBP_SEL_MASK);
	if (ret < 0)
		return ret;

	val = readq(pb->lbp_base + LBP_CTRL);
	ret = dump_lbp_reg_verbose(pb, LBP_CTRL, buf, &written, len,
			"\tLB_INIT 0x%02x LB_RESET 0x%04x RESET %d LBP_ENA "
			"0x%02x\n",
			(val & LBP_CTRL_LB_INIT_MASK) >> LBP_CTRL_LB_INIT_SHIFT,
			(val & LBP_CTRL_LB_RESET_MASK) >>
			LBP_CTRL_LB_RESET_SHIFT,
			!!(val & LBP_CTRL_LBP_RESET_MASK),
			val & LBP_CTRL_LB_ENA_MASK);
	if (ret < 0)
		return ret;

	ret = dump_lbp_reg(pb, LBP_STAT, buf, &written, len);
	if (ret < 0)
		return ret;

	val = readq(pb->lbp_base + LBP_CAP0);
	ret = dump_lbp_reg_verbose(pb, LBP_CAP0, buf, &written, len,
			"\tMAX_RPTR %u MAX_FB_ROWS %u MAX_CHAN %u MAX_LB %u\n",
			(val & LBP_CAP0_MAX_RPTR_MASK) >>
			LBP_CAP0_MAX_RPTR_SHIFT,
			(val & LBP_CAP0_MAX_FB_ROWS_MASK) >>
			LBP_CAP0_MAX_FB_ROWS_SHIFT,
			(val & LBP_CAP0_MAX_CHAN_MASK) >>
			LBP_CAP0_MAX_CHAN_SHIFT,
			val & LBP_CAP0_MAX_LB_MASK);
	if (ret < 0)
		return ret;

	val = readq(pb->lbp_base + LBP_CAP1);
	ret = dump_lbp_reg_verbose(pb, LBP_CAP1, buf, &written, len,
			"\tMEM_SIZE %u bytes\n", val);
	if (ret < 0)
		return ret;

	for (reg_offset = LBP_RAM_CTRL; reg_offset < LBP_POOL_BLOCK_LEN;
			reg_offset += IPU_REG_WIDTH) {
		if (!lbp_reg_names[REG_INDEX(reg_offset)])
			continue;

		ret = dump_lbp_reg(pb, reg_offset, buf, &written, len);
		if (ret < 0)
			return ret;
	}

	return written;
}

int dump_lb_registers(struct paintbox_debug *debug, char *buf, size_t len)
{
	struct paintbox_lb *lb = container_of(debug, struct paintbox_lb, debug);
	struct paintbox_lbp *lbp = lb->lbp;
	struct paintbox_data *pb = debug->pb;
	uint64_t val;
	int ret, written = 0;

	writel(lbp->pool_id | (lb->lb_id << LBP_SEL_LB_SEL_SHIFT),
			pb->lbp_base + LBP_SEL);

	val = readq(pb->lbp_base + LB_CTRL0);
	ret = dump_lbp_reg_verbose(pb, LB_CTRL0, buf, &written, len,
			"\tREUSE_ROWS %u FB_ROWS %u CHANs %u RPTRs %u\n",
			(val & LB_CTRL0_REUSE_ROWS_MASK) >>
			LB_CTRL0_REUSE_ROWS_SHIFT,
			(val & LB_CTRL0_FB_ROWS_MASK) >>
			LB_CTRL0_FB_ROWS_SHIFT,
			(val & LB_CTRL0_NUM_CHAN_MASK) >>
			LB_CTRL0_NUM_CHAN_SHIFT,
			val & LB_CTRL0_NUM_RPTR_MASK);
	if (ret < 0)
		return ret;

	val = readq(pb->lbp_base + LB_OFFSET);
	ret = dump_lbp_reg_verbose(pb, LB_OFFSET, buf, &written, len,
			"\tFB_OFFSET %d OFFSET_CHAN %u OFFSET_Y %d OFFSET_X %d"
			"\n",
			(int8_t)((val & LB_OFFSET_FB_OFFSET_MASK) >>
			LB_OFFSET_FB_OFFSET_SHIFT),
			(val & LB_OFFSET_OFFSET_CHAN_MASK) >>
			LB_OFFSET_OFFSET_CHAN_SHIFT,
			(int16_t)((val & LB_OFFSET_OFFSET_Y_MASK) >>
			LB_OFFSET_OFFSET_Y_SHIFT),
			(int16_t)(val & LB_OFFSET_OFFSET_X_MASK));
	if (ret < 0)
		return ret;

	val = readq(pb->lbp_base + LB_BDRY);
	ret = dump_lbp_reg_verbose(pb, LB_BDRY, buf, &written, len,
			"\tBDRY_VAL 0x%04x BDRY 0x%x\n",
			(val & LB_BDRY_BDRY_VAL_MASK) >> LB_BDRY_BDRY_VAL_SHIFT,
			val & LB_BDRY_BDRY_MASK);
	if (ret < 0)
		return ret;

	val = readq(pb->lbp_base + LB_IMG_SIZE);
	ret = dump_lbp_reg_verbose(pb, LB_IMG_SIZE, buf, &written, len,
			"\tHEIGHT %upx WIDTH %upx\n",
			(val & LB_IMG_SIZE_IMG_HEIGHT_MASK) >>
			LB_IMG_SIZE_IMG_HEIGHT_SHIFT,
			val & LB_IMG_SIZE_IMG_WIDTH_MASK);
	if (ret < 0)
		return ret;

	val = readq(pb->lbp_base + LB_SB_SIZE);
	ret = dump_lbp_reg_verbose(pb, LB_SB_SIZE, buf, &written, len,
			"\tSB_ROWS %u SB_COLS %u\n",
			(val & LB_SB_SIZE_SB_ROWS_MASK) >>
			LB_SB_SIZE_SB_ROWS_SHIFT,
			val & LB_SB_SIZE_SB_COLS_MASK);
	if (ret < 0)
		return ret;

	val = readq(pb->lbp_base + LB_BASE);
	ret = dump_lbp_reg_verbose(pb, LB_BASE, buf, &written, len,
			"\tSB_BASE_ADDR 0x%08x FB_BASE_ADDR 0x%08x\n",
			((val & LB_BASE_SB_BASE_ADDR_MASK) >>
			LB_BASE_SB_BASE_ADDR_SHIFT) << LB_ADDR_ALIGN_SHIFT,
			(val & LB_BASE_FB_BASE_ADDR_MASK) <<
			LB_ADDR_ALIGN_SHIFT);
	if (ret < 0)
		return ret;

	val = readq(pb->lbp_base + LB_STAT);
	ret = dump_lbp_reg_verbose(pb, LB_STAT, buf, &written, len,
			"\tRPTR2 EMPTY %d RPTR1 EMPTY %d RTPR0 EMPTY %d FULL %d"
			"\n",
			!!(val & LB_STAT_EMPTY2), !!(val & LB_STAT_EMPTY1),
			!!(val & LB_STAT_EMPTY0), !!(val & LB_STAT_FULL));
	if (ret < 0)
		return ret;

	val = readq(pb->lbp_base + LB_L_PARAM);
	ret = dump_lbp_reg_verbose(pb, LB_L_PARAM, buf, &written, len,
			"\tL_WIDTH %u L_INC %u\n",
			(val & LB_L_PARAM_L_WIDTH_MASK) >>
			LB_L_PARAM_L_WIDTH_SHIFT, val & LB_L_PARAM_L_INC_MASK);
	if (ret < 0)
		return ret;

	return written;
}

#endif

#ifdef VERBOSE_DEBUG
static void log_lbp_registers(struct paintbox_data *pb,
		struct paintbox_lbp *lbp, struct paintbox_lb *lb,
		const char *msg)
{
	writel(lbp->pool_id | lb->lb_id << LBP_SEL_LB_SEL_SHIFT,
			pb->lbp_base + LBP_SEL);

	dump_lbp_registers(&pb->lbps[lbp->pool_id].debug,
			pb->vdbg_log, pb->vdbg_log_len);
	dev_info(&pb->pdev->dev, "%s\nlbp%u:\n%s", msg, lbp->pool_id,
			pb->vdbg_log);

	dump_lb_registers(&pb->lbps[lbp->pool_id].lbs[lb->lb_id].debug,
			pb->vdbg_log, pb->vdbg_log_len);
	dev_info(&pb->pdev->dev, "lbp%u lb%u:\n%s", lb->lbp->pool_id, lb->lb_id,
			pb->vdbg_log);
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
	if (lb_config->lb_id < 0 || lb_config->lb_id >= pb->lbp_caps.max_lbs) {
		dev_err(&pb->pdev->dev, "%s: invalid line buffer id %d\n",
				__func__, lb_config->lb_id);
		return -EINVAL;
	}

	if (lb_config->num_reuse_rows > LB_REUSE_ROWS_MAX) {
		dev_err(&pb->pdev->dev,
				"%s: lb%u.%u: invalid reuse rows, %u >= %u\n",
				__func__, lb_config->lb_pool_id,
				lb_config->lb_id, lb_config->num_reuse_rows,
				LB_REUSE_ROWS_MAX);
		return -EINVAL;
	}

	if (lb_config->num_read_ptrs > pb->lbp_caps.max_rptrs) {
		dev_err(&pb->pdev->dev,
				"%s: lb%u.%u: invalid max read ptrs, %u > %u\n",
				__func__, lb_config->lb_pool_id,
				lb_config->lb_id, lb_config->num_read_ptrs,
				pb->lbp_caps.max_rptrs);
		return -EINVAL;
	}

	if (lb_config->fb_rows > pb->lbp_caps.max_fb_rows) {
		dev_err(&pb->pdev->dev,
				"%s: lb%u.%u: invalid fb_rows, %u > %u\n",
				__func__, lb_config->lb_pool_id,
				lb_config->lb_id, lb_config->fb_rows,
				pb->lbp_caps.max_fb_rows);
		return -EINVAL;
	}

	if (lb_config->x_offset_pixels > LB_OFFSET_MAX ||
			lb_config->x_offset_pixels < LB_OFFSET_MIN) {
		dev_err(&pb->pdev->dev,
				"%s: lbp%u lb%u: x offset out of bounds, %d <= "
				"%d <= %d\n", __func__, lb_config->lb_pool_id,
				lb_config->lb_id, LB_OFFSET_MIN,
				lb_config->x_offset_pixels, LB_OFFSET_MAX);
		return -ERANGE;
	}

	if (lb_config->y_offset_pixels > LB_OFFSET_MAX ||
			lb_config->y_offset_pixels < LB_OFFSET_MIN) {
		dev_err(&pb->pdev->dev,
				"%s: lbp%u lb%u: y offset out of bounds, %d <= "
				"%d <= %d\n", __func__, lb_config->lb_pool_id,
				lb_config->lb_id, LB_OFFSET_MIN,
				lb_config->y_offset_pixels, LB_OFFSET_MAX);
		return -ERANGE;
	}

	if (lb_config->chan_offset_pixels > LB_OFFSET_CHAN_MAX) {
		dev_err(&pb->pdev->dev,
				"%s: lb%u.%u: invalid CHAN offset, %u > %u\n",
				__func__, lb_config->lb_pool_id,
				lb_config->lb_id, lb_config->chan_offset_pixels,
				LB_OFFSET_CHAN_MAX);
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
			"\tfb offset %u x offset: %d y offset: %d chan offset "
			"%d\n", lb_config->fb_offset_pixels,
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

	if (lb_id >= pb->lbp_caps.max_lbs) {
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

/* The caller to this function must hold pb->lock and must have set the LBP_SEL
 * register.
 */
static void reset_line_buffer(struct paintbox_data *pb, struct paintbox_lb *lb)
{
	uint64_t val, reset_mask;

	reset_mask = 1ULL << (lb->lb_id + LBP_CTRL_LB_RESET_SHIFT);

	/* This register is not self clearing, we need to clear the reset bit */
	val = readq(pb->lbp_base + LBP_CTRL);
	val |= reset_mask;
	writeq(val, pb->lbp_base + LBP_CTRL);
	val &= ~reset_mask;
	writeq(val, pb->lbp_base + LBP_CTRL);
}

/* The caller to this function must hold pb->lock */
void reset_lb(struct paintbox_data *pb, unsigned int lbp_id, unsigned int lb_id)
{
	writel(lbp_id | lb_id << LBP_SEL_LB_SEL_SHIFT, pb->lbp_base + LBP_SEL);
	reset_line_buffer(pb, &pb->lbps[lbp_id].lbs[lb_id]);
}

/* The caller to this function must hold pb->lock */
void release_lbp(struct paintbox_data *pb,
		struct paintbox_session *session, struct paintbox_lbp *lbp)
{
	/* The LBP access control masks are not implemented on the V1 hardware.
	 */
#ifndef CONFIG_PAINTBOX_V1
	disable_stp_access_to_lbp(pb, session, lbp);
#endif
	/* Disable all line buffers within the pool. */
	writel(lbp->pool_id, pb->lbp_base + LBP_SEL);
	writeq(0, pb->lbp_base + LBP_CTRL);

#ifndef CONFIG_PAINTBOX_FPGA_SUPPORT
	ipu_pm_lbp_disable(pb, lbp);
#endif

	/* Remove the line buffer pool from the session. */
	list_del(&lbp->session_entry);
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
	list_add_tail(&lbp->session_entry, &session->lbp_list);

#ifndef CONFIG_PAINTBOX_FPGA_SUPPORT
	ipu_pm_lbp_enable(pb, lbp);
#endif

	writel(pool_id, pb->lbp_base + LBP_SEL);
	writeq(LBP_CTRL_LBP_RESET_MASK, pb->lbp_base + LBP_CTRL);
	writeq(0, pb->lbp_base + LBP_CTRL);

	/* The LBP access control masks are not implemented on the V1 hardware.
	 */
#ifndef CONFIG_PAINTBOX_V1
	/* Grant access to all STPs in this session. */
	enable_stp_access_to_lbp(pb, session, lbp);
#endif

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

static int write_l_param_register(struct paintbox_data *pb,
		struct paintbox_lb *lb)
{
	unsigned int width_rounded, sb_cols_rounded;
	uint32_t l_inc, l_width;

	width_rounded = (lb->width_pixels + LB_BLOCK_TRANSFER_WIDTH - 1) /
			LB_BLOCK_TRANSFER_WIDTH;
	sb_cols_rounded =  (lb->sb_cols + LB_BLOCK_TRANSFER_HEIGHT - 1) /
			LB_BLOCK_TRANSFER_HEIGHT;

	/* Linear address increment.
	 * (ROUND_UP4(img_width) / 4
	 */
	l_inc = width_rounded;
	if (l_inc > LB_L_INC_MAX) {
		dev_err(&pb->pdev->dev,
				"%s: lbp%u lb%u: invalid l_inc valid %u "
				"(max %u)\n", __func__, lb->lbp->pool_id,
				lb->lb_id, l_inc, LB_L_INC_MAX);
		return -EINVAL;
	}

	/* Capacity of the linear space in 256 words.
	 * ROUND_UP4(img_width) / 4 + ROUND_UP4(sb_cols) / 4
	 */
	l_width = width_rounded + sb_cols_rounded;
	if (l_width > LB_L_WIDTH_MAX) {
		dev_err(&pb->pdev->dev,
				"%s: lbp%u lb%u: invalid l_width valid %u "
				"(max %u)\n", __func__, lb->lbp->pool_id,
				lb->lb_id, l_width, LB_L_WIDTH_MAX);
		return -EINVAL;
	}

	writel(l_inc | (l_width << LB_L_PARAM_L_WIDTH_SHIFT), pb->lbp_base +
			LB_L_PARAM);

	return 0;
}

int setup_lb_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct line_buffer_config __user *user_lb_config;
	struct line_buffer_config lb_config;
	struct paintbox_lbp *lbp;
	struct paintbox_lb *lb;
	uint64_t ctrl, lb_offset;
	uint32_t lb_bdry, lb_base;
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
				LB_BDRY_BDRY_VAL_SHIFT) | LB_BDRY_CLAMP;
		break;
	case IPU_PADDING_PERIODIC:
		lb_bdry = (lb_config.padding.value_or_period <<
				LB_BDRY_BDRY_VAL_SHIFT) | LB_BDRY_REPEAT;
		break;
	case IPU_PADDING_SYMMETRIC:
		lb_bdry = (lb_config.padding.value_or_period <<
				LB_BDRY_BDRY_VAL_SHIFT) | LB_BDRY_REFLECT;
		break;
	default:
		dev_err(&pb->pdev->dev,
				"%s: lbp%u lb%u: invalid padding method, %d\n",
				__func__, lb_config.lb_pool_id, lb_config.lb_id,
				lb_config.padding.method);
		mutex_unlock(&pb->lock);
		return ret;
	}

	lb->fb_rows = lb_config.fb_rows;
	lb->num_read_ptrs = lb_config.num_read_ptrs;
	lb->num_channels = lb_config.num_channels;
	lb->width_pixels = lb_config.width_pixels;
	lb->height_pixels = lb_config.height_pixels;
	lb->sb_cols = lb_config.sb_cols;
	lb->sb_rows = lb_config.sb_rows;

	writel(lb_config.lb_pool_id | lb_config.lb_id << LBP_SEL_LB_SEL_SHIFT,
			pb->lbp_base + LBP_SEL);

	/* Disable the line buffer before configuring it in case there is an
	 * active configuration.  Setting the ENA count to the line buffer id
	 * will disable this line buffer and leave all the earlier ones
	 * enabled.
	 */
	ctrl = readq(pb->lbp_base + LBP_CTRL);
	ctrl &= ~LBP_CTRL_LB_ENA_MASK;
	ctrl |= lb_config.lb_id;
	writeq(ctrl, pb->lbp_base + LBP_CTRL);

	reset_line_buffer(pb, lb);

	ctrl = lb_config.num_read_ptrs;
	ctrl |= lb_config.num_channels << LB_CTRL0_NUM_CHAN_SHIFT;
	ctrl |= lb_config.fb_rows << LB_CTRL0_FB_ROWS_SHIFT;
	ctrl |= ((uint64_t)lb_config.num_reuse_rows) <<
			LB_CTRL0_REUSE_ROWS_SHIFT;
	writeq(ctrl, pb->lbp_base + LB_CTRL0);

	lb_offset = ((uint64_t)lb_config.x_offset_pixels) &
			LB_OFFSET_OFFSET_X_MASK;
	lb_offset |= (((uint64_t)lb_config.y_offset_pixels) &
			LB_OFFSET_OFFSET_Y_M ) << LB_OFFSET_OFFSET_Y_SHIFT;
	lb_offset |= ((uint64_t)lb_config.chan_offset_pixels) <<
			LB_OFFSET_OFFSET_CHAN_SHIFT;
	lb_offset |= (((uint64_t)lb_config.fb_offset_pixels) &
			LB_OFFSET_FB_OFFSET_M) << LB_OFFSET_FB_OFFSET_SHIFT;
	writeq(lb_offset, pb->lbp_base + LB_OFFSET);

	writel(lb_bdry, pb->lbp_base + LB_BDRY);

	writel(lb_config.height_pixels << LB_IMG_SIZE_IMG_HEIGHT_SHIFT |
			lb_config.width_pixels, pb->lbp_base + LB_IMG_SIZE);

	writel(lb_config.sb_rows << LB_SB_SIZE_SB_ROWS_SHIFT |
			lb_config.sb_cols, pb->lbp_base + LB_SB_SIZE);

	lb_base = lb_config.ipu_fb_base_addr >> LB_ADDR_ALIGN_SHIFT;
	lb_base |= (lb_config.ipu_sb_base_addr >> LB_ADDR_ALIGN_SHIFT) <<
			LB_BASE_SB_BASE_ADDR_SHIFT;
	writel(lb_base, pb->lbp_base + LB_BASE);

	/* LB_L_PARAM L_INC and L_WIDTH are only set in sliding buffer mode. */
	if (lb->sb_rows > 0 || lb->sb_cols > 0) {
		ret = write_l_param_register(pb, lb);
		if (ret < 0) {
			mutex_unlock(&pb->lock);
			return ret;
		}
	} else {
		/* In full buffer mode the L_INC field is set to zero and the
		 * L_WIDTH field is set to the maximum value.
		 */
		writel(LB_L_WIDTH_MAX << LB_L_PARAM_L_WIDTH_SHIFT,
				pb->lbp_base + LB_L_PARAM);
	}

	/* Enable and initialize the line buffer
	 * The init bit is not self clearing, we need to clear the init bit.
	 */
	ctrl = readq(pb->lbp_base + LBP_CTRL);
	ctrl &= ~LBP_CTRL_LB_ENA_MASK;
	ctrl |= lb_config.lb_id + 1;
	ctrl |= 1ULL << (lb_config.lb_id + LBP_CTRL_LB_INIT_SHIFT);
	writeq(ctrl, pb->lbp_base + LBP_CTRL);
	ctrl &= ~(1ULL << (lb_config.lb_id + LBP_CTRL_LB_INIT_SHIFT));
	writeq(ctrl, pb->lbp_base + LBP_CTRL);

	lb->configured = true;

	LOG_LBP_REGISTERS(pb, lbp, lb);

	mutex_unlock(&pb->lock);

	return 0;
}

/* The caller to this function must hold pb->lock */
static int lbp_sram_write_word(struct paintbox_data *pb,
		struct paintbox_sram_config *sram_config, const uint8_t *buf,
		uint32_t ram_ctrl_addr)
{
	unsigned int attempts = 0;

	writel(sram_config->core_id & LBP_SEL_LBP_SEL_MASK, pb->lbp_base +
			LBP_SEL);

	write_ram_data_registers(pb, buf, pb->lbp_base + LBP_RAM_DATA0,
			LBP_DATA_REG_COUNT);

	writel(LBP_RAM_CTRL_RUN_MASK | LBP_RAM_CTRL_WRITE_MASK | ram_ctrl_addr,
			pb->lbp_base + LBP_RAM_CTRL);

	while (readl(pb->lbp_base + LBP_RAM_CTRL) & LBP_RAM_CTRL_RUN_MASK) {
		if (++attempts >= MAX_MEMORY_ACCESS_ATTEMPTS) {
			dev_err(&pb->pdev->dev, "%s: write timeout\n",
					__func__);
			return -ETIMEDOUT;
		}

		usleep_range(MIN_RAM_ACCESS_SLEEP, MAX_RAM_ACCESS_SLEEP);
	}

	return 0;
}

/* The caller to this function must hold pb->lock */
static int lbp_sram_read_word(struct paintbox_data *pb,
		struct paintbox_sram_config *sram_config, uint8_t *buf,
		uint32_t ram_ctrl_addr)
{
	unsigned int attempts = 0;

	writel(sram_config->core_id & LBP_SEL_LBP_SEL_MASK, pb->lbp_base +
			LBP_SEL);

	writel(LBP_RAM_CTRL_RUN_MASK | ram_ctrl_addr, pb->lbp_base +
			LBP_RAM_CTRL);

	while (readl(pb->lbp_base + LBP_RAM_CTRL) & LBP_RAM_CTRL_RUN_MASK) {
		if (++attempts >= MAX_MEMORY_ACCESS_ATTEMPTS) {
			dev_err(&pb->pdev->dev, "%s: read timeout\n", __func__);
			return -ETIMEDOUT;
		}

		usleep_range(MIN_RAM_ACCESS_SLEEP, MAX_RAM_ACCESS_SLEEP);
	}

	read_ram_data_registers(pb, buf, pb->lbp_base + LBP_RAM_DATA0,
			LBP_DATA_REG_COUNT);

	return 0;
}

static void create_lbp_sram_config(struct paintbox_sram_config *sram_config,
		unsigned int lbp_id, bool pad_to_align)
{
	sram_config->core_id = lbp_id;
	sram_config->ram_ctrl_target = 0;
	sram_config->ram_data_mode = RAM_DATA_MODE_NORMAL;
	sram_config->sram_word_bytes = LBP_DATA_REG_COUNT * IPU_REG_WIDTH_BYTES;
	sram_config->write_word = &lbp_sram_write_word;
	sram_config->read_word = &lbp_sram_read_word;
	sram_config->pad_to_align = pad_to_align;
}

static int validate_sram_transfer(struct paintbox_data *pb,
		struct paintbox_lbp *lbp, uint32_t sram_byte_addr,
		size_t len_bytes)
{
	if (sram_byte_addr + len_bytes > pb->lbp_caps.mem_size_bytes) {
		dev_err(&pb->pdev->dev,
				"%s: lbp%u: memory transfer out of range: SRAM "
				"addr 0x%08x + %lu > %u bytes\n",
				__func__, lbp->pool_id, sram_byte_addr,
				len_bytes, pb->lbp_caps.mem_size_bytes);
		return -ERANGE;
	}

	return 0;
}

int write_lbp_memory_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct ipu_sram_write __user *user_req;
	struct ipu_sram_write req;
	struct paintbox_sram_config sram_config;
	struct paintbox_lbp *lbp;
	int ret;

	user_req = (struct ipu_sram_write __user *)arg;
	if (copy_from_user(&req, user_req, sizeof(req)))
		return -EFAULT;

	mutex_lock(&pb->lock);
	lbp = get_lbp(pb, session, req.id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	ret = validate_sram_transfer(pb, lbp, req.sram_byte_addr,
			req.len_bytes);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	create_lbp_sram_config(&sram_config, req.id, req.pad_to_align);

	ret = sram_write_user_buffer(pb, &sram_config, req.sram_byte_addr,
			req.buf, req.len_bytes);
	if (ret < 0)
		dev_err(&pb->pdev->dev,
				"%s: lbp%u: write error addr: 0x%04x "
				"ram_ctrl 0x%016llx err = %d\n",
				__func__, req.id, req.sram_byte_addr,
				readq(pb->lbp_base + LBP_RAM_CTRL), ret);

	mutex_unlock(&pb->lock);

	return ret;
}

int read_lbp_memory_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct ipu_sram_read __user *user_req;
	struct ipu_sram_read req;
	struct paintbox_sram_config sram_config;
	struct paintbox_lbp *lbp;
	int ret;

	user_req = (struct ipu_sram_read __user *)arg;
	if (copy_from_user(&req, user_req, sizeof(req)))
		return -EFAULT;

	mutex_lock(&pb->lock);
	lbp = get_lbp(pb, session, req.id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	ret = validate_sram_transfer(pb, lbp, req.sram_byte_addr,
			req.len_bytes);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	create_lbp_sram_config(&sram_config, req.id, false /* pad_to_align */);

	ret = sram_read_user_buffer(pb, &sram_config, req.sram_byte_addr,
			req.buf, req.len_bytes);
	if (ret < 0)
		dev_err(&pb->pdev->dev,
				"%s: lbp%u: read error addr: 0x%04x "
				"ram_ctrl 0x%016llx err = %d\n", __func__,
				req.id, req.sram_byte_addr,
				readq(pb->lbp_base + LBP_RAM_CTRL), ret);

	mutex_unlock(&pb->lock);

	return ret;
}

int reset_lbp_ioctl(struct paintbox_data *pb, struct paintbox_session *session,
		unsigned long arg)
{
	unsigned int lbp_id = (unsigned int)arg;
	struct paintbox_lbp *lbp;
	uint64_t ctrl;
	int ret;

	mutex_lock(&pb->lock);
	lbp = get_lbp(pb, session, lbp_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	dev_dbg(&pb->pdev->dev, "%s: lbp%u\n",  __func__, lbp->pool_id);

	writel(lbp->pool_id, pb->lbp_base + LBP_SEL);

	ctrl = readq(pb->lbp_base + LBP_CTRL);
	ctrl |= LBP_CTRL_LBP_RESET_MASK;
	writeq(ctrl, pb->lbp_base + LBP_CTRL);
	ctrl &= ~LBP_CTRL_LBP_RESET_MASK;
	writeq(ctrl, pb->lbp_base + LBP_CTRL);

	mutex_unlock(&pb->lock);

	return 0;
}

int reset_lb_ioctl(struct paintbox_data *pb, struct paintbox_session *session,
		unsigned long arg)
{
	struct line_buffer_reset __user *user_req;
	struct line_buffer_reset req;
	struct paintbox_lbp *lbp;
	uint64_t ctrl;
	int ret;

	user_req = (struct line_buffer_reset __user *)arg;
	if (copy_from_user(&req, user_req, sizeof(struct line_buffer_reset)))
		return -EFAULT;

	mutex_lock(&pb->lock);
	lbp = get_lbp(pb, session, req.lbp_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	if (req.lb_id >= pb->lbp_caps.max_lbs) {
		dev_err(&pb->pdev->dev, "%s: lbp%u: invalid lb id %u, max %u\n",
				__func__, req.lbp_id, req.lb_id,
				pb->lbp_caps.max_lbs);
		mutex_unlock(&pb->lock);
		return -EINVAL;
	}

	dev_dbg(&pb->pdev->dev, "%s: lbp%u lb%u\n",  __func__, lbp->pool_id,
			req.lb_id);

	writel(lbp->pool_id, pb->lbp_base + LBP_SEL);

	ctrl = readq(pb->lbp_base + LBP_CTRL);
	ctrl |= 1ULL << req.lb_id << LBP_CTRL_LB_RESET_SHIFT;
	writeq(ctrl, pb->lbp_base + LBP_CTRL);
	ctrl &= ~(1ULL << req.lb_id << LBP_CTRL_LB_RESET_SHIFT);
	writeq(ctrl, pb->lbp_base + LBP_CTRL);

	mutex_unlock(&pb->lock);

	return 0;
}

static int init_lbp(struct paintbox_data *pb, unsigned int lbp_index)
{
	struct paintbox_lbp *lbp;
	unsigned int i;

	lbp = &pb->lbps[lbp_index];

	/* Store pool id with object as a convenience to avoid doing a lookup
	 * later on.
	 */
	lbp->pool_id = lbp_index;

	lbp->lbs = kzalloc(sizeof(struct paintbox_lb) * pb->lbp_caps.max_lbs,
			GFP_KERNEL);
	if (!lbp->lbs)
		return -ENOMEM;

#ifdef CONFIG_DEBUG_FS
	paintbox_debug_create_entry(pb, &lbp->debug, pb->debug_root, "lbp",
			lbp->pool_id, dump_lbp_registers, NULL, lbp);

	paintbox_debug_create_reg_entries(pb, &lbp->debug, lbp_reg_names,
			LBP_POOL_NUM_REGS, lbp_reg_entry_write,
			lbp_reg_entry_read);
#endif

	for (i = 0; i < pb->lbp_caps.max_lbs; i++) {
		struct paintbox_lb *lb = &lbp->lbs[i];
		lb->lbp = lbp;
		lb->lb_id = i;

#ifdef CONFIG_DEBUG_FS
		paintbox_debug_create_entry(pb, &lb->debug,
				lbp->debug.debug_dir, "lb", lb->lb_id,
				dump_lb_registers, NULL, lb);

		paintbox_debug_create_reg_entries(pb, &lb->debug,
				&lbp_reg_names[REG_INDEX(LB_BLOCK_START)],
				LB_NUM_REGS, lb_reg_entry_write,
				lb_reg_entry_read);
#endif
	}

#ifdef VERBOSE_DEBUG
	paintbox_alloc_debug_buffer(pb, max(LBP_DEBUG_BUFFER_SIZE,
			LB_DEBUG_BUFFER_SIZE));
#endif

	return 0;
}

int paintbox_lbp_init(struct paintbox_data *pb)
{
	unsigned int i;
	uint64_t caps;
	int ret;

	pb->lbp_base = pb->reg_base + IPU_LBP_OFFSET;

	pb->caps.num_lbps = (readl(pb->reg_base + IPU_CAP) &
			IPU_CAP_NUM_LBP_MASK) >> IPU_CAP_NUM_LBP_SHIFT;

	pb->lbps = kzalloc(sizeof(struct paintbox_lbp) * pb->caps.num_lbps,
			GFP_KERNEL);
	if (!pb->lbps)
		return -ENOMEM;


	/* Read LBP/LB capabilities from LBP0 since that is always powered.
	 * The capabilities are the same for the other LBPs.
	 */
	writel(0, pb->lbp_base + LBP_SEL);
	caps = readl(pb->lbp_base + LBP_CAP0);

	pb->lbp_caps.max_lbs = caps & LBP_CAP0_MAX_LB_MASK;
	pb->lbp_caps.max_rptrs = (caps & LBP_CAP0_MAX_RPTR_MASK) >>
			LBP_CAP0_MAX_RPTR_SHIFT;
	pb->lbp_caps.max_channels = (caps & LBP_CAP0_MAX_CHAN_MASK) >>
			LBP_CAP0_MAX_CHAN_SHIFT;
	pb->lbp_caps.max_fb_rows = (caps & LBP_CAP0_MAX_FB_ROWS_MASK) >>
			LBP_CAP0_MAX_FB_ROWS_SHIFT;
	pb->lbp_caps.mem_size_bytes = readl(pb->lbp_base + LBP_CAP1);

	for (i = 0; i < pb->caps.num_lbps; i++) {
		ret = init_lbp(pb, i);
		if (ret < 0)
			return ret;
	}

	dev_dbg(&pb->pdev->dev, "lbp: base %p len %lu max lbs%u\n",
			pb->lbp_base, LBP_BLOCK_LEN, pb->lbp_caps.max_lbs);
	dev_dbg(&pb->pdev->dev, "\trptrs %u ch %u fbrows %u size %u bytes\n",
			pb->lbp_caps.max_rptrs,
			pb->lbp_caps.max_channels,
			pb->lbp_caps.max_fb_rows,
			pb->lbp_caps.mem_size_bytes);

	return 0;
}

void paintbox_lbp_deinit(struct paintbox_data *pb)
{
	unsigned int i;

	for (i = 0; i < pb->caps.num_lbps; i++)
		kfree(pb->lbps[i].lbs);

	kfree(pb->lbps);
}
