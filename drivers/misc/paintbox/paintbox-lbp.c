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

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/stddef.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include "paintbox-common.h"
#include "paintbox-debug.h"
#include "paintbox-lbp.h"
#include "paintbox-lbp-debug.h"
#include "paintbox-power.h"
#include "paintbox-regs.h"
#include "paintbox-regs-supplemental.h"
#include "paintbox-sram.h"
#include "paintbox-stp.h"

/* The IPU uses a 4x4 block when transferring pixels */
#define LB_BLOCK_TRANSFER_HEIGHT 4
#define LB_BLOCK_TRANSFER_WIDTH  4

static int validate_lb_config(struct paintbox_data *pb,
		struct paintbox_lbp *lbp, struct line_buffer_config *lb_config)
{
	/* TODO(ahampson): Need to figure out how the broadcast id will be
	 * expressed.
	 */
	if (lb_config->lb_id < 0 || lb_config->lb_id >= pb->lbp.max_lbs) {
		dev_err(&pb->pdev->dev, "%s: invalid line buffer id %d\n",
				__func__, lb_config->lb_id);
		return -EINVAL;
	}

	if (lb_config->num_reuse_rows > LB_REUSE_ROWS_MAX) {
		dev_err(&pb->pdev->dev,
				"%s: lb%u lb%u: invalid reuse rows, %u >= %u\n",
				__func__, lb_config->lb_pool_id,
				lb_config->lb_id, lb_config->num_reuse_rows,
				LB_REUSE_ROWS_MAX);
		return -EINVAL;
	}

	if (lb_config->num_read_ptrs > pb->lbp.max_rptrs) {
		dev_err(&pb->pdev->dev,
				"%s: lb%u lb%u: invalid max read ptrs, %u > %u"
				"\n", __func__, lb_config->lb_pool_id,
				lb_config->lb_id, lb_config->num_read_ptrs,
				pb->lbp.max_rptrs);
		return -EINVAL;
	}

	if (lb_config->fb_rows > pb->lbp.max_fb_rows) {
		dev_err(&pb->pdev->dev,
				"%s: lb%u lb%u invalid fb_rows, %u > %u\n",
				__func__, lb_config->lb_pool_id,
				lb_config->lb_id, lb_config->fb_rows,
				pb->lbp.max_fb_rows);
		return -EINVAL;
	}

	if (lb_config->x_offset_pixels > LB_OFFSET_MAX ||
			lb_config->x_offset_pixels < LB_OFFSET_MIN) {
		dev_err(&pb->pdev->dev,
				"%s: lbp%u lb%u x offset out of bounds, %d <= "
				"%d <= %d\n", __func__, lb_config->lb_pool_id,
				lb_config->lb_id, LB_OFFSET_MIN,
				lb_config->x_offset_pixels, LB_OFFSET_MAX);
		return -ERANGE;
	}

	if (lb_config->y_offset_pixels > LB_OFFSET_MAX ||
			lb_config->y_offset_pixels < LB_OFFSET_MIN) {
		dev_err(&pb->pdev->dev,
				"%s: lbp%u lb%u y offset out of bounds, %d <= "
				"%d <= %d\n", __func__, lb_config->lb_pool_id,
				lb_config->lb_id, LB_OFFSET_MIN,
				lb_config->y_offset_pixels, LB_OFFSET_MAX);
		return -ERANGE;
	}

	if (lb_config->chan_offset_pixels > LB_OFFSET_CHAN_MAX) {
		dev_err(&pb->pdev->dev,
				"%s: lb%u lb%u invalid CHAN offset, %u > %u\n",
				__func__, lb_config->lb_pool_id,
				lb_config->lb_id, lb_config->chan_offset_pixels,
				LB_OFFSET_CHAN_MAX);
		return -EINVAL;
	}

	if ((lb_config->ipu_fb_base_addr & LB_ADDR_ALIGN_MASK) != 0) {
		dev_err(&pb->pdev->dev,
				"%s: lb%u lb%u FB base alignment error, 0x%x\n",
				__func__, lb_config->lb_pool_id,
				lb_config->lb_id, lb_config->ipu_fb_base_addr);
		return -EINVAL;
	}

	if ((lb_config->ipu_sb_base_addr & LB_ADDR_ALIGN_MASK) != 0) {
		dev_err(&pb->pdev->dev,
				"%s: lbp%u lb%u: SB base alignment error, 0x%x"
				"\n", __func__, lb_config->lb_pool_id,
				lb_config->lb_id, lb_config->ipu_fb_base_addr);
		return -EINVAL;
	}

	return 0;
}

/* The caller to this function must hold pb->lock */
int validate_lbp(struct paintbox_data *pb,
		struct paintbox_session *session, int pool_id)
{
	if (pool_id >= pb->lbp.num_lbps) {
		dev_err(&pb->pdev->dev, "%s: invalid lb pool id %d\n", __func__,
				pool_id);
		return -EINVAL;
	}

	if (pb->lbp.lbps[pool_id].session != session) {
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
	return &pb->lbp.lbps[pool_id];
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

	if (lb_id >= pb->lbp.max_lbs) {
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
	val = readq(pb->lbp.reg_base + LBP_CTRL);
	val |= reset_mask;
	writeq(val, pb->lbp.reg_base + LBP_CTRL);
	val &= ~reset_mask;
	writeq(val, pb->lbp.reg_base + LBP_CTRL);
}

/* The caller to this function must hold pb->lock */
void reset_lb(struct paintbox_data *pb, unsigned int lbp_id, unsigned int lb_id)
{
	writel(lbp_id | lb_id << LBP_SEL_LB_SEL_SHIFT, pb->lbp.reg_base +
			LBP_SEL);
	reset_line_buffer(pb, &pb->lbp.lbps[lbp_id].lbs[lb_id]);
}

/* The caller to this function must hold pb->lock */
void release_lbp(struct paintbox_data *pb, struct paintbox_session *session,
		struct paintbox_lbp *lbp)
{
	dev_dbg(&pb->pdev->dev, "lbp%u release\n", lbp->pool_id);

	/* The LBP access control masks are not implemented on the V1 hardware.
	 */
#ifndef CONFIG_PAINTBOX_V1
	disable_stp_access_to_lbp(pb, session, lbp);
#endif
	/* Disable all line buffers within the pool. */
	writel(lbp->pool_id, pb->lbp.reg_base + LBP_SEL);
	writeq(0, pb->lbp.reg_base + LBP_CTRL);

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

	if (pool_id >= pb->lbp.num_lbps) {
		dev_err(&pb->pdev->dev, "%s: invalid lbp id %d\n", __func__,
				pool_id);
		return -EINVAL;
	}

	mutex_lock(&pb->lock);

	lbp = &pb->lbp.lbps[pool_id];
	if (lbp->session) {
		dev_err(&pb->pdev->dev, "%s: access error, lbp id %d\n",
				__func__, pool_id);
		mutex_unlock(&pb->lock);
		return -EACCES;
	}

	lbp->session = session;
	list_add_tail(&lbp->session_entry, &session->lbp_list);

#ifndef CONFIG_PAINTBOX_FPGA_SUPPORT
	ipu_pm_lbp_enable(pb, lbp);
#endif

	writel(pool_id, pb->lbp.reg_base + LBP_SEL);
	writeq(LBP_CTRL_LBP_RESET_MASK, pb->lbp.reg_base + LBP_CTRL);
	writeq(0, pb->lbp.reg_base + LBP_CTRL);

	/* The LBP access control masks are not implemented on the V1 hardware.
	 */
#ifndef CONFIG_PAINTBOX_V1
	/* Grant access to all STPs in this session. */
	enable_stp_access_to_lbp(pb, session, lbp);
#endif

	dev_dbg(&pb->pdev->dev, "lbp%u allocated\n", pool_id);

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
				"%s: lbp%u lb%u invalid l_inc valid %u (max %u)"
				"\n", __func__, lb->lbp->pool_id, lb->lb_id,
				l_inc, LB_L_INC_MAX);
		return -EINVAL;
	}

	/* Capacity of the linear space in 256 words.
	 * ROUND_UP4(img_width) / 4 + ROUND_UP4(sb_cols) / 4
	 */
	l_width = width_rounded + sb_cols_rounded;
	if (l_width > LB_L_WIDTH_MAX) {
		dev_err(&pb->pdev->dev,
				"%s: lbp%u lb%u: invalid l_width valid %u (max "
				"%u)\n", __func__, lb->lbp->pool_id, lb->lb_id,
				l_width, LB_L_WIDTH_MAX);
		return -EINVAL;
	}

	writel(l_inc | (l_width << LB_L_PARAM_L_WIDTH_SHIFT), pb->lbp.reg_base +
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
			pb->lbp.reg_base + LBP_SEL);

	/* Disable the line buffer before configuring it in case there is an
	 * active configuration.  Setting the ENA count to the line buffer id
	 * will disable this line buffer and leave all the earlier ones
	 * enabled.
	 */
	ctrl = readq(pb->lbp.reg_base + LBP_CTRL);
	ctrl &= ~LBP_CTRL_LB_ENA_MASK;
	ctrl |= lb_config.lb_id;
	writeq(ctrl, pb->lbp.reg_base + LBP_CTRL);

	reset_line_buffer(pb, lb);

	ctrl = lb_config.num_read_ptrs;
	ctrl |= lb_config.num_channels << LB_CTRL0_NUM_CHAN_SHIFT;
	ctrl |= lb_config.fb_rows << LB_CTRL0_FB_ROWS_SHIFT;
	ctrl |= ((uint64_t)lb_config.num_reuse_rows) <<
			LB_CTRL0_REUSE_ROWS_SHIFT;
	writeq(ctrl, pb->lbp.reg_base + LB_CTRL0);

	lb_offset = ((uint64_t)lb_config.x_offset_pixels) &
			LB_OFFSET_OFFSET_X_MASK;
	lb_offset |= (((uint64_t)lb_config.y_offset_pixels) &
			LB_OFFSET_OFFSET_Y_M ) << LB_OFFSET_OFFSET_Y_SHIFT;
	lb_offset |= ((uint64_t)lb_config.chan_offset_pixels) <<
			LB_OFFSET_OFFSET_CHAN_SHIFT;
	lb_offset |= (((uint64_t)lb_config.fb_offset_pixels) &
			LB_OFFSET_FB_OFFSET_M) << LB_OFFSET_FB_OFFSET_SHIFT;
	writeq(lb_offset, pb->lbp.reg_base + LB_OFFSET);

	writel(lb_bdry, pb->lbp.reg_base + LB_BDRY);

	writel(lb_config.height_pixels << LB_IMG_SIZE_IMG_HEIGHT_SHIFT |
			lb_config.width_pixels, pb->lbp.reg_base + LB_IMG_SIZE);

	writel(lb_config.sb_rows << LB_SB_SIZE_SB_ROWS_SHIFT |
			lb_config.sb_cols, pb->lbp.reg_base + LB_SB_SIZE);

	lb_base = lb_config.ipu_fb_base_addr >> LB_ADDR_ALIGN_SHIFT;
	lb_base |= (lb_config.ipu_sb_base_addr >> LB_ADDR_ALIGN_SHIFT) <<
			LB_BASE_SB_BASE_ADDR_SHIFT;
	writel(lb_base, pb->lbp.reg_base + LB_BASE);

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
				pb->lbp.reg_base + LB_L_PARAM);
	}

	/* Enable and initialize the line buffer
	 * The init bit is not self clearing, we need to clear the init bit.
	 */
	ctrl = readq(pb->lbp.reg_base + LBP_CTRL);
	ctrl &= ~LBP_CTRL_LB_ENA_MASK;
	ctrl |= lb_config.lb_id + 1;
	ctrl |= 1ULL << (lb_config.lb_id + LBP_CTRL_LB_INIT_SHIFT);
	writeq(ctrl, pb->lbp.reg_base + LBP_CTRL);
	ctrl &= ~(1ULL << (lb_config.lb_id + LBP_CTRL_LB_INIT_SHIFT));
	writeq(ctrl, pb->lbp.reg_base + LBP_CTRL);

	lb->configured = true;

	LOG_LINE_BUFFER_SETUP(pb, &lb_config);

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

	writel(sram_config->core_id & LBP_SEL_LBP_SEL_MASK, pb->lbp.reg_base +
			LBP_SEL);

	write_ram_data_registers(pb, buf, pb->lbp.reg_base + LBP_RAM_DATA0,
			LBP_DATA_REG_COUNT);

	writel(LBP_RAM_CTRL_RUN_MASK | LBP_RAM_CTRL_WRITE_MASK | ram_ctrl_addr,
			pb->lbp.reg_base + LBP_RAM_CTRL);

	while (readl(pb->lbp.reg_base + LBP_RAM_CTRL) & LBP_RAM_CTRL_RUN_MASK) {
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

	writel(sram_config->core_id & LBP_SEL_LBP_SEL_MASK, pb->lbp.reg_base +
			LBP_SEL);

	writel(LBP_RAM_CTRL_RUN_MASK | ram_ctrl_addr, pb->lbp.reg_base +
			LBP_RAM_CTRL);

	while (readl(pb->lbp.reg_base + LBP_RAM_CTRL) & LBP_RAM_CTRL_RUN_MASK) {
		if (++attempts >= MAX_MEMORY_ACCESS_ATTEMPTS) {
			dev_err(&pb->pdev->dev, "%s: read timeout\n", __func__);
			return -ETIMEDOUT;
		}

		usleep_range(MIN_RAM_ACCESS_SLEEP, MAX_RAM_ACCESS_SLEEP);
	}

	read_ram_data_registers(pb, buf, pb->lbp.reg_base + LBP_RAM_DATA0,
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
	if (sram_byte_addr + len_bytes > pb->lbp.mem_size_bytes) {
		dev_err(&pb->pdev->dev,
				"%s: lbp%u memory transfer out of range: SRAM "
				"addr 0x%08x + %lu > %u bytes\n",
				__func__, lbp->pool_id, sram_byte_addr,
				len_bytes, pb->lbp.mem_size_bytes);
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
				"%s: lbp%u write error addr: 0x%04x "
				"ram_ctrl 0x%016llx err = %d\n",
				__func__, req.id, req.sram_byte_addr,
				readq(pb->lbp.reg_base + LBP_RAM_CTRL), ret);

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
				"%s: lbp%u read error addr: 0x%04x "
				"ram_ctrl 0x%016llx err = %d\n", __func__,
				req.id, req.sram_byte_addr,
				readq(pb->lbp.reg_base + LBP_RAM_CTRL), ret);

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

	dev_dbg(&pb->pdev->dev, "lbp%u reset\n", lbp->pool_id);

	writel(lbp->pool_id, pb->lbp.reg_base + LBP_SEL);

	ctrl = readq(pb->lbp.reg_base + LBP_CTRL);
	ctrl |= LBP_CTRL_LBP_RESET_MASK;
	writeq(ctrl, pb->lbp.reg_base + LBP_CTRL);
	ctrl &= ~LBP_CTRL_LBP_RESET_MASK;
	writeq(ctrl, pb->lbp.reg_base + LBP_CTRL);

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

	if (req.lb_id >= pb->lbp.max_lbs) {
		dev_err(&pb->pdev->dev, "%s: lbp%u: invalid lb id %u, max %u\n",
				__func__, req.lbp_id, req.lb_id,
				pb->lbp.max_lbs);
		mutex_unlock(&pb->lock);
		return -EINVAL;
	}

	dev_dbg(&pb->pdev->dev, "lbp%u lb%u reset\n",  lbp->pool_id, req.lb_id);

	writel(lbp->pool_id, pb->lbp.reg_base + LBP_SEL);

	ctrl = readq(pb->lbp.reg_base + LBP_CTRL);
	ctrl |= 1ULL << req.lb_id << LBP_CTRL_LB_RESET_SHIFT;
	writeq(ctrl, pb->lbp.reg_base + LBP_CTRL);
	ctrl &= ~(1ULL << req.lb_id << LBP_CTRL_LB_RESET_SHIFT);
	writeq(ctrl, pb->lbp.reg_base + LBP_CTRL);

	mutex_unlock(&pb->lock);

	return 0;
}

static int init_lbp(struct paintbox_data *pb, unsigned int lbp_index)
{
	struct paintbox_lbp *lbp;
	unsigned int i;

	lbp = &pb->lbp.lbps[lbp_index];

	/* Store pool id with object as a convenience to avoid doing a lookup
	 * later on.
	 */
	lbp->pool_id = lbp_index;

	lbp->lbs = kzalloc(sizeof(struct paintbox_lb) * pb->lbp.max_lbs,
			GFP_KERNEL);
	if (!lbp->lbs)
		return -ENOMEM;

	paintbox_lbp_debug_init(pb, lbp);

	for (i = 0; i < pb->lbp.max_lbs; i++) {
		struct paintbox_lb *lb = &lbp->lbs[i];
		lb->lbp = lbp;
		lb->lb_id = i;

		paintbox_lb_debug_init(pb, lbp, lb);
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

	pb->lbp.lbps = kzalloc(sizeof(struct paintbox_lbp) * pb->lbp.num_lbps,
			GFP_KERNEL);
	if (!pb->lbp.lbps)
		return -ENOMEM;


	/* Read LBP/LB capabilities from LBP0 since that is always powered.
	 * The capabilities are the same for the other LBPs.
	 */
	writel(0, pb->lbp.reg_base + LBP_SEL);
	caps = readl(pb->lbp.reg_base + LBP_CAP0);

	pb->lbp.max_lbs = caps & LBP_CAP0_MAX_LB_MASK;
	pb->lbp.max_rptrs = (caps & LBP_CAP0_MAX_RPTR_MASK) >>
			LBP_CAP0_MAX_RPTR_SHIFT;
	pb->lbp.max_channels = (caps & LBP_CAP0_MAX_CHAN_MASK) >>
			LBP_CAP0_MAX_CHAN_SHIFT;
	pb->lbp.max_fb_rows = (caps & LBP_CAP0_MAX_FB_ROWS_MASK) >>
			LBP_CAP0_MAX_FB_ROWS_SHIFT;
	pb->lbp.mem_size_bytes = readl(pb->lbp.reg_base + LBP_CAP1);

	for (i = 0; i < pb->lbp.num_lbps; i++) {
		ret = init_lbp(pb, i);
		if (ret < 0)
			return ret;
	}

	dev_dbg(&pb->pdev->dev, "lbp: base %p len %lu max lbs%u\n",
			pb->lbp.reg_base, LBP_BLOCK_LEN, pb->lbp.max_lbs);
	dev_dbg(&pb->pdev->dev, "\trptrs %u ch %u fbrows %u size %u bytes\n",
			pb->lbp.max_rptrs, pb->lbp.max_channels,
			pb->lbp.max_fb_rows, pb->lbp.mem_size_bytes);

	return 0;
}

void paintbox_lbp_deinit(struct paintbox_data *pb)
{
	unsigned int i;

	for (i = 0; i < pb->lbp.num_lbps; i++)
		kfree(pb->lbp.lbps[i].lbs);

	kfree(pb->lbp.lbps);
}
