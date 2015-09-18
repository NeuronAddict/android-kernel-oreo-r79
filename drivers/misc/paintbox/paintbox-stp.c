/*
 * STP support for the Paintbox programmable IPU
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
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include "paintbox-common.h"
#include "paintbox-regs.h"
#include "paintbox-stp.h"
#include "paintbox-stp-sim.h"
#include "paintbox-sim-regs.h"
#include "paintbox-utils.h"


static inline unsigned int stp_id_to_index(unsigned int stp_id)
{
	return stp_id - 1;
}

static inline unsigned int stp_index_to_id(unsigned int stp_index)
{
	return stp_index + 1;
}

#if defined(CONFIG_DEBUG_FS) || defined(VERBOSE_DEBUG)

static const char *stp_reg_names[STP_NUM_REGS] = {
	REG_NAME_ENTRY(STP_SEL),
	REG_NAME_ENTRY(STP_CTRL),
	REG_NAME_ENTRY(STP_STAT_L),
	REG_NAME_ENTRY(STP_STAT_H),
	REG_NAME_ENTRY(STP_CAP),
	REG_NAME_ENTRY(STP_RAM_CTRL),
	REG_NAME_ENTRY(STP_RAM_DATA0_L),
	REG_NAME_ENTRY(STP_RAM_DATA0_H),
	REG_NAME_ENTRY(STP_RAM_DATA1_L),
	REG_NAME_ENTRY(STP_RAM_DATA1_H)
};

static inline void dump_stp_reg(struct paintbox_data *pb, uint32_t reg,
		char *buf, int *written, size_t len)
{
	const char *reg_name = stp_reg_names[REG_INDEX(reg)];
	*written += snprintf(buf + *written, len - *written,
			"0x%04lx: %s\t0x%08x\n",
			pb->stp_base - pb->reg_base + reg,
			reg_name ? reg_name : REG_UNUSED,
			readl(pb->stp_base + reg));
}

int dump_stp_registers(struct paintbox_data *pb, char *buf, size_t len)
{
	uint32_t val;
	int written = 0;

	dump_stp_reg(pb, STP_SEL, buf, &written, len);

	written += snprintf(buf + written, len - written,
			"\tSTP_SEL 0x%02x\n",
			readl(pb->stp_base + STP_SEL) & STP_SEL_MASK);

	dump_stp_reg(pb, STP_CTRL, buf, &written, len);

	val = readl(pb->stp_base + STP_CTRL);
	written += snprintf(buf + written, len - written,
			"\tLBP_MASK 0x%04x RESUME %d RESET %d ENA %d\n",
			(val & STP_LBP_MASK_MASK) >> STP_LBP_MASK_SHIFT,
			!!(val & STP_RESUME), !!(val & STP_RESET),
			!!(val & STP_ENA));

	dump_stp_reg(pb, STP_STAT_L, buf, &written, len);

	val = readl(pb->stp_base + STP_STAT_L);
	written += snprintf(buf + written, len - written,
			"\tINT_CODE 0x%04x PC 0x%04x\n",
			(val & STP_INT_CODE_MASK) >> STP_INT_CODE_SHIFT,
			val & STP_PC_MASK);

	dump_stp_reg(pb, STP_STAT_H, buf, &written, len);

	val = readl(pb->stp_base + STP_STAT_H);
	written += snprintf(buf + written, len - written,
			"\tSTALLED %d\n", val & STP_STALLED);

	dump_stp_reg(pb, STP_CAP, buf, &written, len);

	val = readl(pb->stp_base + STP_CAP);
	written += snprintf(buf + written, len - written,
			"\tINST_MEM %u insts\n", val & STP_INST_MEM_MASK);

	dump_stp_reg(pb, STP_RAM_CTRL, buf, &written, len);
	dump_stp_reg(pb, STP_RAM_DATA0_L, buf, &written, len);
	dump_stp_reg(pb, STP_RAM_DATA0_H, buf, &written, len);
	dump_stp_reg(pb, STP_RAM_DATA1_L, buf, &written, len);
	dump_stp_reg(pb, STP_RAM_DATA1_H, buf, &written, len);

	return written;
}
#endif

#ifdef VERBOSE_DEBUG
static void log_stp_registers(struct paintbox_data *pb,
		struct paintbox_stp *stp, const char *msg)
{
	int written;

	written = snprintf(pb->vdbg_log, pb->vdbg_log_len, "stp%u:\n",
			stp->stp_id);
	dump_stp_registers(pb, pb->vdbg_log + written,
			pb->vdbg_log_len - written);
	dev_vdbg(&pb->pdev->dev, "%s\n%s", msg, pb->vdbg_log);
}

#define LOG_STP_REGISTERS(pb, stp)		\
	log_stp_registers(pb, stp, __func__)

#else
#define LOG_STP_REGISTERS(pb, stp)		\
do { } while (0)
#endif

/* The caller to this function must hold pb->lock */
int validate_stp(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned int stp_id)
{
	unsigned int stp_index = stp_id_to_index(stp_id);

	if (stp_index >= pb->caps.num_stps) {
		dev_err(&pb->pdev->dev, "%s: invalid stp_id %d\n", __func__,
				stp_id);
		return -EINVAL;
	}

	if (pb->stps[stp_index].session != session) {
		dev_err(&pb->pdev->dev, "%s: access error stp_id %d\n",
				__func__, stp_id);
		return -EACCES;
	}

	return 0;
}

/* The caller to this function must hold pb->lock */
struct paintbox_stp *get_stp(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned int stp_id, int *err)
{
	int ret = validate_stp(pb, session, stp_id);
	if (ret < 0) {
		*err = ret;
		return NULL;
	}

	*err = 0;
	return &pb->stps[stp_id_to_index(stp_id)];
}

int allocate_stp_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	unsigned int stp_id = (unsigned int)arg;
	unsigned int stp_index = stp_id_to_index(stp_id);
	struct paintbox_lbp *lbp, *lbp_next;
	struct paintbox_stp *stp;
	uint32_t ctrl, lbp_mask;

	if (stp_index >= pb->caps.num_stps) {
		dev_err(&pb->pdev->dev, "%s: invalid stp_id %d\n", __func__,
				stp_id);
		return -EINVAL;
	}

	mutex_lock(&pb->lock);
	stp = &pb->stps[stp_index];
	if (stp->session) {
		dev_err(&pb->pdev->dev, "%s: access error stp_id %d\n",
				__func__, stp_id);
		mutex_unlock(&pb->lock);
		return -EACCES;
	}

	stp->session = session;
	list_add_tail(&stp->entry, &session->stp_list);

	writel(stp_id, pb->stp_base + STP_SEL);

	/* Grant access to all LBPs associated with this session */
	lbp_mask = 0;
	list_for_each_entry_safe(lbp, lbp_next, &session->lbp_list, entry)
		lbp_mask |= 1 << (lbp->pool_id + STP_LBP_MASK_SHIFT);

	ctrl = readl(pb->stp_base + STP_CTRL);
	ctrl &= ~STP_LBP_MASK_MASK;
	ctrl |= lbp_mask;
	writel(ctrl, pb->stp_base + STP_CTRL);

	mutex_unlock(&pb->lock);

	return 0;
}

/* The caller to this function must hold pb->lock */
void release_stp(struct paintbox_data *pb, struct paintbox_session *session,
		struct paintbox_stp *stp)
{
#ifdef CONFIG_PAINTBOX_SIMULATOR_SUPPORT
	int ret;

	/* Make sure the STP is idle before stopping it.  Currently this is only
	 * done for the Simulator.  The FPGA does not have a similar mechanism
	 * How the post-DMA interrupt cleanup on the hardware will work is TBD.
	 */
	ret = sim_wait_for_idle(pb, stp);
	if (ret < 0)
		return;
#endif

	writel(stp->stp_id, pb->stp_base + STP_SEL);
	writel(0, pb->stp_base + STP_CTRL);

	/* TODO(ahampson): We will probably need to poll this register to wait
	 * for the cancel to take effect.
	 */

	list_del(&stp->entry);
	stp->session = NULL;
}

int release_stp_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	unsigned int stp_id = (unsigned int)arg;
	struct paintbox_stp *stp;
	int ret = 0;

	mutex_lock(&pb->lock);
	stp = get_stp(pb, session, stp_id, &ret);
	if (!ret)
		release_stp(pb, session, stp);
	mutex_unlock(&pb->lock);

	return ret;
}

/* This function is used to set the LBP access control mask for an STP on a
 * newly allocated LBP.
 * The caller to this function must hold pb->lock
 */
void enable_stp_access_to_lbp(struct paintbox_data *pb,
		struct paintbox_session *session, struct paintbox_lbp *lbp)
{
	struct paintbox_stp *stp, *stp_next;
	uint32_t ctrl;

	/* Grant access to the LBP to all STPs in the session. */
	list_for_each_entry_safe(stp, stp_next, &session->stp_list, entry) {
		writel(stp->stp_id, pb->stp_base + STP_SEL);
		ctrl = readl(pb->stp_base + STP_CTRL);
		ctrl |= 1 << (lbp->pool_id + STP_LBP_MASK_SHIFT);
		writel(ctrl, pb->stp_base + STP_CTRL);
	}
}

int stp_ctrl_set_and_clear(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned int stp_id,
		uint32_t set_mask)
{
	struct paintbox_stp *stp;
	uint32_t ctrl;
	int ret;

	mutex_lock(&pb->lock);
	stp = get_stp(pb, session, stp_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	writel(stp->stp_id, pb->stp_base + STP_SEL);

	ctrl = readl(pb->stp_base + STP_CTRL);
	ctrl |= set_mask;
	writel(ctrl, pb->stp_base + STP_CTRL);
	ctrl &= ~set_mask;
	writel(ctrl, pb->stp_base + STP_CTRL);

	mutex_unlock(&pb->lock);

	return 0;
}

int start_stp_ioctl(struct paintbox_data *pb, struct paintbox_session *session,
		unsigned long arg)
{
	unsigned int stp_id = (unsigned int)arg;
	struct paintbox_stp *stp;
	uint32_t ctrl, stat;
	int ret;

	mutex_lock(&pb->lock);
	stp = get_stp(pb, session, stp_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	writel(stp->stp_id, pb->stp_base + STP_SEL);

	stat = readl(pb->stp_base + STP_STAT_H);

	dev_dbg(&pb->pdev->dev, "%s: stp%u\n",  __func__, stp_id);

	ctrl = readl(pb->stp_base + STP_CTRL);
	ctrl |= STP_ENA;

	/* If the processor is stalled (due to a stall.b16 <- src8) instruction
	 * then set and clear the RESUME bit.
	 */
	if (stat & STP_STALLED)
		ctrl |= STP_RESUME;

	writel(ctrl, pb->stp_base + STP_CTRL);

	/* The RESUME bit is not self clearing. */
	if (stat & STP_STALLED) {
		ctrl &= ~STP_RESUME;
		writel(ctrl, pb->stp_base + STP_CTRL);
	}

	mutex_unlock(&pb->lock);

	return 0;
}

int stop_stp_ioctl(struct paintbox_data *pb, struct paintbox_session *session,
		unsigned long arg)
{
	unsigned int stp_id = (unsigned int)arg;
	struct paintbox_stp *stp;
	uint32_t ctrl;
	int ret;

	mutex_lock(&pb->lock);
	stp = get_stp(pb, session, stp_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	dev_dbg(&pb->pdev->dev, "%s: stp%u\n",  __func__, stp->stp_id);

	/* Make sure the STP is idle before stopping it.  Currently this is only
	 * done for the Simulator.  The FPGA does not have a similar mechanism
	 * How the post-DMA interrupt cleanup on the hardware will work is TBD.
	 */
#ifdef CONFIG_PAINTBOX_SIMULATOR_SUPPORT
	ret = sim_wait_for_idle(pb, stp);
	if (ret < 0)
		return ret;
#endif

	writel(stp->stp_id, pb->stp_base + STP_SEL);

	ctrl = readl(pb->stp_base + STP_CTRL);
	ctrl &= ~STP_ENA;
	writel(ctrl, pb->stp_base + STP_CTRL);

	LOG_STP_REGISTERS(pb, stp);

	mutex_unlock(&pb->lock);

	return ret;
}

int resume_stp_ioctl(struct paintbox_data *pb, struct paintbox_session *session,
		unsigned long arg)
{
	unsigned int stp_id = (unsigned int)arg;
	dev_dbg(&pb->pdev->dev, "%s: stp%u\n",  __func__, stp_id);
	return stp_ctrl_set_and_clear(pb, session, stp_id, STP_RESUME);
}

int reset_stp_ioctl(struct paintbox_data *pb, struct paintbox_session *session,
		unsigned long arg)
{
	unsigned int stp_id = (unsigned int)arg;
	dev_dbg(&pb->pdev->dev, "%s: stp%u\n",  __func__, stp_id);
	return stp_ctrl_set_and_clear(pb, session, stp_id, STP_RESET);
}

int setup_stp_ioctl(struct paintbox_data *pb, struct paintbox_session *session,
		unsigned long arg)
{
	struct stp_config __user *user_config;
	struct stp_config config;
	struct paintbox_stp *stp;
	uint32_t ctrl;
	int ret;

	user_config = (struct stp_config __user *)arg;
	if (copy_from_user(&config, user_config, sizeof(config)))
		return -EFAULT;

	mutex_lock(&pb->lock);
	stp = get_stp(pb, session, config.processor_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	/* Make sure the STP is idle before stopping it.  Currently this is only
	 * done for the Simulator.  The FPGA does not have a similar mechanism
	 * How the post-DMA interrupt cleanup on the hardware will work is TBD.
	 */
#ifdef CONFIG_PAINTBOX_SIMULATOR_SUPPORT
	ret = sim_wait_for_idle(pb, stp);
	if (ret < 0)
		return ret;
#endif

	writel(stp->stp_id, pb->stp_base + STP_SEL);

	/* Stop and reset the processor */
	ctrl = readl(pb->stp_base + STP_CTRL);
	ctrl &= ~STP_ENA;
	writel(STP_RESET, pb->stp_base + STP_CTRL);
	writel(ctrl, pb->stp_base + STP_CTRL);

	ret = write_data_common(pb, config.buf, config.len, 0,
			STP_RAM_TARG_INST_RAM << STP_RAM_TARG_SHIFT,
			pb->stp_base + STP_RAM_CTRL, STP_DATA_REG_COUNT);
	if (ret < 0)
		dev_err(&pb->pdev->dev,
				"%s: stp%u: setup stp failed, ret = %d\n",
				__func__, config.processor_id, ret);

	mutex_unlock(&pb->lock);

	return ret;
}

static int set_ram_target(uint32_t *ram_ctrl, uint8_t ram_target)
{
	switch (ram_target) {
	case STP_RAM_TARGET_INSTRUCTION_RAM:
		*ram_ctrl |= (STP_RAM_TARG_INST_RAM << STP_RAM_TARG_SHIFT);
		break;
	case STP_RAM_TARGET_CONSTANT_RAM:
		*ram_ctrl |= (STP_RAM_TARG_CNST_RAM << STP_RAM_TARG_SHIFT);
		break;
	case STP_RAM_TARGET_SCALAR_RAM:
		*ram_ctrl |= (STP_RAM_TARG_DATA_RAM << STP_RAM_TARG_SHIFT);
		break;
	default:
		return -EINVAL;
	};

	return 0;
}

static void set_vector_address(uint32_t *ram_ctrl, uint8_t bank_x,
		uint8_t bank_y, uint8_t offset_in_bank)
{
	/* RAM_ADDR field is composed of the bank y coordinate and the bank
	 * index
	 */
	*ram_ctrl |= ((bank_y << STP_RAM_ADDR_ROW_SHIFT) | offset_in_bank) <<
			STP_RAM_ADDR_SHIFT;

	/* ALU IO channel is the bank x coordinate */
	/* TODO(ahampson):  Add support for ALU Register addressing
	 * b/28729714
	 */
	*ram_ctrl |= (bank_x + STP_RAM_TARG_ALU_IO_RAM_0) << STP_RAM_TARG_SHIFT;
}

/* TODO(ahampson):  This information should be available in the future through
 * capabilities registers.
 */
#define VECTOR_LANE_WIDTH   2
#define VECTOR_BANK_WIDTH   4 /* lanes */
#define VECTOR_BANK_HEIGHT  2 /* lanes */

#define VECTOR_SIMD_WIDTH  16 /* lanes */
#define VECTOR_SIMD_HEIGHT 16 /* lanes */
#define VECTOR_HALO_WIDTH   4 /* lanes */
#define VECTOR_HALO_HEIGHT  4 /* lanes */

#define VECTOR_MEMORY_WIDTH (VECTOR_SIMD_WIDTH + VECTOR_HALO_WIDTH)
#define VECTOR_MEMORY_HEIGHT (VECTOR_SIMD_HEIGHT + VECTOR_HALO_HEIGHT)
#define VECTOR_BANK_SIZE (VECTOR_BANK_WIDTH * VECTOR_BANK_HEIGHT * \
		VECTOR_LANE_WIDTH)

/* The caller to this function must hold pb->lock
 * The caller must also set STP_SEL to the correct STP ID.
 */
static int read_stp_vector_bank(struct paintbox_data *pb,
		struct paintbox_stp *stp, uint8_t bank_x, uint8_t bank_y,
		uint8_t offset_in_bank, uint8_t *buf, bool priority)
{
	unsigned int reg_index, row0_col, row1_col;
	uint32_t ram_ctrl = STP_RAM_RUN;
	int ret;

	if (priority)
		ram_ctrl |= STP_RAM_PRI;

	set_vector_address(&ram_ctrl, bank_x, bank_y, offset_in_bank);

	writel(ram_ctrl, pb->stp_base + STP_RAM_CTRL);

	ret = poll_memory_transfer_complete(pb->stp_base + STP_RAM_CTRL);
	if (ret < 0) {
		dev_err(&pb->pdev->dev, "%s: read timeout\n", __func__);
		return ret;
	}

	/* Each vector bank is a 4 x 2 array of ALU lanes (lanes are 16 bits).
	 * The data provided to the HAL to be written into the vector bank is a
	 * uint8_t* buffer with the uint16_t lane values in row major order:
	 *
	 * 0                   8                15
	 * R0C0 ROC1 R0C2 R0C3 R1C0 R1C1 R1C2 R1C3
	 *
	 * The hardware on the other hand is column major (wiring optimization)
	 * and requires the data to be loaded into the data registers in this
	 * fashion:
	 *
	 * DATA_1_H   DATA_1_L   DATA_0_H   DATA_0_L
	 * R1C3 R0C3  R1C2 R0C2  R1C1 R0C1  R1C0 R0C0
	 */
	row0_col = 0;
	row1_col = VECTOR_BANK_WIDTH * VECTOR_LANE_WIDTH;

	for (reg_index = 0; reg_index < STP_DATA_REG_COUNT; reg_index++,
			row0_col += VECTOR_LANE_WIDTH,
			row1_col += VECTOR_LANE_WIDTH) {
		uint32_t data = readl(pb->stp_base + STP_RAM_DATA0_L +
				reg_index * sizeof(uint32_t));

		buf[row0_col] = (uint8_t)((data & 0xFF000000) >> 24);
		buf[row0_col + 1] = (uint8_t)((data & 0x00FF0000) >> 16);
		buf[row1_col] = (uint8_t)((data & 0x0000FF00) >> 8);
		buf[row1_col + 1] = (uint8_t)(data & 0x000000FF);
	}

	return 0;
}

/* The caller to this function must hold pb->lock
 * The caller must also set STP_SEL to the correct STP ID.
 */
static int write_stp_vector_bank(struct paintbox_data *pb,
		struct paintbox_stp *stp, uint8_t bank_x, uint8_t bank_y,
		uint8_t offset_in_bank, const uint8_t *buf, bool priority)
{
	unsigned int reg_index, row0_col, row1_col;
	uint32_t ram_ctrl = STP_RAM_RUN | STP_RAM_WRITE;
	int ret;

	if (priority)
		ram_ctrl |= STP_RAM_PRI;

	set_vector_address(&ram_ctrl, bank_x, bank_y, offset_in_bank);

	/* Each vector bank is a 4 x 2 array of ALU lanes (lanes are 16 bits).
	 * The data provided to the HAL to be written into the vector bank is a
	 * uint8_t* buffer with the uint16_t lane values in row major order:
	 *
	 * 0                   8                15
	 * R0C0 ROC1 R0C2 R0C3 R1C0 R1C1 R1C2 R1C3
	 *
	 * The hardware on the other hand is column major (wiring optimization)
	 * and requires the data to be loaded into the data registers in this
	 * fashion:
	 *
	 * DATA_1_H   DATA_1_L   DATA_0_H   DATA_0_L
	 * R1C3 R0C3  R1C2 R0C2  R1C1 R0C1  R1C0 R0C0
	 */
	row0_col = 0;
	row1_col = VECTOR_BANK_WIDTH * VECTOR_LANE_WIDTH;

	for (reg_index = 0; reg_index < STP_DATA_REG_COUNT; reg_index++,
			row0_col += VECTOR_LANE_WIDTH,
			row1_col += VECTOR_LANE_WIDTH) {
		uint32_t data;

		data = ((uint32_t)buf[row0_col]) << 24;
		data |= ((uint32_t)buf[row0_col + 1]) << 16;
		data |= ((uint32_t)buf[row1_col]) << 8;
		data |= (uint32_t)buf[row1_col + 1];

		writel(data, pb->stp_base + STP_RAM_DATA0_L + reg_index *
				sizeof(uint32_t));
	}

	writel(ram_ctrl, pb->stp_base + STP_RAM_CTRL);

	ret = poll_memory_transfer_complete(pb->stp_base + STP_RAM_CTRL);
	if (ret < 0)
		dev_err(&pb->pdev->dev, "%s: write timeout\n", __func__);

	return 0;
}

/* The caller to this function must hold pb->lock
 * The caller must also set STP_SEL to the correct STP ID.
 */
static int write_stp_vector_bank_short(struct paintbox_data *pb,
		struct paintbox_stp *stp, uint8_t bank_x, uint8_t bank_y,
		uint8_t offset_in_bank, const uint8_t *buf, size_t len,
		bool priority)
{
	uint8_t temp_buf[VECTOR_BANK_SIZE];
	int ret;

	if (len > VECTOR_BANK_SIZE)
		return -ERANGE;

	ret = read_stp_vector_bank(pb, stp, bank_x, bank_y,
			offset_in_bank, temp_buf, priority);
	if (ret < 0)
		return ret;

	memcpy(temp_buf, buf, len);

	return write_stp_vector_bank(pb, stp, bank_x, bank_y, offset_in_bank,
			temp_buf, priority);
}

static int validate_stp_vector_write_memory_parameters(struct paintbox_data *pb,
		struct ipu_sram_vector_write *req)
{
	/* Multi-bank operations are not supported right now. */
	if (req->len_bytes > STP_DATA_REG_COUNT * sizeof(uint32_t)) {
		dev_err(&pb->pdev->dev,
				"%s: sub-bank memory operations are not "
				"supported\n", __func__);
		return -EINVAL;
	}

	if (req->lane_x_start % VECTOR_BANK_WIDTH != 0 ||
			req->lane_y_start % VECTOR_BANK_HEIGHT != 0) {
		dev_err(&pb->pdev->dev,
				"%s: vector memory operations must start on a "
				"bank boundary x %u y %u\n", __func__,
				req->lane_x_start, req->lane_y_start);
		return -EINVAL;
	}

	if (req->lane_x_end % VECTOR_BANK_WIDTH != VECTOR_BANK_WIDTH - 1 ||
			req->lane_y_end % VECTOR_BANK_HEIGHT !=
			VECTOR_BANK_HEIGHT - 1) {
		dev_err(&pb->pdev->dev,
				"%s: vector memory operations must end on a "
				"bank boundary x %u y %u\n", __func__,
				req->lane_x_end, req->lane_y_end);
		return -EINVAL;
	}

	if (req->lane_x_start > req->lane_x_end ||
			req->lane_y_start > req->lane_y_end) {
		dev_err(&pb->pdev->dev,
				"%s: invalid vector bounding box\n", __func__);
		return -EINVAL;
	}

	/* TODO (ahampson): Need vector memory width and height from CAPS */
	if (req->lane_x_start > VECTOR_MEMORY_WIDTH - 1 ||
			req->lane_y_start > VECTOR_MEMORY_HEIGHT - 1 ||
			req->lane_x_end > VECTOR_MEMORY_WIDTH - 1 ||
			req->lane_y_end > VECTOR_MEMORY_HEIGHT - 1) {
		dev_err(&pb->pdev->dev,
				"%s: vector memory operation out of range\n",
				__func__);
		return -ERANGE;
	}

	/* TODO(ahampson): Need to validate the bank index based on whether this
	 * is a halo lane or a normal lane.
	 */

	return 0;
}

static int validate_stp_vector_read_memory_parameters(struct paintbox_data *pb,
		struct ipu_sram_vector_read *req)
{
	/* Multi-bank operations are not supported right now. */
	if (req->len_bytes > STP_DATA_REG_COUNT * sizeof(uint32_t)) {
		dev_err(&pb->pdev->dev,
				"%s: sub-bank memory operations are not "
				"supported\n", __func__);
		return -EINVAL;
	}

	if (req->lane_x % VECTOR_BANK_WIDTH != 0 ||
			req->lane_y % VECTOR_BANK_HEIGHT != 0) {
		dev_err(&pb->pdev->dev,
				"%s: vector memory operations must start on a "
				"bank boundary x %u y %u\n", __func__,
				req->lane_x, req->lane_y);
		return -EINVAL;
	}

	/* TODO (ahampson): Need vector memory width and height from CAPS */
	if (req->lane_x > VECTOR_MEMORY_WIDTH - 1 ||
			req->lane_y > VECTOR_MEMORY_HEIGHT - 1) {
		dev_err(&pb->pdev->dev,
				"%s: vector memory operation out of range\n",
				__func__);
		return -ERANGE;
	}

	/* TODO(ahampson): Need to validate the bank index based on whether this
	 * is a halo lane or a normal lane.
	 */

	return 0;
}


static const uint8_t lane_logical_x_to_bank_x_map[VECTOR_MEMORY_WIDTH] = {
	1, 1, 1, 1,  /*  0..3  */
	3, 3, 3, 3,  /*  4..7  */
	4, 4, 4, 4,  /*  8..11 */
	2, 2, 2, 2,  /* 12..15 */
	0, 0, 0, 0   /* 16..19 */
};

static const uint8_t lane_logical_y_to_bank_y_map[VECTOR_MEMORY_HEIGHT] = {
	7, 7, 6, 6,  /*  0..3  */
	3, 3, 2, 2,  /*  4..7  */
	0, 0, 1, 1,  /*  8..11 */
	4, 4, 5, 5,  /* 12..15 */
	8, 8, 9, 9   /* 16..19 */
};

int write_stp_vector_memory_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct ipu_sram_vector_write __user *user_req;
	struct ipu_sram_vector_write req;
	struct paintbox_stp *stp;
	uint8_t bank_x, bank_y, x, y;
	uint8_t *buf = NULL;
	int ret;

	user_req = (struct ipu_sram_vector_write __user *)arg;
	if (copy_from_user(&req, user_req, sizeof(req)))
		return -EFAULT;

	mutex_lock(&pb->lock);
	stp = get_stp(pb, session, req.id, &ret);
	if (ret < 0)
		goto exit;

	writel(stp->stp_id, pb->stp_base + STP_SEL);

	ret = validate_stp_vector_write_memory_parameters(pb, &req);
	if (ret < 0)
		goto exit;

	ret = alloc_and_copy_from_user(pb, &buf, req.buf, req.len_bytes);
	if (ret < 0)
		goto exit;

	/* Iterate over all banks in the specified region and write the supplied
	 * user buffer to those banks.
	 */
	for (x = req.lane_x_start; x <= req.lane_x_end;
			x += VECTOR_BANK_WIDTH) {
		for (y = req.lane_y_start; y <= req.lane_y_end;
				y += VECTOR_BANK_HEIGHT) {
			bank_x = lane_logical_x_to_bank_x_map[x];
			bank_y = lane_logical_y_to_bank_y_map[y];

			if (req.len_bytes < VECTOR_BANK_SIZE)
				ret = write_stp_vector_bank_short(pb, stp,
					bank_x, bank_y, req.offset_in_bank, buf,
					req.len_bytes, req.priority);
			else
				ret = write_stp_vector_bank(pb, stp, bank_x,
					bank_y, req.offset_in_bank, buf,
					req.priority);
			if (ret < 0)
				goto exit;
		}
	}

exit:
	mutex_unlock(&pb->lock);

	kfree(buf);

	return ret;
}

int read_stp_vector_memory_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct ipu_sram_vector_read __user *user_req;
	struct ipu_sram_vector_read req;
	struct paintbox_stp *stp;
	uint8_t *buf = NULL;
	uint8_t bank_x, bank_y;
	int ret;

	user_req = (struct ipu_sram_vector_read __user *)arg;
	if (copy_from_user(&req, user_req, sizeof(req)))
		return -EFAULT;

	mutex_lock(&pb->lock);
	stp = get_stp(pb, session, req.id, &ret);
	if (ret < 0)
		goto exit;

	writel(stp->stp_id, pb->stp_base + STP_SEL);

	ret = validate_stp_vector_read_memory_parameters(pb, &req);
	if (ret < 0)
		goto exit;

	buf = kmalloc(max(req.len_bytes, (size_t)VECTOR_BANK_SIZE), GFP_KERNEL);
	if (!buf) {
		dev_err(&pb->pdev->dev, "%s: allocation failure\n", __func__);
		goto exit;
	}

	bank_x = lane_logical_x_to_bank_x_map[req.lane_x];
	bank_y = lane_logical_y_to_bank_y_map[req.lane_y];

	ret = read_stp_vector_bank(pb, stp, bank_x, bank_y, req.offset_in_bank,
			buf, req.priority);
	if (ret < 0)
		goto exit;

	if (copy_to_user((void __user *)req.buf, buf, min(req.len_bytes,
			(size_t)VECTOR_BANK_SIZE)))
		ret = -EFAULT;
exit:
	mutex_unlock(&pb->lock);

	kfree(buf);

	return ret;
}

int write_stp_memory_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct ipu_sram_write __user *user_memory_write;
	struct ipu_sram_write memory_write;
	struct paintbox_stp *stp;
	uint32_t ram_ctrl_mask = 0;
	size_t sram_transfer_width = STP_DATA_REG_COUNT * sizeof(uint32_t);
	uint16_t sram_transfer_addr;
	int ret;

	user_memory_write = (struct ipu_sram_write __user *)arg;
	if (copy_from_user(&memory_write, user_memory_write,
			sizeof(memory_write)))
		return -EFAULT;

	mutex_lock(&pb->lock);
	stp = get_stp(pb, session, memory_write.id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	writel(stp->stp_id, pb->stp_base + STP_SEL);

	/* Verify that the sram_address is aligned on the RAM access registers
	 * address alignment.
	 */
	if (memory_write.sram_addr % sram_transfer_width != 0) {
		dev_err(&pb->pdev->dev,
				"%s: stp%u: memory transfer to unaligned addr, "
				"0x%08x\n", __func__, memory_write.id,
				memory_write.sram_addr);
		mutex_unlock(&pb->lock);
		return -EINVAL;
	}

	sram_transfer_addr = (uint16_t)(memory_write.sram_addr /
			sram_transfer_width);

	if (memory_write.priority)
		ram_ctrl_mask |= STP_RAM_PRI;

	ret = set_ram_target(&ram_ctrl_mask, memory_write.ram_target);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		dev_err(&pb->pdev->dev,
				"%s: stp%u: invalid ram target: 0x%04x "
				"err = %d\n",
				__func__, memory_write.id,
				memory_write.ram_target, ret);
		return ret;
	}

	ret = write_data_common(pb, memory_write.buf, memory_write.len_bytes,
			sram_transfer_addr, ram_ctrl_mask,
			pb->stp_base + STP_RAM_CTRL, STP_DATA_REG_COUNT);
	if (ret < 0)
		dev_err(&pb->pdev->dev,
				"%s: stp%u: write error addr: 0x%04x "
				"type: 0x%02x ram_ctrl 0x%08x err = %d\n",
				__func__, memory_write.id,
				memory_write.sram_addr, memory_write.ram_target,
				readl(pb->stp_base + STP_RAM_CTRL), ret);

	mutex_unlock(&pb->lock);

	return ret;
}

int read_stp_memory_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct ipu_sram_read __user *user_memory_read;
	struct ipu_sram_read memory_read;
	struct paintbox_stp *stp;
	uint32_t ram_ctrl_mask = 0;
	size_t sram_transfer_width = STP_DATA_REG_COUNT * sizeof(uint32_t);
	uint16_t sram_transfer_addr;
	int ret;

	user_memory_read = (struct ipu_sram_read __user *)arg;
	if (copy_from_user(&memory_read, user_memory_read, sizeof(memory_read)))
		return -EFAULT;

	mutex_lock(&pb->lock);
	stp = get_stp(pb, session, memory_read.id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	writel(stp->stp_id, pb->stp_base + STP_SEL);

	/* Verify that the sram_address is aligned on the RAM access registers
	 * address alignment.
	 */
	if (memory_read.sram_addr % sram_transfer_width != 0) {
		dev_err(&pb->pdev->dev,
				"%s: stp%u: memory transfer to unaligned addr, "
				"0x%08x\n", __func__, memory_read.id,
				memory_read.sram_addr);
		mutex_unlock(&pb->lock);
		return -EINVAL;
	}

	sram_transfer_addr = (uint16_t)(memory_read.sram_addr /
			sram_transfer_width);

	if (memory_read.priority)
		ram_ctrl_mask |= STP_RAM_PRI;

	ret = set_ram_target(&ram_ctrl_mask, memory_read.ram_target);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		dev_err(&pb->pdev->dev,
				"%s: stp%u: invalid ram target: 0x%04x "
				"err = %d\n",
				__func__, memory_read.id,
				memory_read.ram_target, ret);
		return ret;
	}

	ret = read_data_common(pb, memory_read.buf, memory_read.len_bytes,
			sram_transfer_addr, ram_ctrl_mask,
			pb->stp_base + STP_RAM_CTRL, STP_DATA_REG_COUNT);
	if (ret < 0)
		dev_err(&pb->pdev->dev,
				"%s: stp%u: read error addr: 0x%04x "
				"type: 0x%02x ram_ctrl 0x%08x err = %d\n",
				__func__, memory_read.id, memory_read.sram_addr,
				memory_read.ram_target,
				readl(pb->stp_base + STP_RAM_CTRL), ret);

	mutex_unlock(&pb->lock);

	return ret;
}

int get_program_state_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct stp_program_state __user *user_program_state;
	struct stp_program_state program_state;
	struct paintbox_stp *stp;
	uint32_t stat;
	int ret;

	user_program_state = (struct stp_program_state __user *)arg;
	if (copy_from_user(&program_state, user_program_state,
			sizeof(program_state)))
		return -EFAULT;

	mutex_lock(&pb->lock);
	stp = get_stp(pb, session, program_state.stp_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	writel(stp->stp_id, pb->stp_base + STP_SEL);

	/* TODO(ahampson):  Check to make sure the processor is not in the
	 * enabled state.  The STP_STAT register does not include a HALT state
	 * to check.  This would be preferrable to relying on the higher level
	 * code to put the processor in the disabled state.
	 * A feature request for a STP_STAT HALT bit is tracked by b/26516030.
	 */
	if (readl(pb->stp_base + STP_CTRL) & STP_ENA) {
		dev_err(&pb->pdev->dev, "%s: stp%u: processor is enabled\n",
				__func__, program_state.stp_id);
		mutex_unlock(&pb->lock);
		return -EBUSY;
	}

	stat = readl(pb->stp_base + STP_STAT_L);
	program_state.program_counter = (uint32_t)(stat & STP_PC_MASK);

	dev_dbg(&pb->pdev->dev, "%s: stp%u pc 0x%08x\n",  __func__,
			stp->stp_id, program_state.program_counter);

	mutex_unlock(&pb->lock);

	if (copy_to_user(user_program_state, &program_state,
			sizeof(program_state)))
		return -EFAULT;

	return 0;
}

#ifdef CONFIG_DEBUG_FS
static int stp_debug_regs_show(struct seq_file *s, void *unused)
{
	struct paintbox_stp *stp = s->private;
	struct paintbox_data *pb = stp->pb;
	char *buf;
	size_t len;
	int written;

	len = seq_get_buf(s, &buf);
	if (!buf)
		return -ENOMEM;

	mutex_lock(&pb->lock);

	written = snprintf(buf, len, "stp%u:\n", stp->stp_id);

	writel(stp->stp_id, pb->stp_base + STP_SEL);
	written += dump_stp_registers(pb, buf, len);

	mutex_unlock(&pb->lock);

	seq_commit(s, written);

	return 0;
}

static int stp_debug_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, stp_debug_regs_show, inode->i_private);
}

static const struct file_operations stp_debug_regs_fops = {
	.open = stp_debug_regs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static void paintbox_stp_debug_init(struct paintbox_data *pb,
		struct paintbox_stp *stp)
{
	char stp_name[RESOURCE_NAME_LEN];

	snprintf(stp_name, RESOURCE_NAME_LEN, "stp%u", stp->stp_id);

	stp->debug_dir = debugfs_create_dir(stp_name, pb->debug_root);

	stp->regs_dentry = debugfs_create_file("regs", S_IRUGO | S_IWUSR,
			stp->debug_dir, stp, &stp_debug_regs_fops);
}
#endif

static int init_stp(struct paintbox_data *pb, unsigned int stp_index)
{
	struct paintbox_stp *stp;

	stp = &pb->stps[stp_index];

	stp->pb = pb;
	stp->stp_id = stp_index_to_id(stp_index);


#ifdef CONFIG_DEBUG_FS
	paintbox_stp_debug_init(pb, stp);
#endif

#ifdef VERBOSE_DEBUG
	paintbox_alloc_debug_buffer(pb, STP_DEBUG_BUFFER_SIZE);
#endif

	dev_dbg(&pb->pdev->dev, "stp%u: base %p len %u\n", stp->stp_id,
			pb->stp_base, STP_BLOCK_LEN);

	return 0;
}

int paintbox_stp_init(struct paintbox_data *pb)
{
	unsigned int stp_index;

	pb->stp_base = pb->reg_base + IPU_STP_OFFSET;

	pb->stps = kzalloc(sizeof(struct paintbox_stp) * pb->caps.num_stps,
			GFP_KERNEL);
	if (!pb->stps)
		return -ENOMEM;

	for (stp_index = 0; stp_index < pb->caps.num_stps; stp_index++)
		init_stp(pb, stp_index);

	return 0;
}
