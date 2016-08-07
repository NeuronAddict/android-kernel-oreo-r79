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
#include "paintbox-debug.h"
#include "paintbox-regs.h"
#include "paintbox-sram.h"
#include "paintbox-stp.h"
#include "paintbox-stp-sim.h"
#include "paintbox-sim-regs.h"


static inline unsigned int stp_id_to_index(unsigned int stp_id)
{
	return stp_id - 1;
}

static inline unsigned int stp_index_to_id(unsigned int stp_index)
{
	return stp_index + 1;
}

#ifdef CONFIG_DEBUG_FS
static uint32_t stp_reg_entry_read(struct paintbox_debug_reg_entry *reg_entry)
{
	struct paintbox_debug *debug = reg_entry->debug;
	struct paintbox_stp *stp = container_of(debug, struct paintbox_stp,
			debug);
	struct paintbox_data *pb = debug->pb;
	uint32_t val;

	mutex_lock(&pb->lock);

	writel(stp->stp_id, pb->stp_base + STP_SEL);
	val = readl(pb->stp_base + reg_entry->reg_offset);

	mutex_unlock(&pb->lock);

	return val;
}

static void stp_reg_entry_write(struct paintbox_debug_reg_entry *reg_entry,
		uint32_t val)
{
	struct paintbox_debug *debug = reg_entry->debug;
	struct paintbox_stp *stp = container_of(debug, struct paintbox_stp,
			debug);
	struct paintbox_data *pb = debug->pb;

	mutex_lock(&pb->lock);

	writel(stp->stp_id, pb->stp_base + STP_SEL);
	writel(val, pb->stp_base + reg_entry->reg_offset);

	mutex_unlock(&pb->lock);
}
#endif

#if defined(CONFIG_DEBUG_FS) || defined(VERBOSE_DEBUG)

static const char *stp_reg_names[STP_NUM_REGS] = {
	REG_NAME_ENTRY(STP_SEL),
	REG_NAME_ENTRY(STP_CTRL),
	REG_NAME_ENTRY(STP_STAT_L),
	REG_NAME_ENTRY(STP_STAT_H),
	REG_NAME_ENTRY(STP_CAP_L),
	REG_NAME_ENTRY(STP_CAP_H),
	REG_NAME_ENTRY(STP_RAM_CTRL),
	REG_NAME_ENTRY(STP_RAM_DATA0_L),
	REG_NAME_ENTRY(STP_RAM_DATA0_H),
	REG_NAME_ENTRY(STP_RAM_DATA1_L),
	REG_NAME_ENTRY(STP_RAM_DATA1_H),
	REG_NAME_ENTRY(STP_PMON_CFG),
	REG_NAME_ENTRY(STP_PMON_CNT_0_CFG),
	REG_NAME_ENTRY(STP_PMON_CNT_0),
	REG_NAME_ENTRY(STP_PMON_CNT_0_STS),
	REG_NAME_ENTRY(STP_PMON_CNT_1_CFG),
	REG_NAME_ENTRY(STP_PMON_CNT_1),
	REG_NAME_ENTRY(STP_PMON_CNT_1_STS)
};

static inline int dump_stp_reg(struct paintbox_data *pb, uint32_t reg_offset,
		char *buf, int *written, size_t len)
{
	const char *reg_name = stp_reg_names[REG_INDEX(reg_offset)];
	return dump_ipu_register(pb, pb->stp_base, reg_offset, reg_name, buf,
			written, len);
}

static int dump_stp_reg_verbose(struct paintbox_data *pb, uint32_t reg_offset,
		char *buf, int *written, size_t len, const char *format, ...)
{
	va_list args;
	int ret;

	ret = dump_stp_reg(pb, reg_offset, buf, written, len);
	if (ret < 0)
		return ret;

	va_start(args, format);

	ret = dump_ipu_vprintf(pb, buf, written, len, format, args);

	va_end(args);

	return ret;
}

int dump_stp_registers(struct paintbox_debug *debug, char *buf, size_t len)
{
	struct paintbox_stp *stp = container_of(debug, struct paintbox_stp,
			debug);
	struct paintbox_data *pb = debug->pb;
	uint32_t val;
	int ret, written = 0;

	writel(stp->stp_id, pb->stp_base + STP_SEL);

	ret = dump_stp_reg_verbose(pb, STP_SEL, buf, &written, len,
			"\tSTP_SEL 0x%02x\n",
			readl(pb->stp_base + STP_SEL) & STP_SEL_MASK);
	if (ret < 0)
		return ret;

	val = readl(pb->stp_base + STP_CTRL);
	ret = dump_stp_reg_verbose(pb, STP_CTRL, buf, &written, len,
			"\tLBP_MASK 0x%04x RESUME %d RESET %d ENA %d\n",
			(val & STP_LBP_MASK_MASK) >> STP_LBP_MASK_SHIFT,
			!!(val & STP_RESUME), !!(val & STP_RESET),
			!!(val & STP_ENA));
	if (ret < 0)
		return ret;

	val = readl(pb->stp_base + STP_STAT_L);
	ret = dump_stp_reg_verbose(pb, STP_STAT_L, buf, &written, len,
			"\tINT_CODE 0x%04x PC 0x%04x\n",
			(val & STP_INT_CODE_MASK) >> STP_INT_CODE_SHIFT,
			val & STP_PC_MASK);
	if (ret < 0)
		return ret;

	val = readl(pb->stp_base + STP_STAT_H);
	ret = dump_stp_reg_verbose(pb, STP_STAT_H, buf, &written, len,
			"\tSTALLED %d\n", val & STP_STALLED);
	if (ret < 0)
		return ret;

	val = readl(pb->stp_base + STP_CAP_L);
	ret = dump_stp_reg_verbose(pb, STP_CAP_L, buf, &written, len,
			"\tSCALAR_MEM %u words INST_MEM %u words\n",
			(val & STP_SCALAR_MEM_MASK) >> STP_SCALAR_MEM_SHIFT,
			val & STP_INST_MEM_MASK);
	if (ret < 0)
		return ret;

	val = readl(pb->stp_base + STP_CAP_H);
	ret = dump_stp_reg_verbose(pb, STP_CAP_H, buf, &written, len,
			"\tHALO_MEM %u words VECTOR_MEM %u words CONST MEM %u "
			"words\n",
			(val & STP_HALO_MEM_MASK) >> STP_HALO_MEM_SHIFT,
			(val & STP_VECTOR_MEM_MASK) >> STP_VECTOR_MEM_SHIFT,
			val & STP_CONST_MEM_MASK);
	if (ret < 0)
		return ret;

	ret = dump_stp_reg(pb, STP_RAM_CTRL, buf, &written, len);
	if (ret < 0)
		return ret;

	ret = dump_stp_reg(pb, STP_RAM_DATA0_L, buf, &written, len);
	if (ret < 0)
		return ret;

	ret = dump_stp_reg(pb, STP_RAM_DATA0_H, buf, &written, len);
	if (ret < 0)
		return ret;

	ret = dump_stp_reg(pb, STP_RAM_DATA1_L, buf, &written, len);
	if (ret < 0)
		return ret;

	ret = dump_stp_reg(pb, STP_RAM_DATA1_H, buf, &written, len);
	if (ret < 0)
		return ret;

	ret = dump_stp_reg(pb, STP_PMON_CFG, buf, &written, len);
	if (ret < 0)
		return ret;

	ret = dump_stp_reg(pb, STP_PMON_CNT_0_CFG, buf, &written, len);
	if (ret < 0)
		return ret;

	ret = dump_stp_reg(pb, STP_PMON_CNT_0, buf, &written, len);
	if (ret < 0)
		return ret;

	ret = dump_stp_reg(pb, STP_PMON_CNT_0_STS, buf, &written, len);
	if (ret < 0)
		return ret;

	ret = dump_stp_reg(pb, STP_PMON_CNT_1_CFG, buf, &written, len);
	if (ret < 0)
		return ret;

	ret = dump_stp_reg(pb, STP_PMON_CNT_1, buf, &written, len);
	if (ret < 0)
		return ret;

	ret = dump_stp_reg(pb, STP_PMON_CNT_1_STS, buf, &written, len);
	if (ret < 0)
		return ret;

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
	dump_stp_registers(&stp->debug, pb->vdbg_log + written,
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

/* This function is used to clear the LBP access control mask for an STP on a
 * released LBP.
 * The caller to this function must hold pb->lock
 */
void disable_stp_access_to_lbp(struct paintbox_data *pb,
		struct paintbox_session *session, struct paintbox_lbp *lbp)
{
	struct paintbox_stp *stp, *stp_next;
	uint32_t ctrl;

	/* Grant access to the LBP to all STPs in the session. */
	list_for_each_entry_safe(stp, stp_next, &session->stp_list, entry) {
		writel(stp->stp_id, pb->stp_base + STP_SEL);
		ctrl = readl(pb->stp_base + STP_CTRL);
		ctrl &= ~(1 << (lbp->pool_id + STP_LBP_MASK_SHIFT));
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
	size_t max_len_bytes;
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

	max_len_bytes = stp->inst_mem_size_in_words * STP_WORD_WIDTH_BYTES;

	if (config.len > max_len_bytes) {
		dev_err(&pb->pdev->dev,
				"%s: stp%u: program too large, %lu > %lu bytes"
				"\n", __func__, config.processor_id, config.len,
				max_len_bytes);
		mutex_unlock(&pb->lock);
		return -ERANGE;
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

	ret = sram_write_user_buffer(pb, config.buf, 0, config.len,
			STP_RAM_TARG_INST_RAM << STP_RAM_TARG_SHIFT,
			pb->stp_base + STP_RAM_CTRL, STP_DATA_REG_COUNT);
	if (ret < 0)
		dev_err(&pb->pdev->dev,
				"%s: stp%u: setup stp failed, ret = %d\n",
				__func__, config.processor_id, ret);

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

static int init_stp(struct paintbox_data *pb, unsigned int stp_index)
{
	struct paintbox_stp *stp = &pb->stps[stp_index];
	uint32_t caps;

	stp->stp_id = stp_index_to_id(stp_index);

#ifdef CONFIG_DEBUG_FS
	paintbox_debug_create_entry(pb, &stp->debug, pb->debug_root, "stp",
			stp->stp_id, dump_stp_registers, stp);

	paintbox_debug_create_reg_entries(pb, &stp->debug, stp_reg_names,
			STP_NUM_REGS, stp_reg_entry_write, stp_reg_entry_read);
#endif

#ifdef VERBOSE_DEBUG
	paintbox_alloc_debug_buffer(pb, STP_DEBUG_BUFFER_SIZE);
#endif

	writel(stp->stp_id, pb->stp_base + STP_SEL);
	caps = readl(pb->stp_base + STP_CAP_L);
	stp->inst_mem_size_in_words = caps & STP_INST_MEM_MASK;
	stp->scalar_mem_size_in_words = (caps & STP_SCALAR_MEM_MASK) >>
			STP_SCALAR_MEM_SHIFT;

	caps = readl(pb->stp_base + STP_CAP_H);
	stp->const_mem_size_in_words = caps & STP_CONST_MEM_MASK;
	stp->vector_mem_size_in_words = (caps & STP_VECTOR_MEM_MASK) >>
			STP_VECTOR_MEM_SHIFT;
	stp->halo_mem_size_in_words = (caps & STP_HALO_MEM_MASK) >>
			STP_HALO_MEM_SHIFT;

	dev_dbg(&pb->pdev->dev, "stp%u: base %p len %u\n", stp->stp_id,
			pb->stp_base, STP_BLOCK_LEN);
	dev_dbg(&pb->pdev->dev, "\tinst mem %u words scalar mem %u words "
			"const mem %u words\n", stp->inst_mem_size_in_words,
			stp->scalar_mem_size_in_words,
			stp->const_mem_size_in_words);
	dev_dbg(&pb->pdev->dev, "\tvector mem %u words halo mem %u words\n",
			stp->const_mem_size_in_words,
			stp->halo_mem_size_in_words);

	return 0;
}

int paintbox_stp_init(struct paintbox_data *pb)
{
	unsigned int stp_index;

	pb->stp_base = pb->reg_base + IPU_STP_OFFSET;

	pb->caps.num_stps = readl(pb->reg_base + IPU_CAP) & NUM_STP_MASK;

	pb->stps = kzalloc(sizeof(struct paintbox_stp) * pb->caps.num_stps,
			GFP_KERNEL);
	if (!pb->stps)
		return -ENOMEM;

	for (stp_index = 0; stp_index < pb->caps.num_stps; stp_index++)
		init_stp(pb, stp_index);

	return 0;
}
