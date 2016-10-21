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
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include "paintbox-common.h"
#include "paintbox-debug.h"
#include "paintbox-io.h"
#include "paintbox-irq.h"
#include "paintbox-regs.h"
#include "paintbox-sram.h"
#include "paintbox-stp.h"
#include "paintbox-stp-sim.h"
#include "paintbox-stp-sram.h"
#include "paintbox-sim-regs.h"


#ifdef CONFIG_DEBUG_FS
static uint64_t stp_reg_entry_read(struct paintbox_debug_reg_entry *reg_entry)
{
	struct paintbox_debug *debug = reg_entry->debug;
	struct paintbox_stp *stp = container_of(debug, struct paintbox_stp,
			debug);
	struct paintbox_data *pb = debug->pb;
	unsigned long irq_flags;
	uint64_t val;

	mutex_lock(&pb->lock);

	spin_lock_irqsave(&pb->stp_lock, irq_flags);

	writel(stp->stp_id, pb->stp_base + STP_SEL);
	val = readq(pb->stp_base + reg_entry->reg_offset);

	spin_unlock_irqrestore(&pb->stp_lock, irq_flags);

	mutex_unlock(&pb->lock);

	return val;
}

static void stp_reg_entry_write(struct paintbox_debug_reg_entry *reg_entry,
		uint64_t val)
{
	struct paintbox_debug *debug = reg_entry->debug;
	struct paintbox_stp *stp = container_of(debug, struct paintbox_stp,
			debug);
	struct paintbox_data *pb = debug->pb;
	unsigned long irq_flags;

	mutex_lock(&pb->lock);

	spin_lock_irqsave(&pb->stp_lock, irq_flags);

	writel(stp->stp_id, pb->stp_base + STP_SEL);
	writeq(val, pb->stp_base + reg_entry->reg_offset);

	spin_unlock_irqrestore(&pb->stp_lock, irq_flags);

	mutex_unlock(&pb->lock);
}
#endif

#if defined(CONFIG_DEBUG_FS) || defined(VERBOSE_DEBUG)

static const char *stp_reg_names[STP_NUM_REGS] = {
	REG_NAME_ENTRY(STP_SEL),
	REG_NAME_ENTRY(STP_CTRL),
	REG_NAME_ENTRY(STP_STAT),
	REG_NAME_ENTRY(STP_CAP),
	REG_NAME_ENTRY(STP_RAM_CTRL),
	REG_NAME_ENTRY(STP_RAM_DATA0),
	REG_NAME_ENTRY(STP_RAM_DATA1),
	REG_NAME_ENTRY(STP_PMON_CFG),
	REG_NAME_ENTRY(STP_PMON_CNT_0_CFG),
	REG_NAME_ENTRY(STP_PMON_CNT_0),
	REG_NAME_ENTRY(STP_PMON_CNT_0_STS),
	REG_NAME_ENTRY(STP_PMON_CNT_1_CFG),
	REG_NAME_ENTRY(STP_PMON_CNT_1),
	REG_NAME_ENTRY(STP_PMON_CNT_1_STS)
};

static inline int dump_stp_reg(struct paintbox_data *pb, uint32_t reg_offset,
		uint64_t reg_value, char *buf, int *written, size_t len)
{
	const char *reg_name = stp_reg_names[REG_INDEX(reg_offset)];
	return dump_ipu_register_with_value64(pb, pb->stp_base, reg_offset,
			reg_value, reg_name, buf, written, len);
}

static int dump_stp_reg_verbose(struct paintbox_data *pb, uint32_t reg_offset,
		uint64_t reg_value, char *buf, int *written, size_t len,
		const char *format, ...)
{
	va_list args;
	int ret;

	ret = dump_stp_reg(pb, reg_offset, reg_value, buf, written, len);
	if (ret < 0)
		return ret;

	va_start(args, format);

	ret = dump_ipu_vprintf(pb, buf, written, len, format, args);

	va_end(args);

	return ret;
}

int dump_stp_registers(struct paintbox_debug *debug, char *buf, size_t len)
{
	uint64_t stp_registers[STP_NUM_REGS];
	struct paintbox_stp *stp = container_of(debug, struct paintbox_stp,
			debug);
	struct paintbox_data *pb = debug->pb;
	unsigned long irq_flags;
	int ret, written = 0;
	unsigned int reg_offset;
	uint64_t val;

	spin_lock_irqsave(&pb->stp_lock, irq_flags);

	writel(stp->stp_id, pb->stp_base + STP_SEL);

	for (reg_offset = 0; reg_offset < STP_BLOCK_LEN; reg_offset +=
			IPU_REG_WIDTH) {
		if (!stp_reg_names[REG_INDEX(reg_offset)])
			continue;

		stp_registers[REG_INDEX(reg_offset)] =  readq(pb->stp_base +
				reg_offset);
	}

	spin_unlock_irqrestore(&pb->stp_lock, irq_flags);

	val = stp_registers[REG_INDEX(STP_SEL)];
	ret = dump_stp_reg_verbose(pb, STP_SEL, val, buf, &written, len,
			"\tSTP_SEL 0x%02x\n", val & STP_SEL_STP_SEL_MASK);
	if (ret < 0)
		return ret;

	val = stp_registers[REG_INDEX(STP_CTRL)];
	ret = dump_stp_reg_verbose(pb, STP_CTRL, val, buf, &written, len,
			"\tLBP_MASK 0x%04x INT %d RESUME %d RESET %d ENA %d\n",
			(val & STP_CTRL_LBP_MASK_MASK) >>
			STP_CTRL_LBP_MASK_SHIFT, !!(val & STP_CTRL_INT_MASK),
			!!(val & STP_CTRL_RESUME_MASK),
			!!(val & STP_CTRL_RESET_MASK),
			!!(val & STP_CTRL_ENA_MASK));
	if (ret < 0)
		return ret;

	val = stp_registers[REG_INDEX(STP_STAT)];
	ret = dump_stp_reg_verbose(pb, STP_STAT, val, buf, &written, len,
			"\tSTALLED %d INT_CODE 0x%04x PC 0x%04x\n",
			(val >> STP_STAT_STALLED_SHIFT) & STP_STAT_STALLED_M,
			(val & STP_STAT_INT_CODE_MASK) >>
			STP_STAT_INT_CODE_SHIFT, val & STP_STAT_PC_MASK);
	if (ret < 0)
		return ret;

	val = stp_registers[REG_INDEX(STP_CAP)];
	ret = dump_stp_reg_verbose(pb, STP_CAP, val, buf, &written, len,
			"\tHALO_MEM %u words VECTOR_MEM %u words CONST MEM %u "
			"words SCALAR_MEM %u words INST_MEM %u instructions\n",
			(val & STP_CAP_HALO_MEM_MASK) >> STP_CAP_HALO_MEM_SHIFT,
			(val & STP_CAP_VECTOR_MEM_MASK) >> STP_CAP_VECTOR_MEM_SHIFT,
			(val & STP_CAP_CONST_MEM_MASK) >> STP_CAP_CONST_MEM_SHIFT,
			(val & STP_CAP_SCALAR_MEM_MASK) >> STP_CAP_SCALAR_MEM_SHIFT,
			val & STP_CAP_INST_MEM_MASK);
	if (ret < 0)
		return ret;

	for (reg_offset = STP_RAM_CTRL; reg_offset < STP_BLOCK_LEN;
			reg_offset += IPU_REG_WIDTH) {
		if (!stp_reg_names[REG_INDEX(reg_offset)])
			continue;

		ret = dump_stp_reg(pb, reg_offset,
				stp_registers[REG_INDEX(reg_offset)], buf,
				&written, len);
		if (ret < 0)
			return ret;
	}

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
	unsigned long irq_flags;
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
	list_add_tail(&stp->session_entry, &session->stp_list);

	spin_lock_irqsave(&pb->stp_lock, irq_flags);

	writel(stp_id, pb->stp_base + STP_SEL);

	/* Grant access to all LBPs associated with this session */
	lbp_mask = 0;
	list_for_each_entry_safe(lbp, lbp_next, &session->lbp_list,
			session_entry)
		lbp_mask |= 1 << (lbp->pool_id + STP_CTRL_LBP_MASK_SHIFT);

	ctrl = readl(pb->stp_base + STP_CTRL);
	ctrl &= ~STP_CTRL_LBP_MASK_MASK;
	ctrl |= lbp_mask;
	writel(ctrl, pb->stp_base + STP_CTRL);

	spin_unlock_irqrestore(&pb->stp_lock, irq_flags);

	mutex_unlock(&pb->lock);

	return 0;
}

/* The caller to this function must hold pb->lock */
void release_stp(struct paintbox_data *pb, struct paintbox_session *session,
		struct paintbox_stp *stp)
{
	unsigned long irq_flags;
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

	spin_lock_irqsave(&pb->stp_lock, irq_flags);

	writel(stp->stp_id, pb->stp_base + STP_SEL);
	writel(0, pb->stp_base + STP_CTRL);

	spin_unlock_irqrestore(&pb->stp_lock, irq_flags);

	/* TODO(ahampson): We will probably need to poll this register to wait
	 * for the cancel to take effect.
	 */

	/* Remove the processor from the session. */
	list_del(&stp->session_entry);
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
	list_for_each_entry_safe(stp, stp_next, &session->stp_list,
			session_entry) {
		unsigned long irq_flags;

		spin_lock_irqsave(&pb->stp_lock, irq_flags);

		writel(stp->stp_id, pb->stp_base + STP_SEL);
		ctrl = readl(pb->stp_base + STP_CTRL);
		ctrl |= 1 << (lbp->pool_id + STP_CTRL_LBP_MASK_SHIFT);
		writel(ctrl, pb->stp_base + STP_CTRL);

		spin_unlock_irqrestore(&pb->stp_lock, irq_flags);
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

	/* Disable access to the LBP for all STPs in the session. */
	list_for_each_entry_safe(stp, stp_next, &session->stp_list,
			session_entry) {
		unsigned long irq_flags;

		spin_lock_irqsave(&pb->stp_lock, irq_flags);

		writel(stp->stp_id, pb->stp_base + STP_SEL);
		ctrl = readl(pb->stp_base + STP_CTRL);
		ctrl &= ~(1 << (lbp->pool_id + STP_CTRL_LBP_MASK_SHIFT));
		writel(ctrl, pb->stp_base + STP_CTRL);

		spin_unlock_irqrestore(&pb->stp_lock, irq_flags);
	}
}

int stp_ctrl_set(struct paintbox_data *pb,
		 struct paintbox_session *session, unsigned int stp_id,
		 uint64_t set_mask)
{
	struct paintbox_stp *stp;
	unsigned long irq_flags;
	uint32_t ctrl;
	int ret;

	mutex_lock(&pb->lock);
	stp = get_stp(pb, session, stp_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	spin_lock_irqsave(&pb->stp_lock, irq_flags);

	writel(stp->stp_id, pb->stp_base + STP_SEL);

	ctrl = readl(pb->stp_base + STP_CTRL);
	ctrl |= set_mask;
	writel(ctrl, pb->stp_base + STP_CTRL);

	spin_unlock_irqrestore(&pb->stp_lock, irq_flags);

	mutex_unlock(&pb->lock);

	return 0;
}

int stp_ctrl_set_and_clear(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned int stp_id,
		uint32_t set_mask)
{
	struct paintbox_stp *stp;
	unsigned long irq_flags;
	uint32_t ctrl;
	int ret;

	mutex_lock(&pb->lock);
	stp = get_stp(pb, session, stp_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	spin_lock_irqsave(&pb->stp_lock, irq_flags);

	writel(stp->stp_id, pb->stp_base + STP_SEL);

	ctrl = readl(pb->stp_base + STP_CTRL);
	ctrl |= set_mask;
	writel(ctrl, pb->stp_base + STP_CTRL);
	ctrl &= ~set_mask;
	writel(ctrl, pb->stp_base + STP_CTRL);

	spin_unlock_irqrestore(&pb->stp_lock, irq_flags);

	mutex_unlock(&pb->lock);

	return 0;
}

int start_stp_ioctl(struct paintbox_data *pb, struct paintbox_session *session,
		unsigned long arg)
{
	unsigned int stp_id = (unsigned int)arg;
	struct paintbox_stp *stp;
	unsigned long irq_flags;
	uint64_t stat;
	uint32_t ctrl;
	int ret;

	mutex_lock(&pb->lock);
	stp = get_stp(pb, session, stp_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	dev_dbg(&pb->pdev->dev, "%s: stp%u\n",  __func__, stp_id);

	spin_lock_irqsave(&pb->stp_lock, irq_flags);

	writel(stp->stp_id, pb->stp_base + STP_SEL);

	stat = readq(pb->stp_base + STP_STAT);

	ctrl = readl(pb->stp_base + STP_CTRL);
	ctrl |= STP_CTRL_ENA_MASK;

	/* If the processor is stalled (due to a stall.b16 <- src8) instruction
	 * then set and clear the RESUME bit.
	 */
	if (stat & STP_STAT_STALLED_MASK)
		ctrl |= STP_CTRL_RESUME_MASK;

	writel(ctrl, pb->stp_base + STP_CTRL);

	/* The RESUME bit is not self clearing. */
	if (stat & STP_STAT_STALLED_MASK) {
		ctrl &= ~STP_CTRL_RESUME_MASK;
		writel(ctrl, pb->stp_base + STP_CTRL);
	}

	spin_unlock_irqrestore(&pb->stp_lock, irq_flags);

	mutex_unlock(&pb->lock);

	return 0;
}

int stop_stp_ioctl(struct paintbox_data *pb, struct paintbox_session *session,
		unsigned long arg)
{
	unsigned int stp_id = (unsigned int)arg;
	struct paintbox_stp *stp;
	uint32_t ctrl;
	unsigned long irq_flags;
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

	spin_lock_irqsave(&pb->stp_lock, irq_flags);

	writel(stp->stp_id, pb->stp_base + STP_SEL);

	ctrl = readl(pb->stp_base + STP_CTRL);
	ctrl &= ~STP_CTRL_ENA_MASK;
	writel(ctrl, pb->stp_base + STP_CTRL);

	spin_unlock_irqrestore(&pb->stp_lock, irq_flags);

	LOG_STP_REGISTERS(pb, stp);

	mutex_unlock(&pb->lock);

	return ret;
}

int resume_stp_ioctl(struct paintbox_data *pb, struct paintbox_session *session,
		unsigned long arg)
{
	unsigned int stp_id = (unsigned int)arg;
	dev_dbg(&pb->pdev->dev, "%s: stp%u\n",  __func__, stp_id);
        /* Resume bit is self-clearing */
	return stp_ctrl_set(pb, session, stp_id, STP_CTRL_RESUME_MASK);
}

int reset_stp_ioctl(struct paintbox_data *pb, struct paintbox_session *session,
		unsigned long arg)
{
	unsigned int stp_id = (unsigned int)arg;
	dev_dbg(&pb->pdev->dev, "%s: stp%u\n",  __func__, stp_id);
	return stp_ctrl_set_and_clear(pb, session, stp_id, STP_CTRL_RESET_MASK);
}

/* The caller to this function must hold pb->lock */
int init_stp(struct paintbox_data *pb, struct paintbox_stp *stp)
{
	unsigned long irq_flags;
	uint64_t ctrl;
#ifdef CONFIG_PAINTBOX_SIMULATOR_SUPPORT
	int ret;

	/* Make sure the STP is idle before stopping it.  Currently this is only
	 * done for the Simulator.  The FPGA does not have a similar mechanism
	 * How the post-DMA interrupt cleanup on the hardware will work is TBD.
	 */
	ret = sim_wait_for_idle(pb, stp);
	if (ret < 0)
		return ret;
#endif

	spin_lock_irqsave(&pb->stp_lock, irq_flags);

	writel(stp->stp_id, pb->stp_base + STP_SEL);

	/* Stop and reset the processor */
	ctrl = readl(pb->stp_base + STP_CTRL);
	ctrl &= ~STP_CTRL_ENA_MASK;
	writel(STP_CTRL_RESET_MASK, pb->stp_base + STP_CTRL);
	writel(ctrl, pb->stp_base + STP_CTRL);

	spin_unlock_irqrestore(&pb->stp_lock, irq_flags);

	return 0;
}

int init_stp_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	unsigned int stp_id = (unsigned int)arg;
	struct paintbox_stp *stp;
	int ret;

	mutex_lock(&pb->lock);
	stp = get_stp(pb, session, stp_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	ret = init_stp(pb, stp);

	mutex_unlock(&pb->lock);

	return ret;
}

int setup_stp_ioctl(struct paintbox_data *pb, struct paintbox_session *session,
		unsigned long arg)
{
	struct stp_config __user *user_config;
	struct stp_config config;
	struct paintbox_sram_config sram_config;
	struct paintbox_stp *stp;
	size_t max_len_bytes;
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

	max_len_bytes = stp->inst_mem_size_in_instructions *
			STP_INST_SRAM_INSTRUCTION_WIDTH_BYTES;

	if (config.len > max_len_bytes) {
		dev_err(&pb->pdev->dev,
				"%s: stp%u: program too large, %lu > %lu bytes"
				"\n", __func__, config.processor_id, config.len,
				max_len_bytes);
		mutex_unlock(&pb->lock);
		return -ERANGE;
	}

	/* TODO(ahampson):  The assembler generates the pISA in the byte order
	 * expected by the DV.  In order for the pISA to be used by the hardware
	 * the instruction buffer needs to be byte swapped.  This will
	 * eventually be fixed in the assembler.  b/30316979
	 */
	ret = create_scalar_sram_config(&sram_config, stp->stp_id,
			SRAM_TARGET_STP_INSTRUCTION_RAM, true /* swap data */);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	ret = init_stp(pb, stp);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	ret = sram_write_user_buffer(pb, &sram_config, 0, config.buf,
			config.len);
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
	unsigned long irq_flags;
	uint64_t stat;
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

	spin_lock_irqsave(&pb->stp_lock, irq_flags);

	writel(stp->stp_id, pb->stp_base + STP_SEL);

	/* TODO(ahampson):  Check to make sure the processor is not in the
	 * enabled state.  The STP_STAT register does not include a HALT state
	 * to check.  This would be preferrable to relying on the higher level
	 * code to put the processor in the disabled state.
	 * A feature request for a STP_STAT HALT bit is tracked by b/26516030.
	 */
	if (readl(pb->stp_base + STP_CTRL) & STP_CTRL_ENA_MASK) {
		dev_err(&pb->pdev->dev, "%s: stp%u: processor is enabled\n",
				__func__, program_state.stp_id);
		spin_unlock_irqrestore(&pb->stp_lock, irq_flags);
		mutex_unlock(&pb->lock);
		return -EBUSY;
	}

	stat = readq(pb->stp_base + STP_STAT);

	spin_unlock_irqrestore(&pb->stp_lock, irq_flags);

	program_state.program_counter = (uint32_t)(stat & STP_STAT_PC_MASK);

	dev_dbg(&pb->pdev->dev, "%s: stp%u pc 0x%08x\n",  __func__,
			stp->stp_id, program_state.program_counter);

	mutex_unlock(&pb->lock);

	if (copy_to_user(user_program_state, &program_state,
			sizeof(program_state)))
		return -EFAULT;

	return 0;
}

int enable_stp_interrupt_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	unsigned int stp_id = (unsigned int)arg;
	int ret;

	mutex_lock(&pb->lock);
	ret = validate_stp(pb, session, stp_id);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	io_enable_stp_interrupt(pb, stp_id);

	mutex_unlock(&pb->lock);

	return 0;
}

int disable_stp_interrupt_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	unsigned int stp_id = (unsigned int)arg;
	int ret;

	mutex_lock(&pb->lock);
	ret = validate_stp(pb, session, stp_id);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	io_disable_stp_interrupt(pb, stp_id);

	mutex_unlock(&pb->lock);

	return 0;
}

int bind_stp_interrupt_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct stp_interrupt_config __user *user_req;
	struct stp_interrupt_config req;
	struct paintbox_stp *stp;
	int ret;

	user_req = (struct stp_interrupt_config __user *)arg;
	if (copy_from_user(&req, user_req, sizeof(req)))
		return -EFAULT;

	mutex_lock(&pb->lock);
	stp = get_stp(pb, session, req.stp_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	ret = bind_stp_interrupt(pb, session, stp, req.interrupt_id);

	init_waiters(pb, stp->irq);

	mutex_unlock(&pb->lock);

	return ret;
}

int unbind_stp_interrupt_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	unsigned int stp_id = (unsigned int)arg;
	struct paintbox_stp *stp;
	int ret = 0;

	mutex_lock(&pb->lock);
	stp = get_stp(pb, session, stp_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	ret = unbind_stp_interrupt(pb, session, stp);
	if (ret < 0) {
		dev_err(&pb->pdev->dev,
				"%s: stp%u: unable to unbind interrupt, %d\n",
				__func__, stp_id, ret);
		mutex_unlock(&pb->lock);
		return ret;
	}

	dev_dbg(&pb->pdev->dev, "%s: stp%u: unbind interrupt\n", __func__,
			stp_id);

	mutex_unlock(&pb->lock);

	return 0;
}

irqreturn_t paintbox_stp_interrupt(struct paintbox_data *pb, uint64_t stp_mask)
{
	unsigned int stp_index;

	for (stp_index = 0; stp_index < pb->caps.num_stps && stp_mask;
			stp_index++, stp_mask >>= 1) {
		struct paintbox_stp *stp;
		uint32_t ctrl;
		int int_code;

		if (!(stp_mask & 0x01))
			continue;

		stp = &pb->stps[stp_index];

		spin_lock(&pb->stp_lock);

		writel(stp->stp_id, pb->stp_base + STP_SEL);
		ctrl = readl(pb->stp_base + STP_CTRL);

		if (!(ctrl & STP_CTRL_INT_MASK)) {
			spin_unlock(&pb->stp_lock);
			continue;
		}

		int_code = (int)((readq(pb->stp_base + STP_STAT) &
				STP_STAT_INT_CODE_MASK) >>
				STP_STAT_INT_CODE_SHIFT);

		writel(ctrl & ~STP_CTRL_INT_MASK, pb->stp_base + STP_CTRL);

		signal_waiters(pb, stp->irq, int_code);

		spin_unlock(&pb->stp_lock);
	}

	return IRQ_HANDLED;
}

static int init_stp_entry(struct paintbox_data *pb, unsigned int stp_index)
{
	struct paintbox_stp *stp = &pb->stps[stp_index];
	uint64_t caps;

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
	caps = readq(pb->stp_base + STP_CAP);
	stp->inst_mem_size_in_instructions = (unsigned int)(caps &
			STP_CAP_INST_MEM_MASK);
	stp->scalar_mem_size_in_words = (unsigned int)((caps &
			STP_CAP_SCALAR_MEM_MASK) >> STP_CAP_SCALAR_MEM_SHIFT);
	stp->const_mem_size_in_words = (unsigned int)((caps &
			STP_CAP_CONST_MEM_MASK) >> STP_CAP_CONST_MEM_SHIFT);
	stp->vector_mem_size_in_words = (unsigned int)((caps &
			STP_CAP_VECTOR_MEM_MASK) >> STP_CAP_VECTOR_MEM_SHIFT);
	stp->halo_mem_size_in_words = (unsigned int)((caps &
			STP_CAP_HALO_MEM_MASK) >> STP_CAP_HALO_MEM_SHIFT);

	dev_dbg(&pb->pdev->dev, "stp%u: base %p len %lu\n", stp->stp_id,
			pb->stp_base, STP_BLOCK_LEN);
	dev_dbg(&pb->pdev->dev, "\tinst mem %u instructions scalar mem %u words"
			" const mem %u words\n",
			stp->inst_mem_size_in_instructions,
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

	spin_lock_init(&pb->stp_lock);

	pb->caps.num_stps = readl(pb->reg_base + IPU_CAP) &
			IPU_CAP_NUM_STP_MASK;

	pb->stps = kzalloc(sizeof(struct paintbox_stp) * pb->caps.num_stps,
			GFP_KERNEL);
	if (!pb->stps)
		return -ENOMEM;

	for (stp_index = 0; stp_index < pb->caps.num_stps; stp_index++)
		init_stp_entry(pb, stp_index);

	return 0;
}
