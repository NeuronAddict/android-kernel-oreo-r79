/*
 * Power management support for the Paintbox programmable IPU
 *
 * Copyright (C) 2017 Google, Inc.
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
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/seq_file.h>
#include <linux/stat.h>
#include <linux/types.h>

#include "paintbox-common.h"
#include "paintbox-debug.h"
#include "paintbox-regs.h"
#include "paintbox-regs-supplemental.h"

/* Delay to prevent in-rush current */
#define CORE_POWER_RAMP_TIME 10 /* us */

/* Delay for rams to wake up */
#define CORE_RAM_POWER_RAIL_RAMP_TIME 1 /* us */

/* Delay for system to stabilize before sending real traffic */
#define CORE_SYSTEM_STABLIZE_TIME 100 /* us */

#ifdef CONFIG_DEBUG_FS
static uint64_t ipu_pm_reg_entry_read(
		struct paintbox_debug_reg_entry *reg_entry)
{
	struct paintbox_debug *debug = reg_entry->debug;
	struct paintbox_data *pb = debug->pb;
	return readq(pb->io.apb_base + reg_entry->reg_offset);
}

static void ipu_pm_reg_entry_write(struct paintbox_debug_reg_entry *reg_entry,
		uint64_t val)
{
	struct paintbox_debug *debug = reg_entry->debug;
	struct paintbox_data *pb = debug->pb;
	writeq(val, pb->io.apb_base + reg_entry->reg_offset);
}
#endif

#if defined(CONFIG_DEBUG_FS) || defined(VERBOSE_DEBUG)
static const char *io_pm_reg_names[IO_APB_NUM_REGS] = {
	REG_NAME_ENTRY(CLK_GATE_CONTROL_STP_IDLE_GATE_DIS),
	REG_NAME_ENTRY(CLK_GATE_CONTROL_LBP_IDLE_GATE_DIS),
	REG_NAME_ENTRY(CLK_GATE_CONTROL),
	REG_NAME_ENTRY(IPU_CORE_PAIRS_EN),
	REG_NAME_ENTRY(CORE_POWER_ON_N),
	REG_NAME_ENTRY(CORE_ISO_ON),
	REG_NAME_ENTRY(CORE_RAM_ON_N),
	REG_NAME_ENTRY(IPU_DMA_CHAN_EN)
};

static inline int ipu_pm_dump_reg(struct paintbox_data *pb, uint32_t reg_offset,
		char *buf, int *written, size_t len)
{
	const char *reg_name = io_pm_reg_names[REG_INDEX(reg_offset)];
	return dump_ipu_register64(pb, pb->io.apb_base, reg_offset, reg_name,
			buf, written, len);
}

int ipu_pm_dump_registers(struct paintbox_debug *debug, char *buf, size_t len)
{
	struct paintbox_data *pb = debug->pb;
	int ret, written = 0;

	ret = ipu_pm_dump_reg(pb, CLK_GATE_CONTROL_STP_IDLE_GATE_DIS, buf,
			&written, len);
	if (ret < 0)
		goto err_exit;

	ret = ipu_pm_dump_reg(pb, CLK_GATE_CONTROL_LBP_IDLE_GATE_DIS, buf,
			&written, len);
	if (ret < 0)
		goto err_exit;

	ret = ipu_pm_dump_reg(pb, CLK_GATE_CONTROL, buf, &written, len);
	if (ret < 0)
		goto err_exit;

	ret = ipu_pm_dump_reg(pb, IPU_CORE_PAIRS_EN, buf, &written, len);
	if (ret < 0)
		goto err_exit;

	ret = ipu_pm_dump_reg(pb, CORE_POWER_ON_N, buf, &written, len);
	if (ret < 0)
		goto err_exit;

	ret = ipu_pm_dump_reg(pb, CORE_ISO_ON, buf, &written, len);
	if (ret < 0)
		goto err_exit;

	ret = ipu_pm_dump_reg(pb, CORE_RAM_ON_N, buf, &written, len);
	if (ret < 0)
		goto err_exit;

	ret = ipu_pm_dump_reg(pb, IPU_DMA_CHAN_EN, buf, &written, len);
	if (ret < 0)
		goto err_exit;

	return written;

err_exit:
	dev_err(&pb->pdev->dev, "%s: register dump error, err = %d", __func__,
			ret);
	return ret;
}
#endif

/* The caller to this function must hold pb->dma.dma_lock */
void ipu_pm_enable_dma_channel(struct paintbox_data *pb,
		unsigned int channel_id)
{
	uint32_t val;

	val = readl(pb->io.apb_base + IPU_DMA_CHAN_EN);
	val |= 1 << channel_id;
	writel(val, pb->io.apb_base + IPU_DMA_CHAN_EN);
}

/* The caller to this function must hold pb->dma.dma_lock */
void ipu_pm_disable_dma_channel(struct paintbox_data *pb,
		unsigned int channel_id)
{
	uint32_t val;

	val = readl(pb->io.apb_base + IPU_DMA_CHAN_EN);
	val &= ~(1 << channel_id);
	writel(val, pb->io.apb_base + IPU_DMA_CHAN_EN);
}

/* The caller to this function must hold pb->lock */
static void ipu_core_power_enable(struct paintbox_data *pb,
		unsigned int requested_cores)
{
	uint32_t max_core_mask, active_core_mask;
	unsigned int active_cores;

	/* If the active core count is already at our requested core count then
	 * there is nothing to do.
	 */
	if (requested_cores <= pb->power.active_core_count)
		return;

	/* The maximum number of cores is equal to the number of STPs. */
	max_core_mask = (1 << pb->caps.num_stps) - 1;

	active_core_mask = (1 << requested_cores) - 1;

	/* Disable STP idle clock gating */
	writel(active_core_mask, pb->io.apb_base +
			CLK_GATE_CONTROL_STP_IDLE_GATE_DIS);

	/* Disable LBP idle clock gating */
	writel((active_core_mask << 1) | 0x1, pb->io.apb_base +
			CLK_GATE_CONTROL_LBP_IDLE_GATE_DIS);

	/* Disable DMA, SSP, MMU, and BIF idle clock gating */
	writel(CLK_GATE_CONTROL_DMA_IDLE_GATE_DIS_MASK |
			CLK_GATE_CONTROL_SSP_IDLE_GATE_DIS_MASK |
			CLK_GATE_CONTROL_MMU_BIF_IDLE_GATE_DIS_MASK,
			pb->io.apb_base + CLK_GATE_CONTROL);

	/* IPU cores need to be enabled in sequence in pairs */
	for (active_cores = pb->power.active_core_count;
			active_cores < requested_cores; active_cores += 2) {
		uint32_t new_core_mask_n = (max_core_mask <<
				(active_cores + 1)) & max_core_mask;
		uint32_t new_core_pairs = (active_cores + 2) / 2;

		/* Power on the odd core first */
		writel(new_core_mask_n, pb->io.apb_base + CORE_POWER_ON_N);
		udelay(CORE_POWER_RAMP_TIME);

		new_core_mask_n = (max_core_mask << (active_cores + 2)) &
				max_core_mask;

		/* Power on the even core next */
		writel(new_core_mask_n, pb->io.apb_base + CORE_POWER_ON_N);
		udelay(CORE_POWER_RAMP_TIME);

		/* We need to run the clock to the core pair that's being
		 * powered on briefly so that all the synchronizers clock
		 * through their data and all the Xs (or random values in the
		 * real HW) clear. Then we need to turn the clock back off so
		 * that we can meet timing on the RAM SD pin -- the setup & hold
		 * on the RAM's SD pin is significantly longer that 1 clock
		 * cycle.
		 */
		writel(new_core_pairs, pb->io.apb_base + IPU_CORE_PAIRS_EN);

		/* Turn clocks off on all active cores */
		writel(0, pb->io.apb_base + IPU_CORE_PAIRS_EN);

		/* Turn on RAMs for the core */
		writel(new_core_mask_n, pb->io.apb_base + CORE_RAM_ON_N);
		udelay(CORE_RAM_POWER_RAIL_RAMP_TIME);

		/* Restore clocks to all active core pairs. */
		writel(new_core_pairs, pb->io.apb_base + IPU_CORE_PAIRS_EN);

		/* Disable core isolation for the requested core */
		writel(new_core_mask_n, pb->io.apb_base + CORE_ISO_ON);
	}

	udelay(CORE_SYSTEM_STABLIZE_TIME);

	/* Enable idle clock gating for MMU and BIF */
	writel(CLK_GATE_CONTROL_DMA_IDLE_GATE_DIS_MASK |
			CLK_GATE_CONTROL_SSP_IDLE_GATE_DIS_MASK,
			pb->io.apb_base + CLK_GATE_CONTROL);

	pb->power.active_core_count = requested_cores;
}

/* The caller to this function must hold pb->lock */
static void ipu_core_power_disable(struct paintbox_data *pb,
		unsigned int requested_cores)
{
	uint32_t max_core_mask;
	unsigned int active_cores;

	/* If the active core count is already at our requested core count then
	 * there is nothing to do.
	 */
	if (requested_cores >= pb->power.active_core_count)
		return;

	/* The maximum number of cores is equal to the number of STPs. */
	max_core_mask = (1 << pb->caps.num_stps) - 1;

	/* Disable MMU, and BIF idle clock gating */
	writel(CLK_GATE_CONTROL_DMA_IDLE_GATE_DIS_MASK |
			CLK_GATE_CONTROL_SSP_IDLE_GATE_DIS_MASK |
			CLK_GATE_CONTROL_MMU_BIF_IDLE_GATE_DIS_MASK,
			pb->io.apb_base + CLK_GATE_CONTROL);

	for (active_cores = pb->power.active_core_count;
			active_cores > requested_cores; active_cores -= 2) {
		uint32_t new_core_mask_n = (max_core_mask <<
				(active_cores - 2)) & max_core_mask;
		uint32_t new_core_pairs = (active_cores - 2) / 2;

		/* Enable core isolation for the disabled cores */
		writel(new_core_mask_n, pb->io.apb_base + CORE_ISO_ON);

		/* Turn off clocks to all cores during the RAM power transition.
		 */
		writel(0, pb->io.apb_base + IPU_CORE_PAIRS_EN);

		/* Turn off RAMs for the disabled core pairs */
		writel(new_core_mask_n, pb->io.apb_base + CORE_RAM_ON_N);

		/* Need to briefly turn on the clocks to the cores being turned
		 * off to propagate the RAM SD pin change into the RAM, then
		 * need to turn the clocks off again, since the cores are being
		 * turned off.
		 */
		writel(new_core_pairs + 1, pb->io.apb_base + IPU_CORE_PAIRS_EN);

		/* Turn off clocks to the disabled core pairs. */
		writel(new_core_pairs, pb->io.apb_base + IPU_CORE_PAIRS_EN);

		/* Turn off the core pair */
		writel(new_core_mask_n, pb->io.apb_base + CORE_POWER_ON_N);
	}

	udelay(CORE_SYSTEM_STABLIZE_TIME);

	/* Enable idle clock gating for MMU and BIF */
	writel(CLK_GATE_CONTROL_DMA_IDLE_GATE_DIS_MASK |
			CLK_GATE_CONTROL_SSP_IDLE_GATE_DIS_MASK,
			pb->io.apb_base + CLK_GATE_CONTROL);

	pb->power.active_core_count = requested_cores;
}

/* The caller to this function must hold pb->lock */
void ipu_pm_stp_enable(struct paintbox_data *pb, struct paintbox_stp *stp)
{
	stp->pm_enabled = true;

	/* IPU cores are power controlled in pairs so round up the requested
	 * cores if it is an odd numbered core.
	 */
	ipu_core_power_enable(pb, (stp->stp_id + 1) & ~1);
}

/* The caller to this function must hold pb->lock */
void ipu_pm_lbp_enable(struct paintbox_data *pb, struct paintbox_lbp *lbp)
{
	lbp->pm_enabled = true;

	/* TODO(ahampson):  Figure out how to handle LBP0 if it is a special
	 * case.
	 */

	/* IPU cores are power controlled in pairs so round up the requested
	 * cores if it is an odd numbered core.
	 */
	ipu_core_power_enable(pb, (lbp->pool_id + 1) & ~1);
}

/* The caller to this function must hold pb->lock */
static void ipu_core_power_down_walk(struct paintbox_data *pb)
{
	unsigned int requested_cores;

	/* Walk backwards starting from the active core count until we find an
	 * enabled STP or LBP.  This is the new requested core count.
	 */
	for (requested_cores = pb->power.active_core_count;
			requested_cores > 0; requested_cores--) {
		struct paintbox_stp *stp = &pb->stp.stps[requested_cores];
		struct paintbox_lbp *lbp = &pb->lbp.lbps[requested_cores];

		if (stp->pm_enabled || lbp->pm_enabled)
			break;
	}

	/* IPU cores are power controlled in pairs so round up the requested
	 * cores if it is an odd numbered core.
	 */
	ipu_core_power_disable(pb, (requested_cores + 1) & ~1);
}

/* The caller to this function must hold pb->lock */
void ipu_pm_stp_disable(struct paintbox_data *pb, struct paintbox_stp *stp)
{
	stp->pm_enabled = false;
	ipu_core_power_down_walk(pb);
}

/* The caller to this function must hold pb->lock */
void ipu_pm_lbp_disable(struct paintbox_data *pb, struct paintbox_lbp *lbp)
{
	lbp->pm_enabled = false;

	/* TODO(ahampson):  Figure out how to handle LBP0 if it is a special
	 * case.
	 */

	ipu_core_power_down_walk(pb);
}

int ipu_pm_init(struct paintbox_data *pb)
{
#ifdef CONFIG_DEBUG_FS
	paintbox_debug_create_entry(pb, &pb->power.debug, pb->debug_root,
			"power", -1, ipu_pm_dump_registers, NULL, &pb->power);

	paintbox_debug_create_reg_entries(pb, &pb->power.debug, io_pm_reg_names,
			IO_APB_NUM_REGS, ipu_pm_reg_entry_write,
			ipu_pm_reg_entry_read);
#endif

	/* TODO(ahampson):  Add support for debugfs entry that allows a user to
	 * force a certain number of active cores.
	 */
	/* TODO(ahampson):  Add support for force leave enable */

	return 0;
}
