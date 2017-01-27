/*
 * MMU support for the Paintbox programmable IPU
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
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/iommu.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/seq_file.h>
#include <linux/types.h>

#include "paintbox-common.h"
#include "paintbox-debug.h"
#include "paintbox-dma.h"
#include "paintbox-io.h"
#include "paintbox-regs.h"
#include "paintbox-regs-supplemental.h"

#ifdef CONFIG_DEBUG_FS
static uint64_t paintbox_mmu_reg_entry_read(
		struct paintbox_debug_reg_entry *reg_entry)
{
	struct paintbox_debug *debug = reg_entry->debug;
	struct paintbox_data *pb = debug->pb;
	return readq(pb->io.axi_base + reg_entry->reg_offset);
}

static void paintbox_mmu_reg_entry_write(
		struct paintbox_debug_reg_entry *reg_entry, uint64_t val)
{
	struct paintbox_debug *debug = reg_entry->debug;
	struct paintbox_data *pb = debug->pb;
	writeq(val, pb->io.axi_base + reg_entry->reg_offset);
}
#endif

#if defined(CONFIG_DEBUG_FS) || defined(VERBOSE_DEBUG)
static const char *paintbox_mmu_reg_names[IO_AXI_NUM_REGS] = {
	REG_NAME_ENTRY(MMU_CTRL),
	REG_NAME_ENTRY(MMU_TABLE_BASE),
	REG_NAME_ENTRY(MMU_ERR_BASE),
	REG_NAME_ENTRY(MMU_SYNC),
	REG_NAME_ENTRY(MMU_FLUSH_CHANNEL),
	REG_NAME_ENTRY(MMU_FLUSH_ADDRESS),
	REG_NAME_ENTRY(MMU_FLUSH_FIFO_LEVEL),
	REG_NAME_ENTRY(MMU_FLUSH_FIFO_FULL),
	REG_NAME_ENTRY(MMU_ISR),
	REG_NAME_ENTRY(MMU_IMR),
	REG_NAME_ENTRY(MMU_ISR_OVF),
	REG_NAME_ENTRY(MMU_ERR_LOG),
	REG_NAME_ENTRY(MMU_PMON_CFG),
	REG_NAME_ENTRY(MMU_PMON_CNT_0_CFG),
	REG_NAME_ENTRY(MMU_PMON_CNT_0),
	REG_NAME_ENTRY(MMU_PMON_CNT_0_STS_ACC),
	REG_NAME_ENTRY(MMU_PMON_CNT_0_STS),
	REG_NAME_ENTRY(MMU_PMON_CNT_1_CFG),
	REG_NAME_ENTRY(MMU_PMON_CNT_1),
	REG_NAME_ENTRY(MMU_PMON_CNT_1_STS_ACC),
	REG_NAME_ENTRY(MMU_PMON_CNT_1_STS)
};

static inline int paintbox_mmu_dump_reg(struct paintbox_data *pb,
		uint32_t reg_offset, char *buf, int *written, size_t len)
{
	const char *reg_name = paintbox_mmu_reg_names[REG_INDEX(reg_offset)];
	return dump_ipu_register64(pb, pb->io.axi_base, reg_offset, reg_name,
			buf, written, len);
}

int paintbox_dump_mmu_registers(struct paintbox_debug *debug, char *buf,
		size_t len)
{
	struct paintbox_data *pb = debug->pb;
	unsigned int i;
	int ret, written = 0;

	for (i = 0; i < IO_AXI_NUM_REGS; i++) {
		if (paintbox_mmu_reg_names[i] == NULL)
			continue;

		ret = paintbox_mmu_dump_reg(pb, i * IPU_REG_WIDTH, buf,
				&written, len);
		if (ret < 0) {
			dev_err(&pb->pdev->dev,
					"%s: register dump error, err = %d",
					__func__, ret);
			return ret;
		}
	}

	return written;
}
#endif

static void paintbox_mmu_table_walk_error_interrupt(struct paintbox_data *pb,
		const char *error, bool overflow)
{
	if (!overflow) {
		uint64_t err_log;
		unsigned long iova;
		unsigned int tid;

		err_log = readq(pb->io.axi_base + MMU_ERR_LOG);
		writeq(err_log, pb->io.axi_base + MMU_ERR_LOG);

		tid = (unsigned int)((err_log & MMU_ERR_LOG_ID_MASK) >>
				MMU_ERR_LOG_ID_SHIFT);

		iova = (unsigned long)((err_log & MMU_ERR_LOG_VPAGEADDR_MASK) <<
				MMU_IOVA_SHIFT);

		dev_err(&pb->pdev->dev,
				"%s: %s occurred on dma %s, tid 0x%02x iova "
				"0x%016lx\n", __func__, error,
				(err_log & MMU_ERR_LOG_RD_WR_N_MASK) ? "read" :
				"write", tid, iova);
	} else {
		dev_err(&pb->pdev->dev,
				"%s: table walk engine %s occurred (interrupt "
				"overflow)", __func__, error);
	}

	/* An MMU table walk error is a catastrophic error for the IPU.
	 * Complete and report the error on all DMA channels.
	 */
	dma_report_error_all_channels(pb, -ENOTRECOVERABLE);

	/* TODO(ahampson):  Initiate a reset of the IPU and block new operations
	 * until the IPU comes out of reset.  As part of the reset process any
	 * MIPI or STP waiters should be released.  b/33455713
	 */
}

static void paintbox_mmu_flush_error_interrupt(struct paintbox_data *pb,
		const char *error, bool overflow)
{
	if (!overflow) {
		uint64_t err_log;
		unsigned long iova;

		err_log = readq(pb->io.axi_base + MMU_ERR_LOG);
		writeq(err_log, pb->io.axi_base + MMU_ERR_LOG);

		iova = (unsigned long)((err_log & MMU_ERR_LOG_VPAGEADDR_MASK) <<
				MMU_IOVA_SHIFT);

		dev_err(&pb->pdev->dev,
				"%s: mmu flush %s error occurred, iova 0x%016lx"
				"\n", __func__, error, iova);
	} else {
		dev_err(&pb->pdev->dev,
				"%s: mmu flush %s error occurred (interrupt "
				"overflow)", __func__, error);
	}

	/* An MMU flush error is a catastrophic error for the IPU.
	 * Complete and report the error on all DMA channels.
	 */
	dma_report_error_all_channels(pb, -ENOTRECOVERABLE);

	/* TODO(ahampson):  Initiate a reset of the IPU and block new operations
	 * until the IPU comes out of reset.  As part of the reset process any
	 * MIPI or STP waiters should be released.  b/33455713
	 */
}

static void paintbox_mmu_prefetch_error_interrupt(struct paintbox_data *pb,
		bool overflow)
{
	if (!overflow) {
		uint64_t err_log;
		unsigned long iova;

		err_log = readq(pb->io.axi_base + MMU_ERR_LOG);
		writeq(err_log, pb->io.axi_base + MMU_ERR_LOG);

		iova = (unsigned long)((err_log & MMU_ERR_LOG_VPAGEADDR_MASK) <<
				MMU_IOVA_SHIFT);

		dev_err(&pb->pdev->dev,
				"%s: mmu prefetch read error occurred, iova "
				"0x%016lx\n", __func__, iova);
	} else {
		dev_err(&pb->pdev->dev,
				"%s: mmu pretch read error occurred (interrupt "
				"overflow)", __func__);
	}

	/* An MMU prefetch error is a catastrophic error for the IPU.
	 * Complete and report the error on all DMA channels.
	 */
	dma_report_error_all_channels(pb, -ENOTRECOVERABLE);

	/* TODO(ahampson):  Initiate a reset of the IPU and block new operations
	 * until the IPU comes out of reset.  As part of the reset process any
	 * MIPI or STP waiters should be released.  b/33455713
	 */
}

/* This function must be called in an interrupt context. */
void paintbox_mmu_interrupt(struct paintbox_data *pb)
{
	uint32_t status = readl(pb->io.axi_base + MMU_ISR);
	writel(status, pb->io.axi_base + MMU_ISR);

	if (status & MMU_ISR_FLUSH_FULL_ERR_MASK)
		dev_err(&pb->pdev->dev, "%s: mmu flush fifo full\n", __func__);

	if (status & MMU_ISR_FLUSH_MEMRD_ERR_MASK)
		paintbox_mmu_flush_error_interrupt(pb, "memory read", false);

	if (status & MMU_ISR_FLUSH_INVALID_TABLE_MASK)
		paintbox_mmu_flush_error_interrupt(pb, "invalid table", false);

	if (status & MMU_ISR_TWE_MEMRD_ERR_MASK)
		paintbox_mmu_table_walk_error_interrupt(pb, "read error",
				false);

	if (status & MMU_ISR_TWE_ACCESS_VIO_MASK)
		paintbox_mmu_table_walk_error_interrupt(pb, "permission error",
				false);

	if (status & MMU_ISR_TWE_INVALID_TABLE_MASK)
		paintbox_mmu_table_walk_error_interrupt(pb, "invalid table",
				false);

	if (status & MMU_ISR_PREFETCH_MEMRD_ERR_MASK)
		paintbox_mmu_prefetch_error_interrupt(pb, false);

	status = readl(pb->io.axi_base + MMU_ISR_OVF);
	writel(status, pb->io.axi_base + MMU_ISR_OVF);

	if (status & MMU_ISR_OVF_FLUSH_FULL_ERR_MASK)
		dev_err(&pb->pdev->dev, "%s: mmu flush fifo full (interrupt "
				"overflow)\n", __func__);

	if (status & MMU_ISR_OVF_FLUSH_MEMRD_ERR_MASK)
		paintbox_mmu_flush_error_interrupt(pb, "memory read", true);

	if (status & MMU_ISR_OVF_FLUSH_INVALID_TABLE_MASK)
		paintbox_mmu_flush_error_interrupt(pb, "invalid table", true);

	if (status & MMU_ISR_OVF_TWE_MEMRD_ERR_MASK)
		paintbox_mmu_table_walk_error_interrupt(pb, "read error", true);

	if (status & MMU_ISR_OVF_TWE_ACCESS_VIO_MASK)
		paintbox_mmu_table_walk_error_interrupt(pb, "permission error",
				true);

	if (status & MMU_ISR_OVF_TWE_INVALID_TABLE_MASK)
		paintbox_mmu_table_walk_error_interrupt(pb, "invalid table",
				true);

	if (status & MMU_ISR_OVF_PREFETCH_MEMRD_ERR_MASK)
		paintbox_mmu_prefetch_error_interrupt(pb, true);
}

#ifdef CONFIG_DEBUG_FS
void paintbox_mmu_debug_init(struct paintbox_data *pb)
{
	paintbox_debug_create_entry(pb, &pb->mmu.debug, pb->debug_root,
			"mmu", -1, paintbox_dump_mmu_registers, NULL,
			&pb->mmu);

	paintbox_debug_create_reg_entries(pb, &pb->mmu.debug,
			paintbox_mmu_reg_names, IO_AXI_NUM_REGS,
			paintbox_mmu_reg_entry_write,
			paintbox_mmu_reg_entry_read);
}
#endif

int paintbox_mmu_init(struct paintbox_data *pb)
{
	writel(MMU_IMR_PREFETCH_MEMRD_ERR_MASK |
			MMU_IMR_TWE_ACCESS_VIO_MASK |
			MMU_IMR_TWE_MEMRD_ERR_MASK |
			MMU_IMR_FLUSH_MEMRD_ERR_MASK |
			MMU_IMR_TWE_INVALID_TABLE_MASK |
			MMU_IMR_FLUSH_FULL_ERR_MASK |
			MMU_IMR_FLUSH_INVALID_TABLE_MASK,
			pb->io.axi_base + MMU_IMR);

	paintbox_enable_mmu_interrupt(pb);

#ifdef CONFIG_DEBUG_FS
	paintbox_mmu_debug_init(pb);
#endif

#ifdef VERBOSE_DEBUG
	paintbox_alloc_debug_buffer(pb, IO_AXI_DEBUG_BUFFER_SIZE);
#endif

	return 0;
}
