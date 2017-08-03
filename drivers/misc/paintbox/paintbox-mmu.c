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
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/iommu.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/seq_file.h>
#include <linux/paintbox-iommu.h>
#include <linux/types.h>

#include "paintbox-common.h"
#include "paintbox-debug.h"
#include "paintbox-dma.h"
#include "paintbox-io.h"
#include "paintbox-power.h"
#include "paintbox-regs.h"

/* Paintbox IO virtual address space bounds
 * TODO(ahampson):  These are place holder values.  I need to figure out the
 * correct value for these.  This comes out to 512MB right now.
 */
#define PAINTBOX_IOVA_START		0x20000000
#define PAINTBOX_IOVA_SIZE		0x40000000

/* TOOD(ahampson):  The error base is specific to the platform and should
 * be passed in through the platform data.
 */
#define PAINTBOX_ERROR_BASE		0x8000000000

/* TODO(ahampson):  Figure out if there is a way to get this information from
 * the system.
 */
#define PAINTBOX_INPUT_ADDR_SIZE	43 /* bits */

/* TODO(ahampson):  This will need to be configurable.  The output address size
 * on Easel will be 32 bits but on a normal system it will be 40 bits.
 */
#define PAINTBOX_OUTPUT_ADDR_SIZE	32 /* bits */

/* Easel will use 4K pages.  This may change in future versions. */
#define PAINTBOX_PAGE_SIZE_BITMAP	SZ_4K

#define MMU_FLUSH_DELAY 10 /* us */
#define MMU_FLUSH_MAX_ATTEMPTS 30

#define MMU_SYNC_DELAY 10 /* us */
#define MMU_SYNC_MAX_ATTEMPTS 3

#ifdef CONFIG_PAINTBOX_DEBUG
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
	REG_NAME_ENTRY(MMU_PMON_CNT_1_STS),
#if CONFIG_PAINTBOX_VERSION_MAJOR >= 1
	REG_NAME_ENTRY(MMU_IER),
	REG_NAME_ENTRY(MMU_ITR),
#endif
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
	 * MIPI or STP waiters should be released.  b/34518459
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
	 * MIPI or STP waiters should be released.  b/34518459
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
	 * MIPI or STP waiters should be released.  b/34518459
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

#ifdef CONFIG_PAINTBOX_IOMMU
/* MMU operation hooks
 *
 * The MMU registers and interrupts are part of the core IPU driver's
 * register space.  Hooks are provided through platform data to the
 * IOMMU driver for MMU operations.
 */
/* Called with page table spinlock held. */
static void paintbox_mmu_tlb_sync(void *priv)
{
	struct paintbox_data *pb = (struct paintbox_data *)priv;
	int attempts = 0;

	dev_dbg(&pb->pdev->dev, "%s\n", __func__);

#if CONFIG_PAINTBOX_VERSION_MAJOR == 0
	paintbox_disable_mmu_bif_idle_clock_gating(pb);
#endif

	/* TODO(ahampson):  There is no field bit defined for MMU_SYNC so we
	 * just write a 1 into the register in the interim.
	 */
	writel(0x01, pb->io.axi_base + MMU_SYNC);
	while (readl(pb->io.axi_base + MMU_SYNC)) {
		if (++attempts >= MMU_SYNC_MAX_ATTEMPTS) {
			dev_err(&pb->pdev->dev,
					"%s: timeout waiting for MMU sync\n",
					__func__);
			/* TODO(ahampson):  A proper recovery path for a sync
			 * timeout should be developed for this case.
			 * b/35470877
			 */
			break;
		}
		udelay(MMU_SYNC_DELAY);
	}

#if CONFIG_PAINTBOX_VERSION_MAJOR == 0
	paintbox_enable_mmu_bif_idle_clock_gating(pb);
#endif
}

/* Called with page table spinlock held. */
static void paintbox_mmu_tlb_flush_all(void *priv)
{
	struct paintbox_data *pb = (struct paintbox_data *)priv;
	unsigned int channel_id;
	int attempts = 0;

	dev_dbg(&pb->pdev->dev, "%s\n", __func__);

#if CONFIG_PAINTBOX_VERSION_MAJOR == 0
	paintbox_disable_mmu_bif_idle_clock_gating(pb);
#endif

	for (channel_id = 0; channel_id < pb->dma.num_channels; channel_id++) {
		while (readl(pb->io.axi_base + MMU_FLUSH_FIFO_FULL)) {
			if (++attempts >= MMU_FLUSH_MAX_ATTEMPTS) {
				dev_err(&pb->pdev->dev,
						"%s: timeout waiting for flush "
						"FIFO to clear\n", __func__);
				/* TODO(ahampson):  A proper recovery path for a
				 * flush FIFO timeout should be developed for
				 * this case.  b/35470877
				 */
				goto err_exit;
			}
			udelay(MMU_FLUSH_DELAY);
		}

		writel(channel_id, pb->io.axi_base + MMU_FLUSH_CHANNEL);
	}

err_exit:
#if CONFIG_PAINTBOX_VERSION_MAJOR == 0
	paintbox_enable_mmu_bif_idle_clock_gating(pb);
#endif
	return;
}

/* Called with page table spinlock held. */
static void paintbox_mmu_tlb_invalidate_range_nosync(void *priv,
		unsigned long iova, size_t size, bool leaf)
{
	struct paintbox_data *pb = (struct paintbox_data *)priv;
	unsigned long offset;
	int attempts = 0;

	dev_dbg(&pb->pdev->dev, "%s:iova 0x%016lx sz %zu leaf %d\n", __func__,
			iova, size, leaf);

#if CONFIG_PAINTBOX_VERSION_MAJOR == 0
	paintbox_disable_mmu_bif_idle_clock_gating(pb);
#endif

	for (offset = 0; offset < size; offset += PAGE_SIZE) {
		while (readl(pb->io.axi_base + MMU_FLUSH_FIFO_FULL)) {
			if (++attempts >= MMU_FLUSH_MAX_ATTEMPTS) {
				dev_err(&pb->pdev->dev,
						"%s: timeout waiting for flush "
						"FIFO to clear\n", __func__);
				/* TODO(ahampson):  A proper recovery path for a
				 * flush FIFO timeout should be developed for
				 * this case.  b/35470877
				 */
				goto err_exit;
			}

			udelay(MMU_FLUSH_DELAY);
		}

		writel((iova + offset) >> MMU_FLUSH_ADDRESS_RSHIFT,
				pb->io.axi_base + MMU_FLUSH_ADDRESS);
	}

err_exit:
#if CONFIG_PAINTBOX_VERSION_MAJOR == 0
	paintbox_enable_mmu_bif_idle_clock_gating(pb);
#endif
	return;
}

static void paintbox_mmu_enable_locked(struct paintbox_data *pb,
		uint64_t table_base_paddr)
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

#if CONFIG_PAINTBOX_VERSION_MAJOR == 0
	paintbox_disable_mmu_bif_idle_clock_gating(pb);
#endif

	writel((uint32_t)(PAINTBOX_ERROR_BASE >> MMU_ERROR_BASE_RSHIFT),
			pb->io.axi_base + MMU_ERR_BASE);

	writel((uint32_t)(table_base_paddr >> MMU_TABLE_BASE_RSHIFT),
			pb->io.axi_base + MMU_TABLE_BASE);

	writel(MMU_CTRL_MMU_ENABLE_MASK | MMU_CTRL_PREFETCH_ENABLE_MASK,
			pb->io.axi_base + MMU_CTRL);

#if CONFIG_PAINTBOX_VERSION_MAJOR == 0
	paintbox_enable_mmu_bif_idle_clock_gating(pb);
#endif
}


static void paintbox_mmu_enable(void *priv, uint64_t table_base_paddr)
{
	struct paintbox_data *pb = (struct paintbox_data *)priv;

	mutex_lock(&pb->mmu.lock);

	pb->mmu.table_base_paddr = table_base_paddr;

	paintbox_mmu_enable_locked(pb, table_base_paddr);

	pb->mmu.hw_enabled = true;

	mutex_unlock(&pb->mmu.lock);
}

static void paintbox_mmu_disable(void *priv)
{
	struct paintbox_data *pb = (struct paintbox_data *)priv;

	mutex_lock(&pb->mmu.lock);

	pb->mmu.hw_enabled = false;
	pb->mmu.table_base_paddr = 0;

	writel(0, pb->io.axi_base + MMU_CTRL);
	writel(0, pb->io.axi_base + MMU_IMR);

	paintbox_disable_mmu_interrupt(pb);

	mutex_unlock(&pb->mmu.lock);
}

/* Normally an IOMMU is a separate device that provides translation services for
 * an entire bus, however the Paintbox IOMMU is integrated with the IPU.  To
 * make the Paintbox IOMMU fit within the Linux IOMMU framework it exists as a
 * separate driver.  The core driver communicates with the IOMMU driver across
 * a shared virtual bus.
 */
int paintbox_mmu_iommu_init(struct paintbox_data *pb)
{
	struct paintbox_iommu_pdata *pdata = &pb->mmu.pdata;
	struct device *iommu_dev = &pb->mmu.iommu_dev;
	int ret;

	/* The MMU registers and interrupts are part of the core IPU driver's
	 * register space.  Hooks are provided through platform data to the
	 * IOMMU driver for MMU operations.
	 */
	pdata->mmu_ops.tlb_flush_all = &paintbox_mmu_tlb_flush_all;
	pdata->mmu_ops.tlb_invalidate_range_nosync =
			&paintbox_mmu_tlb_invalidate_range_nosync;
	pdata->mmu_ops.tlb_sync = &paintbox_mmu_tlb_sync;
	pdata->mmu_ops.enable = &paintbox_mmu_enable;
	pdata->mmu_ops.disable = &paintbox_mmu_disable;
	pdata->mmu_ops.priv = pb;

	/* Configuration for the IOMMU driver is provided through platform
	 * data.
	 */
	pdata->config.page_size_bitmap = PAINTBOX_PAGE_SIZE_BITMAP;
	pdata->config.input_address_size = PAINTBOX_INPUT_ADDR_SIZE;
	pdata->config.output_address_size = PAINTBOX_OUTPUT_ADDR_SIZE;

	iommu_dev->platform_data = pdata;
	iommu_dev->bus = &paintbox_bus_type;

	/* TODO(ahampson):  Look for a better way to do this.  Normally it is
	 * done in OF but since we are manually constructing the IOMMU device we
	 * need to do it here.
	 */
	iommu_dev->coherent_dma_mask = DMA_BIT_MASK(PAINTBOX_INPUT_ADDR_SIZE);
	iommu_dev->dma_mask = &iommu_dev->coherent_dma_mask;

	dev_set_name(iommu_dev, "paintbox-iommu");

	ret = device_register(iommu_dev);
	if (ret < 0) {
		put_device(iommu_dev);
		return ret;
	}

	/* Normally an IOMMU driver can recover its driver data using
	 * dev->bus->iommu_ops->priv but the Paintbox IOMMU is internal to the
	 * IPU and does not sit on the same bus.  To workaround this the IPU
	 * device will store the IOMMU device in its platform data.
	 */
	pb->pdev->dev.platform_data = iommu_dev;

	/* Clear the dma ops for the IOMMU device and setup the dma_ops for the
	 * IOMMU.  The arm64 dma map code will set up the swiotlb dma map for
	 * dma device.
	 */
	iommu_dev->archdata.dma_ops = NULL;
	arch_setup_dma_ops(iommu_dev, 0, 0, NULL, false /* coherent */);

	return 0;
}

int paintbox_mmu_start(struct paintbox_data *pb)
{
	struct bus_type *bus = &paintbox_bus_type;
	const struct iommu_ops *iommu_ops = bus->iommu_ops;
	int ret;

	if (!iommu_ops || !iommu_ops->device_group) {
		dev_err(&pb->pdev->dev, "%s: no iommu_ops\n", __func__);
		return -EINVAL;
	}

	pb->mmu.group = iommu_ops->device_group(&pb->pdev->dev);
	if (IS_ERR_OR_NULL(pb->mmu.group)) {
		dev_err(&pb->pdev->dev,
				"%s: failed to get group, ret = %ld\n",
				__func__, PTR_ERR(pb->mmu.group));
		return PTR_ERR(pb->mmu.group);
	}

	ret = iommu_group_add_device(pb->mmu.group, &pb->pdev->dev);
	if (ret < 0) {
		iommu_group_put(pb->mmu.group);
		dev_err(&pb->pdev->dev,
				"%s: unable to attach IPU to IOMMU, ret = %d",
				__func__, ret);
		return ret;
	}

	/* iommu_group_alloc() adds a reference to the reference count for the
	 * kobject in the IOMMU group.  Now that the IPU device has been added
	 * to the group we can remove this reference.  The only reference on the
	 * IOMMU group should now be the IPU device.
	 */
	iommu_group_put(pb->mmu.group);

	/* Change the dma ops for the IPU device to the newly created IOMMU
	 * device.
	 */
	arch_setup_dma_ops(&pb->pdev->dev, PAINTBOX_IOVA_START,
			PAINTBOX_IOVA_SIZE,
			(struct iommu_ops *)paintbox_bus_type.iommu_ops,
			false /* coherent */);

	pb->mmu.iommu_enabled = true;

	return 0;
}

/* Note this this a debug inferface and it should only be used when the DMA and
 * MMU blocks do not have active transfers.
 */
void paintbox_mmu_shutdown(struct paintbox_data *pb)
{
	/* Teardown the DMA ops.  This will detach the IPU device from the
	 * Paintbox IOMMU.
	 */
	arch_teardown_dma_ops(&pb->pdev->dev);

	/* Remove the IPU device from the IOMMU group.  This will release the
	 * last reference to the group causing it to be freed.
	 */
	iommu_group_remove_device(&pb->pdev->dev);

	/* Set the DMA ops for the IPU device back to the default, swiotlb */
	arch_setup_dma_ops(&pb->pdev->dev, 0, 0, NULL, false /* coherent */);

	pb->mmu.iommu_enabled = false;
}
#endif

#ifdef CONFIG_PAINTBOX_DEBUG
#ifdef CONFIG_PAINTBOX_IOMMU
static int paintbox_mmu_enable_show(struct seq_file *s, void *p)
{
	struct paintbox_data *pb = s->private;
	seq_printf(s, "%d\n", pb->mmu.iommu_enabled);
	return 0;
}

static int paintbox_mmu_enable_open(struct inode *inode, struct file *file)
{
	return single_open(file, paintbox_mmu_enable_show, inode->i_private);
}

static ssize_t paintbox_mmu_enable_write(struct file *file,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	struct seq_file *s = (struct seq_file *)file->private_data;
	struct paintbox_data *pb = s->private;
	int ret, val;

	ret = kstrtoint_from_user(user_buf, count, 0, &val);
	if (ret == 0) {
		if (!pb->mmu.iommu_enabled && val == 1) {
			ret = paintbox_mmu_start(pb);
			return ret < 0 ? ret : count;
		}

		if (pb->mmu.iommu_enabled && val == 0) {
			paintbox_mmu_shutdown(pb);
			return count;
		}
	}

	dev_err(&pb->pdev->dev, "%s: invalid value, err = %d", __func__, ret);
	return ret < 0 ? ret : count;
}

static const struct file_operations enable_fops = {
	.open = paintbox_mmu_enable_open,
	.write = paintbox_mmu_enable_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.owner = THIS_MODULE,
};
#endif

int paintbox_mmu_debug_init(struct paintbox_data *pb)
{
	paintbox_debug_create_entry(pb, &pb->mmu.debug, pb->debug_root,
			"mmu", -1, paintbox_dump_mmu_registers, NULL,
			&pb->mmu);

	paintbox_debug_create_reg_entries(pb, &pb->mmu.debug,
			paintbox_mmu_reg_names, IO_AXI_NUM_REGS,
			paintbox_mmu_reg_entry_write,
			paintbox_mmu_reg_entry_read);

#ifdef CONFIG_PAINTBOX_IOMMU
	pb->mmu.enable_dentry = debugfs_create_file("enable",
			S_IRUSR | S_IRGRP | S_IWUSR, pb->mmu.debug.debug_dir,
			pb, &enable_fops);
	if (IS_ERR(pb->mmu.enable_dentry)) {
		dev_err(&pb->pdev->dev, "%s: err = %ld",__func__,
				PTR_ERR(pb->mmu.enable_dentry));
		return PTR_ERR(pb->mmu.enable_dentry);
	}
#endif

	return 0;
}
#endif

int paintbox_mmu_init(struct paintbox_data *pb)
{
	int ret = 0;

#ifdef CONFIG_PAINTBOX_DEBUG
	paintbox_mmu_debug_init(pb);
#endif

#ifdef CONFIG_PAINTBOX_IOMMU
	mutex_init(&pb->mmu.lock);

	ret = paintbox_mmu_iommu_init(pb);
	if (ret < 0)
		return ret;

#ifdef CONFIG_PAINTBOX_IOMMU_ENABLED
	ret = paintbox_mmu_start(pb);
#endif
#endif

	return ret;
}

/* All sessions must be released before remove can be called. */
void paintbox_mmu_remove(struct paintbox_data *pb)
{
#ifdef CONFIG_PAINTBOX_IOMMU
	paintbox_mmu_shutdown(pb);
#endif

#ifdef CONFIG_PAINTBOX_DEBUG
	debugfs_remove(pb->mmu.enable_dentry);
	paintbox_debug_free_reg_entries(&pb->mmu.debug);
	paintbox_debug_free_entry(&pb->mmu.debug);
#endif

#ifdef CONFIG_PAINTBOX_IOMMU
	device_unregister(&pb->mmu.iommu_dev);
	mutex_destroy(&pb->mmu.lock);
#endif
}

/* The Linux IOMMU is designed around an IOMMU providing translation services to
 * all devices on a particular bus.  The Paintbox IOMMU is integrated into the
 * Paintbox IPU.  To make the Paintbox IOMMU fit within the Linux IOMMU
 * framework we will create a virtual bus between the core paintbox drver and
 * the IOMMU driver.
 */
static int paintbox_bus_match(struct device *dev, struct device_driver *drv)
{
	/* Just do a simple match based on the device and driver names */
	if (strcmp(dev_name(dev), drv->name) == 0)
		return 1;

	return 0;
}

struct bus_type paintbox_bus_type = {
	.name	= "paintbox",
	.match	= paintbox_bus_match,
};
EXPORT_SYMBOL(paintbox_bus_type);

static int __init paintbox_bus_driver_init(void)
{
	return bus_register(&paintbox_bus_type);
}

postcore_initcall(paintbox_bus_driver_init);
