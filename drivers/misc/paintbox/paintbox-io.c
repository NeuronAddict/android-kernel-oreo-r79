/*
 * IO support for the Paintbox programmable IPU
 *
 * Copyright (C) 2016 Google, Inc.
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
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include "paintbox-common.h"
#include "paintbox-dma.h"
#include "paintbox-io.h"
#include "paintbox-regs.h"
#include "paintbox-utils.h"


#if defined(CONFIG_DEBUG_FS) || defined(VERBOSE_DEBUG)

#ifdef CONFIG_PAINTBOX_FPGA_SUPPORT
static const char *io_fpga_reg_names[IO_FPGA_NUM_REGS] = {
	REG_NAME_ENTRY(INPUT_FRAME_WIDTH),
	REG_NAME_ENTRY(INPUT_FRAME_HEIGHT),
	REG_NAME_ENTRY(DST_LBP),
	REG_NAME_ENTRY(DST_LB_ID),
	REG_NAME_ENTRY(DST_CHAN_ID),
	REG_NAME_ENTRY(NUM_INPUT_CHANNELS),
	REG_NAME_ENTRY(OUTPUT_FRAME_WIDTH),
	REG_NAME_ENTRY(OUTPUT_FRAME_HEIGHT),
	REG_NAME_ENTRY(SRC_LBP),
	REG_NAME_ENTRY(SRC_LB_ID),
	REG_NAME_ENTRY(SRC_RPTR_ID),
	REG_NAME_ENTRY(NUM_OUTPUT_CHANNELS)
};

static inline void dump_io_fpga_reg(struct paintbox_data *pb, uint32_t reg,
		char *buf, int *written, size_t len)
{
	const char *reg_name = io_fpga_reg_names[REG_INDEX(reg)];
	*written += snprintf(buf + *written, len - *written,
			"0x%04lx: %s\t0x%08x\n",
			pb->io.fpga_base - pb->reg_base + reg,
			reg_name ? reg_name : REG_UNUSED,
			readl(pb->io.fpga_base + reg));
}

int dump_io_fpga_registers(struct paintbox_data *pb, char *buf, size_t len)
{
	int written;

	written = snprintf(buf, len, "io:\n");
	dump_io_fpga_reg(pb, INPUT_FRAME_WIDTH, buf, &written, len);
	dump_io_fpga_reg(pb, INPUT_FRAME_HEIGHT, buf, &written, len);
	dump_io_fpga_reg(pb, DST_LBP, buf, &written, len);
	dump_io_fpga_reg(pb, DST_LB_ID, buf, &written, len);
	dump_io_fpga_reg(pb, DST_CHAN_ID, buf, &written, len);
	dump_io_fpga_reg(pb, NUM_INPUT_CHANNELS, buf, &written, len);
	dump_io_fpga_reg(pb, OUTPUT_FRAME_WIDTH, buf, &written, len);
	dump_io_fpga_reg(pb, OUTPUT_FRAME_HEIGHT, buf, &written, len);
	dump_io_fpga_reg(pb, SRC_LBP, buf, &written, len);
	dump_io_fpga_reg(pb, SRC_LB_ID, buf, &written, len);
	dump_io_fpga_reg(pb, SRC_RPTR_ID, buf, &written, len);
	dump_io_fpga_reg(pb, NUM_OUTPUT_CHANNELS, buf, &written, len);

	return written;
}
#endif

static const char *io_axi_reg_names[IO_AXI_NUM_REGS] = {
	REG_NAME_ENTRY(MMU_CTRL),
	REG_NAME_ENTRY(MMU_TABLE_BASE),
	REG_NAME_ENTRY(MMU_ERR_BASE),
	REG_NAME_ENTRY(BIF_AXI_CTRL_DMA),
	REG_NAME_ENTRY(BIF_AXI_CTRL_MMU),
	REG_NAME_ENTRY(BIF_ERR_CFG_STS),
	REG_NAME_ENTRY(BIF_ERR_LOG),
	REG_NAME_ENTRY(BIF_ERR_LOG_BUS_ADDR)
};

static inline void dump_io_axi_reg(struct paintbox_data *pb, uint32_t reg,
		char *buf, int *written, size_t len)
{
	const char *reg_name = io_axi_reg_names[REG_INDEX(reg)];
	*written += snprintf(buf + *written, len - *written,
			"0x%04lx: %s\t0x%08x\n",
			pb->io.axi_base - pb->reg_base + reg,
			reg_name ? reg_name : REG_UNUSED,
			readl(pb->io.axi_base + reg));
}

int dump_io_axi_registers(struct paintbox_data *pb, char *buf, size_t len)
{
	int written = 0;

	dump_io_axi_reg(pb, MMU_CTRL, buf, &written, len);
	dump_io_axi_reg(pb, MMU_TABLE_BASE, buf, &written, len);
	dump_io_axi_reg(pb, MMU_ERR_BASE, buf, &written, len);
	dump_io_axi_reg(pb, BIF_AXI_CTRL_DMA, buf, &written, len);
	dump_io_axi_reg(pb, BIF_AXI_CTRL_MMU, buf, &written, len);
	dump_io_axi_reg(pb, BIF_ERR_LOG, buf, &written, len);
	dump_io_axi_reg(pb, BIF_ERR_LOG_BUS_ADDR, buf, &written, len);

	return written;
}
#endif

#ifdef VERBOSE_DEBUG
static void log_io_axi_registers(struct paintbox_data *pb, const char *msg)
{
	int written;

	written = snprintf(buf, len, "io_axi:\n");
	dump_io_axi_registers(pb, pb->vdbg_log + written, pb->vdbg_log_len -
			written);
	dev_vdbg(&pb->pdev->dev, "%s\n%s", msg, pb->vdbg_log);
}

#define LOG_IO_AXI_REGISTERS(pb)		\
	log_io_axi_registers(pb, __func__)

#else
#define LOG_IO_AXI_REGISTERS(pb)		\
do { } while (0)
#endif

#ifdef CONFIG_DEBUG_FS

#ifdef CONFIG_PAINTBOX_FPGA_SUPPORT
static int io_fpga_debug_regs_show(struct seq_file *s, void *unused)
{
	struct paintbox_data *pb = s->private;
	char *buf;
	size_t len;
	int written;

	len = seq_get_buf(s, &buf);
	if (!buf)
		return -ENOMEM;

	mutex_lock(&pb->lock);

	written = dump_io_fpga_registers(pb, buf, len);

	mutex_unlock(&pb->lock);

	seq_commit(s, written);

	return 0;
}

static int io_fpga_debug_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, io_fpga_debug_regs_show, inode->i_private);
}

static const struct file_operations io_fpga_debug_regs_fops = {
	.open = io_fpga_debug_regs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
#endif

static int io_debug_regs_show(struct seq_file *s, void *unused)
{
	struct paintbox_data *pb = s->private;
	char *buf;
	size_t len;
	int written;

	len = seq_get_buf(s, &buf);
	if (!buf)
		return -ENOMEM;

	mutex_lock(&pb->lock);

	written = snprintf(buf, len, "io_axi:\n");
	written += dump_io_axi_registers(pb, buf + written, len - written);

	mutex_unlock(&pb->lock);

	seq_commit(s, written);

	return 0;
}

static int io_debug_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, io_debug_regs_show, inode->i_private);
}

static const struct file_operations io_debug_regs_fops = {
	.open = io_debug_regs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static void paintbox_io_debug_init(struct paintbox_data *pb)
{
	pb->io.debug_dir = debugfs_create_dir("io", pb->debug_root);

	pb->io.regs_dentry = debugfs_create_file("regs", S_IRUGO | S_IWUSR,
			pb->io.debug_dir, pb, &io_debug_regs_fops);

#ifdef CONFIG_PAINTBOX_FPGA_SUPPORT
	pb->io.fpga_debug_dir = debugfs_create_dir("io_fpga", pb->debug_root);

	pb->io.fpga_regs_dentry = debugfs_create_file("regs", S_IRUGO | S_IWUSR,
			pb->io.fpga_debug_dir, pb, &io_fpga_debug_regs_fops);
#endif
}
#endif

static irqreturn_t paintbox_io_interrupt(int irq, void *arg)
{
	struct paintbox_data *pb = (struct paintbox_data *)arg;
	uint32_t status;

	spin_lock(&pb->irq_lock);

	status = readl(pb->io.apb_base + IPU_ISR);
	writel(status, pb->io.apb_base + IPU_ISR);

	if (status & pb->io.dma_mask)
		paintbox_dma_interrupt(pb, status & pb->io.dma_mask);

	/* TODO(ahampson):  Check for STP, BIF, MMU, and MIPI interrupts */

	spin_unlock(&pb->irq_lock);

	return IRQ_HANDLED;
}

int paintbox_io_init(struct paintbox_data *pb)
{
	int ret;

	pb->io.apb_base = pb->reg_base + IPU_IO_APB_OFFSET;
	pb->io.axi_base = pb->reg_base + IPU_IO_AXI_OFFSET;

#ifdef CONFIG_PAINTBOX_FPGA_SUPPORT
	pb->io.fpga_base = pb->reg_base + IPU_IO_ADB_OFFSET;

	dev_dbg(&pb->pdev->dev, "io_fpga: base %p len %u\n", pb->io.fpga_base,
			IO_FPGA_BLOCK_LEN);
#endif

#ifdef CONFIG_DEBUG_FS
	paintbox_io_debug_init(pb);
#endif

#ifdef VERBOSE_DEBUG
	paintbox_alloc_debug_buffer(pb, IO_AXI_DEBUG_BUFFER_SIZE);
#endif

#ifndef CONFIG_PAINTBOX_FPGA_SUPPORT
	ret = devm_request_irq(&pb->pdev->dev, pb->io.irq,
			paintbox_io_interrupt, IRQF_SHARED, pb->pdev->name, pb);
	if (ret < 0)
		return ret;
#endif

	dev_dbg(&pb->pdev->dev, "io_axi: base %p len %u\n",
			pb->io.axi_base, IO_AXI_BLOCK_LEN);

	return 0;
}
