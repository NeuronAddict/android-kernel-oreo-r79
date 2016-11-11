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
#include "paintbox-debug.h"
#include "paintbox-dma.h"
#include "paintbox-io.h"
#include "paintbox-mipi.h"
#include "paintbox-regs.h"
#include "paintbox-regs-supplemental.h"
#include "paintbox-stp.h"


#ifdef CONFIG_DEBUG_FS
static uint64_t io_apb_reg_entry_read(
		struct paintbox_debug_reg_entry *reg_entry)
{
	struct paintbox_debug *debug = reg_entry->debug;
	struct paintbox_io *io = container_of(debug, struct paintbox_io,
			apb_debug);
	return readq(io->apb_base + reg_entry->reg_offset);
}

static void io_apb_reg_entry_write(struct paintbox_debug_reg_entry *reg_entry,
		uint64_t val)
{
	struct paintbox_debug *debug = reg_entry->debug;
	struct paintbox_io *io = container_of(debug, struct paintbox_io,
			apb_debug);
	writeq(val, io->apb_base + reg_entry->reg_offset);
}

static uint64_t io_axi_reg_entry_read(
		struct paintbox_debug_reg_entry *reg_entry)
{
	struct paintbox_debug *debug = reg_entry->debug;
	struct paintbox_io *io = container_of(debug, struct paintbox_io,
			axi_debug);
	return readq(io->axi_base + reg_entry->reg_offset);
}

static void io_axi_reg_entry_write(struct paintbox_debug_reg_entry *reg_entry,
		uint64_t val)
{
	struct paintbox_debug *debug = reg_entry->debug;
	struct paintbox_io *io = container_of(debug, struct paintbox_io,
			axi_debug);
	writeq(val, io->axi_base + reg_entry->reg_offset);
}
#endif

#if defined(CONFIG_DEBUG_FS) || defined(VERBOSE_DEBUG)
static const char *io_apb_reg_names[IO_APB_NUM_REGS] = {
	REG_NAME_ENTRY(IPU_VERSION),
	REG_NAME_ENTRY(IPU_CHECKSUM),
	REG_NAME_ENTRY(IPU_ISR),
	REG_NAME_ENTRY(IPU_IMR),
	REG_NAME_ENTRY(IPU_CAP),
	REG_NAME_ENTRY(CLK_GATE_CONTROL_STP_IDLE_GATE_DIS),
	REG_NAME_ENTRY(CLK_GATE_CONTROL_LBP_IDLE_GATE_DIS),
	REG_NAME_ENTRY(CLK_GATE_CONTROL),
	REG_NAME_ENTRY(IPU_CORE_PAIRS_EN),
	REG_NAME_ENTRY(CORE_POWER_ON_N),
	REG_NAME_ENTRY(CORE_RAM_ON_N),
	REG_NAME_ENTRY(IPU_CORE_PAIRS_EN),
	REG_NAME_ENTRY(IPU_DMA_CHAN_EN)
};

static inline int dump_io_apb_reg(struct paintbox_data *pb, uint32_t reg_offset,
		char *buf, int *written, size_t len)
{
	const char *reg_name = io_apb_reg_names[REG_INDEX(reg_offset)];
	return dump_ipu_register64(pb, pb->io.apb_base, reg_offset, reg_name,
			buf, written, len);
}

int dump_io_apb_registers(struct paintbox_debug *debug, char *buf, size_t len)
{
	struct paintbox_data *pb = debug->pb;
	int ret, written = 0;

	ret = dump_io_apb_reg(pb, IPU_VERSION, buf, &written, len);
	if (ret < 0)
		goto err_exit;

	ret = dump_io_apb_reg(pb, IPU_CHECKSUM, buf, &written, len);
	if (ret < 0)
		goto err_exit;

	ret = dump_io_apb_reg(pb, IPU_ISR, buf, &written, len);
	if (ret < 0)
		goto err_exit;

	ret = dump_io_apb_reg(pb, IPU_IMR, buf, &written, len);
	if (ret < 0)
		goto err_exit;

	ret = dump_io_apb_reg(pb, IPU_CAP, buf, &written, len);
	if (ret < 0)
		goto err_exit;

	ret = dump_io_apb_reg(pb, CLK_GATE_CONTROL_STP_IDLE_GATE_DIS, buf,
			&written, len);
	if (ret < 0)
		goto err_exit;

	ret = dump_io_apb_reg(pb, CLK_GATE_CONTROL_LBP_IDLE_GATE_DIS, buf,
			&written, len);
	if (ret < 0)
		goto err_exit;

	ret = dump_io_apb_reg(pb, CLK_GATE_CONTROL, buf, &written, len);
	if (ret < 0)
		goto err_exit;

	ret = dump_io_apb_reg(pb, IPU_CORE_PAIRS_EN, buf, &written, len);
	if (ret < 0)
		goto err_exit;

	ret = dump_io_apb_reg(pb, CORE_POWER_ON_N, buf, &written, len);
	if (ret < 0)
		goto err_exit;

	ret = dump_io_apb_reg(pb, CORE_RAM_ON_N, buf, &written, len);
	if (ret < 0)
		goto err_exit;

	ret = dump_io_apb_reg(pb, IPU_CORE_PAIRS_EN, buf, &written, len);
	if (ret < 0)
		goto err_exit;

	ret = dump_io_apb_reg(pb, IPU_DMA_CHAN_EN, buf, &written, len);
	if (ret < 0)
		goto err_exit;

	return written;

err_exit:
	dev_err(&pb->pdev->dev, "%s: register dump error, err = %d", __func__,
			ret);
	return ret;
}

static const char *io_axi_reg_names[IO_AXI_NUM_REGS] = {
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
	REG_NAME_ENTRY(BIF_AXI_CTRL_DMA0),
	REG_NAME_ENTRY(BIF_AXI_CTRL_DMA1),
	REG_NAME_ENTRY(BIF_AXI_CTRL_DMA2),
	REG_NAME_ENTRY(BIF_AXI_CTRL_DMA3),
	REG_NAME_ENTRY(BIF_AXI_CTRL_DMA4),
	REG_NAME_ENTRY(BIF_AXI_CTRL_DMA5),
	REG_NAME_ENTRY(BIF_AXI_CTRL_MMU),
	REG_NAME_ENTRY(BIF_IMR),
	REG_NAME_ENTRY(BIF_ISR),
	REG_NAME_ENTRY(BIF_ISR_OVF),
	REG_NAME_ENTRY(BIF_TO_ERR_CFG),
	REG_NAME_ENTRY(BIF_ERR_LOG),
	REG_NAME_ENTRY(BIF_ERR_LOG_BUS_ADDR),
	REG_NAME_ENTRY(BIF_PMON_CFG),
	REG_NAME_ENTRY(BIF_PMON_CNT_0_CFG),
	REG_NAME_ENTRY(BIF_PMON_CNT_0),
	REG_NAME_ENTRY(BIF_PMON_CNT_0_STS_ACC),
	REG_NAME_ENTRY(BIF_PMON_CNT_0_STS),
	REG_NAME_ENTRY(BIF_PMON_CNT_1_CFG),
	REG_NAME_ENTRY(BIF_PMON_CNT_1),
	REG_NAME_ENTRY(BIF_PMON_CNT_1_STS_ACC),
	REG_NAME_ENTRY(BIF_PMON_CNT_1_STS),
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

int dump_io_apb_stats(struct paintbox_debug *debug, char *buf,
		size_t len)
{
	struct paintbox_data *pb = debug->pb;

	return snprintf(buf, len, "IPU interrupts: %u IRQ activations %u\n",
			pb->io.ipu_interrupts, pb->io.irq_activations);
}

static inline int dump_io_axi_reg(struct paintbox_data *pb, uint32_t reg_offset,
		char *buf, int *written, size_t len)
{
	const char *reg_name = io_axi_reg_names[REG_INDEX(reg_offset)];
	return dump_ipu_register64(pb, pb->io.axi_base, reg_offset, reg_name,
			buf, written, len);
}

int dump_io_axi_registers(struct paintbox_debug *debug, char *buf, size_t len)
{
	struct paintbox_data *pb = debug->pb;
	unsigned int i;
	int ret, written = 0;

	for (i = 0; i < IO_AXI_NUM_REGS; i++) {
		if (io_axi_reg_names[i] != NULL) {
			ret = dump_io_axi_reg(pb, i * IPU_REG_WIDTH, buf,
					&written, len);
			if (ret < 0) {
				dev_err(&pb->pdev->dev,
						"%s: register dump error, err ="
						"%d", __func__, ret);
				return ret;
			}
		}
	}

	return written;
}
#endif

#ifdef VERBOSE_DEBUG
static void log_io_apb_registers(struct paintbox_data *pb, const char *msg)
{
	int ret, written;

	ret = snprintf(pb->vdbg_log, pb->vdbg_log_len, "io_apb:\n");
	if (ret < 0)
		return;

	written = ret;

	dump_io_apb_registers(&pb->io.apb_debug, pb->vdbg_log + written,
			pb->vdbg_log_len - written);
	dev_vdbg(&pb->pdev->dev, "%s\n%s", msg, pb->vdbg_log);
}

static void log_io_axi_registers(struct paintbox_data *pb, const char *msg)
{
	int ret, written;

	ret = snprintf(pb->vdbg_log, pb->vdbg_log_len, "io_axi:\n");
	if (ret < 0)
		return;

	written = ret;

	dump_io_axi_registers(&pb->io.axi_debug, pb->vdbg_log + written,
			pb->vdbg_log_len - written);
	dev_vdbg(&pb->pdev->dev, "%s\n%s", msg, pb->vdbg_log);
}

#define LOG_IO_APB_REGISTERS(pb)		\
	log_io_apb_registers(pb, __func__)

#define LOG_IO_AXI_REGISTERS(pb)		\
	log_io_axi_registers(pb, __func__)

#else
#define LOG_IO_APB_REGISTERS(pb)		\
do { } while (0)

#define LOG_IO_AXI_REGISTERS(pb)		\
do { } while (0)
#endif


static irqreturn_t paintbox_io_interrupt(int irq, void *arg)
{
	struct paintbox_data *pb = (struct paintbox_data *)arg;
	uint32_t status;

	pb->io.irq_activations++;

	status = readl(pb->io.apb_base + IPU_ISR);
	if (status == 0)
		return IRQ_NONE;

	writel(status, pb->io.apb_base + IPU_ISR);

	pb->io.ipu_interrupts++;

	/* MIPI interrupts need to be processed before DMA interrupts so error
	 * conditions like MIPI input overflow can be reported properly.  A
	 * MIPI OVF interrupt will cause an early EOF on the associated DMA
	 * channel.  This DMA EOF interrupt needs to be reported as an error and
	 * not as a normal completion.
	 */
	if (status & pb->io.mipi_input_mask)
		paintbox_mipi_input_interrupt(pb, (status &
				pb->io.mipi_input_mask) >>
				pb->io.mipi_input_start);

	if (status & pb->io.mipi_output_mask)
		paintbox_mipi_output_interrupt(pb, (status &
				pb->io.mipi_output_mask) >>
				pb->io.mipi_output_start);

	if (status & pb->io.dma_mask)
		paintbox_dma_interrupt(pb, status & pb->io.dma_mask);

	if (status & pb->io.stp_mask)
		paintbox_stp_interrupt(pb, (status & pb->io.stp_mask) >>
				pb->io.stp_start);

	/* TODO(ahampson):  Check for BIF, and MMU interrupts */

	return IRQ_HANDLED;
}

/* The caller to this function must hold pb->dma.dma_lock */
void io_enable_dma_channel(struct paintbox_data *pb, unsigned int channel_id)
{
	uint32_t val;

	val = readl(pb->io.apb_base + IPU_DMA_CHAN_EN);
	val |= 1 << channel_id;
	writel(val, pb->io.apb_base + IPU_DMA_CHAN_EN);

}

/* The caller to this function must hold pb->dma.dma_lock */
void io_disable_dma_channel(struct paintbox_data *pb, unsigned int channel_id)
{
	uint32_t val;

	val = readl(pb->io.apb_base + IPU_DMA_CHAN_EN);
	val &= ~(1 << channel_id);
	writel(val, pb->io.apb_base + IPU_DMA_CHAN_EN);
}

void io_enable_dma_channel_interrupt(struct paintbox_data *pb,
		unsigned int channel_id)
{
	unsigned long irq_flags;
	uint32_t ipu_imr;

	spin_lock_irqsave(&pb->io.io_lock, irq_flags);

	ipu_imr = readl(pb->io.apb_base + IPU_IMR);
	ipu_imr |= 1 << channel_id;
	writel(ipu_imr, pb->io.apb_base + IPU_IMR);

	spin_unlock_irqrestore(&pb->io.io_lock, irq_flags);
}

void io_disable_dma_channel_interrupt(struct paintbox_data *pb,
		unsigned int channel_id)
{
	unsigned long irq_flags;
	uint32_t ipu_imr;

	spin_lock_irqsave(&pb->io.io_lock, irq_flags);

	ipu_imr = readl(pb->io.apb_base + IPU_IMR);
	ipu_imr &= ~(1 << channel_id);
	writel(ipu_imr, pb->io.apb_base + IPU_IMR);

	spin_unlock_irqrestore(&pb->io.io_lock, irq_flags);
}

void io_enable_stp_interrupt(struct paintbox_data *pb, unsigned int stp_id)
{
	unsigned long irq_flags;
	uint32_t ipu_imr;

	spin_lock_irqsave(&pb->io.io_lock, irq_flags);

	ipu_imr = readl(pb->io.apb_base + IPU_IMR);
	ipu_imr |= 1 << (stp_id_to_index(stp_id) + pb->io.stp_start);
	writel(ipu_imr, pb->io.apb_base + IPU_IMR);

	spin_unlock_irqrestore(&pb->io.io_lock, irq_flags);
}

void io_disable_stp_interrupt(struct paintbox_data *pb, unsigned int stp_id)
{
	unsigned long irq_flags;
	uint32_t ipu_imr;

	spin_lock_irqsave(&pb->io.io_lock, irq_flags);

	ipu_imr = readl(pb->io.apb_base + IPU_IMR);
	ipu_imr &= ~(1 << (stp_id_to_index(stp_id) + pb->io.stp_start));
	writel(ipu_imr, pb->io.apb_base + IPU_IMR);

	spin_unlock_irqrestore(&pb->io.io_lock, irq_flags);
}

static void io_enable_mipi_interface_interrupt(struct paintbox_data *pb,
		unsigned int interface_offset)
{
	unsigned long irq_flags;
	uint32_t ipu_imr;

	spin_lock_irqsave(&pb->io.io_lock, irq_flags);

	ipu_imr = readl(pb->io.apb_base + IPU_IMR);
	ipu_imr |= 1 << interface_offset;
	writel(ipu_imr, pb->io.apb_base + IPU_IMR);

	spin_unlock_irqrestore(&pb->io.io_lock, irq_flags);
}

static void io_disable_mipi_interface_interrupt(struct paintbox_data *pb,
		unsigned int interface_offset)
{
	unsigned long irq_flags;
	uint32_t ipu_imr;

	spin_lock_irqsave(&pb->io.io_lock, irq_flags);

	ipu_imr = readl(pb->io.apb_base + IPU_IMR);
	ipu_imr &= ~(1 << interface_offset);
	writel(ipu_imr, pb->io.apb_base + IPU_IMR);

	spin_unlock_irqrestore(&pb->io.io_lock, irq_flags);
}

void io_enable_mipi_input_interface_interrupt(struct paintbox_data *pb,
		unsigned int interface_id)
{
	io_enable_mipi_interface_interrupt(pb, interface_id +
			pb->io.mipi_input_start);
}

void io_disable_mipi_input_interface_interrupt(struct paintbox_data *pb,
		unsigned int interface_id)
{
	io_disable_mipi_interface_interrupt(pb, interface_id +
			pb->io.mipi_input_start);
}

void io_enable_mipi_output_interface_interrupt(struct paintbox_data *pb,
		unsigned int interface_id)
{
	io_enable_mipi_interface_interrupt(pb, interface_id +
			pb->io.mipi_output_start);
}

void io_disable_mipi_output_interface_interrupt(struct paintbox_data *pb,
		unsigned int interface_id)
{
	io_disable_mipi_interface_interrupt(pb, interface_id +
			pb->io.mipi_output_start);
}

int paintbox_io_axi_init(struct paintbox_data *pb)
{
	pb->io.axi_base = pb->reg_base + IPU_IO_AXI_OFFSET;

#ifdef CONFIG_DEBUG_FS
	paintbox_debug_create_entry(pb, &pb->io.axi_debug, pb->debug_root,
			"axi", -1, dump_io_axi_registers, NULL, &pb->io);

	paintbox_debug_create_reg_entries(pb, &pb->io.axi_debug,
			io_axi_reg_names, IO_AXI_NUM_REGS,
			io_axi_reg_entry_write, io_axi_reg_entry_read);
#endif

#ifdef VERBOSE_DEBUG
	paintbox_alloc_debug_buffer(pb, IO_AXI_DEBUG_BUFFER_SIZE);
#endif

	dev_dbg(&pb->pdev->dev, "io_axi: base %p len %lu\n",
			pb->io.axi_base, IO_AXI_BLOCK_LEN);

	return 0;
}

int paintbox_io_apb_init(struct paintbox_data *pb)
{
	int ret;

	spin_lock_init(&pb->io.io_lock);

	pb->io.apb_base = pb->reg_base + IPU_IO_APB_OFFSET;

#ifdef CONFIG_DEBUG_FS
	paintbox_debug_create_entry(pb, &pb->io.apb_debug, pb->debug_root,
			"apb", -1, dump_io_apb_registers, dump_io_apb_stats,
			&pb->io);

	paintbox_debug_create_reg_entries(pb, &pb->io.apb_debug,
			io_apb_reg_names, IO_APB_NUM_REGS,
			io_apb_reg_entry_write, io_apb_reg_entry_read);
#endif

#ifdef VERBOSE_DEBUG
	paintbox_alloc_debug_buffer(pb, IO_APB_DEBUG_BUFFER_SIZE);
#endif

	pb->io.dma_mask = (1 << pb->dma.num_channels) - 1;

	pb->io.stp_start = pb->dma.num_channels;
	pb->io.stp_mask = ((1 << pb->caps.num_stps) - 1) << pb->io.stp_start;
	pb->io.bif_start = pb->io.stp_start + pb->caps.num_stps;
	pb->io.bif_mask = ((1 << NUM_BIF_INTERRUPTS) - 1) << pb->io.bif_start;
	pb->io.mmu_start = pb->io.bif_start + NUM_BIF_INTERRUPTS;
	pb->io.mmu_mask = ((1 << NUM_MMU_INTERRUPTS) - 1) << pb->io.mmu_start;

	/* Update the number of available interrupts reported to the user space.
	 * This value is also used to allocate the number of IRQ waiter objects.
	 *
	 * TODO(ahampson):  The IRQ waiter code should be modified to allocate
	 * IRQ waiter objects on demand.  The fixed relationship between the
	 * number of IRQ waiters and the number of interrupts is arbitrary and
	 * should be cleaned up.  b/31684858
	 */
	pb->caps.num_interrupts = pb->io.mmu_start + NUM_MMU_INTERRUPTS;

	if (pb->io_ipu.num_mipi_input_interfaces > 0) {
		pb->io.mipi_input_start = pb->io.mmu_start + NUM_MMU_INTERRUPTS;
		pb->io.mipi_input_mask = ((1 <<
				pb->io_ipu.num_mipi_input_interfaces) -
				1) << pb->io.mipi_input_start;

		/* For the number of interrupts available that is reported to
		 * the user space we want to have an interrupt per MIPI stream
		 * rather than per interface.
		 */
		pb->caps.num_interrupts += pb->io_ipu.num_mipi_input_streams;
	}

	if (pb->io_ipu.num_mipi_output_interfaces > 0) {
		pb->io.mipi_output_start = pb->io.mipi_input_start +
				pb->io_ipu.num_mipi_input_interfaces;
		pb->io.mipi_output_mask = ((1 <<
				pb->io_ipu.num_mipi_output_interfaces) -
				1) << pb->io.mipi_output_start;

		/* For the number of interrupts available that is reported to
		 * the user space we want to have an interrupt per MIPI stream
		 * rather than per interface.
		 */
		pb->caps.num_interrupts += pb->io_ipu.num_mipi_output_streams;
	}

	ret = devm_request_irq(&pb->pdev->dev, pb->io.irq,
			paintbox_io_interrupt, IRQF_SHARED, pb->pdev->name, pb);
	if (ret < 0)
		return ret;

	dev_dbg(&pb->pdev->dev, "io_apb: base %p len %lu\n",
			pb->io.apb_base, IO_APB_BLOCK_LEN);

	return 0;
}
