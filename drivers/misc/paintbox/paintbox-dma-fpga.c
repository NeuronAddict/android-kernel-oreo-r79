/*
 * FPGA DMA support for the Paintbox programmable IPU
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
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/paintbox.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include "paintbox-dma.h"
#include "paintbox-dma-fpga.h"
#include "paintbox-io.h"
#include "paintbox-lbp.h"
#include "paintbox-regs.h"
#include "paintbox-xilinx-regs.h"


#define DMA_SETUP_RETRY_MAX 3
#define IO_NAME_LEN         6
#define XILINX_NAME_LEN     10


#if defined(CONFIG_DEBUG_FS) || defined(VERBOSE_DEBUG)

int dump_dma_registers(struct paintbox_data *pb, char *buf, size_t len)
{
	return 0;
}

int dump_dma_channel_registers(struct paintbox_data *pb,
		unsigned int channel_id, char *buf, size_t len)
{
	return 0;
}

static const char *xilinx_reg_names[XILINX_NUM_REGS] = {
	REG_NAME_ENTRY(MM2S_DMACR),
	REG_NAME_ENTRY(MM2S_DMASR),
	REG_NAME_ENTRY(MM2S_SA),
	REG_NAME_ENTRY(MM2S_SA_MSB),
	REG_NAME_ENTRY(MM2S_LENGTH),
	REG_NAME_ENTRY(S2MM_DMACR),
	REG_NAME_ENTRY(S2MM_DMASR),
	REG_NAME_ENTRY(S2MM_DA),
	REG_NAME_ENTRY(S2MM_DA_MSB),
	REG_NAME_ENTRY(S2MM_LENGTH)
};

static inline void dump_xilinx_reg(struct paintbox_data *pb, uint32_t reg,
		char *buf, int *written, size_t len)
{
	const char *reg_name = xilinx_reg_names[REG_INDEX(reg)];
	*written += snprintf(buf + *written, len - *written,
			"0x%p: %s\t0x%08x\n",
			pb->dma.dma_base + reg,
			reg_name ? reg_name : REG_UNUSED,
			readl(pb->dma.dma_base + reg));
}

int dump_xilinx_registers(struct paintbox_data *pb, char *buf,
		size_t len)
{
	int written;

	written = snprintf(buf, len, "xilinx:\n");
	dump_xilinx_reg(pb, MM2S_DMACR, buf, &written, len);
	dump_xilinx_reg(pb, MM2S_DMASR, buf, &written, len);
	dump_xilinx_reg(pb, MM2S_SA, buf, &written, len);
	dump_xilinx_reg(pb, MM2S_SA_MSB, buf, &written, len);
	dump_xilinx_reg(pb, MM2S_LENGTH, buf, &written, len);
	dump_xilinx_reg(pb, S2MM_DMACR, buf, &written, len);
	dump_xilinx_reg(pb, S2MM_DMASR, buf, &written, len);
	dump_xilinx_reg(pb, S2MM_DA, buf, &written, len);
	dump_xilinx_reg(pb, S2MM_DA_MSB, buf, &written, len);
	dump_xilinx_reg(pb, S2MM_LENGTH, buf, &written, len);

	return written;
}
#endif

#ifdef VERBOSE_DEBUG
static void log_io_fpga_registers(struct paintbox_data *pb, const char *msg)
{
	dump_io_fpga_registers(pb, pb->vdbg_log, pb->vdbg_log_len);
	dev_vdbg(&pb->pdev->dev, "%s\n%s", msg, pb->vdbg_log);
}

static void log_xilinx_registers(struct paintbox_data *pb, const char *msg)
{
	dump_xilinx_registers(pb, pb->vdbg_log, pb->vdbg_log_len);
	dev_vdbg(&pb->pdev->dev, "%s\n%s", msg, pb->vdbg_log);
}


#define LOG_IO_FPGA_REGISTERS(pb)			\
	log_io_fpga_registers(pb, __func__)

#define LOG_XILINX_REGISTERS(pb)		\
	log_xilinx_registers(pb, __func__)

#else
#define LOG_IO_FPGA_REGISTERS(pb)			\
do { } while (0)

#define LOG_XILINX_REGISTERS(pb)		\
do { } while (0)
#endif


/* The caller to this function must hold pb->lock */
static int dma_setup_transfer_common(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer,
		struct dma_transfer_config *config)
{
	struct paintbox_fpga_dma_channel *fpga = &channel->fpga;
	uint32_t val;
	unsigned int bytes_per_component, count = 0;
	unsigned long irq_flags;

	switch (config->img.bit_depth) {
	case 8:
		bytes_per_component = 1;
		break;
	case 16:
		bytes_per_component = 2;
		break;
	default:
		dev_err(&pb->pdev->dev, "%s: dma%u: unsupported bit depth %u",
			__func__, channel->channel_id, config->img.bit_depth);
		return -EINVAL;
	};

	spin_lock_irqsave(&pb->irq_lock, irq_flags);

	/* Store transfer information in the channel object for later use when
	 * the channels starts. Setup and start are presented as
	 * separate operations in this driver so this information needs to be
	 * cached.
	 */
	fpga->requested_transfer_len = config->img.width_pixels *
			config->img.height_pixels * bytes_per_component *
			config->img.components;

	/* MM2S and S2MM channel registers are identical but are at different
	 * offsets.  The code below uses the MM2S mnemonics bits but these will
	 * work for both channels.
	 */

	/* Enable the channel */
	writel(1 << DMACR_IRQ_THRESH_SHIFT | DMACR_RS, fpga->xilinx_base +
			MM2S_DMACR);

	/* Wait for the HALTED bit to clear following enable */
	do {
		val = readl(fpga->xilinx_base + MM2S_DMASR);
		if ((val & DMASR_HALTED) == 0)
			break;
		spin_unlock_irqrestore(&pb->irq_lock, irq_flags);
		usleep_range(100, 500);

		if (count++ > DMA_SETUP_RETRY_MAX) {
			dev_err(&pb->pdev->dev,  "%s: dma%u: channel not ready",
				__func__, config->channel_id);
			return -EIO;
		}
		spin_lock_irqsave(&pb->irq_lock, irq_flags);
	} while (1);

	/* Enable IOC and ERR interrupts for the channel. */
	val = readl(fpga->xilinx_base + MM2S_DMACR);
	val |= DMACR_IOC_IRQ_EN | DMACR_ERR_IRQ_EN;
	writel(val, fpga->xilinx_base + MM2S_DMACR);

	/* Program the paddr of the channel dma mem buffer */
	writel((uint32_t)transfer->buf_paddr, fpga->xilinx_base + MM2S_SA);
	writel((uint32_t)((uint64_t)transfer->buf_paddr >> 32),
			fpga->xilinx_base + MM2S_SA_MSB);

	/* The Xilinx controller will start the transfer when the length field
	 * is set.  This will be done as part of the start transfer operation.
	 */

	spin_unlock_irqrestore(&pb->irq_lock, irq_flags);

	return 0;
}

/* The caller to this function must hold pb->lock */
int dma_setup_dram_to_lbp_transfer(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer,
		struct dma_transfer_config *config)
{
	struct paintbox_fpga_dma_channel *fpga = &channel->fpga;
	int ret;

	if (!fpga->to_ipu) {
		dev_err(&pb->pdev->dev,
				"%s: dma%u: channel direction mismatch "
				"transfer type %u supported %u\n",
				__func__, config->channel_id,
				config->transfer_type, fpga->to_ipu);
		return -EINVAL;
	}

	ret = validate_lbp(pb, session, config->dst.lbp.lbp_id);
	if (ret < 0)
		return ret;

	writel(config->img.width_pixels, pb->io.fpga_base + INPUT_FRAME_WIDTH);
	writel(config->img.height_pixels, pb->io.fpga_base +
			INPUT_FRAME_HEIGHT);
	writel(lbp_id_to_noc_id(config->dst.lbp.lbp_id), pb->io.fpga_base +
			DST_LBP);
	writel(config->dst.lbp.lb_id, pb->io.fpga_base + DST_LB_ID);

	/* TODO(ahampson): DST_CHAN_ID currently does not do anything on the
	 * FPGA.  It should correspond to the plane_index but this will need to
	 * be checked when the DST_CHAN_ID is hooked up in the FPGA.
	 */
	writel(0, pb->io.fpga_base + DST_CHAN_ID);
	writel(config->img.components, pb->io.fpga_base + NUM_INPUT_CHANNELS);

	LOG_IO_FPGA_REGISTERS(pb);

	dev_dbg(&pb->pdev->dev, "%s: dma%u: DRAM -> lbp%u lb%u\n", __func__,
			config->channel_id, config->dst.lbp.lbp_id,
			config->dst.lbp.lb_id);

	return dma_setup_transfer_common(pb, channel, transfer, config);
}

/* The caller to this function must hold pb->lock */
int dma_setup_dram_to_stp_transfer(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer,
		struct dma_transfer_config *config)
{
	/* Not supported by the interim I/O block */
	return -ENOSYS;
}

/* The caller to this function must hold pb->lock */
int dma_setup_lbp_to_dram_transfer(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer,
		struct dma_transfer_config *config)
{
	struct paintbox_fpga_dma_channel *fpga = &channel->fpga;
	struct paintbox_lb *lb;
	int ret;

	if (fpga->to_ipu) {
		dev_err(&pb->pdev->dev,
				"%s: dma%u: channel direction mismatch "
				"transfer type %u supported %u\n",
				__func__, config->channel_id,
				config->transfer_type, fpga->to_ipu);
		return -EINVAL;
	}

	lb = get_lb(pb, session, config->src.lbp.lbp_id,
			config->src.lbp.lb_id, &ret);
	if (ret < 0)
		return ret;

	/* TODO(ahampson): The FPGA IO Block is built with the expectation that
	 * the number of channels in an output line buffer is 3.  This code
	 * should be removed once complex DMA is in the FPGA.  b/27101916
	 */
	if (lb->num_channels != 3) {
		dev_err(&pb->pdev->dev,
				"%s: lbp%u lb %u: num channels is not 3, num = "
				"%u\n", __func__, config->src.lbp.lbp_id,
				config->src.lbp.lb_id, lb->num_channels);
		return -EINVAL;
	}

	writel(config->img.width_pixels, pb->io.fpga_base + OUTPUT_FRAME_WIDTH);
	writel(config->img.height_pixels, pb->io.fpga_base +
			OUTPUT_FRAME_HEIGHT);
	writel(lbp_id_to_noc_id(config->src.lbp.lbp_id), pb->io.fpga_base +
			SRC_LBP);
	writel(config->src.lbp.lb_id, pb->io.fpga_base + SRC_LB_ID);
	writel(config->src.lbp.read_ptr_id, pb->io.fpga_base + SRC_RPTR_ID);
	writel(config->img.components, pb->io.fpga_base + NUM_OUTPUT_CHANNELS);

	LOG_IO_FPGA_REGISTERS(pb);

	dev_dbg(&pb->pdev->dev, "%s: dma%u: lbp%u lb%u rptr %u -> DRAM\n",
			__func__, config->channel_id, config->src.lbp.lbp_id,
			config->src.lbp.lb_id, config->src.lbp.read_ptr_id);

	return dma_setup_transfer_common(pb, channel, transfer, config);
}

/* The caller to this function must hold pb->lock */
int dma_setup_stp_to_dram_transfer(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer,
		struct dma_transfer_config *config)
{
	/* Not supported by the interim I/O block */
	return -ENOSYS;
}

/* The caller to this function must hold pb->lock */
int dma_setup_mipi_to_lbp_transfer(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer,
		struct dma_transfer_config *config)
{
	/* Not supported by the interim I/O block */
	return -ENOSYS;
}

/* The caller to this function must hold pb->lock */
int dma_start_transfer(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel)
{
	struct paintbox_fpga_dma_channel *fpga = &channel->fpga;
	unsigned long irq_flags;

	spin_lock_irqsave(&pb->irq_lock, irq_flags);

	fpga->bytes_transferred = 0;
	fpga->transfer_len = min_t(uint32_t, fpga->requested_transfer_len,
			XILINX_MAX_DMA_TRANSFER_LEN);

	/* Xilinx dma transfers will begin when the length field is set. */
	writel(fpga->transfer_len, fpga->xilinx_base + MM2S_LENGTH);

	spin_unlock_irqrestore(&pb->irq_lock, irq_flags);

	LOG_XILINX_REGISTERS(pb);

	return 0;
}

int dma_stop_transfer(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel)
{
	/* TODO(ahampson): Figure out how to stop a transfer with the Xilinx
	 * dma.
	 */

	return 0;
}

/* Caller must hold pb->irq_lock */
static void paintbox_fpga_completion_interrupt(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel)
{
	struct paintbox_fpga_dma_channel *fpga = &channel->fpga;

	fpga->bytes_transferred += fpga->transfer_len;

	/* Reprogram the channel if there is more data to transfer in the
	 * request.
	 */
	if (fpga->bytes_transferred < fpga->requested_transfer_len) {
		dma_addr_t transfer_paddr;

		fpga->transfer_len = min(fpga->requested_transfer_len -
				fpga->bytes_transferred,
				(uint32_t)XILINX_MAX_DMA_TRANSFER_LEN);
		transfer_paddr = channel->transfer.buf_paddr +
				fpga->bytes_transferred;
		writel((uint32_t)transfer_paddr, fpga->xilinx_base + MM2S_SA);
		writel((uint32_t)((uint64_t)transfer_paddr >> 32),
				fpga->xilinx_base + MM2S_SA_MSB);
		writel(fpga->transfer_len, fpga->xilinx_base + MM2S_LENGTH);
		return;
	}

	dma_report_completion(pb, channel, 0);
}

/* Caller must hold pb->irq_lock */
static void paintbox_fpga_error_interrupt(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel, uint32_t status)
{
	if (status & DMASR_DMA_INT_ERR)
		dev_err(&pb->pdev->dev,
				"%s: dma%u: internal error DMASR 0x%08x\n",
				__func__, channel->channel_id, status);
	if (status & DMASR_DMA_SLV_ERR)
		dev_err(&pb->pdev->dev, "%s: dma%u: slave error DMASR 0x%08x\n",
				__func__, channel->channel_id, status);
	if (status & DMASR_DMA_DEC_ERR)
		dev_err(&pb->pdev->dev,
				"%s: dma%u: decode error DMASR 0x%08x\n",
				__func__, channel->channel_id, status);
	if (status & DMASR_SG_INT_ERR)
		dev_err(&pb->pdev->dev,
				"%s: dma%u: SG internal error DMASR 0x%08x\n",
				__func__, channel->channel_id, status);
	if (status & DMASR_SG_SLV_ERR)
		dev_err(&pb->pdev->dev,
				"%s: dma%u: SG slave error DMASR 0x%08x\n",
				__func__, channel->channel_id, status);
	if (status & DMASR_SG_DEC_ERR)
		dev_err(&pb->pdev->dev,
				"%s: dma%u: SG decode error DMASR 0x%08x\n",
				__func__, channel->channel_id, status);
	/* TODO(ahampson): Figure out DMA cleanup in error situations.
	 */

	if (status & DMASR_HALTED) {
		writel(0, channel->fpga.xilinx_base + MM2S_DMACR);
		dma_report_completion(pb, channel, -EIO);
	}
}

static irqreturn_t paintbox_fpga_interrupt(int irq, void *arg)
{
	struct paintbox_data *pb = (struct paintbox_data *)arg;
	uint32_t status;
	unsigned int i;

	spin_lock(&pb->irq_lock);

	for (i = 0; i < pb->dma.num_channels; i++) {
		struct paintbox_dma_channel *channel = &pb->dma.channels[i];
		status = readl(channel->fpga.xilinx_base + MM2S_DMASR);
		writel(status, channel->fpga.xilinx_base + MM2S_DMASR);

		if (status & DMASR_ERR_IRQ)
			paintbox_fpga_error_interrupt(pb, channel, status);

		if (status & DMASR_IOC_IRQ)
			paintbox_fpga_completion_interrupt(pb, channel);
	}

	spin_unlock(&pb->irq_lock);

	return IRQ_HANDLED;
}

#ifdef CONFIG_DEBUG_FS
static int xilinx_debug_regs_show(struct seq_file *s, void *unused)
{
	struct paintbox_data *pb = s->private;
	char *buf;
	size_t len;
	int written;

	len = seq_get_buf(s, &buf);
	if (!buf)
		return -ENOMEM;

	mutex_lock(&pb->lock);

	written = dump_xilinx_registers(pb, buf, len);

	mutex_unlock(&pb->lock);

	seq_commit(s, written);

	return 0;
}

static int xilinx_debug_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, xilinx_debug_regs_show, inode->i_private);
}

static const struct file_operations xilinx_debug_regs_fops = {
	.open = xilinx_debug_regs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static void paintbox_dma_debug_init(struct paintbox_data *pb)
{
	char name[XILINX_NAME_LEN];

	snprintf(name, XILINX_NAME_LEN, "xilinx");
	pb->dma.fpga.xilinx_debug_dir = debugfs_create_dir(name,
			pb->debug_root);

	pb->dma.fpga.xilinx_regs_dentry = debugfs_create_file("regs",
			S_IRUGO | S_IWUSR, pb->dma.fpga.xilinx_debug_dir, pb,
			&xilinx_debug_regs_fops);
}
#endif

int paintbox_dma_init(struct paintbox_data *pb)
{
	struct resource *r;
	void __iomem *offset;
	unsigned int i;
	int ret;

	r = platform_get_resource(pb->pdev, IORESOURCE_MEM, 1);
	if (r == NULL) {
		dev_err(&pb->pdev->dev, "platform_get_resource failed\n");
		return -ENODEV;
	}

	pb->dma.dma_base = devm_ioremap(&pb->pdev->dev, r->start,
			resource_size(r));
	if (pb->dma.dma_base == NULL) {
		dev_err(&pb->pdev->dev, "unable to remap MMIO\n");
		return -ENOMEM;
	}

	ret = devm_request_irq(&pb->pdev->dev, pb->io.irq,
			paintbox_fpga_interrupt,
			IRQF_SHARED | IRQF_TRIGGER_HIGH, pb->pdev->name, pb);
	if (ret < 0)
		return ret;

#ifdef CONFIG_DEBUG_FS
	paintbox_dma_debug_init(pb);
#endif

#ifdef VERBOSE_DEBUG
	paintbox_alloc_debug_buffer(pb, max(IO_DEBUG_BUFFER_SIZE,
			XILINX_DEBUG_BUFFER_SIZE));
#endif

	/* TODO(ahampson): The current FPGA does not have a caps register to
	 * report the number of DMA channels so we are going to hard code this
	 * for now.
	 */
	pb->dma.num_channels = FPGA_DMA_COUNT;
	pb->caps.num_dma_channels = pb->dma.num_channels;

	pb->dma.channels = kzalloc(sizeof(struct paintbox_dma_channel) *
			pb->dma.num_channels, GFP_KERNEL);
	if (!pb->dma.channels)
		return -ENOMEM;

	/* Store channel id with object as a convenience to avoid doing a
	 * lookup later on.
	 */
	for (i = 0, offset = pb->dma.dma_base; i < pb->dma.num_channels; i++) {
		pb->dma.channels[i].pb = pb;
		pb->dma.channels[i].channel_id = i;
		pb->dma.channels[i].interrupt_id = DMA_NO_INTERRUPT;
		pb->dma.channels[i].fpga.xilinx_base = offset;
		offset += S2MM_OFFSET;

		/* to_ipu DMA channels will be even channels, from_ipu will be
		 * odd channels.
		 */
		pb->dma.channels[i].fpga.to_ipu = !(i & 0x01);

		dev_dbg(&pb->pdev->dev, "dma%u: base %p len %u\n", i,
				pb->dma.channels[i].fpga.xilinx_base,
				S2MM_OFFSET);
	}

	return 0;
}
