/*
 * Core driver for the Paintbox programmable IPU
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

#include <linux/atomic.h>
#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/kernel.h>
#include <linux/paintbox.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/version.h>

#include "paintbox-dma.h"
#include "paintbox-dma-debug.h"
#include "paintbox-dma-dram.h"
#include "paintbox-dma-lbp.h"
#include "paintbox-dma-stp.h"
#include "paintbox-io.h"
#include "paintbox-irq.h"
#include "paintbox-lbp.h"
#include "paintbox-regs.h"
#include "paintbox-sim-regs.h"


/* The caller to this function must hold pb->lock */
void dma_select_channel(struct paintbox_data *pb, uint32_t channel_id)
{
	writel((channel_id << DMA_CHAN_SEL_SHIFT) & DMA_CHAN_SEL_MASK,
			pb->dma.dma_base + DMA_CTRL);
}

/* The caller to this function must hold pb->lock */
int validate_dma_channel(struct paintbox_data *pb,
		struct paintbox_session *session, uint8_t channel_id)
{
	if (channel_id >= pb->dma.num_channels) {
		dev_err(&pb->pdev->dev, "%s: invalid dma channel id %d\n",
				__func__, channel_id);
		return -EINVAL;
	}

	if (pb->dma.channels[channel_id].session != session) {
		dev_err(&pb->pdev->dev, "%s: access error, dma channel id %d\n",
				__func__, channel_id);
		return -EACCES;
	}

	return 0;
}

/* The caller to this function must hold pb->lock */
struct paintbox_dma_channel *get_dma_channel(struct paintbox_data *pb,
		struct paintbox_session *session, uint8_t channel_id, int *err)
{
	int ret = validate_dma_channel(pb, session, channel_id);
	if (ret < 0) {
		*err = ret;
		return NULL;
	}

	*err = 0;
	return &pb->dma.channels[channel_id];
}

/* The caller to this function must hold pb->lock */
int dma_stop_transfer(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel)
{
	uint32_t mode;

	io_disable_dma_interrupts(pb);

	dma_select_channel(pb, channel->channel_id);

	writel(0, pb->dma.dma_base + DMA_CHAN_IMR);

	mode = readl(pb->dma.dma_base + DMA_CHAN_MODE);
	mode &= ~DMA_CHAN_ENA;
	writel(mode, pb->dma.dma_base + DMA_CHAN_MODE);

	io_enable_dma_interrupts(pb);

	return 0;
}

/* The caller to this function must hold pb->lock */
int release_dma_channel(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_dma_channel *channel)
{
	struct paintbox_dma_transfer *transfer;
	int ret;

	ret = dma_stop_transfer(pb, channel);
	if (ret < 0)
		return ret;

	io_disable_dma_channel(pb, channel->channel_id);

	/* TODO(ahampson):  We will need to walk the transfer queue here and
	 * unmap any buffers associated with transfers.  In the interim, just
	 * free the cma buffer and NULL out the single transfer associated with
	 * the channel.  Note that this will have to be conditional on whether
	 * IOMMU is enabled or not.
	 */
	transfer = &channel->transfer;

	if (transfer->buf_vaddr) {
		dma_free_coherent(&pb->pdev->dev,
				transfer->len_bytes,
				transfer->buf_vaddr,
				transfer->buf_paddr);

		transfer->len_bytes = 0;
		transfer->buf_vaddr = NULL;
		transfer->buf_paddr = 0;
	}

	list_del(&channel->entry);

	channel->session = NULL;

	return ret;
}

int release_dma_channel_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	unsigned int channel_id = (unsigned int)arg;
	struct paintbox_dma_channel *channel;
	int ret = 0;

	mutex_lock(&pb->lock);
	channel = get_dma_channel(pb, session, channel_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	/* Unbind any associated interrupt */
	unbind_dma_interrupt(pb, session, channel);

	release_dma_channel(pb, session, channel);

	mutex_unlock(&pb->lock);

	return ret;
}

int allocate_dma_channel_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	unsigned int channel_id = (unsigned int)arg;
	struct paintbox_dma_channel *channel;

	if (channel_id >= pb->dma.num_channels) {
		dev_err(&pb->pdev->dev, "%s: invalid dma channel id %d\n",
				__func__, channel_id);
		return -EINVAL;
	}

	mutex_lock(&pb->lock);

	channel = &pb->dma.channels[channel_id];
	if (channel->session) {
		dev_err(&pb->pdev->dev, "%s: access error, dma channel id %d\n",
				__func__, channel_id);
		mutex_unlock(&pb->lock);
		return -EACCES;
	}

	channel->session = session;
	list_add_tail(&channel->entry, &session->dma_list);

	io_enable_dma_channel(pb, channel_id);
	mutex_unlock(&pb->lock);

	return 0;
}

int setup_dma_transfer_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct dma_transfer_config __user *user_config;
	struct dma_transfer_config config;
	struct paintbox_dma_channel *channel;
	struct paintbox_dma_transfer *transfer;
	int ret;

	user_config = (struct dma_transfer_config __user *)arg;
	if (copy_from_user(&config, user_config, sizeof(config)))
		return -EFAULT;

	mutex_lock(&pb->lock);

	channel = get_dma_channel(pb, session, config.channel_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	/* TODO(ahampson):  Eventually we are going to need to allocate a
	 * transfer object, populate it, and enqueue it.  In the interim we are
	 * going to just have a single transfer associated with each DMA
	 * channel.
	 */
	transfer = &channel->transfer;

	/* TODO(ahampson):  We are temporarily going to have one transfer
	 * hardcoded with the channel.  The CMA memory associated with the
	 * transfer is freed lazily.  If there is a buffer associated with the
	 * channel then free it here.  This will get cleaned up once the
	 * interrupt code is refactored.
	 */
	if (transfer->buf_vaddr) {
		dma_free_coherent(&pb->pdev->dev,
				transfer->len_bytes,
				transfer->buf_vaddr,
				transfer->buf_paddr);
	}

	memset(transfer, 0, sizeof(struct paintbox_dma_transfer));

	switch (config.transfer_type) {
	case DMA_DRAM_TO_LBP:
		channel->read_transfer = false;
		ret = dma_setup_dram_to_lbp_transfer(pb, session, channel,
				transfer, &config);
		break;
	case DMA_DRAM_TO_STP:
		channel->read_transfer = false;
		ret = dma_setup_dram_to_stp_transfer(pb, session, channel,
				transfer, &config);
		break;
	case DMA_LBP_TO_DRAM:
		channel->read_transfer = true;
		ret = dma_setup_lbp_to_dram_transfer(pb, session, channel,
				transfer, &config);
		break;
	case DMA_MIPI_TO_LBP:
		ret = dma_setup_mipi_to_lbp_transfer(pb, session, channel,
				transfer, &config);
		break;
	case DMA_LBP_TO_MIPI:
		ret = dma_setup_lbp_to_mipi_transfer(pb, session, channel,
				transfer, &config);
		break;
	default:
		dev_err(&pb->pdev->dev, "dma: invalid transfer type %u\n",
			config.transfer_type);
		ret = -EINVAL;
	}

	mutex_unlock(&pb->lock);

	return ret;
}

int read_dma_transfer_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct dma_transfer_read __user *user_req;
	struct dma_transfer_read req;
	struct paintbox_dma_channel *channel;
	int ret;

	user_req = (struct dma_transfer_read __user *)arg;
	if (copy_from_user(&req, user_req, sizeof(req)))
		return -EFAULT;

	mutex_lock(&pb->lock);

	channel = get_dma_channel(pb, session, req.channel_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	smp_mb();

	/* TODO(ahampson):  This will need to be changed when the DMA code is
	 * changed to have a done queue.
	 */
	if (atomic_cmpxchg(&channel->completed_unread, 1, 0) == 0) {
		mutex_unlock(&pb->lock);
		return -EAGAIN;
	}

	if (!access_ok(VERIFY_READ, req.host_vaddr, req.len_bytes)) {
		mutex_unlock(&pb->lock);
		return -EFAULT;
	}

	/* TODO(ahampson):  I think the way this will work eventually is that
	 * there will be a queue of completed transfers.  The transfers in the
	 * queue will be unmapped and freed if they are CMA buffers.  This will
	 * have to be different for IOMMU buffers.
	 * We may need to have some sort of transaction id as well.
	 */
	ret = dma_unmap_buffer_cma(pb, &channel->transfer, req.host_vaddr,
			req.len_bytes);

	mutex_unlock(&pb->lock);

	return ret;
}

static void commit_transfer_to_hardware(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel)
{
	struct paintbox_dma_transfer *transfer;
	struct paintbox_dma *dma = &pb->dma;
	uint32_t val;

	io_disable_dma_interrupts(pb);

	/* TODO(ahampson):  There should be a transfer queue that we dequeue
	 * from here.  Currently only a single transfer is supported.
	 */
	transfer = &channel->transfer;

	dma_select_channel(pb, channel->channel_id);

	/* Load the transfer into the DMA channel registers */
	writel(transfer->chan_img_format, dma->dma_base + DMA_CHAN_IMG_FORMAT);
	writel(transfer->chan_img_size, dma->dma_base + DMA_CHAN_IMG_SIZE);
	writel(transfer->chan_img_pos_low, dma->dma_base + DMA_CHAN_IMG_POS_L);
	writel(transfer->chan_img_pos_high, dma->dma_base + DMA_CHAN_IMG_POS_H);
	writel(transfer->chan_img_layout_low, dma->dma_base +
			DMA_CHAN_IMG_LAYOUT_L);
	writel(transfer->chan_img_layout_high, dma->dma_base +
			DMA_CHAN_IMG_LAYOUT_H);

	val = readl(dma->dma_base + DMA_CHAN_BIF_XFER);
	val &= ~DMA_CHAN_STRIPE_HEIGHT_MASK;
	val |= transfer->chan_bif_xfer;
	writel(val, dma->dma_base + DMA_CHAN_BIF_XFER);

	writel(transfer->chan_va_low, dma->dma_base + DMA_CHAN_VA_L);
	writel(transfer->chan_va_high, dma->dma_base + DMA_CHAN_VA_H);
	writel(transfer->chan_va_bdry_low, dma->dma_base + DMA_CHAN_VA_BDRY_L);
	writel(transfer->chan_va_bdry_high, dma->dma_base + DMA_CHAN_VA_BDRY_H);
	writel(transfer->chan_noc_xfer_low, dma->dma_base +
			DMA_CHAN_NOC_XFER_L);

	writel(transfer->chan_node, dma->dma_base + DMA_CHAN_NODE);

	/* Enable interrupts for the channel */
	writel(DMA_CHAN_INT_EOF | DMA_CHAN_INT_VA_ERR, dma->dma_base +
			DMA_CHAN_IMR);

	/* Write the channel mode register last as this will enqueue the
	 * transfer into the hardware.
	 */
	writel(transfer->chan_mode, dma->dma_base + DMA_CHAN_MODE);

	io_enable_dma_channel_interrupt(pb, channel->channel_id);

	io_enable_dma_interrupts(pb);

	LOG_DMA_REGISTERS(pb, channel);
}

int start_dma_transfer_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	unsigned int channel_id = (unsigned int)arg;
	struct paintbox_dma_channel *channel;
	int ret;

	mutex_lock(&pb->lock);
	channel = get_dma_channel(pb, session, channel_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	dev_dbg(&pb->pdev->dev, "%s: dma%u: start\n",  __func__, channel_id);

	/* TODO(ahampson):  Implement support for double buffering b/28316153 */
	commit_transfer_to_hardware(pb, channel);

	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	mutex_unlock(&pb->lock);

	return 0;
}

int bind_dma_interrupt_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct dma_interrupt_config __user *user_req;
	struct dma_interrupt_config req;
	struct paintbox_dma_channel *channel;
	int ret;

	user_req = (struct dma_interrupt_config __user *)arg;
	if (copy_from_user(&req, user_req, sizeof(req)))
		return -EFAULT;

	mutex_lock(&pb->lock);
	channel = get_dma_channel(pb, session, req.channel_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	ret = bind_dma_interrupt(pb, session, channel, req.interrupt_id);

	init_waiters(pb, channel->irq);

	mutex_unlock(&pb->lock);

	return ret;
}

int unbind_dma_interrupt_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	unsigned int channel_id = (unsigned int)arg;
	struct paintbox_dma_channel *channel;
	int ret = 0;

	mutex_lock(&pb->lock);
	channel = get_dma_channel(pb, session, channel_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	ret = unbind_dma_interrupt(pb, session, channel);
	if (ret < 0) {
		dev_err(&pb->pdev->dev,
				"%s: dma channel%u: unable to unbind interrupt,"
				" %d\n", __func__, channel_id, ret);
		mutex_unlock(&pb->lock);
		return ret;
	}

	dev_dbg(&pb->pdev->dev, "%s: dma channel%u: unbind interrupt\n",
			__func__, channel_id);

	mutex_unlock(&pb->lock);

	return 0;
}

/* Returns the number of DMA transfers that have been completed and are ready
 * to be read out by the HAL.
 */
int get_completed_transfer_count_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	unsigned int channel_id = (unsigned int)arg;
	struct paintbox_dma_channel *channel;
	int ret;

	mutex_lock(&pb->lock);
	channel = get_dma_channel(pb, session, channel_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	/* TODO(ahampson): When the driver is switched over to using a transfer
	 * queue this should return the number of transfers in the completed
	 * queue.
	 */
	ret = atomic_read(&channel->completed_unread);

	mutex_unlock(&pb->lock);

	return ret;
}

/* This function is called in an interrupt context */
static void dma_report_completion(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel, int err)
{
	if (channel->read_transfer) {
		smp_wmb();
		atomic_inc(&channel->completed_unread);
	}

	signal_waiters(channel->irq, err);
}

irqreturn_t paintbox_dma_interrupt(struct paintbox_data *pb,
		uint32_t channel_mask)
{
	unsigned int channel_id;

	for (channel_id = 0; channel_id < pb->dma.num_channels && channel_mask;
			channel_id++, channel_mask >>= 1) {
		struct paintbox_dma_channel *channel;
		uint32_t status;

		if (!(channel_mask & 0x01))
			continue;

		channel = &pb->dma.channels[channel_id];

		dma_select_channel(pb, channel_id);

		status = readl(pb->dma.dma_base + DMA_CHAN_ISR);
		writel(status, pb->dma.dma_base + DMA_CHAN_ISR);

		if (status & DMA_CHAN_INT_VA_ERR)
			dma_report_completion(pb, channel, -EIO);
		else if (status & DMA_CHAN_INT_EOF)
			dma_report_completion(pb, channel, 0);
	}

	return IRQ_HANDLED;
}

int paintbox_dma_init(struct paintbox_data *pb)
{
	unsigned int i;

	pb->dma.dma_base = pb->reg_base + IPU_DMA_OFFSET;

	pb->dma.num_channels = readl(pb->dma.dma_base + DMA_CAP0) &
			MAX_DMA_CHAN_MASK;

#ifdef CONFIG_DEBUG_FS
	paintbox_dma_debug_init(pb);
#endif

	/* TODO(ahampson):  refactor out the storage of the caps structure  */
	pb->caps.num_dma_channels = pb->dma.num_channels;

	pb->dma.channels = kzalloc(sizeof(struct paintbox_dma_channel) *
			pb->dma.num_channels, GFP_KERNEL);
	if (!pb->dma.channels)
		return -ENOMEM;

	/* Store channel id with object as a convenience to avoid doing a
	 * lookup later on.
	 */
	for (i = 0; i < pb->dma.num_channels; i++) {
		struct paintbox_dma_channel *channel = &pb->dma.channels[i];
		channel->channel_id = i;
		atomic_set(&channel->completed_unread, 0);

#ifdef CONFIG_DEBUG_FS
		paintbox_dma_channel_debug_init(pb, channel);
#endif
	}

	dev_dbg(&pb->pdev->dev, "dma: base %p len %u dma channels %u\n",
			pb->dma.dma_base, DMA_BLOCK_LEN,
			pb->dma.num_channels);

	return 0;
}
