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
#include "paintbox-dma-mipi.h"
#include "paintbox-dma-stp.h"
#include "paintbox-io.h"
#include "paintbox-irq.h"
#include "paintbox-lbp.h"
#include "paintbox-mipi.h"
#include "paintbox-regs.h"
#include "paintbox-sim-regs.h"


#define MAX_DMA_STOP_ATTEMPTS 2

/* The caller to this function must hold pb->dma.dma_lock. */
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

static void drain_queue(struct paintbox_data *pb,
		struct list_head *transfer_list, unsigned int *count)
{
	unsigned long irq_flags;

	spin_lock_irqsave(&pb->dma.dma_lock, irq_flags);

	while (!list_empty(transfer_list)) {
		struct paintbox_dma_transfer *transfer;

		transfer = list_entry(transfer_list->next,
				struct paintbox_dma_transfer, entry);
		list_del(&transfer->entry);
		(*count)--;
		WARN_ON(*count < 0);

		spin_unlock_irqrestore(&pb->dma.dma_lock, irq_flags);

		if (transfer->buf_vaddr)
			dma_unmap_buffer_cma(pb, transfer, NULL, 0);

		kfree(transfer);

		spin_lock_irqsave(&pb->dma.dma_lock, irq_flags);
	}

	spin_unlock_irqrestore(&pb->dma.dma_lock, irq_flags);
}

/* The caller to this function must hold pb->dma.dma_lock. */
static void dma_report_completion(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer, int err)
{
	if (transfer->notify_on_completion) {
		list_add_tail(&transfer->entry, &channel->completed_list);

		channel->completed_count++;
		channel->stats.reported_completions++;

		signal_waiters(pb, channel->irq, err);
	} else {
		list_add_tail(&transfer->entry, &pb->dma.discard_list);
		channel->stats.reported_discards++;
	}
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

/* The caller to this function must hold pb->lock and pb->dma.dma_lock */
static void dma_reset_channel_locked(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel)
{
	struct paintbox_dma *dma = &pb->dma;
	struct paintbox_dma_transfer *transfer;
	uint32_t ctrl;

	/* If there are no active transfers then there is no need to reset the
	 * channel.
	 */
	if (channel->active_count == 0) {
		/* Disable the DMA channel interrupts in the local IMR and in
		 * the top-level IPU_IMR.
		 */
		writel(0, dma->dma_base + DMA_CHAN_IMR);
		io_disable_dma_channel_interrupt(pb, channel->channel_id);
		io_disable_dma_channel(pb, channel->channel_id);
		return;
	}

	channel->active_count--;
	if (WARN_ON(channel->active_count < 0))
		channel->active_count = 0;

	dma_select_channel(pb, channel->channel_id);

	ctrl = readl(dma->dma_base + DMA_CHAN_CTRL_L);
	ctrl |= 1 << channel->channel_id;
	writel(ctrl, dma->dma_base + DMA_CHAN_CTRL_L);
	ctrl &= ~(1 << channel->channel_id);
	writel(ctrl, dma->dma_base + DMA_CHAN_CTRL_L);

	transfer = list_entry(channel->active_list.next,
			struct paintbox_dma_transfer, entry);
	if (!WARN_ON(transfer == NULL)) {
		list_del(&transfer->entry);
		dma_report_completion(pb, channel, transfer, -ECANCELED);
	}
}

/* The caller to this function must hold pb->lock */
void dma_reset_channel(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel)
{
	unsigned long irq_flags;

	spin_lock_irqsave(&pb->dma.dma_lock, irq_flags);

	dma_reset_channel_locked(pb, channel);

	spin_unlock_irqrestore(&pb->dma.dma_lock, irq_flags);
}

/* The caller to this function must hold pb->lock */
void dma_stop_transfer(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel)
{
	struct paintbox_dma *dma = &pb->dma;
	uint32_t ctrl, mode, src, dst;
	unsigned long irq_flags;

	spin_lock_irqsave(&pb->dma.dma_lock, irq_flags);

	dma_select_channel(pb, channel->channel_id);

	/* Disable the channel to prevent any pending transfers from starting.
	 */
	mode = readl(dma->dma_base + DMA_CHAN_MODE);
	if (mode & DMA_CHAN_ENA) {
		mode &= ~DMA_CHAN_ENA;
		writel(mode, dma->dma_base + DMA_CHAN_MODE);
	}

	/* If there is no active transfer then disable the interrupt and
	 * power down the channel.
	 */
	if (channel->active_count == 0) {
		/* Disable the DMA channel interrupts in the local IMR and in
		 * the top-level IPU_IMR.
		 */
		writel(0, dma->dma_base + DMA_CHAN_IMR);
		io_disable_dma_channel_interrupt(pb, channel->channel_id);
		io_disable_dma_channel(pb, channel->channel_id);

		spin_unlock_irqrestore(&pb->dma.dma_lock, irq_flags);
		return;
	}

	mode = readl(dma->dma_base + DMA_CHAN_MODE_RO);
	src = (mode & DMA_CHAN_SRC_MASK) >> DMA_CHAN_SRC_SHIFT;
	dst = (mode & DMA_CHAN_DST_MASK) >> DMA_CHAN_DST_SHIFT;

	/* If MIPI is the source or destination of the transfer then cleanup the
	 * MIPI stream first before initiating the DMA stop request.
	 */
	if (src == DMA_CHAN_SRC_MIPI_IN || dst == DMA_CHAN_DST_MIPI_OUT) {
		/* It should not be possible to program a MIPI transfer without
		 * having a MIPI stream associated with the channel.
		 */
		if (!WARN_ON(!channel->mipi_stream))
			mipi_request_cleanup(pb, channel->mipi_stream);
	}

	/* If there is an active transfer then issue a stop request. */
	ctrl = readl(dma->dma_base + DMA_CHAN_CTRL_H);
	ctrl |= 1 << channel->channel_id;
	writel(ctrl, dma->dma_base + DMA_CHAN_CTRL_H);

	reinit_completion(&channel->stop_completion);

	channel->stop_request = true;

	spin_unlock_irqrestore(&pb->dma.dma_lock, irq_flags);

	/* Wait for the EOF frame interrupt to occur.  If we timeout waiting
	 * for the interrupt or the completion is interrupted by a signal then
	 * the DMA channel will be reset below.
	 */
	wait_for_completion_interruptible_timeout(&channel->stop_completion,
			usecs_to_jiffies(max(DMA_STOP_TIMEOUT_US,
			MIPI_CLEANUP_TIMEOUT_US)));

	spin_lock_irqsave(&pb->dma.dma_lock, irq_flags);

	/* Re-select the channel, the DMA_SEL field might have changed while the
	 * lock was not held.
	 */
	dma_select_channel(pb, channel->channel_id);

	/* Stop bits are not self-clearing so clear it here. */
	ctrl = readl(dma->dma_base + DMA_CHAN_CTRL_H);
	ctrl &= ~(1 << channel->channel_id);
	writel(ctrl, dma->dma_base + DMA_CHAN_CTRL_H);

	/* Disable the DMA channel interrupts in the local IMR and in the
	 * top-level IPU_IMR.
	 */
	writel(0, dma->dma_base + DMA_CHAN_IMR);
	io_disable_dma_channel_interrupt(pb, channel->channel_id);

	/* If the transfer was to or from a line buffer then reset the
	 * line buffer too.
	 */
	if (src == DMA_CHAN_SRC_LBP || dst == DMA_CHAN_DST_LBP) {
		uint32_t chan_node = readl(dma->dma_base + DMA_CHAN_NODE_RO);
		reset_lb(pb, chan_node & DMA_CHAN_CORE_ID_MASK,
				(chan_node & DMA_CHAN_LB_ID_MASK) >>
				DMA_CHAN_LB_ID_SHIFT);
	}

	/* If this was a MIPI related transfer then verify that the cleanup
	 * operation completed.
	 */
	if (src == DMA_CHAN_SRC_MIPI_IN || dst == DMA_CHAN_DST_MIPI_OUT) {
		/* It should not be possible to program a MIPI transfer without
		 * having a MIPI stream associated with the channel.
		 */
		if (!WARN_ON(!channel->mipi_stream))
			verify_cleanup_completion(pb, channel->mipi_stream);
	}

	/* If the channel didn't stop within the timeout then reset it.  This
	 * situation can occur if there are no outstanding requests between the
	 * src and dst or if the transfer has not started.
	 */
	if (channel->stop_request) {
		channel->stop_request = false;

		dma_reset_channel_locked(pb, channel);
	}

	/* If there are no active transfers then power down the DMA channel. */
	if (channel->active_count == 0)
		io_disable_dma_channel(pb, channel->channel_id);

	spin_unlock_irqrestore(&pb->dma.dma_lock, irq_flags);
}

/* The caller to this function must hold pb->lock */
void release_dma_channel(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_dma_channel *channel)
{
	unsigned long irq_flags;

	dma_stop_transfer(pb, channel);

	drain_queue(pb, &channel->pending_list, &channel->pending_count);
	drain_queue(pb, &channel->active_list, &channel->active_count);
	drain_queue(pb, &channel->completed_list, &channel->completed_count);
	drain_queue(pb, &pb->dma.discard_list, &pb->dma.discard_count);

	/* If there is a MIPI stream associated with this DMA channel then
	 * notify it that the DMA channel has been released.
	 */
	if (channel->mipi_stream) {
		mipi_handle_dma_channel_released(pb, channel->mipi_stream);

		spin_lock_irqsave(&pb->dma.dma_lock, irq_flags);

		channel->mipi_stream = NULL;

		spin_unlock_irqrestore(&pb->dma.dma_lock, irq_flags);
	}

	/* Remove the DMA channel from the session. */
	list_del(&channel->session_entry);

	channel->session = NULL;

	spin_lock_irqsave(&pb->dma.dma_lock, irq_flags);

	/* Make sure the DMA channel is powered down. */
	if (!WARN_ON(channel->active_count != 0))
		io_disable_dma_channel(pb, channel->channel_id);

	spin_unlock_irqrestore(&pb->dma.dma_lock, irq_flags);
}

/* The caller to this function must hold pb->lock */
void dma_handle_mipi_stream_released(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel)
{
	unsigned long irq_flags;

	spin_lock_irqsave(&pb->dma.dma_lock, irq_flags);

	channel->mipi_stream = NULL;

	spin_unlock_irqrestore(&pb->dma.dma_lock, irq_flags);
}

/* The caller to this function must hold pb->lock */
struct paintbox_dma_channel *dma_handle_mipi_stream_allocated(
		struct paintbox_data *pb, struct paintbox_session *session,
		struct paintbox_mipi_stream *stream, unsigned int channel_id,
		int *ret)
{
	struct paintbox_dma_channel *channel;
	unsigned long irq_flags;

	if (channel_id >= pb->dma.num_channels) {
		dev_err(&pb->pdev->dev, "%s: invalid dma channel id %d\n",
				__func__, channel_id);
		*ret = -EINVAL;
		return NULL;
	}

	/* If the channel has not been allocated to a session then return with
	 * no error.  This is not an error if the MIPI stream is allocated
	 * before the DMA channel.  Cross-linking the DMA channel with the MIPI
	 * stream will be handled in mipi_dma_channel_allocated() once the DMA
	 * channel is allocated.
	 */
	if (pb->dma.channels[channel_id].session == NULL) {
		*ret = 0;
		return NULL;
	}

	/* If the DMA channel is owned by a different session then it can not be
	 * associated with this MIPI stream.
	 */
	if (pb->dma.channels[channel_id].session != session) {
		dev_err(&pb->pdev->dev,
				"%s: DMA channel%u is not part of this session,"
				" MIPI %s stream%u can not be used.\n",
				__func__, channel_id,
				stream->is_input ? "input" : "output",
				stream->stream_id);
		*ret = -EACCES;
		return NULL;
	}

	channel = &pb->dma.channels[channel_id];

	/* There should not be a stream object already associated with this
	 * channel.  If there is then there is a bug in cleanup.
	 */
	WARN_ON(channel->mipi_stream);

	spin_lock_irqsave(&pb->dma.dma_lock, irq_flags);

	channel->mipi_stream = stream;

	spin_unlock_irqrestore(&pb->dma.dma_lock, irq_flags);

	return channel;
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

int flush_dma_transfers_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg) {
	struct dma_transfer_flush __user *user_req;
	struct dma_transfer_flush req;
	struct paintbox_dma_channel *channel;
	int ret = 0;

	user_req = (struct dma_transfer_flush __user *)arg;
	if (copy_from_user(&req, user_req, sizeof(req)))
		return -EFAULT;

	mutex_lock(&pb->lock);

	channel = get_dma_channel(pb, session, req.channel_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	if (req.flush_pending)
		drain_queue(pb, &channel->pending_list,
				&channel->pending_count);

	if (req.flush_active) {
		/* Stop any running transfers before flushing the active queue.
		 */
		dma_stop_transfer(pb, channel);
		drain_queue(pb, &channel->active_list, &channel->active_count);
	}

	if (req.flush_completed)
		drain_queue(pb, &channel->completed_list,
				&channel->completed_count);

	mutex_unlock(&pb->lock);

	return 0;
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
	list_add_tail(&channel->session_entry, &session->dma_list);

	/* MIPI streams and DMA channels have a fixed mapping, i.e. a specific
	 * DMA channel must be used for transfers from a particular MIPI stream.
	 * Notify the MIPI code that this DMA channel has been allocated to the
	 * session.
	 */
	channel->mipi_stream = mipi_handle_dma_channel_allocated(pb, session,
			channel);

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
	unsigned long irq_flags;
	int ret = 0;

	user_config = (struct dma_transfer_config __user *)arg;
	if (copy_from_user(&config, user_config, sizeof(config)))
		return -EFAULT;

	mutex_lock(&pb->lock);

	channel = get_dma_channel(pb, session, config.channel_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	transfer = kzalloc(sizeof(struct paintbox_dma_transfer), GFP_KERNEL);
	if (!transfer) {
		mutex_unlock(&pb->lock);
		dev_err(&pb->pdev->dev, "%s: allocation failure\n", __func__);
		return -ENOMEM;
	}

	switch (config.transfer_type) {
	case DMA_DRAM_TO_LBP:
		ret = dma_setup_dram_to_lbp_transfer(pb, session, channel,
				transfer, &config);
		break;
	case DMA_DRAM_TO_STP:
		ret = dma_setup_dram_to_stp_transfer(pb, session, channel,
				transfer, &config);
		break;
	case DMA_LBP_TO_DRAM:
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
	case DMA_MIPI_TO_DRAM:
		ret = dma_setup_mipi_to_dram_transfer(pb, session, channel,
				transfer, &config);
		break;
	default:
		dev_err(&pb->pdev->dev, "dma: invalid transfer type %u\n",
			config.transfer_type);
		ret = -EINVAL;
	}

	if (ret < 0) {
		mutex_unlock(&pb->lock);

		kfree(transfer);
		return ret;
	}

	transfer->notify_on_completion = config.notify_on_completion;
	transfer->auto_load_transfer = config.auto_load_transfer;

	/* Add the transfer to the pending queue */
	spin_lock_irqsave(&pb->dma.dma_lock, irq_flags);

	list_add_tail(&transfer->entry, &channel->pending_list);
	channel->pending_count++;

	spin_unlock_irqrestore(&pb->dma.dma_lock, irq_flags);

	mutex_unlock(&pb->lock);

	return 0;
}

int read_dma_transfer_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct dma_transfer_read __user *user_req;
	struct dma_transfer_read req;
	struct paintbox_dma_channel *channel;
	struct paintbox_dma_transfer *transfer;
	unsigned long irq_flags;
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

	if (!access_ok(VERIFY_READ, req.host_vaddr, req.len_bytes)) {
		mutex_unlock(&pb->lock);
		return -EFAULT;
	}

	spin_lock_irqsave(&pb->dma.dma_lock, irq_flags);

	if (list_empty(&channel->completed_list)) {
		spin_unlock_irqrestore(&pb->dma.dma_lock, irq_flags);
		mutex_unlock(&pb->lock);
		return -ENOENT;
	}

	channel->completed_count--;
	WARN_ON(channel->completed_count < 0);

	transfer = list_entry(channel->completed_list.next,
			struct paintbox_dma_transfer, entry);
	list_del(&transfer->entry);

	spin_unlock_irqrestore(&pb->dma.dma_lock, irq_flags);

	ret = dma_unmap_buffer_cma(pb, transfer, req.host_vaddr, req.len_bytes);

	kfree(transfer);

	/* Take this opportunity to drain the discard queue */
	drain_queue(pb, &pb->dma.discard_list, &pb->dma.discard_count);

	mutex_unlock(&pb->lock);

	return ret;
}

/* The caller to this function must hold pb->dma.dma_lock. */
static void commit_transfer_to_hardware(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel)
{
	struct paintbox_dma_transfer *transfer;
	struct paintbox_dma *dma = &pb->dma;
	uint32_t val;

	transfer = list_entry(channel->pending_list.next,
			struct paintbox_dma_transfer, entry);
	list_del(&transfer->entry);

	list_add_tail(&transfer->entry, &channel->active_list);

	channel->pending_count--;
	WARN_ON(channel->pending_count < 0);

	channel->active_count++;

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
	writel(transfer->chan_noc_xfer_high, dma->dma_base +
			DMA_CHAN_NOC_XFER_H);

	writel(transfer->chan_node, dma->dma_base + DMA_CHAN_NODE);

	/* Enable interrupts for the channel */
	writel(DMA_CHAN_INT_EOF | DMA_CHAN_INT_VA_ERR, dma->dma_base +
			DMA_CHAN_IMR);

	/* Write the channel mode register last as this will enqueue the
	 * transfer into the hardware.
	 */
	writel(transfer->chan_mode, dma->dma_base + DMA_CHAN_MODE);

	/* Enable the DMA channel interrupt in the top-level IPU_IMR */
	io_enable_dma_channel_interrupt(pb, channel->channel_id);

	LOG_DMA_REGISTERS(pb, channel);
}

int stop_dma_transfer_ioctl(struct paintbox_data *pb,
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

	dev_dbg(&pb->pdev->dev, "%s: dma%u: stop\n",  __func__, channel_id);

	dma_stop_transfer(pb, channel);

	mutex_unlock(&pb->lock);

	return 0;
}

int start_dma_transfer_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	unsigned int channel_id = (unsigned int)arg;
	struct paintbox_dma_channel *channel;
	unsigned long irq_flags;
	int ret = 0;

	mutex_lock(&pb->lock);
	channel = get_dma_channel(pb, session, channel_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	dev_dbg(&pb->pdev->dev, "%s: dma%u: start\n",  __func__, channel_id);

	spin_lock_irqsave(&pb->dma.dma_lock, irq_flags);

	if (list_empty(&channel->pending_list)) {
		dev_err(&pb->pdev->dev, "dma ch%u: no pending transfers\n",
				channel->channel_id);
		ret = -ENOENT;
		goto err_exit;
	}

	if (channel->active_count >= MAX_ACTIVE_TRANSFERS) {
		dev_err(&pb->pdev->dev, "dma ch%u: too many transfers\n",
				channel->channel_id);
		ret = -EAGAIN;
		goto err_exit;
	}

	io_enable_dma_channel(pb, channel_id);

	commit_transfer_to_hardware(pb, channel);

err_exit:
	spin_unlock_irqrestore(&pb->dma.dma_lock, irq_flags);

	mutex_unlock(&pb->lock);

	return ret;
}

/* Returns the number of DMA transfers that have been completed and are ready
 * to be read out by the HAL.
 */
int get_completed_transfer_count_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	unsigned int channel_id = (unsigned int)arg;
	struct paintbox_dma_channel *channel;
	unsigned long irq_flags;
	int ret;

	mutex_lock(&pb->lock);
	channel = get_dma_channel(pb, session, channel_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	spin_lock_irqsave(&pb->dma.dma_lock, irq_flags);

	ret = channel->completed_count;

	spin_unlock_irqrestore(&pb->dma.dma_lock, irq_flags);

	mutex_unlock(&pb->lock);

	return ret;
}

#ifdef CONFIG_PAINTBOX_TEST_SUPPORT
int dma_test_reset_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	unsigned long irq_flags;
	uint32_t ctrl;

	mutex_lock(&pb->lock);

	spin_lock_irqsave(&pb->dma.dma_lock, irq_flags);

	ctrl = readl(pb->dma.dma_base + DMA_CHAN_CTRL_L);
	ctrl |= DMA_RESET;
	writel(ctrl, pb->dma.dma_base + DMA_CHAN_CTRL_L);
	ctrl &= ~DMA_RESET;
	writel(ctrl, pb->dma.dma_base + DMA_CHAN_CTRL_L);

	spin_unlock_irqrestore(&pb->dma.dma_lock, irq_flags);

	mutex_unlock(&pb->lock);

	return 0;
}

int dma_test_channel_reset_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	unsigned int channel_id = (unsigned int)arg;
	struct paintbox_dma_channel *channel;
	int ret = 0;

	mutex_lock(&pb->lock);
	channel = get_dma_channel(pb, session, channel_id, &ret);
	if (ret >= 0)
		dma_reset_channel(pb, channel);

	mutex_unlock(&pb->lock);

	return ret;
}
#endif

/* This function must be called in an interrupt context */
void dma_set_mipi_error(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel, int err)
{
	struct paintbox_dma_transfer *transfer;

	spin_lock(&pb->dma.dma_lock);

	/* Verify that there is an active transfer. */
	if (!channel->active_count) {
		dev_err(&pb->pdev->dev,
				"%s: dma channel%u unable to set error, no"
				"active transfer\n", __func__,
				channel->channel_id);
		spin_unlock(&pb->dma.dma_lock);
		return;
	}

	transfer = list_entry(channel->active_list.next,
			struct paintbox_dma_transfer, entry);
	if (!WARN_ON(transfer == NULL)) {
		uint32_t src, dst;

		/* Verify that the active transfer is a MIPI transfer */
		src = (transfer->chan_mode & DMA_CHAN_SRC_MASK) >>
				DMA_CHAN_SRC_SHIFT;
		dst = (transfer->chan_mode & DMA_CHAN_DST_MASK) >>
				DMA_CHAN_DST_SHIFT;

		if (src == DMA_CHAN_SRC_MIPI_IN || dst == DMA_CHAN_DST_MIPI_OUT)
			transfer->error = err;
	}

	spin_unlock(&pb->dma.dma_lock);
}

/* This function must be called in an interrupt context */
irqreturn_t paintbox_dma_interrupt(struct paintbox_data *pb,
		uint32_t channel_mask)
{
	unsigned int channel_id;

	for (channel_id = 0; channel_id < pb->dma.num_channels && channel_mask;
			channel_id++, channel_mask >>= 1) {
		struct paintbox_dma_channel *channel;
		struct paintbox_dma_transfer *transfer;
		uint32_t status;

		if (!(channel_mask & 0x01))
			continue;

		spin_lock(&pb->dma.dma_lock);

		channel = &pb->dma.channels[channel_id];
		channel->stats.irq_activations++;

		transfer = list_entry(channel->active_list.next,
				struct paintbox_dma_transfer, entry);
		list_del(&transfer->entry);

		channel->active_count--;
		WARN_ON(channel->active_count < 0);

		dma_select_channel(pb, channel_id);

		status = readl(pb->dma.dma_base + DMA_CHAN_ISR);
		writel(status, pb->dma.dma_base + DMA_CHAN_ISR);

		/* If there is a stop request then stop the current transfers
		 * and signal the waiters if there are any.  Do not load any
		 * new transfers either.  The runtime will need to start the
		 * channel again to load the next transfer.
		 */
		if (channel->stop_request) {
			channel->stop_request = false;

			complete(&channel->stop_completion);
			dma_report_completion(pb, channel, transfer,
					-ECANCELED);
			spin_unlock(&pb->dma.dma_lock);
			continue;
		}

		if (status & DMA_CHAN_INT_VA_ERR) {
			channel->stats.va_interrupts++;
			dma_report_completion(pb, channel, transfer, -EIO);
		} else if (status & DMA_CHAN_INT_EOF) {
			channel->stats.eof_interrupts++;

			/* If the transfer error field is set then there has
			 * been an error for this transfer reported from outside
			 * of DMA, e.g. MIPI overflow error.
			 */
			dma_report_completion(pb, channel, transfer,
					transfer->error ?
					transfer->error : 0);

			if (list_empty(&channel->pending_list)) {
				/* If there are no active transfers then power
				 * down the DMA channel.
				 */
				if (channel->active_count == 0)
					io_disable_dma_channel(pb, channel_id);

				spin_unlock(&pb->dma.dma_lock);
				continue;
			}

			if (channel->active_count >= MAX_ACTIVE_TRANSFERS) {
				spin_unlock(&pb->dma.dma_lock);
				dev_err(&pb->pdev->dev,
						"dma channel%u: too many "
						"transfers\n", channel_id);
				continue;
			}

			transfer = list_entry(channel->pending_list.next,
				struct paintbox_dma_transfer, entry);

			if (transfer->auto_load_transfer)
				commit_transfer_to_hardware(pb, channel);
		}

		/* If there are no active transfers then power down the DMA
		 * channel.
		 */
		if (channel->active_count == 0)
			io_disable_dma_channel(pb, channel->channel_id);

		spin_unlock(&pb->dma.dma_lock);
	}

	return IRQ_HANDLED;
}

int paintbox_dma_init(struct paintbox_data *pb)
{
	unsigned int i;

	pb->dma.dma_base = pb->reg_base + IPU_DMA_OFFSET;

	pb->dma.num_channels = readl(pb->dma.dma_base + DMA_CAP0) &
			MAX_DMA_CHAN_MASK;

	spin_lock_init(&pb->dma.dma_lock);
	INIT_LIST_HEAD(&pb->dma.discard_list);

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
		INIT_LIST_HEAD(&channel->pending_list);
		INIT_LIST_HEAD(&channel->active_list);
		INIT_LIST_HEAD(&channel->completed_list);
		init_completion(&channel->stop_completion);
#ifdef CONFIG_DEBUG_FS
		paintbox_dma_channel_debug_init(pb, channel);
#endif
	}

	dev_dbg(&pb->pdev->dev, "dma: base %p len %u dma channels %u\n",
			pb->dma.dma_base, DMA_BLOCK_LEN,
			pb->dma.num_channels);

	return 0;
}
