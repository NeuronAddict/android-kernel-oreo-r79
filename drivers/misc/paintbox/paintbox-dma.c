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
#include "paintbox-io.h"
#include "paintbox-lbp.h"
#include "paintbox-regs.h"
#include "paintbox-sim-regs.h"


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
int unbind_dma_interrupt(struct paintbox_data *pb,
		struct paintbox_session *session, uint8_t interrupt_id,
		uint8_t channel_id)
{
	struct paintbox_dma_channel *dma;
	struct paintbox_irq *irq;
	unsigned long irq_flags;
	int ret = 0;

	dma = get_dma_channel(pb, session, channel_id, &ret);
	if (ret < 0)
		return ret;

	irq = get_interrupt(pb, session, interrupt_id, &ret);
	if (ret < 0)
		return ret;

	spin_lock_irqsave(&pb->irq_lock, irq_flags);

	dma->interrupt_id = DMA_NO_INTERRUPT;
	irq->channel_id = IRQ_NO_DMA_CHANNEL;

	spin_unlock_irqrestore(&pb->irq_lock, irq_flags);

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
	struct paintbox_dma_channel *dma;
	int ret = 0;

	mutex_lock(&pb->lock);
	dma = get_dma_channel(pb, session, channel_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	/* Unbind any associated interrupt */
	if (dma->interrupt_id != DMA_NO_INTERRUPT) {
		ret = unbind_dma_interrupt(pb, session, dma->interrupt_id,
				dma->channel_id);
		if (ret < 0) {
			mutex_unlock(&pb->lock);
			return ret;
		}
	}

	release_dma_channel(pb, session, dma);
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

/* The caller to this function must hold pb->lock */
static int dma_map_buffer_cma(struct paintbox_data *pb,
		struct paintbox_dma_transfer *transfer, void __user *buf,
		size_t len_bytes, enum dma_data_direction dir)
{
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

	transfer->buf_vaddr = dma_alloc_coherent(&pb->pdev->dev, len_bytes,
			&transfer->buf_paddr, GFP_KERNEL);
	if (!transfer->buf_vaddr) {
		dev_err(&pb->pdev->dev, "%s: allocation failure\n", __func__);
		return -ENOMEM;
	}

	/* Copy the entire user buffer into the transfer buffer for both
	 * directions in case the actual transfer is just a stripe.  This will
	 * ensure that portions of the buffer outside of the stripe will be
	 * consistent between the user buffer and the transfer buffer.
	 *
	 * TODO(ahampson):  Once the driver switches to locked user buffers this
	 * will no longer be necessary.  b/28405438
	 */
	if (copy_from_user(transfer->buf_vaddr, buf, len_bytes)) {
		dma_free_coherent(&pb->pdev->dev, len_bytes,
				transfer->buf_vaddr, transfer->buf_paddr);

		transfer->len_bytes = 0;
		transfer->buf_vaddr = NULL;
		transfer->buf_paddr = 0;

		return -EFAULT;
	}

	if (dir == DMA_TO_DEVICE)
		dma_sync_single_for_device(&pb->pdev->dev, transfer->buf_paddr,
				len_bytes, DMA_TO_DEVICE);

	transfer->len_bytes = len_bytes;

	dev_dbg(&pb->pdev->dev, "%s: len %lu\n", __func__,  len_bytes);
	dev_dbg(&pb->pdev->dev, "\tva 0x%p pa 0x%pa\n",
			transfer->buf_vaddr, &transfer->buf_paddr);

	return 0;
}

/* The caller to this function must hold pb->lock */
static int dma_unmap_buffer_cma(struct paintbox_data *pb,
		struct paintbox_dma_transfer *transfer, void __user *buf,
		size_t len_bytes, enum dma_data_direction dir)
{
	if (dir == DMA_FROM_DEVICE) {
		dma_sync_single_for_cpu(&pb->pdev->dev,
			transfer->buf_paddr, transfer->len_bytes,
			DMA_FROM_DEVICE);

		if (copy_to_user(buf, transfer->buf_vaddr,
				min(transfer->len_bytes, len_bytes)))
			return -EFAULT;
	}

	dev_dbg(&pb->pdev->dev, "%s: len %lu\n", __func__, transfer->len_bytes);
	dev_dbg(&pb->pdev->dev, "\tva 0x%p pa 0x%pa\n",
			transfer->buf_vaddr,
			&transfer->buf_paddr);

	dma_free_coherent(&pb->pdev->dev, transfer->len_bytes,
			transfer->buf_vaddr, transfer->buf_paddr);

	/* TODO(ahampson):  We are going to temporarily have one hardcoded
	 * transfer associated with each channel.  NULL out the transfer instead
	 * of freeing it.
	 */
	transfer->len_bytes = 0;
	transfer->buf_vaddr = NULL;
	transfer->buf_paddr = 0;

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
	memset(transfer, 0, sizeof(struct paintbox_dma_transfer));

	switch (config.transfer_type) {
	case DMA_DRAM_TO_LBP:
		if (config.src.dram.len_bytes > DMA_MAX_IMG_TRANSFER_LEN) {
			mutex_unlock(&pb->lock);
			return -ERANGE;
		}

		if (!access_ok(VERIFY_READ, config.src.dram.host_vaddr,
				config.src.dram.len_bytes)) {
			mutex_unlock(&pb->lock);
			return -EFAULT;
		}

		/* TODO(ahampson):  This needs to be conditionalized for IOMMU
		 * or CMA.
		 */
		ret = dma_map_buffer_cma(pb, transfer,
				config.src.dram.host_vaddr,
				config.src.dram.len_bytes, DMA_TO_DEVICE);
		if (ret < 0) {
			mutex_unlock(&pb->lock);
			return ret;
		}

		ret = dma_setup_dram_to_lbp_transfer(pb, session, channel,
				transfer, &config);
		break;
	case DMA_DRAM_TO_STP:
		ret = dma_setup_dram_to_stp_transfer(pb, session, channel,
				transfer, &config);
		break;
	case DMA_LBP_TO_DRAM:
		if (config.dst.dram.len_bytes > DMA_MAX_IMG_TRANSFER_LEN) {
			mutex_unlock(&pb->lock);
			return -ERANGE;
		}

		if (!access_ok(VERIFY_WRITE, config.dst.dram.host_vaddr,
				config.dst.dram.len_bytes)) {
			mutex_unlock(&pb->lock);
			return -EFAULT;
		}

		/* TODO(ahampson):  This needs to be conditionalized for IOMMU
		 * or CMA.
		 */
		ret = dma_map_buffer_cma(pb, transfer,
				config.dst.dram.host_vaddr,
				config.dst.dram.len_bytes, DMA_FROM_DEVICE);
		if (ret < 0) {
			mutex_unlock(&pb->lock);
			return ret;
		}

		ret = dma_setup_lbp_to_dram_transfer(pb, session, channel,
				transfer, &config);
		break;
	case DMA_STP_TO_DRAM:
		ret = dma_setup_stp_to_dram_transfer(pb, session, channel,
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
			req.len_bytes, DMA_FROM_DEVICE);

	mutex_unlock(&pb->lock);

	return ret;
}

int start_dma_transfer_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	unsigned int channel_id = (unsigned int)arg;
	struct paintbox_dma_channel *dma;
	int ret;

	mutex_lock(&pb->lock);
	dma = get_dma_channel(pb, session, channel_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	dev_dbg(&pb->pdev->dev, "%s: dma%u: start\n",  __func__, channel_id);

	ret = dma_start_transfer(pb, dma);
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
	struct interrupt_config __user *user_config;
	struct interrupt_config config;
	struct paintbox_dma_channel *dma;
	struct paintbox_irq *irq;
	unsigned long irq_flags;
	int ret;

	user_config = (struct interrupt_config __user *)arg;
	if (copy_from_user(&config, user_config, sizeof(config)))
		return -EFAULT;

	mutex_lock(&pb->lock);
	dma = get_dma_channel(pb, session, config.channel_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	irq = get_interrupt(pb, session, config.interrupt_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	if (dma->interrupt_id != DMA_NO_INTERRUPT) {
		dev_err(&pb->pdev->dev,
				"%s: dma%u: unable to bind int%u, int%u is "
				"already bound to this channel\n", __func__,
				config.channel_id, config.interrupt_id,
				dma->interrupt_id);
		mutex_unlock(&pb->lock);
		return -EEXIST;
	}

	if (irq->channel_id != IRQ_NO_DMA_CHANNEL) {
		dev_err(&pb->pdev->dev,
				"%s: dma%u: unable to bind int%u, the interrupt"
				" is already bound to dma%u\n",
				__func__, config.channel_id,
				config.interrupt_id,
				irq->channel_id);
		mutex_unlock(&pb->lock);
		return -EEXIST;
	}

	dev_dbg(&pb->pdev->dev, "%s: dma%u: bind int%u" , __func__,
			config.channel_id, config.interrupt_id);

	spin_lock_irqsave(&pb->irq_lock, irq_flags);

	irq->channel_id = config.channel_id;
	dma->interrupt_id = config.interrupt_id;

	/* TODO(ahampson): This should be cleaned up when the QEMU kernel is
	 * updated to 3.13 or greater.
	 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 13, 0)
	INIT_COMPLETION(irq->completion);
#else
	reinit_completion(&irq->completion);
#endif

	spin_unlock_irqrestore(&pb->irq_lock, irq_flags);

	mutex_unlock(&pb->lock);

	return 0;
}

int unbind_dma_interrupt_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	uint8_t channel_id = (uint8_t)arg;
	struct paintbox_dma_channel *dma;
	int ret = 0;

	mutex_lock(&pb->lock);
	dma = get_dma_channel(pb, session, channel_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	if (dma->interrupt_id == DMA_NO_INTERRUPT) {
		dev_err(&pb->pdev->dev, "%s: dma%u: no interrupt to unbind\n",
				__func__, channel_id);
		mutex_unlock(&pb->lock);
		return -ENOENT;
	}

	dev_dbg(&pb->pdev->dev, "%s: dma%u: unbind int%u" , __func__,
			channel_id, dma->interrupt_id);

	ret = unbind_dma_interrupt(pb, session, dma->interrupt_id,
			dma->channel_id);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	mutex_unlock(&pb->lock);

	return 0;
}

void dma_report_completion(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel, int err)
{
	struct paintbox_irq *irq;

	if (channel->interrupt_id == DMA_NO_INTERRUPT)
		return;

	irq = &pb->irqs[channel->interrupt_id];
	irq->error = err;

	complete_all(&irq->completion);
}
