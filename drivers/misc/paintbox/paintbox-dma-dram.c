/*
 * DMA DRAM support for the Paintbox programmable IPU
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

#include <linux/atomic.h>
#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include "paintbox-dma-dram.h"

int dma_map_buffer_cma(struct paintbox_data *pb,
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

	transfer->dir = dir;
	transfer->len_bytes = len_bytes;

	dev_dbg(&pb->pdev->dev, "%s: len %lu\n", __func__,  len_bytes);
	dev_dbg(&pb->pdev->dev, "\tva 0x%p pa 0x%pa\n",
			transfer->buf_vaddr, &transfer->buf_paddr);

	return 0;
}

/* The caller to this function must hold pb->lock */
int dma_unmap_buffer_cma(struct paintbox_data *pb,
		struct paintbox_dma_transfer *transfer, void __user *buf,
		size_t len_bytes)
{
	int ret = 0;

	if (transfer->dir == DMA_FROM_DEVICE && buf != NULL) {
		dma_sync_single_for_cpu(&pb->pdev->dev,
			transfer->buf_paddr, transfer->len_bytes,
			DMA_FROM_DEVICE);

		if (copy_to_user(buf, transfer->buf_vaddr,
				min(transfer->len_bytes, len_bytes)))
			ret = -EFAULT;
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

	return ret;
}
