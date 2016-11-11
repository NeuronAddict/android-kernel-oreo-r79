/*
 * DMA MIPI driver support for the Paintbox programmable IPU
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

#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/paintbox.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include "paintbox-common.h"
#include "paintbox-dma.h"
#include "paintbox-dma-common.h"
#include "paintbox-dma-dram.h"
#include "paintbox-dma-mipi.h"
#include "paintbox-regs.h"

/* The caller to this function must hold pb->lock */
int dma_setup_mipi_to_dram_transfer(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer,
		struct dma_transfer_config *config)
{
	int ret;

	if (config->dst.dram.len_bytes > DMA_MAX_IMG_TRANSFER_LEN)
		return -ERANGE;

	if (!access_ok(VERIFY_WRITE, config->dst.dram.host_vaddr,
			config->dst.dram.len_bytes))
		return -EFAULT;

	/* A MIPI transfer can not be initiated if the corresponding MIPI
	 * stream for this channel is not in the session.
	 */
	if (!channel->mipi_stream) {
		dev_err(&pb->pdev->dev,
				"%s: dma channel%u no associated mipi stream\n",
				__func__, channel->channel_id);
		return -EINVAL;
	}

	/* TODO(ahampson): This needs to be conditionalized for IOMMU or CMA. */
	ret = dma_map_buffer_cma(pb, transfer, config->dst.dram.host_vaddr,
			config->dst.dram.len_bytes, DMA_FROM_DEVICE);
	if (ret < 0)
		return ret;

	set_dma_channel_mode(transfer, DMA_CHAN_SRC_MIPI_IN, DMA_CHAN_DST_DRAM,
			false);

	ret = set_dma_image_parameters(pb, channel, transfer, &config->img);
	if (ret < 0)
		goto err_exit;

	ret = set_dma_transfer_region_parameters(pb, channel, transfer, config);
	if (ret < 0)
		goto err_exit;

	set_dma_dram_parameters(pb, channel, transfer);

	dev_dbg(&pb->pdev->dev,
			"%s: dma%u: mipi input -> va %p pa %pa %llu bytes\n",
			__func__, config->channel_id, transfer->buf_vaddr,
			&transfer->buf_paddr, config->dst.dram.len_bytes);

	return 0;

err_exit:
	dma_unmap_buffer_cma(pb, transfer, NULL, 0);

	return ret;
}
