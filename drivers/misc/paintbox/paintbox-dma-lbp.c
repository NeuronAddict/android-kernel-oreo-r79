/*
 * DMA LBP support for the Paintbox programmable IPU
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
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <uapi/paintbox.h>

#include "paintbox-common.h"
#include "paintbox-debug.h"
#include "paintbox-dma.h"
#include "paintbox-dma-common.h"
#include "paintbox-dma-debug.h"
#include "paintbox-dma-dram.h"
#include "paintbox-dma-lbp.h"
#include "paintbox-io.h"
#include "paintbox-lbp.h"
#include "paintbox-regs.h"
#include "paintbox-stp.h"

/* The caller to this function must hold pb->lock */
static int set_dma_lbp_parameters(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer,
		struct dma_lbp_config *config)
{
	/* TODO:  Temporarily make LBP DMA configuration validation a
	 * debug only operation.  b/62353362
	 */
#ifdef DEBUG
	struct paintbox_lb *lb;
	int ret;

	lb = get_lb(pb, session, config->lbp_id, config->lb_id, &ret);
	if (ret < 0)
		return ret;

	if (config->read_ptr_id >= lb->num_read_ptrs) {
		dev_err(&pb->pdev->dev,
				"%s: dma channel%u invalid rptr id, %u, num read ptrs %u\n",
				__func__, channel->channel_id,
				config->read_ptr_id, lb->num_read_ptrs);
		return -EINVAL;
	}

	if (config->start_x_pixels < DMA_CHAN_IMG_POS_LB_START_MIN ||
			config->start_x_pixels >
			DMA_CHAN_IMG_POS_LB_START_MAX) {
		dev_err(&pb->pdev->dev,
				"%s: dma channel%u lb_start x out of bounds, %d min %d max %d\n",
				__func__, channel->channel_id,
				config->start_x_pixels,
				DMA_CHAN_IMG_POS_LB_START_MIN,
				DMA_CHAN_IMG_POS_LB_START_MAX);
		return -ERANGE;
	}

	if (config->start_y_pixels < DMA_CHAN_IMG_POS_LB_START_MIN ||
			config->start_y_pixels >
			DMA_CHAN_IMG_POS_LB_START_MAX) {
		dev_err(&pb->pdev->dev,
				"%s: dma channel%u lb_start y out of bounds, %d min %d max %d\n",
				 __func__, channel->channel_id,
				config->start_y_pixels,
				DMA_CHAN_IMG_POS_LB_START_MIN,
				DMA_CHAN_IMG_POS_LB_START_MAX);
		return -ERANGE;
	}
#endif

	/* Set the LBP node configuration */
	transfer->chan_node = config->lbp_id;
	transfer->chan_node |= config->lb_id << DMA_CHAN_NODE_LB_ID_SHIFT;
	transfer->chan_node |= config->read_ptr_id <<
			DMA_CHAN_NODE_RPTR_ID_SHIFT;

	/* Set the line buffer image position */
	paintbox_dma_set_lb_start(transfer, (uint64_t)config->start_x_pixels,
			(uint64_t)config->start_y_pixels);

	return 0;
}

/* The caller to this function must hold pb->lock */
int dma_setup_dram_to_lbp_transfer(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer,
		struct dma_transfer_config *config)
{
	int ret;

	/* TODO:  Temporarily make LBP DMA configuration validation a
	 * debug only operation.  b/62353362
	 */
#ifdef DEBUG
	if (config->src.dram.len_bytes > DMA_CHAN_VA_BDRY_LEN_MAX) {
		dev_err(&pb->pdev->dev,
				"%s: dma channel%u transfer too large, %llu max %llu bytes",
				__func__, channel->channel_id,
				config->dst.dram.len_bytes,
				DMA_CHAN_VA_BDRY_LEN_MAX);
		return -ERANGE;
	}
#endif

	ret = ipu_dma_attach_buffer(pb, channel, transfer, &config->src.dram,
			DMA_TO_DEVICE);
	if (ret < 0)
		return ret;

	if (channel->stats.time_stats_enabled)
		channel->stats.non_dram_setup_start_time = ktime_get_boottime();

	paintbox_dma_set_channel_mode(pb, session, channel, transfer,
			DMA_CHAN_MODE_SRC_DRAM, DMA_CHAN_MODE_DST_LBP,
			config->dst.lbp.gather);

	ret = set_dma_lbp_parameters(pb, session, channel, transfer,
			&config->dst.lbp);
	if (ret < 0)
		goto err_exit;

	ret = set_dma_image_parameters(pb, channel, transfer, &config->img);
	if (ret < 0)
		goto err_exit;

	ret = set_dma_transfer_region_parameters(pb, channel, transfer, config);
	if (ret < 0)
		goto err_exit;

	LOG_DMA_DRAM_TO_LBP_TRANSFER(pb, channel, transfer, config);

	if (channel->stats.time_stats_enabled)
		channel->stats.non_dram_setup_finish_time =
				ktime_get_boottime();

	return 0;

err_exit:
	ipu_dma_release_buffer(pb, transfer);

	return ret;
}

/* The caller to this function must hold pb->lock */
int dma_setup_lbp_to_dram_transfer(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer,
		struct dma_transfer_config *config)
{
	int ret;

	/* TODO:  Temporarily make LBP DMA configuration validation a
	 * debug only operation.  b/62353362
	 */
#ifdef DEBUG
	if (config->dst.dram.len_bytes > DMA_CHAN_VA_BDRY_LEN_MAX) {
		dev_err(&pb->pdev->dev,
				"%s: dma channel%u transfer too large, %llu max %llu bytes",
				__func__, channel->channel_id,
				config->dst.dram.len_bytes,
				DMA_CHAN_VA_BDRY_LEN_MAX);
		return -ERANGE;
	}
#endif

	ret = ipu_dma_attach_buffer(pb, channel, transfer, &config->dst.dram,
			DMA_FROM_DEVICE);
	if (ret < 0)
		return ret;

	if (channel->stats.time_stats_enabled)
		channel->stats.non_dram_setup_start_time = ktime_get_boottime();

	paintbox_dma_set_channel_mode(pb, session, channel, transfer,
			DMA_CHAN_MODE_SRC_LBP, DMA_CHAN_MODE_DST_DRAM,
			config->src.lbp.gather);

	ret = set_dma_lbp_parameters(pb, session, channel, transfer,
			&config->src.lbp);
	if (ret < 0)
		goto err_exit;

	ret = set_dma_image_parameters(pb, channel, transfer, &config->img);
	if (ret < 0)
		goto err_exit;

	ret = set_dma_transfer_region_parameters(pb, channel, transfer, config);
	if (ret < 0)
		goto err_exit;

	LOG_DMA_LBP_TO_DRAM_TRANSFER(pb, channel, transfer, config);

	if (channel->stats.time_stats_enabled)
		channel->stats.non_dram_setup_finish_time =
				ktime_get_boottime();

	return 0;

err_exit:
	ipu_dma_release_buffer(pb, transfer);

	return ret;
}

/* The caller to this function must hold pb->lock */
int dma_setup_mipi_to_lbp_transfer(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer,
		struct dma_transfer_config *config)
{
	int ret;

	if (channel->stats.time_stats_enabled)
		channel->stats.non_dram_setup_start_time = ktime_get_boottime();

	/* TODO:  Temporarily make LBP DMA configuration validation a
	 * debug only operation.  b/62353362
	 */
#ifdef DEBUG
	if (config->dst.lbp.gather) {
		dev_err(&pb->pdev->dev,
				"%s: dma channel%u gather mode not supported for MIPI transfers",
				__func__, channel->channel_id);
		return -EINVAL;
	}

	/* A MIPI transfer can not be initiated if the corresponding MIPI
	 * stream for this channel is not in the session.
	 */
	if (!channel->mipi_stream) {
		dev_err(&pb->pdev->dev,
				"%s: dma channel%u no associated mipi stream\n",
				__func__, channel->channel_id);
		return -EINVAL;
	}
#endif

	paintbox_dma_set_channel_mode(pb, session, channel, transfer,
			DMA_CHAN_MODE_SRC_MIPI_IN, DMA_CHAN_MODE_DST_LBP,
			false);

	ret = set_dma_lbp_parameters(pb, session, channel, transfer,
			&config->dst.lbp);
	if (ret < 0)
		return ret;

	ret = set_dma_image_parameters(pb, channel, transfer, &config->img);
	if (ret < 0)
		return ret;

	ret = set_dma_transfer_region_parameters(pb, channel, transfer, config);
	if (ret < 0)
		return ret;

	LOG_DMA_MIPI_TO_LBP_TRANSFER(pb, channel, transfer, config);

	if (channel->stats.time_stats_enabled)
		channel->stats.non_dram_setup_finish_time =
				ktime_get_boottime();

	return 0;
}

/* The caller to this function must hold pb->lock */
int dma_setup_lbp_to_mipi_transfer(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer,
		struct dma_transfer_config *config)
{
	int ret;

	if (channel->stats.time_stats_enabled)
		channel->stats.non_dram_setup_start_time = ktime_get_boottime();

	/* TODO:  Temporarily make LBP DMA configuration validation a
	 * debug only operation.  b/62353362
	 */
#ifdef DEBUG
	if (config->src.lbp.gather) {
		dev_err(&pb->pdev->dev,
				"%s: dma channel%u gather mode not supported for MIPI transfers",
				__func__, channel->channel_id);
		return -EINVAL;
	}

	/* A MIPI transfer can not be initiated if the corresponding MIPI
	 * stream for this channel is not in the session.
	 */
	if (!channel->mipi_stream) {
		dev_err(&pb->pdev->dev,
				"%s: dma channel%u no associated mipi stream\n",
				__func__, channel->channel_id);
		return -EINVAL;
	}
#endif

	paintbox_dma_set_channel_mode(pb, session, channel, transfer,
			DMA_CHAN_MODE_SRC_LBP, DMA_CHAN_MODE_DST_MIPI_OUT,
			false);

	ret = set_dma_lbp_parameters(pb, session, channel, transfer,
			&config->src.lbp);
	if (ret < 0)
		return ret;

	ret = set_dma_image_parameters(pb, channel, transfer, &config->img);
	if (ret < 0)
		return ret;

	ret = set_dma_transfer_region_parameters(pb, channel, transfer, config);
	if (ret < 0)
		return ret;

	LOG_DMA_LBP_TO_MIPI_TRANSFER(pb, channel, transfer, config);

	if (channel->stats.time_stats_enabled)
		channel->stats.non_dram_setup_finish_time =
				ktime_get_boottime();

	return 0;
}
