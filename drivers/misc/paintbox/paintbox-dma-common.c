/*
 * DMA common support for the Paintbox programmable IPU
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
#include <linux/paintbox.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include "paintbox-debug.h"
#include "paintbox-dma.h"
#include "paintbox-common.h"
#include "paintbox-dma-debug.h"
#include "paintbox-io.h"
#include "paintbox-lbp.h"
#include "paintbox-regs.h"
#include "paintbox-stp.h"


/* The caller to this function must hold pb->lock */
void set_dma_channel_mode(struct paintbox_dma_transfer *transfer,
		uint32_t src_type, uint32_t dst_type, bool gather)
{
	transfer->chan_mode = src_type << DMA_CHAN_SRC_SHIFT;
	transfer->chan_mode |= dst_type << DMA_CHAN_DST_SHIFT;
	transfer->chan_mode |= DMA_CHAN_ENA;

	if (gather)
		transfer->chan_mode |= DMA_CHAN_GATHER;
}

/* The caller to this function must hold pb->lock */
static int validate_dma_image_parameters(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel,
		struct dma_image_config *config)
{
	/* Image Size */
	if (config->width_pixels > DMA_CHAN_IMG_SIZE_MAX) {
		dev_err(&pb->pdev->dev,
				"%s: dma channel%u image width too large, %u > "
				"%u\n", __func__, channel->channel_id,
				config->width_pixels, DMA_CHAN_IMG_SIZE_MAX);
		return -ERANGE;
	}

	if (config->height_pixels > DMA_CHAN_IMG_SIZE_MAX) {
		dev_err(&pb->pdev->dev,
				"%s: dma channel%u image height too large, %u >"
				"%u\n", __func__, channel->channel_id,
				config->height_pixels, DMA_CHAN_IMG_SIZE_MAX);
		return -ERANGE;
	}

	/* Image Position */
	if (config->start_x_pixels > DMA_CHAN_START_MAX ||
			config->start_x_pixels < DMA_CHAN_START_MIN) {
		dev_err(&pb->pdev->dev,
				"%s: dma channel%u start x out of bounds, %d "
				"(%d..%d)\n", __func__, channel->channel_id,
				config->start_x_pixels, DMA_CHAN_START_MIN,
				DMA_CHAN_START_MAX);
		return -ERANGE;
	}

	if (config->start_y_pixels > DMA_CHAN_START_MAX ||
			config->start_y_pixels < DMA_CHAN_START_MIN) {
		dev_err(&pb->pdev->dev,
				"%s: dma channel%u start y out of bounds, %d "
				"(%d..%d)\n", __func__, channel->channel_id,
				config->start_y_pixels, DMA_CHAN_START_MIN,
				DMA_CHAN_START_MAX);
		return -ERANGE;
	}

	/* Image Format */
	if (config->components < DMA_CHAN_MIN_COMPONENTS ||
			config->components > DMA_CHAN_MAX_COMPONENTS) {
		dev_err(&pb->pdev->dev,
				"%s: dma channel%u invalid number of components"
				", %u (%u..%u)\n", __func__,
				channel->channel_id, config->components,
				DMA_CHAN_MIN_COMPONENTS,
				DMA_CHAN_MAX_COMPONENTS);
		return -EINVAL;
	}

	if (config->planes < DMA_CHAN_MIN_PLANES ||
			config->planes > DMA_CHAN_MAX_PLANES) {
		dev_err(&pb->pdev->dev,
				"%s: dma channel%u invalid number of planes, %u"
				" (%u..%u)\n", __func__, channel->channel_id,
				config->planes, DMA_CHAN_MIN_PLANES,
				DMA_CHAN_MAX_PLANES);
		return -EINVAL;
	}

	/* Image Layout */
	if (config->row_stride_bytes > DMA_CHAN_ROW_STRIDE_MAX) {
		dev_err(&pb->pdev->dev,
				"%s: dma channel%u invalid row stride, %u >= %u"
				"\n", __func__, channel->channel_id,
				config->row_stride_bytes,
				DMA_CHAN_ROW_STRIDE_MAX);
		return -EINVAL;
	}

	if (config->plane_stride_bytes > DMA_CHAN_PLANE_STRIDE_MAX) {
		dev_err(&pb->pdev->dev,
				"%s: dma channel%u invalid plane stride, %llu "
				" >= %llu\n", __func__, channel->channel_id,
				config->plane_stride_bytes,
				DMA_CHAN_PLANE_STRIDE_MAX);
		return -EINVAL;
	}

	return 0;
}

/* The caller to this function must hold pb->lock */
int set_dma_image_parameters(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer,
		struct dma_image_config *config)
{
	int ret;

	ret = validate_dma_image_parameters(pb, channel, config);
	if (ret < 0)
		return ret;

	/* Image Start Position */
	transfer->chan_img_pos_low = (uint16_t)config->start_y_pixels;
	transfer->chan_img_pos_low <<= DMA_CHAN_START_Y_SHIFT;
	transfer->chan_img_pos_low |= (uint16_t)config->start_x_pixels;


	/* Image Size */
	transfer->chan_img_size = config->width_pixels;
	transfer->chan_img_size |= config->height_pixels <<
			DMA_CHAN_IMG_HEIGHT_SHIFT;

	/* Image Format */
	transfer->chan_img_format = config->components - 1;
	transfer->chan_img_format |= (config->planes - 1) <<
			DMA_CHAN_PLANES_SHIFT;

	switch (config->bit_depth) {
	case 8:
		transfer->chan_img_format |= DMA_CHAN_BIT_DEPTH8 <<
				DMA_CHAN_BIT_DEPTH_SHIFT;
		break;
	case 10:
		transfer->chan_img_format |= DMA_CHAN_BIT_DEPTH10 <<
				DMA_CHAN_BIT_DEPTH_SHIFT;
		break;
	case 12:
		transfer->chan_img_format |= DMA_CHAN_BIT_DEPTH12 <<
				DMA_CHAN_BIT_DEPTH_SHIFT;
		break;
	case 14:
		transfer->chan_img_format |= DMA_CHAN_BIT_DEPTH14 <<
				DMA_CHAN_BIT_DEPTH_SHIFT;
		break;
	case 16:
		transfer->chan_img_format |= DMA_CHAN_BIT_DEPTH16 <<
				DMA_CHAN_BIT_DEPTH_SHIFT;
		break;
	default:
		dev_err(&pb->pdev->dev,
			"%s: dma channel%u: unsupported bit depth %u", __func__,
			channel->channel_id, config->bit_depth);
		return -EINVAL;
	};

	switch (config->rgba_format) {
	case RGBA_FORMAT_DISABLED:
		transfer->chan_img_format |=
				DMA_CHAN_RGBA_FORMAT_DISABLED <<
				DMA_CHAN_RGBA_FORMAT_SHIFT;
		break;
	case RGBA_FORMAT_RGBA:
		transfer->chan_img_format |= DMA_CHAN_RGBA_FORMAT_RGBA <<
				DMA_CHAN_RGBA_FORMAT_SHIFT;
		break;
	case RGBA_FORMAT_ARGB:
		transfer->chan_img_format |= DMA_CHAN_RGBA_FORMAT_ARGB <<
				DMA_CHAN_RGBA_FORMAT_SHIFT;
		break;
	default:
		dev_err(&pb->pdev->dev,
			"%s: dma channel%u: invalid RGBA format %u", __func__,
			channel->channel_id, config->rgba_format);
		return -EINVAL;
	};

	if (config->block4x4)
		transfer->chan_img_format |= DMA_CHAN_BLOCK_4X4;

	if (config->mipi_raw_format)
		transfer->chan_img_format |= DMA_CHAN_MIPI_RAW_FORMAT;

	/* Image Layout */
	transfer->chan_img_layout_low = config->row_stride_bytes;
	transfer->chan_img_layout_low |= (config->plane_stride_bytes &
			DMA_CHAN_PLANE_STRIDE_LOW_M) <<
			DMA_CHAN_PLANE_STRIDE_LOW_SHIFT;
	transfer->chan_img_layout_high |= config->plane_stride_bytes >>
			DMA_CHAN_PLANE_STRIDE_LOW_WIDTH;

	return 0;
}

/* The caller to this function must hold pb->lock */
int set_dma_transfer_region_parameters(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer,
		struct dma_transfer_config *config)
{
	if (config->stripe_height > DMA_CHAN_MAX_STRIPES) {
		dev_err(&pb->pdev->dev,
				"%s: dma channel%u: invalid stripe height %u > "
				"%u\n", __func__, channel->channel_id,
				config->stripe_height, DMA_CHAN_MAX_STRIPES);
		return -EINVAL;
	}

	if (config->sheet_width > DMA_CHAN_MAX_SHEET_WIDTH) {
		dev_err(&pb->pdev->dev,
				"%s: dma channel%u: invalid sheet width %u > "
				"%u", __func__, channel->channel_id,
				config->sheet_width, DMA_CHAN_MAX_SHEET_WIDTH);
		return -EINVAL;
	}

	if (config->sheet_height > DMA_CHAN_MAX_SHEET_HEIGHT) {
		dev_err(&pb->pdev->dev,
				"%s: dma channel%u: invalid sheet height %u > "
				"%u\n", __func__, channel->channel_id,
				config->sheet_height,
				DMA_CHAN_MAX_SHEET_HEIGHT);
		return -EINVAL;
	}

	if (config->noc_outstanding < DMA_CHAN_NOC_OUTSTANDING_MIN ||
		config->noc_outstanding > DMA_CHAN_NOC_OUTSTANDING_MAX) {
		dev_err(&pb->pdev->dev,
				"%s: dma channel%u: invalid NOC outstanding "
				"value %u, (%u..%u)\n", __func__,
				channel->channel_id, config->noc_outstanding,
				DMA_CHAN_NOC_OUTSTANDING_MIN,
				DMA_CHAN_NOC_OUTSTANDING_MAX);
		return -EINVAL;
	}

	if (config->retry_interval > DMA_CHAN_RETRY_INTERVAL_MAX ||
			config->retry_interval < DMA_CHAN_RETRY_INTERVAL_MIN) {
		dev_err(&pb->pdev->dev,
				"%s: dma channel%u: invalid retry interval "
				"value %u, (%u..%u\n", __func__,
				channel->channel_id, config->retry_interval,
				DMA_CHAN_RETRY_INTERVAL_MIN,
				DMA_CHAN_RETRY_INTERVAL_MAX);
		return -EINVAL;
	}

	transfer->chan_bif_xfer = config->stripe_height;
	transfer->chan_bif_xfer |= (pb->dma.bif_outstanding - 1) <<
			DMA_CHAN_OUTSTANDING_SHIFT;
	transfer->chan_noc_xfer_low = config->sheet_width;
	transfer->chan_noc_xfer_low |= config->sheet_height <<
			DMA_CHAN_SHEET_HEIGHT_SHIFT;
	transfer->chan_noc_xfer_low |= config->noc_outstanding <<
			DMA_CHAN_NOC_OUTSTANDING_SHIFT;
	transfer->chan_noc_xfer_high = config->retry_interval;

	/* TODO(ahampson):  DMA_CHAN_DYN_OUTSTANDING is currently set for all
	 * DMA transfers at this time.  This may change in the future to give
	 * priority to MIPI transfers.
	 */
	transfer->chan_noc_xfer_high |= DMA_CHAN_NOC_XFER_DYN_OUTSTANDING_MASK;

	return 0;
}
