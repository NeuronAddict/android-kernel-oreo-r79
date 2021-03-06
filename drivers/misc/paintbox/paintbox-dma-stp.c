/*
 * DMA STP driver support for the Paintbox programmable IPU
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

#include "paintbox-debug.h"
#include "paintbox-dma.h"
#include "paintbox-dma-common.h"
#include "paintbox-dma-debug.h"
#include "paintbox-dma-dram.h"
#include "paintbox-dma-stp.h"
#include "paintbox-io.h"
#include "paintbox-lbp.h"
#include "paintbox-regs.h"
#include "paintbox-stp.h"

/* TODO:  Temporarily make stp dma configuration validation a debug
 * only operation.  b/62353362
 */
#ifdef DEBUG
/* DRAM to STP transfers must be aligned to 32 byte SRAM offset */
#define DMA_STP_SRAM_ADDR_ALIGN_MASK 0x1F

static int validate_stp_inst_sram_transfer(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel, struct paintbox_stp *stp,
		struct dma_dram_config *dram_config,
		struct dma_stp_config *stp_config)
{
	size_t sram_size_bytes = pb->stp.inst_mem_size_in_instructions *
			STP_INST_SRAM_INSTRUCTION_WIDTH_BYTES;

	if ((stp_config->sram_addr & DMA_STP_SRAM_ADDR_ALIGN_MASK) != 0) {
		dev_err(&pb->pdev->dev,
				"%s: dma channel%u SRAM address 0x%04x is not aligned\n",
				__func__, channel->channel_id,
				stp_config->sram_addr);
		return -EINVAL;
	}

	if (stp_config->sram_addr + dram_config->len_bytes > sram_size_bytes) {
		dev_err(&pb->pdev->dev,
				"%s: dma channel%u not enough space in target SRAM, SRAM ADDR 0x%04x + %llu bytes > %lu bytes\n",
				__func__, channel->channel_id,
				stp_config->sram_addr, dram_config->len_bytes,
				sram_size_bytes);
		return -ERANGE;
	}

	return 0;
}

static int validate_stp_cnst_sram_transfer(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel, struct paintbox_stp *stp,
		struct dma_dram_config *dram_config,
		struct dma_stp_config *stp_config)
{
	size_t sram_size_bytes = pb->stp.const_mem_size_in_words *
			STP_CONST_SRAM_WORD_WIDTH_BYTES;

	if ((stp_config->sram_addr & DMA_STP_SRAM_ADDR_ALIGN_MASK) != 0) {
		dev_err(&pb->pdev->dev,
				"%s: dma channel%u SRAM address 0x%04x is not aligned\n",
				__func__, channel->channel_id,
				stp_config->sram_addr);
		return -EINVAL;
	}

	if (stp_config->sram_addr + dram_config->len_bytes > sram_size_bytes) {
		dev_err(&pb->pdev->dev,
				"%s: dma channel%u not enough space in target SRAM, SRAM ADDR 0x%04x + %llu bytes > %lu bytes\n",
				__func__, channel->channel_id,
				stp_config->sram_addr, dram_config->len_bytes,
				sram_size_bytes);
		return -ERANGE;
	}

	return 0;
}

static int validate_stp_scalar_sram_transfer(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel, struct paintbox_stp *stp,
		struct dma_dram_config *dram_config,
		struct dma_stp_config *stp_config)
{
	size_t sram_size_bytes = pb->stp.scalar_mem_size_in_words *
			STP_SCALAR_SRAM_WORD_WIDTH_BYTES;

	if ((stp_config->sram_addr & DMA_STP_SRAM_ADDR_ALIGN_MASK) != 0) {
		dev_err(&pb->pdev->dev,
				"%s: dma channel%u SRAM address 0x%04x is not aligned\n",
				__func__, channel->channel_id,
				stp_config->sram_addr);
		return -EINVAL;
	}

	if (stp_config->sram_addr + dram_config->len_bytes > sram_size_bytes) {
		dev_err(&pb->pdev->dev,
				"%s: dma channel%u not enough space in target SRAM, SRAM ADDR 0x%04x + %llu bytes > %lu bytes\n",
				__func__, channel->channel_id,
				stp_config->sram_addr,
				dram_config->len_bytes, sram_size_bytes);
		return -ERANGE;
	}

	return 0;
}
#endif

/* The caller to this function must hold pb->lock */
static int set_dma_stp_parameters(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer,
		struct dma_dram_config *dram_config,
		struct dma_stp_config *stp_config)
{
	struct paintbox_stp *stp;
	int ret = 0;

	stp = get_stp(pb, session, stp_config->stp_id, &ret);
	if (ret < 0)
		return ret;

	/* For STP DMA transfers the SRAM address and target are encoded into
	 * the LB_START_X and LB_START_Y fields.
	 */
	switch (stp_config->sram_target) {
	case SRAM_TARGET_STP_INSTRUCTION_RAM:
		/* TODO:  Temporarily make stp dma configuration
		 * validation a debug only operation.  b/62353362
		 */
#ifdef DEBUG
		ret = validate_stp_inst_sram_transfer(pb, channel, stp,
				dram_config, stp_config);
		if (ret < 0)
			return ret;
#endif
		paintbox_dma_set_lb_start(transfer,
				(uint64_t)stp_config->sram_addr,
				DMA_CHAN_LB_START_Y_STP_IRAM);
		break;
	case SRAM_TARGET_STP_CONSTANT_RAM:
		/* TODO:  Temporarily make stp dma configuration
		 * validation a debug only operation.  b/62353362
		 */
#ifdef DEBUG
		ret = validate_stp_cnst_sram_transfer(pb, channel, stp,
				dram_config, stp_config);
		if (ret < 0)
			return ret;
#endif
		paintbox_dma_set_lb_start(transfer,
				(uint64_t)stp_config->sram_addr,
				DMA_CHAN_LB_START_Y_STP_CRAM);
		break;
	case SRAM_TARGET_STP_SCALAR_RAM:
		/* TODO:  Temporarily make stp dma configuration
		 * validation a debug only operation.  b/62353362
		 */
#ifdef DEBUG
		ret = validate_stp_scalar_sram_transfer(pb, channel, stp,
				dram_config, stp_config);
		if (ret < 0)
			return ret;
#endif
		paintbox_dma_set_lb_start(transfer,
				(uint64_t)stp_config->sram_addr,
				DMA_CHAN_LB_START_Y_STP_DRAM);
		break;
	case SRAM_TARGET_STP_VECTOR_RAM:
		/* TODO:  Add parameter checks for vector b/30969166
		 */
		paintbox_dma_set_lb_start(transfer,
				(uint64_t)stp_config->sram_addr,
				stp_config->include_halo ?
				DMA_CHAN_LB_START_Y_STP_ARRAY_32x32 :
				DMA_CHAN_LB_START_Y_STP_ARRAY_16x16);
		break;
	default:
		dev_err(&pb->pdev->dev,
				"%s: dma channel%u invalid STP SRAM type, %u\n",
				__func__, channel->channel_id,
				stp_config->sram_target);
		return -EINVAL;
	};

	/* Set the STP node configuration */
	transfer->chan_node = stp_config->stp_id;

	return 0;
}

/* The caller to this function must hold pb->lock */
int dma_setup_dram_to_stp_transfer(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer,
		struct dma_transfer_config *config)
{
	int ret;

	/* TODO:  Temporarily make stp dma configuration validation a
	 * debug only operation.  b/62353362
	 */
#ifdef DEBUG
	if (config->src.dram.len_bytes > DMA_CHAN_VA_BDRY_LEN_MAX) {
		dev_err(&pb->pdev->dev,
				"%s: dma channel%u: transfer too large, %llu max %llu bytes",
				__func__, channel->channel_id,
				config->dst.dram.len_bytes,
				DMA_CHAN_VA_BDRY_LEN_MAX);
		return -ERANGE;
	}

	/* Verify that the target STP is part of the session. */
	ret = validate_stp(pb, session, config->dst.stp.stp_id);
	if (ret < 0) {
		dev_err(&pb->pdev->dev,
				"%s: dma channel%u: stp%u is not part of the session, err %d\n",
				__func__, channel->channel_id,
				config->dst.stp.stp_id, ret);
		return ret;
	}
#endif

	ret = ipu_dma_attach_buffer(pb, channel, transfer, &config->src.dram,
			DMA_TO_DEVICE);
	if (ret < 0)
		return ret;

	paintbox_dma_set_channel_mode(pb, session, channel, transfer,
			DMA_CHAN_MODE_SRC_DRAM, DMA_CHAN_MODE_DST_STP, false);

	ret = set_dma_stp_parameters(pb, session, channel, transfer,
			&config->src.dram, &config->dst.stp);
	if (ret < 0)
		goto err_exit;

	ret = set_dma_image_parameters(pb, channel, transfer, &config->img);
	if (ret < 0)
		goto err_exit;

	ret = set_dma_transfer_region_parameters(pb, channel, transfer, config);
	if (ret < 0)
		goto err_exit;

	LOG_DMA_DRAM_TO_STP_TRANSFER(pb, channel, transfer, config);

	return 0;

err_exit:
	ipu_dma_release_buffer(pb, transfer);

	return ret;
}
