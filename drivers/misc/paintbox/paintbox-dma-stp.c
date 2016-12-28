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
#include <linux/paintbox.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>

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


static int validate_stp_inst_sram_transfer(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel, struct paintbox_stp *stp,
		struct dma_dram_config *dram_config,
		struct dma_stp_config *stp_config)
{
	size_t sram_size_bytes = stp->inst_mem_size_in_instructions *
			STP_INST_SRAM_INSTRUCTION_WIDTH_BYTES;

	if ((stp_config->sram_addr & DMA_STP_SRAM_ADDR_ALIGN_MASK) != 0) {
		dev_err(&pb->pdev->dev,
				"%s: dma channel%u SRAM address is not aligned,"
				" 0x%04x\n", __func__, channel->channel_id,
				stp_config->sram_addr);
		return -EINVAL;
	}

	if (stp_config->sram_addr + dram_config->len_bytes > sram_size_bytes) {
		dev_err(&pb->pdev->dev,
				"%s: dma channel%u not enough space in target "
				"SRAM, SRAM ADDR 0x%04x + %llu bytes > %lu "
				"bytes\n", __func__, channel->channel_id,
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
	size_t sram_size_bytes = stp->const_mem_size_in_words *
			STP_CONST_SRAM_WORD_WIDTH_BYTES;

	if ((stp_config->sram_addr & DMA_STP_SRAM_ADDR_ALIGN_MASK) != 0) {
		dev_err(&pb->pdev->dev,
				"%s: dma channel%u SRAM address is not aligned,"
				" 0x%04x\n", __func__, channel->channel_id,
				stp_config->sram_addr);
		return -EINVAL;
	}

	if (stp_config->sram_addr + dram_config->len_bytes > sram_size_bytes) {
		dev_err(&pb->pdev->dev,
				"%s: dma channel%u not enough space in target "
				"SRAM, SRAM ADDR 0x%04x + %llu bytes > %lu "
				"bytes\n", __func__, channel->channel_id,
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
	size_t sram_size_bytes = stp->scalar_mem_size_in_words *
			STP_SCALAR_SRAM_WORD_WIDTH_BYTES;

	if ((stp_config->sram_addr & DMA_STP_SRAM_ADDR_ALIGN_MASK) != 0) {
		dev_err(&pb->pdev->dev,
				"%s: dma channel%u SRAM address is not aligned,"
				" 0x%04x\n", __func__, channel->channel_id,
				stp_config->sram_addr);
		return -EINVAL;
	}

	if (stp_config->sram_addr + dram_config->len_bytes > sram_size_bytes) {
		dev_err(&pb->pdev->dev,
				"%s: dma channel%u not enough space in target "
				"SRAM, SRAM ADDR 0x%04x + %llu bytes > %lu "
				"bytes\n", __func__, channel->channel_id,
				stp_config->sram_addr,
				dram_config->len_bytes, sram_size_bytes);
		return -ERANGE;
	}

	return 0;
}

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
		ret = validate_stp_inst_sram_transfer(pb, channel, stp,
				dram_config, stp_config);
		if (ret < 0)
			return ret;

		transfer->chan_img_pos_high = DMA_CHAN_LB_START_Y_STP_IRAM;
		transfer->chan_img_pos_high <<= DMA_CHAN_LB_START_Y_SHIFT;
		transfer->chan_img_pos_high |= (uint16_t)stp_config->sram_addr;
		break;
	case SRAM_TARGET_STP_CONSTANT_RAM:
		ret = validate_stp_cnst_sram_transfer(pb, channel, stp,
				dram_config, stp_config);
		if (ret < 0)
			return ret;

		transfer->chan_img_pos_high = DMA_CHAN_LB_START_Y_STP_CRAM;
		transfer->chan_img_pos_high <<= DMA_CHAN_LB_START_Y_SHIFT;
		transfer->chan_img_pos_high |= (uint16_t)stp_config->sram_addr;
		break;
	case SRAM_TARGET_STP_SCALAR_RAM:
		ret = validate_stp_scalar_sram_transfer(pb, channel, stp,
				dram_config, stp_config);
		if (ret < 0)
			return ret;

		transfer->chan_img_pos_high = DMA_CHAN_LB_START_Y_STP_DRAM;
		transfer->chan_img_pos_high <<= DMA_CHAN_LB_START_Y_SHIFT;
		transfer->chan_img_pos_high |= (uint16_t)stp_config->sram_addr;
		break;
	case SRAM_TARGET_STP_VECTOR_RAM:
		/* TODO(ahampson):  Add parameter checks for vector b/30969166 */

		transfer->chan_img_pos_high = stp_config->include_halo ?
				DMA_CHAN_LB_START_Y_STP_ARRAY_32x32 :
				DMA_CHAN_LB_START_Y_STP_ARRAY_16x16;

		transfer->chan_img_pos_high <<= DMA_CHAN_LB_START_Y_SHIFT;
		transfer->chan_img_pos_high |= (uint16_t)stp_config->sram_addr;
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

	if (config->src.dram.len_bytes > DMA_MAX_IMG_TRANSFER_LEN)
		return -ERANGE;

	if (!access_ok(VERIFY_READ, config->src.dram.host_vaddr,
			config->src.dram.len_bytes))
		return -EFAULT;

	ret = ipu_dma_attach_buffer(pb, transfer, &config->src.dram,
			DMA_TO_DEVICE);
	if (ret < 0)
		return ret;

	set_dma_channel_mode(transfer, DMA_CHAN_SRC_DRAM, DMA_CHAN_DST_STP,
			false);

	ret = set_dma_stp_parameters(pb, session, channel, transfer,
			&config->src.dram, &config->dst.stp);
	if (ret < 0)
		goto err_exit;

	ret = set_dma_image_parameters(pb, channel, transfer, &config->img);
	if (ret < 0)
		goto err_exit;

	/* TODO(ahampson):  Not supported in the simulator (b/28197242).  Once
	 * simulator support is added this code will need to be completed.
	 * Kernel support for this transfer mode is tracked in b/28341158.
	 */
	ret = set_dma_transfer_region_parameters(pb, channel, transfer, config);
	if (ret < 0)
		goto err_exit;

	return 0;

err_exit:
	ipu_dma_release_buffer(pb, transfer);

	return ret;
}
