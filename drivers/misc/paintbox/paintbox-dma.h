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

#ifndef __PAINTBOX_DMA_H__
#define __PAINTBOX_DMA_H__

#include <linux/interrupt.h>
#include <linux/types.h>

#include "paintbox-common.h"


#define DMA_DEBUG_BUFFER_SIZE (DMA_NUM_REGS * REG_DEBUG_BUFFER_SIZE)

/* DMA Ioctl Handlers */
int allocate_dma_channel_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);

int release_dma_channel_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);

int bind_dma_interrupt_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);

int unbind_dma_interrupt_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);

int setup_dma_transfer_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);

int read_dma_transfer_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);

int start_dma_transfer_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);

/* The caller to these function must hold pb->lock */
int validate_dma_channel(struct paintbox_data *pb,
		struct paintbox_session *session, uint8_t channel_id);

struct paintbox_dma_channel *get_dma_channel(struct paintbox_data *pb,
		struct paintbox_session *session, uint8_t channel_id, int *err);

int unbind_dma_interrupt(struct paintbox_data *pb,
		struct paintbox_session *session, uint8_t interrupt_id,
		uint8_t channel_id);

int release_dma_channel(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_dma_channel *dma);

int dma_bind_interrupt(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel, struct paintbox_irq *irq);

int dma_unbind_interrupt(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel, struct paintbox_irq *irq);

int dma_setup_dram_to_lbp_transfer(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer,
		struct dma_transfer_config *config);

int dma_setup_dram_to_stp_transfer(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer,
		struct dma_transfer_config *config);

int dma_setup_lbp_to_dram_transfer(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer,
		struct dma_transfer_config *config);

int dma_setup_stp_to_dram_transfer(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer,
		struct dma_transfer_config *config);

int dma_setup_mipi_to_lbp_transfer(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer,
		struct dma_transfer_config *config);

int dma_start_transfer(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel);

int dma_stop_transfer(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel);

void dma_report_completion(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel, int err);

irqreturn_t paintbox_dma_interrupt(struct paintbox_data *pb,
		uint32_t channel_mask);

int paintbox_dma_init(struct paintbox_data *pb);

#if defined(CONFIG_DEBUG_FS) || defined(VERBOSE_DEBUG)
int dump_dma_registers(struct paintbox_data *pb, char *buf, size_t len);
int dump_dma_channel_registers(struct paintbox_data *pb,
		unsigned int channel_id, char *buf, size_t len);
#endif

#endif  /* __PAINTBOX_DMA_H__ */
