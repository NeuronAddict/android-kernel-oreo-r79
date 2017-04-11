/*
 * IO support for the Paintbox programmable IPU
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

#ifndef __PAINTBOX_IO_H__
#define __PAINTBOX_IO_H__

#include <linux/io.h>

#include "paintbox-common.h"
#include "paintbox-regs.h"

void io_enable_dma_channel_interrupt(struct paintbox_data *pb,
		unsigned int channel_id);
void io_disable_dma_channel_interrupt(struct paintbox_data *pb,
		unsigned int channel_id);
void io_enable_stp_interrupt(struct paintbox_data *pb, unsigned int stp_id);
void io_disable_stp_interrupt(struct paintbox_data *pb, unsigned int stp_id);

bool get_mipi_input_interface_interrupt_state(struct paintbox_data *pb,
		unsigned int interface_id);
bool get_mipi_output_interface_interrupt_state(struct paintbox_data *pb,
		unsigned int interface_id);
void io_enable_mipi_input_interface_interrupt(struct paintbox_data *pb,
		unsigned int interface_id);
void io_disable_mipi_input_interface_interrupt(struct paintbox_data *pb,
		unsigned int interface_id);
void io_enable_mipi_output_interface_interrupt(struct paintbox_data *pb,
		unsigned int interface_id);
void io_disable_mipi_output_interface_interrupt(struct paintbox_data *pb,
		unsigned int interface_id);
void paintbox_enable_mmu_interrupt(struct paintbox_data *pb);
void paintbox_disable_mmu_interrupt(struct paintbox_data *pb);
void paintbox_enable_bif_interrupt(struct paintbox_data *pb);
void paintbox_disable_bif_interrupt(struct paintbox_data *pb);

/* The caller to these functions must hold pb->lock */
void io_enable_dma_channel(struct paintbox_data *pb, unsigned int channel_id);
void io_disable_dma_channel(struct paintbox_data *pb, unsigned int channel_id);

int paintbox_io_apb_init(struct paintbox_data *pb);

#ifdef CONFIG_PAINTBOX_DEBUG
int paintbox_dump_io_apb_registers(struct paintbox_debug *debug, char *buf,
		size_t len);
#endif

#endif /* __PAINTBOX_IO_H__ */
