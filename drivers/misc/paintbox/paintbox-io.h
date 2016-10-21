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
#include "paintbox-regs-supplemental.h"


/* Size of the debug buffer used for debugfs or verbose logging.  These values
 * should be reevaluated whenever the dump_*_registers functions are changed.
 */
#define IO_AXI_DEBUG_BUFFER_SIZE (IO_AXI_NUM_REGS * REG_DEBUG_BUFFER_SIZE)
#define IO_APB_DEBUG_BUFFER_SIZE (IO_APB_NUM_REGS * REG_DEBUG_BUFFER_SIZE)

void io_enable_dma_channel_interrupt(struct paintbox_data *pb,
		unsigned int channel_id);
void io_disable_dma_channel_interrupt(struct paintbox_data *pb,
		unsigned int channel_id);
void io_enable_stp_interrupt(struct paintbox_data *pb, unsigned int stp_id);
void io_disable_stp_interrupt(struct paintbox_data *pb, unsigned int stp_id);
void io_enable_mipi_input_interface_interrupt(struct paintbox_data *pb,
		unsigned int interface_id);
void io_disable_mipi_input_interface_interrupt(struct paintbox_data *pb,
		unsigned int interface_id);
void io_enable_mipi_output_interface_interrupt(struct paintbox_data *pb,
		unsigned int interface_id);
void io_disable_mipi_output_interface_interrupt(struct paintbox_data *pb,
		unsigned int interface_id);
void io_enable_dma_interrupts(struct paintbox_data *pb);
void io_disable_dma_interrupts(struct paintbox_data *pb);
void io_enable_mipi_interrupts(struct paintbox_data *pb);
void io_disable_mipi_interrupts(struct paintbox_data *pb);


/* The caller to these functions must hold pb->lock */
void io_enable_dma_channel(struct paintbox_data *pb, unsigned int channel_id);
void io_disable_dma_channel(struct paintbox_data *pb, unsigned int channel_id);

int paintbox_io_axi_init(struct paintbox_data *pb);
int paintbox_io_apb_init(struct paintbox_data *pb);

#if defined(CONFIG_DEBUG_FS) || defined(VERBOSE_DEBUG)
int dump_io_axi_registers(struct paintbox_debug *debug, char *buf, size_t len);
int dump_io_apb_registers(struct paintbox_debug *debug, char *buf, size_t len);
#endif

#endif /* __PAINTBOX_IO_H__ */
