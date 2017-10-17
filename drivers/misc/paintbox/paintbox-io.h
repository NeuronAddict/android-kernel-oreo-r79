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
#include "paintbox-stp.h"

#if CONFIG_PAINTBOX_VERSION_MAJOR == 0
bool get_mipi_input_interface_interrupt_state(struct paintbox_data *pb,
		unsigned int interface_id);
bool get_mipi_output_interface_interrupt_state(struct paintbox_data *pb,
		unsigned int interface_id);
#endif

void paintbox_io_enable_interrupt(struct paintbox_data *pb,
		uint64_t enable_mask);
void paintbox_io_disable_interrupt(struct paintbox_data *pb,
		uint64_t disable_mask);

static inline void paintbox_enable_dma_channel_interrupt(
		struct paintbox_data *pb, unsigned int channel_id)
{
	paintbox_io_enable_interrupt(pb, 1ULL << channel_id);
}

static inline void paintbox_disable_dma_channel_interrupt(
		struct paintbox_data *pb, unsigned int channel_id)
{
	paintbox_io_disable_interrupt(pb, 1ULL << channel_id);
}

#if CONFIG_PAINTBOX_VERSION_MAJOR >= 1
void paintbox_enable_dma_channel_error_interrupt(struct paintbox_data *pb,
		unsigned int channel_id);

void paintbox_disable_dma_channel_error_interrupt(struct paintbox_data *pb,
		unsigned int channel_id);
#endif

static inline void paintbox_enable_stp_interrupt(struct paintbox_data *pb,
		unsigned int stp_id)
{
	paintbox_io_enable_interrupt(pb, 1ULL << (stp_id_to_index(stp_id) +
			IPU_IMR_STP_INTR_SHIFT));
}

static inline void paintbox_disable_stp_interrupt(struct paintbox_data *pb,
		unsigned int stp_id)
{
	paintbox_io_disable_interrupt(pb, 1ULL << (stp_id_to_index(stp_id) +
			IPU_IMR_STP_INTR_SHIFT));
}

#if CONFIG_PAINTBOX_VERSION_MAJOR >= 1
void paintbox_enable_stp_error_interrupt(struct paintbox_data *pb,
		unsigned int stp_id);

void paintbox_disable_stp_error_interrupt(struct paintbox_data *pb,
		unsigned int stp_id);
#endif

static inline void paintbox_enable_bif_interrupt(struct paintbox_data *pb)
{
	paintbox_io_enable_interrupt(pb, IPU_IMR_BIF_INTR_MASK);
}

static inline void paintbox_disable_bif_interrupt(struct paintbox_data *pb)
{
	paintbox_io_disable_interrupt(pb, IPU_IMR_BIF_INTR_MASK);
}

static inline void paintbox_enable_mmu_interrupt(struct paintbox_data *pb)
{
	paintbox_io_enable_interrupt(pb, IPU_IMR_MMU_INTR_MASK);
}

static inline void paintbox_disable_mmu_interrupt(struct paintbox_data *pb)
{
	paintbox_io_disable_interrupt(pb, IPU_IMR_MMU_INTR_MASK);
}

static inline void paintbox_enable_mipi_input_interface_interrupt(
		struct paintbox_data *pb, unsigned int interface_id)
{
	paintbox_io_enable_interrupt(pb, 1ULL << (interface_id +
			IPU_IMR_MPI_INTR_SHIFT));
}

static inline void paintbox_disable_mipi_input_interface_interrupt(
		struct paintbox_data *pb, unsigned int interface_id)
{
	paintbox_io_disable_interrupt(pb, 1ULL << (interface_id +
			IPU_IMR_MPI_INTR_SHIFT));
}

#if CONFIG_PAINTBOX_VERSION_MAJOR >= 1
void paintbox_enable_mipi_input_interface_error_interrupt(
		struct paintbox_data *pb, unsigned int interface_id);

void paintbox_disable_mipi_input_interface_error_interrupt(
		struct paintbox_data *pb, unsigned int interface_id);
#endif

static inline void paintbox_enable_mipi_output_interface_interrupt(
		struct paintbox_data *pb, unsigned int interface_id)
{
	paintbox_io_enable_interrupt(pb, 1ULL << (interface_id +
			IPU_IMR_MPO_INTR_SHIFT));
}

static inline void paintbox_disable_mipi_output_interface_interrupt(
		struct paintbox_data *pb, unsigned int interface_id)
{
	paintbox_io_disable_interrupt(pb, 1ULL << (interface_id +
			IPU_IMR_MPO_INTR_SHIFT));
}

void paintbox_io_apb_post_ipu_reset(struct paintbox_data *pb);

int paintbox_io_apb_init(struct paintbox_data *pb);

/* All sessions must be released before remove can be called. */
void paintbox_io_apb_remove(struct paintbox_data *pb);

#ifdef CONFIG_PAINTBOX_DEBUG
int paintbox_dump_io_apb_registers(struct paintbox_debug *debug, char *buf,
		size_t len);
#endif

#endif /* __PAINTBOX_IO_H__ */
