/*
 * Paintbox V0 specific MIPI Support for Paintbox programmable IPU
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

#ifndef __PAINTBOX_MIPI_V0_H__
#define __PAINTBOX_MIPI_V0_H__

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <uapi/paintbox.h>

#include "paintbox-dma.h"
#include "paintbox-debug.h"
#include "paintbox-io.h"
#include "paintbox-irq.h"
#include "paintbox-mipi.h"
#include "paintbox-mipi-debug.h"
#include "paintbox-regs.h"

#define MIPI_INVALID_FRAME_NUMBER -1

#define MIPI_INPUT_SOF_IMR	MPI_STRM_CTRL_SOF_IMR_MASK
#define MIPI_INPUT_SOF_ISR	MPI_STRM_CTRL_SOF_ISR_MASK
#define MIPI_INPUT_OVF_IMR	MPI_STRM_CTRL_OVF_IMR_MASK
#define MIPI_INPUT_OVF_ISR	MPI_STRM_CTRL_OVF_ISR_MASK

#define MIPI_OUTPUT_EOF_IMR	MPO_STRM_CTRL_EOF_IMR_MASK
#define MIPI_OUTPUT_EOF_ISR	MPO_STRM_CTRL_EOF_ISR_MASK

/* TODO(ahampson):  The mapping here below applies to the V0 IPU but may not be
 * the same in future versions of the hardware.  This mapping should be moved
 * into the device tree.  b/32283059
 *
 * MIPI input streams 0..11 have a fixed mapping to DMA channels 0..11.
 * MIPI output streams 0..1 have a fixed mapping to DMA channels 14..15.
 */
#define MIPI_INPUT_DMA_CHANNEL_ID_START  0
#define MIPI_INPUT_DMA_CHANNEL_ID_END    11
#define MIPI_OUTPUT_DMA_CHANNEL_ID_START 14
#define MIPI_OUTPUT_DMA_CHANNEL_ID_END   15

static inline unsigned int mipi_stream_to_dma_channel_id(
		struct paintbox_mipi_stream *stream)
{
	return stream->is_input ? stream->stream_id +
			MIPI_INPUT_DMA_CHANNEL_ID_START:
			stream->stream_id + MIPI_OUTPUT_DMA_CHANNEL_ID_START;
}

/* Mitigation for hardware bug b/32640344
 *
 * On the V0 IPU, the MPI_STRM_CTRL register contains bits that are set by the
 * hardware, notably the ISR bits and the FRAME_NUM field.  During a read,
 * modify, write operation by the CPU it is possible to clear these bits without
 * knowing that they have been set.  This could cause the driver to miss an
 * interrupt.  It is possible to determine that an interrupt had occurred on the
 * MIPI interface to which the stream belongs using the IPU_ISR register.  If
 * there is a change in the interface bit for the IPU_ISR around a read, modify,
 * write operation then a stream interrupt may have been missed.
 *
 * This function returns -EINTR if an interrupt occurred while doing a read,
 * modify, write of the MPI_STRM_CTRL register.
 *
 * The caller to these functions must hold pb->io_ipu.mipi_lock.
 */
int mipi_input_set_control(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream, uint32_t ctrl_set_mask);

int mipi_input_clear_control(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream, uint32_t ctrl_clear_mask);

/* Mitigation for hardware bug b/32640344
 *
 * On the V0 IPU, the MPO_STRM_CTRL register contains an EOF ISR bit that is set
 * by the hardware.  During a read, modify, write operation by the CPU it is
 * possible to clear these bit without knowing that it has been set.
 * This could cause the driver to miss an EOF interrupt.  On the V0 IPU, there
 * is only one stream per MIPI output interface.  If the interface bit is set in
 * the IPU_ISR register but the ISR bit is not set in the MPO_STRM_CTRL
 * interrupt then it can be determined that an interrupt was missed.
 *
 * This function returns -EINTR if an interrupt occurred while doing a read,
 * modify, write of the MPO_STRM_CTRL register.
 *
 * The caller to these functions must hold pb->io_ipu.mipi_lock.
 */
int mipi_output_clear_and_set_control(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream, uint32_t ctrl_clear_mask,
		uint32_t ctrl_set_mask);

static inline int mipi_output_set_control(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream, uint32_t ctrl_set_mask)
{
	return mipi_output_clear_and_set_control(pb, stream, 0, ctrl_set_mask);
}

static inline int mipi_output_clear_control(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream, uint32_t ctrl_clear_mask)
{
	return mipi_output_clear_and_set_control(pb, stream, ctrl_clear_mask,
			0);
}

/* The caller to the mipi_{in,out}put_(ack|{en,dis}able)_xxx functions must hold
 * pb->io_ipu.mipi_lock.
 */
static inline int mipi_input_enable_stream(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream)
{
	paintbox_mipi_select_input_stream(pb, stream->stream_id);

	return mipi_input_set_control(pb, stream, MPI_STRM_CTRL_EN_MASK);
}

static inline int mipi_input_enable_irqs_and_stream(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream, uint32_t imr_mask)
{
	paintbox_mipi_select_input_stream(pb, stream->stream_id);

	return mipi_input_set_control(pb, stream,
			imr_mask | MPI_STRM_CTRL_EN_MASK);
}

static inline int mipi_input_disable_stream(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream)
{
	paintbox_mipi_select_input_stream(pb, stream->stream_id);

	return mipi_input_clear_control(pb, stream, MPI_STRM_CTRL_EN_MASK);
}

static inline int mipi_input_disable_irqs_and_stream(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream, uint32_t imr_mask)
{
	paintbox_mipi_select_input_stream(pb, stream->stream_id);

	return mipi_input_clear_control(pb, stream,
			imr_mask | MPI_STRM_CTRL_EN_MASK);
}

static inline int mipi_output_ack_irqs(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream, uint32_t isr_mask)
{
	paintbox_mipi_select_output_stream(pb, stream->stream_id);

	return mipi_output_clear_control(pb, stream, isr_mask);
}

static inline int mipi_output_enable_stream(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream)
{
	paintbox_mipi_select_output_stream(pb, stream->stream_id);

	return mipi_output_set_control(pb, stream, MPO_STRM_CTRL_EN_MASK);
}

static inline int mipi_output_enable_irqs_and_stream(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream, uint32_t imr_mask,
		bool row_sync)
{
	uint32_t ctrl_clear_mask = 0;
	uint32_t ctrl_set_mask = imr_mask | MPO_STRM_CTRL_EN_MASK;

	paintbox_mipi_select_output_stream(pb, stream->stream_id);

	/* {set,clear} RSYNC_EN in case previously {cleared,set} */
	if (row_sync)
		ctrl_set_mask |= MPO_STRM_CTRL_RSYNC_EN_MASK;
	else
		ctrl_clear_mask |= MPO_STRM_CTRL_RSYNC_EN_MASK;

	return mipi_output_clear_and_set_control(pb, stream, ctrl_clear_mask,
			ctrl_set_mask);
}

static inline int mipi_output_disable_stream(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream)
{
	paintbox_mipi_select_output_stream(pb, stream->stream_id);

	return mipi_output_clear_control(pb, stream, MPO_STRM_CTRL_EN_MASK);
}

static inline int mipi_output_disable_irqs(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream, uint32_t imr_mask)
{
	paintbox_mipi_select_output_stream(pb, stream->stream_id);

	return mipi_output_clear_control(pb, stream, imr_mask);
}

#endif /* __PAINTBOX_MIPI_V0_H__ */
