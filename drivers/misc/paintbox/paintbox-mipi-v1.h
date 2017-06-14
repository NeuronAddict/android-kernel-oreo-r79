/*
 * Paintbox V1 specific MIPI support for Paintbox programmable IPU
 *
 * Copyright (C) 2017 Google, Inc.
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

#ifndef __PAINTBOX_MIPI_V1_H__
#define __PAINTBOX_MIPI_V1_H__

#include <linux/io.h>

#include "paintbox-common.h"
#include "paintbox-mipi.h"
#include "paintbox-regs.h"

#define MIPI_INVALID_FRAME_NUMBER -1

#define MIPI_INPUT_SOF_IMR	(1 << 0)
#define MIPI_INPUT_SOF_ISR	(1 << 0)
#define MIPI_INPUT_OVF_IMR	(1 << 1)
#define MIPI_INPUT_OVF_ISR	(1 << 1)

#define MIPI_OUTPUT_EOF_IMR	(1 << 0)
#define MIPI_OUTPUT_EOF_ISR	(1 << 0)

/* MIPI input streams 0..11 have a fixed mapping to DMA channels 0..11.
 * MIPI output streams 0..3 and 4..7 share fixed mappings to DMA channels
 * 12..15.
 */
#define MIPI_INPUT_DMA_CHANNEL_ID_START  0
#define MIPI_INPUT_DMA_CHANNEL_ID_END    11
#define MIPI_OUTPUT_DMA_CHANNEL_ID_START 12
#define MIPI_OUTPUT_DMA_CHANNEL_ID_END   15

static inline unsigned int mipi_stream_to_dma_channel_id(
		struct paintbox_mipi_stream *stream)
{
	return stream->is_input
		? MIPI_INPUT_DMA_CHANNEL_ID_START + stream->stream_id
		: MIPI_OUTPUT_DMA_CHANNEL_ID_START + (stream->stream_id % 4);
}

/* The caller to this function must hold pb->io_ipu.mipi_lock. */
static inline int mipi_output_clear_and_set_control(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream, uint32_t ctrl_clear_mask,
		uint32_t ctrl_set_mask)
{
	uint32_t ctrl;

	ctrl = readl(pb->io_ipu.ipu_base + MPO_STRM_CTRL);
	ctrl &= ~ctrl_clear_mask;
	ctrl |= ctrl_set_mask;
	writel(ctrl, pb->io_ipu.ipu_base + MPO_STRM_CTRL);

	return 0;
}

/* The caller to the mipi_{in,out}put_(ack|{en,dis}able)_xxx functions must hold
 * pb->io_ipu.mipi_lock.
 */
static inline int mipi_input_enable_stream(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream)
{
	writeq((MPI_CTRL_STRM_CONTINUOUS_SET0_MASK |
			MPI_CTRL_STRM_ENA_SET0_MASK) << stream->stream_id,
			pb->io_ipu.ipu_base + MPI_CTRL);

	return 0;
}

static inline void mipi_input_set_imr(struct paintbox_data *pb,
		uint32_t set_mask)
{
	uint32_t val;

	val = readl(pb->io_ipu.ipu_base + MPI_IMR);
	val |= set_mask;
	writel(val, pb->io_ipu.ipu_base + MPI_IMR);

	/* With MIPI we enable and disable at the IER, as well as the IMR. */
	val = readl(pb->io_ipu.ipu_base + MPI_IER);
	val |= set_mask;
	writel(val, pb->io_ipu.ipu_base + MPI_IER);
}

static inline void mipi_input_set_err_imr(struct paintbox_data *pb,
		uint32_t set_mask)
{
	uint32_t val;

	val = readl(pb->io_ipu.ipu_base + MPI_ERR_IMR);
	val |= set_mask;
	writel(val, pb->io_ipu.ipu_base + MPI_ERR_IMR);

	/* With MIPI we enable and disable at the IER, as well as the IMR. */
	val = readl(pb->io_ipu.ipu_base + MPI_ERR_IER);
	val |= set_mask;
	writel(val, pb->io_ipu.ipu_base + MPI_ERR_IER);
}

static inline void mipi_input_clear_imr(struct paintbox_data *pb,
		uint32_t clear_mask)
{
	uint32_t val;

	/* With MIPI we enable and disable at the IER, as well as the IMR. */
	val = readl(pb->io_ipu.ipu_base + MPI_IER);
	val &= ~clear_mask;
	writel(val, pb->io_ipu.ipu_base + MPI_IER);

	val = readl(pb->io_ipu.ipu_base + MPI_IMR);
	val &= ~clear_mask;
	writel(val, pb->io_ipu.ipu_base + MPI_IMR);
}

static inline void mipi_input_clear_err_imr(struct paintbox_data *pb,
		uint32_t clear_mask)
{
	uint32_t val;

	/* With MIPI we enable and disable at the IER, as well as the IMR. */
	val = readl(pb->io_ipu.ipu_base + MPI_ERR_IER);
	val &= ~clear_mask;
	writel(val, pb->io_ipu.ipu_base + MPI_ERR_IER);

	val = readl(pb->io_ipu.ipu_base + MPI_ERR_IMR);
	val &= ~clear_mask;
	writel(val, pb->io_ipu.ipu_base + MPI_ERR_IMR);
}

static inline void mipi_output_set_imr(struct paintbox_data *pb,
		uint32_t set_mask)
{
	uint32_t val;

	val = readl(pb->io_ipu.ipu_base + MPO_IMR);
	val |= set_mask;
	writel(val, pb->io_ipu.ipu_base + MPO_IMR);

	/* With MIPI we enable and disable at the IER, as well as the IMR. */
	val = readl(pb->io_ipu.ipu_base + MPO_IER);
	val |= set_mask;
	writel(val, pb->io_ipu.ipu_base + MPO_IER);
}

static inline void mipi_output_clear_imr(struct paintbox_data *pb,
		uint32_t clear_mask)
{
	uint32_t val;

	/* With MIPI we enable and disable at the IER, as well as the IMR. */
	val = readl(pb->io_ipu.ipu_base + MPO_IER);
	val &= ~clear_mask;
	writel(val, pb->io_ipu.ipu_base + MPO_IER);

	val = readl(pb->io_ipu.ipu_base + MPO_IMR);
	val &= ~clear_mask;
	writel(val, pb->io_ipu.ipu_base + MPO_IMR);
}

static inline int mipi_input_enable_irqs_and_stream(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream, uint32_t imr_mask)
{
	if (imr_mask & MIPI_INPUT_SOF_IMR)
		mipi_input_set_imr(pb, 1 << (stream->stream_id +
				MPI_IMR_SOF0_SHIFT));

	if (imr_mask & MIPI_INPUT_OVF_IMR)
		mipi_input_set_err_imr(pb, 1 << (stream->stream_id +
				MPI_ERR_IMR_OVF0_SHIFT));

	mipi_input_enable_stream(pb, stream);

	return 0;
}

static inline int mipi_input_disable_stream(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream)
{
	writeq(MPI_CTRL_STRM_CONTINUOUS_CLR0_MASK << stream->stream_id,
			pb->io_ipu.ipu_base + MPI_CTRL);

	return 0;
}

static inline int mipi_input_disable_irqs_and_stream(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream, uint32_t imr_mask)
{
	if (imr_mask & MIPI_INPUT_SOF_IMR)
		mipi_input_clear_imr(pb, 1 << (stream->stream_id +
				MPI_IMR_SOF0_SHIFT));

	if (imr_mask & MIPI_INPUT_OVF_IMR)
		mipi_input_clear_err_imr(pb, 1 << (stream->stream_id +
				MPI_ERR_IMR_OVF0_SHIFT));

	mipi_input_disable_stream(pb, stream);

	return 0;
}

static inline int mipi_output_ack_irqs(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream, uint32_t isr_mask)
{
	if (isr_mask & MIPI_OUTPUT_EOF_ISR)
		writel(1 << (stream->stream_id + MPO_ISR_EOF0_SHIFT),
				pb->io_ipu.ipu_base + MPO_ISR);

	return 0;
}

static inline int mipi_output_enable_stream(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream)
{
	writeq((MPO_CTRL_STRM_CONTINUOUS_SET0_MASK |
			MPO_CTRL_STRM_ENA_SET0_MASK) << stream->stream_id,
			pb->io_ipu.ipu_base + MPO_CTRL);

	return 0;
}

static inline int mipi_output_enable_irqs_and_stream(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream, uint32_t imr_mask,
		bool row_sync)
{
	uint32_t val;

	if (imr_mask & MIPI_OUTPUT_EOF_IMR) {
		val = readl(pb->io_ipu.ipu_base + MPO_IMR);
		val |= 1 << (stream->stream_id + MPO_IMR_EOF0_SHIFT);
		writel(val, pb->io_ipu.ipu_base + MPO_IMR);
	}

	paintbox_mipi_select_output_stream(pb, stream->stream_id);

	/* {set,clear} RSYNC_EN in case previously {cleared,set} */
	mipi_output_clear_and_set_control(pb, stream,
			row_sync ? 0 : MPO_STRM_CTRL_RSYNC_EN_MASK,
			row_sync ? MPO_STRM_CTRL_RSYNC_EN_MASK : 0);

	mipi_output_enable_stream(pb, stream);

	return 0;
}

static inline int mipi_output_disable_stream(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream)
{
	writeq(MPO_CTRL_STRM_CONTINUOUS_CLR0_MASK << stream->stream_id,
			pb->io_ipu.ipu_base + MPO_CTRL);

	return 0;
}

static inline int mipi_output_disable_irqs(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream, uint32_t imr_mask)
{
	uint32_t val;

	if (imr_mask & MIPI_OUTPUT_EOF_IMR) {
		val = readl(pb->io_ipu.ipu_base + MPO_IMR);
		val &= ~(1 << (stream->stream_id + MPO_IMR_EOF0_SHIFT));
		writel(val, pb->io_ipu.ipu_base + MPO_IMR);
	}

	return 0;
}

#endif /* __PAINTBOX_MIPI_V1_H__ */
