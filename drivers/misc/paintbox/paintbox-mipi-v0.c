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

#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <uapi/paintbox.h>

#include "paintbox-dma.h"
#include "paintbox-io.h"
#include "paintbox-mipi.h"
#include "paintbox-mipi-debug.h"
#include "paintbox-mipi-v0.h"
#include "paintbox-regs.h"

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
 * This function is used to determine if an interrupt has occurred on any of the
 * streams that are part of MIPI interface.  On the V0 IPU, each input interface
 * has four streams associated with it.  If more than one stream has an
 * interrupt then it will not be possible to determine the source of the
 * interrupt if the ISR bits of one stream have been inadvertently cleared.
 *
 * The caller to this function must hold pb->lock and pb->io_ipu.mipi_lock.
 */
static bool mipi_input_interface_has_interrupt(struct paintbox_data *pb,
		struct paintbox_mipi_interface *interface)
{
	unsigned int old_stream_id, stream_id;
	bool interrupt_occurred = false;

	old_stream_id = pb->io_ipu.selected_input_stream_id;

	for (stream_id = 0; stream_id < interface->num_streams; stream_id++) {
		struct paintbox_mipi_stream *stream =
				interface->streams[stream_id];
		uint32_t ctrl;

		/* If the stream is not allocated then skip it.  */
		if (!stream->allocated)
			continue;

		paintbox_mipi_select_input_stream(pb, stream->stream_id);

		ctrl = readl(pb->io_ipu.ipu_base + MPI_STRM_CTRL);
		if (ctrl & (MPI_STRM_CTRL_SOF_ISR_MASK |
				MPI_STRM_CTRL_OVF_ISR_MASK)) {
			interrupt_occurred = true;
			break;
		}
	}

	paintbox_mipi_select_input_stream(pb, old_stream_id);

	return interrupt_occurred;
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
 * This function is used to determine the type of stream interrupt, SOF, or
 * OVF (overflow), that occurred on the stream based on the stream state.  If it
 * is determined that an ISR bit was inadvertently cleared then a flag will be
 * set that will be cleared in the interrupt handler when the interrupt handler
 * fires.
 *
 * The caller to this function must hold pb->lock and pb->io_ipu.mipi_lock.
 */
static int evaluate_missing_input_interrupt(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream)
{
	uint32_t ctrl = readl(pb->io_ipu.ipu_base + MPI_STRM_CTRL);

	/* If both the SOF and OVF interrupts have been reported then there is
	 * nothing to do here, everthing can be handled normally in the
	 * interrupt handler.
	 */
	if (ctrl & (MPI_STRM_CTRL_SOF_ISR_MASK | MPI_STRM_CTRL_OVF_ISR_MASK)) {
		return 0;

	/* If only the OVF interrupt has occurred then try to determine if the
	 * SOF interrupt was lost.
	 */
	} else if (ctrl & MPI_STRM_CTRL_OVF_ISR_MASK) {
		/* The OVF interrupt occurred but there is not frame currently
		 * being processed then we likely missed the SOF.
		 */
		if (!stream->input.frame_in_progress) {
			stream->input.missed_sof_interrupt = true;
			return -EINTR;
		}

		/* If there is a frame currently being processed and an OVF
		 * interrupt occurred and is reported in the ISR then there is
		 * nothing to do here, everything can be handled in the
		 * interrupt handler.
		 */
		return 0;

	/* If an SOF interrupt has been reported but no OVF interrupt then
	 * check to make sure an OVF interrupt wasn't lost.
	 */
	} else if (ctrl & MPI_STRM_CTRL_SOF_ISR_MASK) {
		/* If the cleanup bit is set but the stream->cleanup_in_progress
		 * flag is not then we might have missed an OVF interrupt.
		 */
		if ((ctrl & MPI_STRM_CTRL_CLEANUP_MASK) &&
				!stream->cleanup_in_progress) {
			stream->input.missed_ovf_interrupt = true;
			return -EINTR;
		}

		/* If an SOF interrupt has been reported and there is no
		 * cleanup in progress then there is nothing to do here.
		 *
		 * Note: It is possible that the cleanup operation has already
		 * concluded, however there is no way to detect that condition.
		 */
		return 0;

	/* If neither the SOF nor the OVF ISR bits are set then the interrupt
	 * was either for another stream or the ISR bits were cleared by the
	 * read, modify, write.
	 */
	} else {
		/* If the cleanup bit is set but the stream->cleanup_in_progress
		 * flag is not then we might have missed an OVF interrupt.
		 */
		if ((ctrl & MPI_STRM_CTRL_CLEANUP_MASK) &&
				!stream->cleanup_in_progress) {
			stream->input.missed_ovf_interrupt = true;

			/* If there is no active frame being processed then we
			 * have also missed the SOF.
			 */
			if (!stream->input.frame_in_progress)
				stream->input.missed_sof_interrupt = true;

			return -EINTR;
		}

		/* If there is no cleanup in progress then check to see if any
		 * other streams in the interface have interrupts pending.  If
		 * another stream has an interrupt then there is nothing we can
		 * do.
		 */
		if (mipi_input_interface_has_interrupt(pb, stream->interface))
			return 0;

		/* If none of the other streams have interrupts pending
		 * then check to see if this stream is actively processing a
		 * frame or not.
		 */
		if (stream->input.frame_in_progress)
			stream->input.missed_ovf_interrupt = true;
		else
			stream->input.missed_sof_interrupt = true;

		return -EINTR;
	}
}

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
 * This function is used to determine if an EOF interrupt was missed.  If it is
 * determined that the EOF ISR bit was inadvertently cleared then a flag will be
 * set that will be cleared in the interrupt handler when the interrupt handler
 * fires.
 *
 * The caller to this function must hold pb->lock and pb->io_ipu.mipi_lock.
 */
static int evaluate_missing_output_interrupt(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream)
{
	uint32_t ctrl = readl(pb->io_ipu.ipu_base + MPO_STRM_CTRL);

	if (!(ctrl & MPO_STRM_CTRL_EOF_ISR_MASK)) {
		stream->output.missed_eof_interrupt = true;
		return -EINTR;
	}

	return 0;
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
 * The caller to this function must hold pb->io_ipu.mipi_lock.
 */
int mipi_input_set_control(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream, uint32_t ctrl_set_mask)
{
	unsigned int interface_id = stream->interface->interface_id;
	bool start_interrupt_state, end_interrupt_state;
	uint32_t ctrl;

	start_interrupt_state = get_mipi_input_interface_interrupt_state(pb,
			interface_id);

	ctrl = readl(pb->io_ipu.ipu_base + MPI_STRM_CTRL);
	writel(ctrl | ctrl_set_mask, pb->io_ipu.ipu_base + MPI_STRM_CTRL);

	end_interrupt_state = get_mipi_input_interface_interrupt_state(pb,
			interface_id);

	/* An interface interrupt has occurred during the read, modify, write
	 * operation above.  Determine if a stream interrupt was missed.
	 */
	if (!start_interrupt_state && end_interrupt_state)
		return evaluate_missing_input_interrupt(pb, stream);

	return 0;
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
 * The caller to this function must hold pb->io_ipu.mipi_lock.
 */
int mipi_input_clear_control(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream, uint32_t ctrl_clear_mask)
{
	unsigned int interface_id = stream->interface->interface_id;
	bool start_interrupt_state, end_interrupt_state;
	uint32_t ctrl;

	start_interrupt_state = get_mipi_input_interface_interrupt_state(pb,
			interface_id);

	ctrl = readl(pb->io_ipu.ipu_base + MPI_STRM_CTRL);
	writel(ctrl & ~ctrl_clear_mask, pb->io_ipu.ipu_base + MPI_STRM_CTRL);

	end_interrupt_state = get_mipi_input_interface_interrupt_state(pb,
			interface_id);

	/* An interface interrupt has occurred during the read, modify, write
	 * operation above.  Determine if a stream interrupt was missed.
	 */
	if (!start_interrupt_state && end_interrupt_state)
		return evaluate_missing_input_interrupt(pb, stream);

	return 0;
}

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
 * The caller to this function must hold pb->io_ipu.mipi_lock.
 */
int mipi_output_clear_and_set_control(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream, uint32_t ctrl_clear_mask,
		uint32_t ctrl_set_mask)
{
	unsigned int interface_id = stream->interface->interface_id;
	bool start_interrupt_state, end_interrupt_state;
	uint32_t ctrl;

	start_interrupt_state = get_mipi_output_interface_interrupt_state(pb,
			interface_id);

	ctrl = readl(pb->io_ipu.ipu_base + MPO_STRM_CTRL);
	ctrl &= ~ctrl_clear_mask;
	ctrl |= ctrl_set_mask;
	writel(ctrl, pb->io_ipu.ipu_base + MPO_STRM_CTRL);

	end_interrupt_state = get_mipi_output_interface_interrupt_state(pb,
			interface_id);

	/* An interface interrupt has occurred during the read, modify, write
	 * operation above.  Determine if a stream interrupt was missed.
	 */
	if (!start_interrupt_state && end_interrupt_state)
		return evaluate_missing_output_interrupt(pb, stream);

	return 0;
}

/* On the V0 IPU it is also not possible to enable all streams at once, streams
 * will be enabled sequentially.
 *
 * TODO(ahampson):  This function does not verify that the streams being enabled
 * are all in the same state.
 */
void paintbox_mipi_enable_multiple_input(struct paintbox_data *pb,
		struct paintbox_session *session, uint32_t stream_id_mask,
		struct mipi_stream_enable_multiple *req)
{
	struct paintbox_mipi_stream *stream, *stream_next;
	unsigned long irq_flags;

	spin_lock_irqsave(&pb->io_ipu.mipi_lock, irq_flags);

	list_for_each_entry_safe(stream, stream_next,
			&session->mipi_input_list, session_entry) {
		int ret;

		if (!req->enable_all &&
				!(stream_id_mask & (1 << stream->stream_id)))
			continue;

		if (stream->enabled) {
			paintbox_mipi_update_stream_count(pb, stream,
					req->free_running, req->frame_count);
			continue;
		}

		paintbox_mipi_enable_input_stream_common(pb, stream,
				req->free_running, req->frame_count,
				req->input.disable_on_error);

		/* TODO(ahampson):  A future performance optimization would be
		 * to construct an array of ctrl register masks here and then
		 * write that all at once in a second loop.  This would reduce
		 * the amount of time between stream enables.
		 */
		ret = mipi_input_enable_irqs_and_stream(pb, stream,
			MIPI_INPUT_SOF_IMR | MIPI_INPUT_OVF_IMR);

		/* Log a missed interrupt if it occurred but don't propagate the
		 * error up.
		*/
		if (ret == -EINTR)
			dev_warn(&pb->pdev->dev, "%s: mipi input stream%u may have missed an interrupt\n",
				__func__, stream->stream_id);
	}

	spin_unlock_irqrestore(&pb->io_ipu.mipi_lock, irq_flags);
}

/* On the V0 IPU it is also not possible to enable all streams at once, streams
 * will be enabled sequentially.
 *
 * TODO(ahampson):  This function does not verify that the streams being enabled
 * are all in the same state.
 */
void paintbox_mipi_enable_multiple_output(struct paintbox_data *pb,
		struct paintbox_session *session, uint32_t stream_id_mask,
		struct mipi_stream_enable_multiple *req)
{
	struct paintbox_mipi_stream *stream, *stream_next;
	unsigned long irq_flags;

	spin_lock_irqsave(&pb->io_ipu.mipi_lock, irq_flags);

	list_for_each_entry_safe(stream, stream_next,
			&session->mipi_output_list, session_entry) {
		int ret;

		if (!req->enable_all &&
				!(stream_id_mask & (1 << stream->stream_id)))
			continue;

		if (stream->enabled) {
			paintbox_mipi_update_stream_count(pb, stream,
					req->free_running, req->frame_count);
			continue;
		}

		ret = enable_mipi_output_stream(pb, stream, req->free_running,
				req->frame_count, req->output.enable_row_sync);

		/* Log a missed interrupt if it occurred but don't propagate the
		 * error up.
		*/
		if (ret == -EINTR)
			dev_warn(&pb->pdev->dev, "%s: mipi output stream%u may have missed an interrupt\n",
				__func__, stream->stream_id);
	}

	spin_unlock_irqrestore(&pb->io_ipu.mipi_lock, irq_flags);
}

/* On the V0 IPU it is also not possible to disable all streams at once, streams
 * will be disabled sequentially.
 *
 * TODO(ahampson):  This function does not verify that the streams being
 * disables are all in the same state.
 */
void paintbox_mipi_disable_multiple_input(struct paintbox_data *pb,
		struct paintbox_session *session, uint32_t stream_id_mask,
		struct mipi_stream_disable_multiple *req)
{
	struct paintbox_mipi_stream *stream, *stream_next;
	unsigned long irq_flags;

	spin_lock_irqsave(&pb->io_ipu.mipi_lock, irq_flags);

	list_for_each_entry_safe(stream, stream_next, &session->mipi_input_list,
			session_entry) {
		int ret;

		if (!req->disable_all &&
				!(stream_id_mask & (1 << stream->stream_id)))
			continue;

		paintbox_mipi_disable_stream_common(pb, stream);

		/* Do not disable the overflow interrupt on disable. This
		 * interrupt reports on the state of the current frame and needs
		 * to be left enabled.
		 */
		ret = mipi_input_disable_irqs_and_stream(pb, stream,
				MIPI_INPUT_SOF_IMR);

		/* Log a missed interrupt if it occurred but don't propagate the
		 * error up.
		 */
		if (ret == -EINTR)
			dev_warn(&pb->pdev->dev,
					"%s: mipi output stream%u may have missed an interrupt\n",
					__func__, stream->stream_id);
	}

	spin_unlock_irqrestore(&pb->io_ipu.mipi_lock, irq_flags);
}

/* On the V0 IPU it is also not possible to disable all streams at once, streams
 * will be disabled sequentially.
 *
 * TODO(ahampson):  This function does not verify that the streams being
 * disables are all in the same state.
 */
void paintbox_mipi_disable_multiple_output(struct paintbox_data *pb,
		struct paintbox_session *session, uint32_t stream_id_mask,
		struct mipi_stream_disable_multiple *req)
{
	struct paintbox_mipi_stream *stream, *stream_next;
	unsigned long irq_flags;

	spin_lock_irqsave(&pb->io_ipu.mipi_lock, irq_flags);

	list_for_each_entry_safe(stream, stream_next,
			&session->mipi_output_list, session_entry) {
		int ret;

		if (!req->disable_all &&
				!(stream_id_mask & (1 << stream->stream_id)))
			continue;

		paintbox_mipi_disable_stream_common(pb, stream);

		/* Do not disable the EOF interrupt on disable. This interrupt
		 * reports on the state of the current frame and needs to be
		 * left enabled.
		 */
		ret = mipi_output_disable_stream(pb, stream);

		/* Log a missed interrupt if it occurred but don't propagate the
		 * error up.
		 */
		if (ret == -EINTR)
			dev_warn(&pb->pdev->dev,
					"%s: mipi output stream%u may have missed an interrupt\n",
					__func__, stream->stream_id);
	}

	spin_unlock_irqrestore(&pb->io_ipu.mipi_lock, irq_flags);
}
