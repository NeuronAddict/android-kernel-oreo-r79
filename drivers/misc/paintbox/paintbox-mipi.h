/*
 * MIPI Support for Paintbox programmable IPU
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

#ifndef __PAINTBOX_MIPI_H__
#define __PAINTBOX_MIPI_H__

#include <linux/types.h>

#include "paintbox-common.h"


/* Size of the debug buffer used for debugfs or verbose logging.  This value
 * should be reevaluated whenever the dump_stp_registers function is changed.
 */
#define MIPI_DEBUG_BUFFER_SIZE (IO_IPU_NUM_REGS * REG_DEBUG_BUFFER_SIZE)

int allocate_mipi_input_stream_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);
int allocate_mipi_output_stream_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);
int release_mipi_stream_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg,
		bool is_input);
int setup_mipi_stream_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg,
		bool is_input);
int enable_mipi_stream_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg,
		bool is_input);
int disable_mipi_stream_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg,
		bool is_input);
int reset_mipi_stream_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg,
		bool is_input);
int cleanup_mipi_stream_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg,
		bool is_input);
int enable_mipi_interrupt_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg,
		bool is_input);
int disable_mipi_interrupt_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg,
		bool is_input);
int wait_for_mipi_interrupt_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg,
		bool is_input);

void release_mipi_stream(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_mipi_stream *stream);

int dump_io_ipu_registers(struct paintbox_debug *debug, char *buf, size_t len);
int dump_mipi_input_stream_registers(struct paintbox_debug *debug, char *buf,
		size_t len);
int dump_mipi_output_stream_registers(struct paintbox_debug *debug, char *buf,
		size_t len);

irqreturn_t paintbox_mipi_input_interrupt(struct paintbox_data *pb,
		uint32_t stream_mask);
irqreturn_t paintbox_mipi_output_interrupt(struct paintbox_data *pb,
		uint32_t stream_mask);

int paintbox_mipi_init(struct paintbox_data *pb);

#endif  /* __PAINTBOX_MIPI_H__ */
