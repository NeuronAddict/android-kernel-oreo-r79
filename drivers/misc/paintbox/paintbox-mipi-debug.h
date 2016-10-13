/*
 * MIPI Debug Support for Paintbox programmable IPU
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

#ifndef __PAINTBOX_MIPI_DEBUG_H__
#define __PAINTBOX_MIPI_DEBUG_H__

#include <linux/types.h>

#include "paintbox-common.h"


/* Size of the debug buffer used for debugfs or verbose logging.  This value
 * should be reevaluated whenever the dump_stp_registers function is changed.
 */
#define MIPI_DEBUG_BUFFER_SIZE (IO_IPU_NUM_REGS * REG_DEBUG_BUFFER_SIZE)

int dump_io_ipu_registers(struct paintbox_debug *debug, char *buf, size_t len);
int dump_mipi_input_stream_registers(struct paintbox_debug *debug, char *buf,
		size_t len);
int dump_mipi_output_stream_registers(struct paintbox_debug *debug, char *buf,
		size_t len);


#ifdef VERBOSE_DEBUG
void log_mipi_registers(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream, const char *msg);

#define LOG_MIPI_REGISTERS(pb, stream)		\
	log_mipi_registers(pb, stream, __func__)

#else
#define LOG_MIPI_REGISTERS(pb, stream)		\
do { } while (0)
#endif

void paintbox_mipi_input_stream_debug_init(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream);
void paintbox_mipi_output_stream_debug_init(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream);
void paintbox_mipi_debug_init(struct paintbox_data *pb);

#endif  /* __PAINTBOX_MIPI_DEBUG_H__ */
