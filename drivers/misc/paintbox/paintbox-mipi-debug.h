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

#ifdef DEBUG
void paintbox_log_mipi_input_setup(struct paintbox_data *pb,
		struct mipi_stream_setup *setup);
void paintbox_log_mipi_output_setup(struct paintbox_data *pb,
		struct mipi_stream_setup *setup);

#define LOG_MIPI_INPUT_SETUP(pb, config)				\
	paintbox_log_mipi_input_setup(pb, config)
#define LOG_MIPI_OUTPUT_SETUP(pb, config)				\
	paintbox_log_mipi_output_setup(pb, config)
#else
#define LOG_MIPI_INPUT_SETUP(pb, config)				\
do { } while (0)
#define LOG_MIPI_OUTPUT_SETUP(pb, config)				\
do { } while (0)
#endif

#ifdef VERBOSE_DEBUG
void paintbox_log_mipi_registers(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream, const char *msg);

#define LOG_MIPI_REGISTERS(pb, stream)		\
	paintbox_log_mipi_registers(pb, stream, __func__)

#else
#define LOG_MIPI_REGISTERS(pb, stream)		\
do { } while (0)
#endif

#ifdef CONFIG_PAINTBOX_DEBUG
int paintbox_dump_mipi_common_registers(struct paintbox_debug *debug, char *buf,
		size_t len);
int paintbox_dump_mipi_input_stream_registers(struct paintbox_debug *debug,
		char *buf, size_t len);
int paintbox_dump_mipi_output_stream_registers(struct paintbox_debug *debug,
		char *buf, size_t len);

void paintbox_mipi_input_stream_debug_init(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream);
void paintbox_mipi_output_stream_debug_init(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream);

void paintbox_mipi_stream_debug_remove(struct paintbox_data *pb,
		struct paintbox_mipi_stream *stream);

void paintbox_mipi_debug_init(struct paintbox_data *pb);
void paintbox_mipi_debug_remove(struct paintbox_data *pb);
#else
static inline void paintbox_mipi_input_stream_debug_init(
		struct paintbox_data *pb, struct paintbox_mipi_stream *stream)
{ }

static inline void paintbox_mipi_output_stream_debug_init(
		struct paintbox_data *pb, struct paintbox_mipi_stream *stream)
{ }

static inline void paintbox_mipi_stream_debug_remove(
		struct paintbox_data *pb, struct paintbox_mipi_stream *stream)
{ }

static inline void paintbox_mipi_debug_init(struct paintbox_data *pb) { }
static inline void paintbox_mipi_debug_remove(struct paintbox_data *pb) { }
#endif

#endif  /* __PAINTBOX_MIPI_DEBUG_H__ */
