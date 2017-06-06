/*
 * STP debug support for the Paintbox programmable IPU
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

#ifndef __PAINTBOX_STP_DEBUG_H__
#define __PAINTBOX_STP_DEBUG_H__

#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ktime.h>
#include <linux/types.h>

#include "paintbox-common.h"

#ifdef CONFIG_PAINTBOX_DEBUG
void paintbox_stp_debug_init(struct paintbox_data *pb,
		struct paintbox_stp *stp);
void paintbox_stp_debug_deinit(struct paintbox_data *pb,
		struct paintbox_stp *stp);

int paintbox_dump_stp_registers(struct paintbox_debug *debug, char *buf,
		size_t len);
#else
static inline void paintbox_stp_debug_init(struct paintbox_data *pb,
		struct paintbox_stp *stp) { }
static inline void paintbox_stp_debug_deinit(struct paintbox_data *pb,
		struct paintbox_stp *stp) { }
#endif

#endif /* __PAINTBOX_STP_DEBUG_H__ */
