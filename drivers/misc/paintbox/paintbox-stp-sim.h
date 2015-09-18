/*
 * STP support (Simulator Specific) for the Paintbox programmable IPU
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

#ifndef __PAINTBOX_STP_SIM_H__
#define __PAINTBOX_STP_SIM_H__

#include <linux/io.h>

#include "paintbox-common.h"

/* The caller to these functions must hold pb->lock */
int sim_wait_for_idle(struct paintbox_data *pb, struct paintbox_stp *stp);
void sim_execute(struct paintbox_data *pb, unsigned int channel_id,
		uint64_t timeout_ns);

int sim_get_stp_idle_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);
int sim_wait_for_idle_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);

#endif /* __PAINTBOX_STP_SIM_H__ */
