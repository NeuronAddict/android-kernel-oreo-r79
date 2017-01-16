/*
 * Power management support for the Paintbox programmable IPU
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

#ifndef __PAINTBOX_POWER_H__
#define __PAINTBOX_POWER_H__

#include <linux/io.h>

#include "paintbox-common.h"

/* The caller to these functions must hold pb->dma.dma_lock */
void ipu_pm_enable_dma_channel(struct paintbox_data *pb,
		unsigned int channel_id);
void ipu_pm_disable_dma_channel(struct paintbox_data *pb,
		unsigned int channel_id);

/* The caller to these functions must hold pb->lock */
void ipu_pm_stp_enable(struct paintbox_data *pb, struct paintbox_stp *stp);
void ipu_pm_lbp_enable(struct paintbox_data *pb, struct paintbox_lbp *lbp);
void ipu_pm_stp_disable(struct paintbox_data *pb, struct paintbox_stp *stp);
void ipu_pm_lbp_disable(struct paintbox_data *pb, struct paintbox_lbp *lbp);

int ipu_pm_init(struct paintbox_data *pb);

#if defined(CONFIG_DEBUG_FS) || defined(VERBOSE_DEBUG)
int ipu_pm_dump_registers(struct paintbox_debug *debug, char *buf, size_t len);
#endif

#endif /* __PAINTBOX_POWER_H__ */
