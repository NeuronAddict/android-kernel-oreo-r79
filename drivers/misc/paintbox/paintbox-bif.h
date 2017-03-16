/*
 * BIF support for the Paintbox programmable IPU
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

#ifndef __PAINTBOX_BIF_H__
#define __PAINTBOX_BIF_H__

#include <linux/io.h>

#include "paintbox-common.h"
#include "paintbox-regs.h"

/* Size of the debug buffer used for debugfs or verbose logging.  These values
 * should be reevaluated whenever the paintbox_dump_*_registers functions are
 * changed.
 */
#define BIF_DEBUG_BUFFER_SIZE (IO_AXI_NUM_REGS * REG_DEBUG_BUFFER_SIZE)

#ifdef CONFIG_DEBUG_FS
int paintbox_dump_bif_registers(struct paintbox_debug *debug, char *buf,
		size_t len);
#else
static inline int paintbox_dump_bif_registers(struct paintbox_debug *debug,
		char *buf, size_t len)
{
	return 0;
}
#endif

/* This function must be called from an interrupt context */
void paintbox_bif_interrupt(struct paintbox_data *pb);

void paintbox_bif_start(struct paintbox_data *pb);
void paintbox_bif_shutdown(struct paintbox_data *pb);

int paintbox_bif_init(struct paintbox_data *pb);
void paintbox_bif_remove(struct paintbox_data *pb);


#endif /* __PAINTBOX_BIF_H__ */
