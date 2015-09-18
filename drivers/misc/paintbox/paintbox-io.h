/*
 * IO support for the Paintbox programmable IPU
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

#ifndef __PAINTBOX_IO_H__
#define __PAINTBOX_IO_H__

#include <linux/io.h>

#include "paintbox-common.h"


/* Size of the debug buffer used for debugfs or verbose logging.  These values
 * should be reevaluated whenever the dump_*_registers functions are changed.
 */
#define IO_AXI_DEBUG_BUFFER_SIZE (IO_AXI_NUM_REGS * REG_DEBUG_BUFFER_SIZE)
#define IO_DEBUG_BUFFER_SIZE (IO_FPGA_BLOCK_LEN * REG_DEBUG_BUFFER_SIZE)

int paintbox_io_init(struct paintbox_data *pb);

#if defined(CONFIG_DEBUG_FS) || defined(VERBOSE_DEBUG)
int dump_io_axi_registers(struct paintbox_data *pb, char *buf, size_t len);
int dump_io_fpga_registers(struct paintbox_data *pb, char *buf, size_t len);
#endif

#endif /* __PAINTBOX_IO_H__ */
