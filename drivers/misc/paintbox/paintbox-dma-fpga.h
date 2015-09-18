/*
 * FPGA DMA support for the Paintbox programmable IPU
 *
 * Copyright (C) 2015 Google, Inc.
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

#ifndef __PAINTBOX_FPGA_DMA_H__
#define __PAINTBOX_FPGA_DMA_H__

#include <linux/types.h>

#include "paintbox-common.h"
#include "paintbox-xilinx-regs.h"


/* Size of the debug buffer used for debugfs or verbose logging.  This value
 * should be reevaluated whenever the dump_*_registers functions are changed.
 */
#define XILINX_DEBUG_BUFFER_SIZE (XILINX_NUM_REGS * REG_DEBUG_BUFFER_SIZE)

#if defined(CONFIG_DEBUG_FS) || defined(VERBOSE_DEBUG)
int dump_xilinx_registers(struct paintbox_data *pb, char *buf, size_t len);
#endif

#endif  /* __PAINTBOX_FPGA_DMA_H__ */
