/*
 * Paintbox Simulator Register Definitions
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

#ifndef __PAINTBOX_SIM_REGS_H__
#define __PAINTBOX_SIM_REGS_H__

#include "paintbox-regs.h"

#define SIM_GROUP_OFFSET IPU_RESERVED_OFFSET

/* Simulator Group Register Offsets */
#define SIM_CTRL               0x00
#define SIM_STAT               0x08
#define SIM_BLOCK_LEN          0x10
#define SIM_NUM_REGS           (SIM_BLOCK_LEN / sizeof(uint64_t))

/* SIM_CTRL Register Bits */
#define SIM_WAIT_IDLE          (1 << 1)

/* SIM_STAT Register Bits */
#define SIM_STP_IDLE_MASK      0xFF

#endif /* __PAINTBOX_SIM_REGS_H__ */
