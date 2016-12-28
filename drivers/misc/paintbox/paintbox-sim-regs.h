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


/* Simulator Group Register Offsets */
#define SIM_ID                 0x00
#define SIM_CTRL               0x08
#define SIM_STAT               0x10
#define SIM_TIMEOUT            0x18
#define SIM_INTERRUPT_MASK_ALL 0x20
#define SIM_INTERRUPT_MASK_ANY 0x28
#define SIM_BLOCK_LEN          0x30
#define SIM_NUM_REGS           (SIM_BLOCK_LEN / sizeof(uint64_t))

/* SIM_CTRL Register Bits */
#define SIM_RUN                (1 << 0)
#define SIM_WAIT_IDLE          (1 << 1)
#define SIM_INT_ID_SHIFT       8
#define SIM_INT_ID_M           0xFF
#define SIM_INT_ID_MASK        (SIM_INT_ID_M << SIM_INT_ID_SHIFT)

/* SIM_STAT Register Bits */
#define SIM_STP_IDLE_MASK      0xFF

#endif /* __PAINTBOX_SIM_REGS_H__ */
