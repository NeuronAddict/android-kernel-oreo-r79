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
#define SIM_ID                0x00
#define SIM_CTRL              0x04
#define SIM_STAT              0x08
#define SIM_TIMEOUT_L         0x0C
#define SIM_TIMEOUT_H         0x10
#define SIM_BLOCK_LEN         0x14
#define SIM_NUM_REGS          (SIM_BLOCK_LEN / sizeof(uint32_t))

/* SIM_CTRL Register Bits */
#define SIM_RUN               (1 << 0)
#define SIM_WAIT_IDLE         (1 << 1)
#define SIM_INT_SRC_M         0xFF
#define SIM_INT_SRC_SHIFT     16
#define SIM_INT_SRC_DMA       0
#define SIM_INT_SRC_MIPI_IN   1
#define SIM_INT_SRC_MIPI_OUT  2
#define SIM_INT_SRC_STP       3
#define SIM_INT_SRC_MMU       4
#define SIM_INT_SRC_BIF       5
#define SIM_INT_SRC_MASK      (SIM_INT_SRC_M << SIM_INT_SRC_SHIFT)
#define SIM_INT_ID_SHIFT      24
#define SIM_INT_ID_M          0xFF
#define SIM_INT_ID_MASK       (SIM_INT_ID_M << SIM_INT_ID_SHIFT)

/* SIM_STAT Register Bits */
#define SIM_STP_IDLE_MASK     0xFF

#endif /* __PAINTBOX_SIM_REGS_H__ */
