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
#define SIM_CAP0              0x04
#define SIM_CTRL              0x08
#define SIM_STAT              0x0C
#define SIM_TIMEOUT_L         0x10
#define SIM_TIMEOUT_H         0x14
#define SIM_BLOCK_LEN         0x18
#define SIM_NUM_REGS          (SIM_BLOCK_LEN / sizeof(uint32_t))

/* SIM ID Register Bits */
#define SIM_ID_HW_TYPE_MASK  (1 << 31)
#define SIM_ID_HW_TYPE_SIM   (1 << 31)
#define SIM_ID_HW_TYPE_FPGA  (0 << 31)

/* SIM CAP0 Register Bits */
#define SIM_CAP0_STP         0xFF
#define SIM_CAP0_STP_SHIFT   0
#define SIM_CAP0_STP_MASK    SIM_CAP0_STP
#define SIM_CAP0_INT         0xFF
#define SIM_CAP0_INT_SHIFT   8
#define SIM_CAP0_INT_MASK    (SIM_CAP0_INT << SIM_CAP0_INT_SHIFT)
#define SIM_CAP0_LBP         0xFF
#define SIM_CAP0_LBP_SHIFT   16
#define SIM_CAP0_LBP_MASK    (SIM_CAP0_LBP << SIM_CAP0_LBP_SHIFT)

/* SIM_CTRL Register Bits */
#define SIM_RUN               (1 << 0)
#define SIM_WAIT_IDLE         (1 << 1)
#define SIM_INT_SHIFT         24
#define SIM_INT               0xFF
#define SIM_INT_MASK          (SIM_INT << SIM_INT_SHIFT)

/* SIM_STAT Register Bits */
#define SIM_STP_IDLE_MASK     0xFF

#endif /* __PAINTBOX_SIM_REGS_H__ */
