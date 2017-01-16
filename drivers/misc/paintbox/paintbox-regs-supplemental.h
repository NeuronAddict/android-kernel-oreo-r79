/* Supplement to the generated register header file
 *
 * Copyright (C) 2016 The Android Open Source Project
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

#ifndef _PAINTBOX_REGS_SUPPLEMENTAL_H
#define _PAINTBOX_REGS_SUPPLEMENTAL_H

#include <linux/types.h>

#include "paintbox-regs.h"

#define IPU_REG_WIDTH_BYTES sizeof(uint64_t)

/* TODO(ahampson): Switch these to IPU_REG_WIDTH_BYTES when the trace code is
 * fully switched over.
 */
#define IO_APB_BLOCK_LEN     (APB_SPARE + IPU_REG_WIDTH)
#define IO_APB_NUM_REGS      (IO_APB_BLOCK_LEN / IPU_REG_WIDTH)

/* TODO(ahampson): Switch these to IPU_REG_WIDTH_BYTES when the trace code is
 * fully switched over.
 */
#define IO_AXI_BLOCK_LEN     (AXI_SPARE + IPU_REG_WIDTH)
#define IO_AXI_NUM_REGS      (IO_AXI_BLOCK_LEN / IPU_REG_WIDTH)

/* TODO(ahampson): Switch these to IPU_REG_WIDTH_BYTES when the trace code is
 * fully switched over.
 */
#define STP_BLOCK_LEN           (STP_PMON_CNT_1_STS + IPU_REG_WIDTH)
#define STP_NUM_REGS            (STP_BLOCK_LEN / IPU_REG_WIDTH)

/* TODO(ahampson): Switch these to IPU_REG_WIDTH_BYTES when the trace code is
 * fully switched over.
 */
#define LBP_POOL_BLOCK_LEN   (LBP_PMON_CNT_1_STS + IPU_REG_WIDTH)
#define LBP_POOL_NUM_REGS    (LBP_POOL_BLOCK_LEN / IPU_REG_WIDTH)

#define LBP_BLOCK_LEN        (LB_L_PARAM + IPU_REG_WIDTH)
#define LBP_NUM_REGS         (LBP_BLOCK_LEN / IPU_REG_WIDTH)

/* Block length for just the line buffer registers */
#define LB_BLOCK_START       LB_CTRL0
#define LB_BLOCK_LEN         (LBP_BLOCK_LEN - LB_BLOCK_START)
#define LB_NUM_REGS          (LB_BLOCK_LEN / IPU_REG_WIDTH)

#define LB_BDRY_CLAMP         0x00000000
#define LB_BDRY_REPEAT        0x00000001
#define LB_BDRY_REFLECT       0x00000002
#define LB_BDRY_RESERVED      0x00000003

#define LB_ADDR_ALIGN_MASK    0x1F
#define LB_ADDR_ALIGN_SHIFT   5

#define LBP_LB_ENA_DEF       8
#define LB_FB_ROWS_DEF       40
#define LB_NUM_RPTR_DEF      1
#define LB_NUM_CHAN_DEF      2
#define LB_IMG_HEIGHT_DEF    480
#define LB_IMG_WIDTH_DEF     640

#define LBP_MAX_RPTR          0xF
#define LBP_MAX_FB_ROWS       0x0FFF
#define LBP_MAX_CHAN          0x1FF
#define LBP_MAX_LB            0xF
#define LB_REUSE_ROWS_MAX     0x1F

#define LB_OFFSET_MIN         -32767
#define LB_OFFSET_MAX         32767

#define LB_OFFSET_CHAN_WIDTH  9
#define LB_OFFSET_CHAN_MAX    ((1 << LB_OFFSET_CHAN_WIDTH) - 1)

/* LB_STAT Register Bits */
#define LB_STAT_FULL          (1 << 0)
#define LB_STAT_EMPTY0        (1 << 1)
#define LB_STAT_EMPTY1        (1 << 2)
#define LB_STAT_EMPTY2        (1 << 3)

#define LB_L_INC_WIDTH        12
#define LB_L_WIDTH_WIDTH      12
#define LB_L_INC_MAX          ((1 << LB_L_INC_WIDTH) - 1)
#define LB_L_WIDTH_MAX        ((1 << LB_L_WIDTH_WIDTH) - 1)

#define LBP_DATA_REG_COUNT    4

/* MMU_ERR_LOG */
#define MMU_IOVA_SHIFT        12

/* MMU_ISR / MMU_IMR */
#define NUM_MMU_INTERRUPTS 1

/* BIF_ISR / BIF_IMR */
#define NUM_BIF_INTERRUPTS 1

#define STP_LBP_MASK_DEF        0xFFFF
#define STP_DATA_REG_COUNT      2

/* Vector Memory Address Bits */
#define STP_RAM_ADDR_ROW      0x0F
#define STP_RAM_ADDR_ROW_SHIFT 7
#define STP_RAM_ADDR_ROW_MASK (STP_RAM_ADDR_ROW << STP_RAM_ADDR_ROW_SHIFT)
#define STP_RAM_ADDR_OFFSET_MASK 0x7F

#define STP_RAM_TARG_INST_RAM 0
#define STP_RAM_TARG_CNST_RAM 1
#define STP_RAM_TARG_DATA_RAM 2
#define STP_RAM_TARG_ALU_IO_RF_0 3
#define STP_RAM_TARG_ALU_IO_RF_1 4
#define STP_RAM_TARG_ALU_IO_RF_2 5
#define STP_RAM_TARG_ALU_IO_RF_3 6
#define STP_RAM_TARG_ALU_IO_RF_4 7
#define STP_RAM_TARG_ALU_IO_RAM_0 11
#define STP_RAM_TARG_ALU_IO_RAM_1 12
#define STP_RAM_TARG_ALU_IO_RAM_2 13
#define STP_RAM_TARG_ALU_IO_RAM_3 14
#define STP_RAM_TARG_ALU_IO_RAM_4 15

#define COMMON_RAM_ADDR       0xFFFF
#define COMMON_RAM_ADDR_SHIFT 16

#define STP_LANE_GROUP_WIDTH 4
#define VECTOR_GROUP_ROW_OFFSET_BYTES 4

#endif /* __PAINTBOX_REGS_SUPPLEMENTAL_H__ */
