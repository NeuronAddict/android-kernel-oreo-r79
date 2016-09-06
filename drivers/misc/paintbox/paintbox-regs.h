/*
 * Register definitions for the Paintbox programmable IPU
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

#ifndef __PAINTBOX_REGS_H__
#define __PAINTBOX_REGS_H__

#include <linux/types.h>


#define REG_UNUSED "UNKNOWN"
#define IPU_REG_WIDTH sizeof(uint32_t)

/* Number of bytes allocated for each register trace when dumping the register
 * contents through verbose logging or debugfs.
 */
#define REG_DEBUG_BUFFER_SIZE 96

#define REG_INDEX(r) (r / IPU_REG_WIDTH)
#define REG_NAME_ENTRY(r) [r / IPU_REG_WIDTH] = #r

#define STP_INST_SRAM_INSTRUCTION_WIDTH_BYTES 16
#define STP_CONST_SRAM_WORD_WIDTH_BYTES       2
#define STP_SCALAR_SRAM_WORD_WIDTH_BYTES      2
#define STP_VECTOR_SRAM_WORD_WIDTH_BYTES      2

/* Width of STP RAM transfer through PIO interface */
#define STP_PIO_WORD_WIDTH_BYTES  16

/* Register Group Offsets */
#define IPU_IO_APB_OFFSET              0x0000
#define IPU_IO_AXI_OFFSET              0x0400
#define IPU_IO_IPU_OFFSET              0x0800
#define IPU_DMA_OFFSET                 0x0C00
#define IPU_STP_OFFSET                 0x1000
#define IPU_LBP_OFFSET                 0x1400
#define IPU_IO_ADB_OFFSET              0x1800
#define IPU_RESERVED_OFFSET            0x1C00
#define IPU_REGISTER_SPACE_LEN         0x3FFF

/* IO APB Register Group Offsets */
#define IPU_VERSION           0x00
#define IPU_CHECKSUM_L        0x08
#define IPU_CHECKSUM_H        0x0C
#define IPU_ISR               0x10
#define IPU_IMR               0x18
#define IPU_CAP               0x20
#define CLK_GATE_CONTROL_STP_IDLE_GATE_DIS 0x28
#define CLK_GATE_CONTROL_LBP_IDLE_GATE_DIS 0x30
#define CLK_GATE_CONTROL      0x38
#define IPU_CORE_PAIRS_EN     0x40
#define CORE_POWER_ON_N       0x48
#define CORE_ISO_ON           0x50
#define CORE_RAM_ON_N         0x58
#define IPU_DMA_CHAN_EN       0x60
#define IO_APB_BLOCK_LEN      0x68
#define IO_APB_NUM_REGS      (IO_APB_BLOCK_LEN / IPU_REG_WIDTH)

/* IPU_VERSION Register Bits */
#define IPU_VERSION_INCR_MASK 0xFF
#define IPU_VERSION_MINOR_M   0xFF
#define IPU_VERSION_MINOR_SHIFT 8
#define IPU_VERSION_MINOR_MASK (IPU_VERSION_MINOR_M << IPU_VERSION_MINOR_SHIFT)
#define IPU_VERSION_FPGA_BUILD (1 << 23)
#define IPU_VERSION_MAJOR_M   0xFF
#define IPU_VERSION_MAJOR_SHIFT 24
#define IPU_VERSION_MAJOR_MASK (IPU_VERSION_MAJOR_M << IPU_VERSION_MAJOR_SHIFT)

/* IPU_CAP Register Bits */
#define NUM_STP_MASK          0xFF
#define NUM_LBP_M             0xFF
#define NUM_LBP_SHIFT         8
#define NUM_LBP_MASK          (NUM_LBP_M << NUM_LBP_SHIFT)

/* IO AXI Register Group Offsets */
#define MMU_CTRL              0x00
#define MMU_TABLE_BASE        0x08
#define MMU_ERR_BASE          0x10
#define MMU_SYNC              0x18
#define MMU_FLUSH_CHANNEL     0x20
#define MMU_FLUSH_ADDRESS     0x28
#define MMU_FLUSH_FIFO_LEVEL  0x30
#define MMU_FLUSH_FIFO_FULL   0x38
#define MMU_ISR               0x40
#define MMU_IMR               0x48
#define MMU_ISR_OVF           0x50
#define MMU_ERR_LOG_L         0x58
#define MMU_ERR_LOG_H         0x5C
#define BIF_AXI_CTRL_DMA0     0x80
#define BIF_AXI_CTRL_DMA1     0x88
#define BIF_AXI_CTRL_DMA2     0x90
#define BIF_AXI_CTRL_DMA3     0x98
#define BIF_AXI_CTRL_DMA4     0xA0
#define BIF_AXI_CTRL_DMA5     0xA8
#define BIF_AXI_CTRL_MMU      0xB0
#define BIF_IMR               0xB8
#define BIF_ISR               0xC0
#define BIF_ISR_OVF           0xC8
#define BIF_ERR_CFG_STS       0xD0
#define BIF_ERR_LOG_L         0xD8
#define BIF_ERR_LOG_H         0xDC
#define BIF_ERR_LOG_BUS_ADDR  0xE0
#define BIF_PMON_CFG          0x100
#define BIF_PMON_CNT_0_CFG    0x108
#define BIF_PMON_CNT_0        0x110
#define BIF_PMON_CNT_0_STS_ACC 0x118
#define BIF_PMON_CNT_0_STS    0x120
#define BIF_PMON_CNT_1_CFG    0x128
#define BIF_PMON_CNT_1        0x130
#define BIF_PMON_CNT_1_STS_ACC 0x138
#define BIF_PMON_CNT_1_STS    0x140
#define MMU_PMON_CFG          0x1C0
#define MMU_PMON_CNT_0_CFG    0x1C8
#define MMU_PMON_CNT_0        0x1D0
#define MMU_PMON_CNT_0_STS_ACC 0x1D8
#define MMU_PMON_CNT_0_STS    0x1E0
#define MMU_PMON_CNT_1_CFG    0x1E8
#define MMU_PMON_CNT_1        0x1F0
#define MMU_PMON_CNT_1_STS_ACC 0x1F8
#define MMU_PMON_CNT_1_STS    0x200
#define IO_AXI_BLOCK_LEN      0x208
#define IO_AXI_NUM_REGS      (IO_AXI_BLOCK_LEN / IPU_REG_WIDTH)

/* MMU_CTRL Register Bits */
#define MMU_ENABLE            (1 << 0)

/* MMU_TABLE_BASE Register Bits */
#define MMU_TABLE_BASE_RSHIFT 12

/* MMU_ERROR_BASE_Register Bits */
#define MMU_ERROR_BASE_RSHIFT 12

/* MMU_ISR / MMU_IMR */
#define NUM_MMU_INTERRUPTS 1

/* BIF_ISR / BIF_IMR */
#define NUM_BIF_INTERRUPTS 1

/* IO IPU Register Group Offsets */
#define MPI_CAP               0x00
#define MPI_STRM_SEL          0x08
#define MPI_COMMON_BLOCK_END  0x10
#define MPI_COMMON_BLOCK_LEN  MPI_COMMON_BLOCK_END
#define MPI_COMMON_NUM_REGS  (MPI_COMMON_BLOCK_LEN / IPU_REG_WIDTH)

#define MPI_STRM_BLOCK_START MPI_COMMON_BLOCK_LEN
#define MPI_STRM_CTRL         MPI_STRM_BLOCK_START
#define MPI_STRM_CNFG0_L      0x18
#define MPI_STRM_CNFG0_H      0x1C
#define MPI_STRM_CNFG1_L      0x20
#define MPI_STRM_CNFG1_H      0x24
#define MPI_STRM_CNFG0_L_RO   0x28
#define MPI_STRM_CNFG0_H_RO   0x2C
#define MPI_STRM_CNFG1_L_RO   0x30
#define MPI_STRM_CNFG1_H_RO   0x34
#define MPI_STRM_BLOCK_END    0x38
#define MPI_STRM_BLOCK_LEN    (MPI_STRM_BLOCK_END - MPI_STRM_BLOCK_START)
#define MPI_STRM_NUM_REGS     (MPI_STRM_BLOCK_LEN / IPU_REG_WIDTH)

#define MPO_COMMON_BLOCK_START MPI_STRM_BLOCK_END
#define MPO_CAP               MPO_COMMON_BLOCK_START
#define MPO_STRM_SEL          (MPO_COMMON_BLOCK_START + 0x08)
#define MPO_COMMON_BLOCK_END  (MPO_COMMON_BLOCK_START + 0x10)
#define MPO_COMMON_BLOCK_LEN  (MPO_COMMON_BLOCK_END - MPO_COMMON_BLOCK_START)
#define MPO_COMMON_NUM_REGS   (MPO_COMMON_BLOCK_LEN / IPU_REG_WIDTH)
#define MPO_STRM_BLOCK_START  MPO_COMMON_BLOCK_END
#define MPO_STRM_CTRL         MPO_STRM_BLOCK_START
#define MPO_STRM_CNFG0_L      0x50
#define MPO_STRM_CNFG0_H      0x54
#define MPO_STRM_CNFG1        0x58
#define MPO_STRM_CNFG0_L_RO   0x60
#define MPO_STRM_CNFG0_H_RO   0x64
#define MPO_STRM_CNFG1_RO     0x68
#define MPO_STRM_BLOCK_END    0x70
#define MPO_STRM_BLOCK_LEN    (MPO_STRM_BLOCK_END - MPO_STRM_BLOCK_START)
#define MPO_STRM_NUM_REGS     (MPO_STRM_BLOCK_LEN / IPU_REG_WIDTH)

#define IO_IPU_BLOCK_LEN       MPO_STRM_BLOCK_END
#define IO_IPU_NUM_REGS       (IO_IPU_BLOCK_LEN / IPU_REG_WIDTH)

/* MPI_CAP Register Bits */
#define MPI_MAX_IFC_MASK      0x0F
#define MPI_MAX_STRM_M        0x0F
#define MPI_MAX_STRM_SHIFT    4
#define MPI_MAX_STRM_MASK     (MPI_MAX_STRM_M << MPI_MAX_STRM_SHIFT)

/* MPI_STRM_SEL Register Bits */
#define MPI_STRM_SEL_MASK     0x0F
#define MPI_STRM_SEL_DEF      15

/* MPI_STRM_CTRL Register Bits */
#define MPI_STRM_EN           (1 << 0)
#define MPI_STRM_CLEANUP      (1 << 1)
#define MPI_STRM_RST          (1 << 2)
#define MPI_STRM_SOF_ISR      (1 << 3)
#define MPI_STRM_SOF_IMR      (1 << 4)
#define MPI_STRM_OVF_ISR      (1 << 5)
#define MPI_STRM_OVF_IMR      (1 << 6)
#define MPI_STRM_NUM_FRAME_M  0xFFFF
#define MPI_STRM_NUM_FRAME_SHIFT 8
#define MPI_STRM_NUM_FRAME_MASK (MPI_STRM_NUM_FRAME_M <<                       \
		MPI_STRM_NUM_FRAME_SHIFT)

/* MPI_STRM_CNFG0_L Register Bits */
#define MPI_VC_MASK           0x03
#define MPI_DT_IN_M           0x3F
#define MPI_DT_IN_SHIFT       4
#define MPI_DT_IN_MASK        (MPI_DT_IN_M << MPI_DT_IN_SHIFT)
#define MPI_DT_PROC_M         0x3F
#define MPI_DT_PROC_SHIFT     10
#define MPI_DT_PROC_MASK      (MPI_DT_PROC_M << MPI_DT_PROC_SHIFT)
#define MPI_STRP_HEIGHT_M     0x3F
#define MPI_STRP_HEIGHT_SHIFT 16
#define MPI_STRP_HEIGHT_MASK (MPI_STRP_HEIGHT_M << MPI_STRP_HEIGHT_SHIFT)

#define MPI_VC_MAX            MPI_VC_MASK
#define MPI_DT_IN_MAX         MPI_DT_IN_M
#define MPI_DT_IN_DEF         42
#define MPI_DT_PROC_MAX       MPI_DT_PROC_M
#define MPI_DT_PROC_DEF       46
#define MPI_STRP_HEIGHT_MAX   63
#define MPI_STRP_HEIGHT_DEF   3

/* MPI_STRM_CNFG0_H Register Bits */
#define MPI_IMG_WIDTH_MASK    0xFFFF
#define MPI_IMG_HEIGHT_M      0xFFFF
#define MPI_IMG_HEIGHT_SHIFT  16
#define MPI_IMG_HEIGHT_MASK   (MPI_IMG_HEIGHT_M << MPI_IMG_HEIGHT_SHIFT)

#define MPI_IMG_WIDTH_MAX     MPI_IMG_WIDTH_MASK
#define MPI_IMG_WIDTH_DEF     640
#define MPI_IMG_HEIGHT_MAX    MPI_IMG_HEIGHT_M
#define MPI_IMG_HEIGHT_DEF    480

/* MPI_STRM_CNFG1_L Register Bits */
#define MPI_SEG_START_MASK    0x7F
#define MPI_SEG_END_M         0x7F
#define MPI_SEG_END_SHIFT     16
#define MPI_SEG_END_MASK      (MPI_SEG_END_M << MPI_SEG_END_SHIFT)

#define MPI_SEG_START_MAX     MPI_SEG_START_MASK
#define MPI_SEG_END_MAX       MPI_SEG_END_M

/* MPI_STRM_CNFG1_H Register Bits */
#define MPI_SEGS_PER_ROW_MASK 0x7F
#define MPI_SEG_WORDS_PER_ROW_M 0xFFF
#define MPI_SEG_WORDS_PER_ROW_SHIFT 16
#define MPI_SEG_WORDS_PER_ROW_MASK (MPI_SEG_WORDS_PER_ROW_M <<                 \
		MPI_SEG_WORDS_PER_ROW_SHIFT)

#define MPI_SEGS_PER_ROW_MAX  127
#define MPI_SEG_WORDS_PER_ROW_MAX MPI_SEG_WORDS_PER_ROW_M
#define MPI_SEG_WORDS_PER_ROW_DEF 39

/* MPO_CAP Register Bits */
#define MPO_MAX_IFC_MASK      0x0F
#define MPO_MAX_STRM_M        0x0F
#define MPO_MAX_STRM_SHIFT    4
#define MPO_MAX_STRM_MASK     (MPO_MAX_STRM_M << MPO_MAX_STRM_SHIFT)

/* MPO_STRM_SEL Register Bits */
#define MPO_STRM_SEL_MASK     0x0F
#define MPO_STRM_SEL_DEF      3

/* MPO_STRM_CTRL Register Bits */
#define MPO_STRM_EN           (1 << 0)
#define MPO_STRM_RSYNC_EN     (1 << 1)
#define MPO_STRM_CLEANUP      (1 << 2)
#define MPO_STRM_RST          (1 << 3)
#define MPO_STRM_EOF_ISR      (1 << 4)
#define MPO_STRM_EOF_IMR      (1 << 5)

/* MPO_STRM_CNFG0_L Register Bits */
#define MPO_VC_MASK           0x03
#define MPO_DT_OUT_M          0x3F
#define MPO_DT_OUT_SHIFT      4
#define MPO_DT_OUT_MASK       (MPO_DT_OUT_M << MPO_DT_OUT_SHIFT)
#define MPO_DT_PROC_M         0x3F
#define MPO_DT_PROC_SHIFT     10
#define MPO_DT_PROC_MASK      (MPO_DT_PROC_M << MPO_DT_PROC_SHIFT)
#define MPO_STRP_HEIGHT_M     0x3F
#define MPO_STRP_HEIGHT_SHIFT 16
#define MPO_STRP_HEIGHT_MASK  (MPO_STRP_HEIGHT_M << MPO_STRP_HEIGHT_SHIFT)

#define MPO_DT_OUT_DEF        42
#define MPO_DT_PROC_DEF       46
#define MPO_STRP_HEIGHT_DEF   3

/* MPO_STRM_CNFG0_H Register Bits */
#define MPO_IMG_WIDTH_MASK    0xFFFF
#define MPO_IMG_HEIGHT_M      0xFFFF
#define MPO_IMG_HEIGHT_SHIFT  16
#define MPO_IMG_HEIGHT_MASK   (MPO_IMG_HEIGHT_M << MPO_IMG_HEIGHT_SHIFT)

#define MPO_IMG_HEIGHT_DEF    480
#define MPO_IMG_WIDTH_DEF     640

/* MPO_STRM_CNFG1 Register Bits */
#define MPO_SEG_END_MASK      0x7F
#define MPO_SEGS_PER_ROW_M    0x7F
#define MPO_SEGS_PER_ROW_SHIFT 16
#define MPO_SEGS_PER_ROW_MASK (MPO_SEGS_PER_ROW_M << MPO_SEGS_PER_ROW_SHIFT)

#define MPO_SEG_END_MAX       127

#define MPO_SEGS_PER_ROW_MAX  MPO_SEGS_PER_ROW_M
#define MPO_SEGS_PER_ROW_DEF  2

/* DMA Register Group Offsets */
#define DMA_CTRL              0x00
#define DMA_CHAN_CTRL_L       0x08
#define DMA_CHAN_CTRL_H       0x0C
#define DMA_CAP0              0x10
#define DMA_PMON_CFG          0x100
#define DMA_PMON_CNT_0_CFG    0x108
#define DMA_PMON_CNT_0        0x110
#define DMA_PMON_CNT_0_STS_ACC 0x118
#define DMA_PMON_CNT_0_STS    0x120
#define DMA_PMON_CNT_1_CFG    0x128
#define DMA_PMON_CNT_1        0x130
#define DMA_PMON_CNT_1_STS_ACC 0x138
#define DMA_PMON_CNT_1_STS    0x140
#define DMA_PMON_CNT_2_CFG    0x148
#define DMA_PMON_CNT_2        0x150
#define DMA_PMON_CNT_2_STS_ACC 0x158
#define DMA_PMON_CNT_2_STS    0x160
#define DMA_PMON_CNT_3_CFG    0x168
#define DMA_PMON_CNT_3        0x170
#define DMA_PMON_CNT_3_STS_ACC 0x178
#define DMA_PMON_CNT_3_STS    0x180
#define DMA_CTRL_BLOCK_LEN    0x188
#define DMA_CTRL_NUM_REGS     (DMA_CTRL_BLOCK_LEN / IPU_REG_WIDTH)

#define DMA_CHAN_BLOCK_START  0x188
#define DMA_CHAN_MODE         0x188
#define DMA_CHAN_IMG_FORMAT   0x190
#define DMA_CHAN_IMG_SIZE     0x198
#define DMA_CHAN_IMG_POS_L    0x1A0
#define DMA_CHAN_IMG_POS_H    0x1A4
#define DMA_CHAN_IMG_LAYOUT_L 0x1A8
#define DMA_CHAN_IMG_LAYOUT_H 0x1AC
#define DMA_CHAN_BIF_XFER     0x1B0
#define DMA_CHAN_VA_L         0x1B8
#define DMA_CHAN_VA_H         0x1BC
#define DMA_CHAN_VA_BDRY_L    0x1C0
#define DMA_CHAN_VA_BDRY_H    0x1C4
#define DMA_CHAN_NOC_XFER_L   0x1C8
#define DMA_CHAN_NOC_XFER_H   0x1CC
#define DMA_CHAN_NODE         0x1D0
#define DMA_CHAN_IMR          0x1D8
#define DMA_CHAN_ISR          0x1E0
#define DMA_CHAN_ISR_OVF      0x1E8
#define DMA_CHAN_MODE_RO      0x1F0
#define DMA_CHAN_IMG_FORMAT_RO 0x1F8
#define DMA_CHAN_IMG_SIZE_L_RO 0x200
#define DMA_CHAN_IMG_SIZE_H_RO 0x204
#define DMA_CHAN_IMG_POS_L_RO 0x208
#define DMA_CHAN_IMG_POS_H_RO 0x20C
#define DMA_CHAN_IMG_LAYOUT_L_RO 0x210
#define DMA_CHAN_IMG_LAYOUT_H_RO 0x214
#define DMA_CHAN_BIF_XFER_RO  0x218
#define DMA_CHAN_VA_L_RO      0x220
#define DMA_CHAN_VA_H_RO      0x224
#define DMA_CHAN_VA_BDRY_L_RO 0x228
#define DMA_CHAN_VA_BDRY_H_RO 0x22C
#define DMA_CHAN_NOC_XFER_L_RO 0x230
#define DMA_CHAN_NOC_XFER_H_RO 0x234
#define DMA_CHAN_NODE_RO      0x238
#define DMA_CHAN_DEPENDENCY   0x240
#define DMA_CHAN_BLOCK_END    0x248

#define DMA_CHAN_BLOCK_LEN    (DMA_CHAN_BLOCK_END - DMA_CHAN_BLOCK_START)
#define DMA_CHAN_NUM_REGS     (DMA_CHAN_BLOCK_LEN / IPU_REG_WIDTH)

#define DMA_STS_BLOCK_START   0x248
#define DMA_STS_CTRL          0x248
#define DMA_STS_STATE         0x250
#define DMA_STS_PTR_L         0x258
#define DMA_STS_PTR_H         0x25C
#define DMS_STS_ADDR_L        0x260
#define DMA_STS_ADDR_H        0x264
#define DMA_SPARE             0x268
#define DMA_STS_BLOCK_END     0x270

#define DMA_STS_BLOCK_LEN     (DMA_STS_BLOCK_END - DMA_STS_BLOCK_START)
#define DMA_STS_NUM_REGS      (DMA_STS_BLOCK_LEN / IPU_REG_WIDTH)

#define DMA_BLOCK_LEN         DMA_STS_BLOCK_END
#define DMA_NUM_REGS          (DMA_BLOCK_LEN / IPU_REG_WIDTH)

/* DMA_CTRL Register Bits */
#define DMA_RESET             (1 << 0)
#define DMA_CHAN_SEL_WIDTH    0xFF
#define DMA_CHAN_SEL_SHIFT    8
#define DMA_CHAN_SEL_MASK     (DMA_CHAN_SEL_WIDTH << DMA_CHAN_SEL_SHIFT)
#define DMA_AXI_SWIZZLE_NONE  0
#define DMA_AXI_SWIZZLE_BIG_ENDIAN 1
#define DMA_AXI_SWIZZLE_NEIGHBOR_BYTES 2
#define DMA_AXI_SWIZZLE_M     0x03
#define DMA_AXI_SWIZZLE_SHIFT 16
#define DMA_AXI_SWIZZLE_MASK (DMA_AXI_SWIZZLE_M << DMA_AXI_SWIZZLE_SHIFT)

#define DMA_CHAN_SEL_DEF      31

/* DMA_CHAN_CTRL_L Register Bits */
#define DMA_CHAN_RESET_MASK   0xFFFF
#define DMA_CHAN_DOUBLE_BUF_M 0xFFFF
#define DMA_CHAN_DOUBLE_BUF_SHIFT 16
#define DMA_CHAN_DOUBLE_BUF_MASK (DMA_CHAN_DOUBLE_BUF_M <<                     \
		DMA_CHAN_DOUBLE_BUF_SHIFT)

#define DMA_CHAN_DOUBLE_BUF_DEF 0xFFFF

/* DMA_CHAN_CTRL_H Register Bits */
#define DMA_STOP_MASK         0xFFFF

/* DMA_CAP0 Register Bits */
#define MAX_DMA_CHAN_MASK     0xFF

/* DMA_CHAN_MODE Register Bits */
#define DMA_CHAN_ENA          (1 << 0)
#define DMA_CHAN_SRC_DRAM     0
#define DMA_CHAN_SRC_LBP      1
#define DMA_CHAN_SRC_STP      2
#define DMA_CHAN_SRC_MIPI_IN  3
#define DMA_CHAN_SRC_SHIFT    4
#define DMA_CHAN_SRC_M        3
#define DMA_CHAN_SRC_MASK     (DMA_CHAN_SRC_M << DMA_CHAN_SRC_SHIFT)
#define DMA_CHAN_DST_DRAM     0
#define DMA_CHAN_DST_LBP      1
#define DMA_CHAN_DST_STP      2
#define DMA_CHAN_DST_MIPI_OUT 3
#define DMA_CHAN_DST_SHIFT    6
#define DMA_CHAN_DST_M        3
#define DMA_CHAN_DST_MASK     (DMA_CHAN_DST_M << DMA_CHAN_DST_SHIFT)
#define DMA_CHAN_ADDR_MODE_ABSTRACT (0 << 8)
#define DMA_CHAN_ADDR_MODE_PHYSICAL (1 << 8)
#define DMA_CHAN_GATHER       (1 << 9)

#define DMA_CHAN_SRC_DEF      DMA_CHAN_SRC_DRAM
#define DMA_CHAN_DST_DEF      DMA_CHAN_DST_LBP

/* DMA_CHAN_IMG_FORMAT */
#define DMA_CHAN_COMPONENTS_MASK 0x03
#define DMA_CHAN_PLANES_M     0x3F
#define DMA_CHAN_PLANES_SHIFT 4
#define DMA_CHAN_PLANES_MASK (DMA_CHAN_PLANES_M << DMA_CHAN_PLANES_SHIFT)
#define DMA_CHAN_BIT_DEPTH8   0
#define DMA_CHAN_BIT_DEPTH10  1
#define DMA_CHAN_BIT_DEPTH12  2
#define DMA_CHAN_BIT_DEPTH14  3
#define DMA_CHAN_BIT_DEPTH16  4
#define DMA_CHAN_BIT_DEPTH_M 0x7
#define DMA_CHAN_BIT_DEPTH_SHIFT 12
#define DMA_CHAN_BIT_DEPTH_MASK (DMA_CHAN_BIT_DEPTH_M <<                       \
		DMA_CHAN_BIT_DEPTH_SHIFT)
#define DMA_CHAN_MIPI_RAW_FORMAT (1 << 20)
#define DMA_CHAN_RGBA_FORMAT_DISABLED 0
#define DMA_CHAN_RGBA_FORMAT_RGBA 1
#define DMA_CHAN_RGBA_FORMAT_ARGB 2
#define DMA_CHAN_RGBA_FORMAT_M 0x03
#define DMA_CHAN_RGBA_FORMAT_SHIFT 24
#define DMA_CHAN_RGBA_FORMAT_MASK (DMA_CHAN_RGBA_FORMAT_M <<                   \
		DMA_CHAN_RGBA_FORMAT_SHIFT)
#define DMA_CHAN_BLOCK_4X4    (1 << 26)

#define DMA_CHAN_MIN_COMPONENTS 1
#define DMA_CHAN_MAX_COMPONENTS 4
#define DMA_CHAN_MIN_PLANES   1
#define DMA_CHAN_MAX_PLANES   64

/* DMA_CHAN_IMG_SIZE Register Bits */
#define DMA_CHAN_IMG_SIZE_MASK  0xFFFF
#define DMA_CHAN_IMG_WIDTH_MASK DMA_CHAN_IMG_SIZE_MASK
#define DMA_CHAN_IMG_HEIGHT_SHIFT 16
#define DMA_CHAN_IMG_HEIGHT_MASK (DMA_CHAN_IMG_SIZE_MASK <<                    \
		DMA_CHAN_IMG_HEIGHT_SHIFT)

#define DMA_CHAN_IMG_SIZE_MAX  0xFFFF

#define DMA_CHAN_IMG_WIDTH_DEF 128
#define DMA_CHAN_IMG_HEIGHT_DEF 32

/* DMA_CHAN_IMG_POS_L Register Bits */
#define DMA_CHAN_START_X_MASK  0xFFFF
#define DMA_CHAN_START_Y_M     0xFFFF
#define DMA_CHAN_START_Y_SHIFT 16
#define DMA_CHAN_START_Y_MASK  (DMA_CHAN_START_Y_M << DMA_CHAN_START_Y_SHIFT)

#define DMA_CHAN_START_MAX     32767
#define DMA_CHAN_START_MIN     -32767

/* DMA_CHAN_IMG_POS_H Register Bits */
#define DMA_CHAN_LB_START_X_MASK  0xFFFF
#define DMA_CHAN_LB_START_Y_M     0xFFFF
#define DMA_CHAN_LB_START_Y_SHIFT 16
#define DMA_CHAN_LB_START_Y_MASK  (DMA_CHAN_LB_START_Y_M <<                    \
		DMA_CHAN_LB_START_Y_SHIFT)

#define DMA_CHAN_LB_START_MAX     32767
#define DMA_CHAN_LB_START_MIN     -32767

/* DMA CHAN_IMG_POS_H STP DRAM Register Bits */
#define DMA_CHAN_LB_START_Y_STP_IRAM        (0 << 2)
#define DMA_CHAN_LB_START_Y_STP_CRAM        (1 << 2)
#define DMA_CHAN_LB_START_Y_STP_DRAM        (2 << 2)
#define DMA_CHAN_LB_START_Y_STP_ARRAY_16x16 (3 << 2)
#define DMA_CHAN_LB_START_Y_STP_ARRAY_32x32 (4 << 2)

#define DMA_STP_SRAM_ADDR_ALIGN_MASK 0x1F

/* DMA_CHAN_IMG_LAYOUT */
#define DMA_CHAN_PLANE_STRIDE_WIDTH 35
#define DMA_CHAN_PLANE_STRIDE_MAX   ((1ULL << DMA_CHAN_PLANE_STRIDE_WIDTH) - 1)

/* DMA_CHAN_IMG_LAYOUT_L Register Bits */
#define DMA_CHAN_ROW_STRIDE_WIDTH 16
#define DMA_CHAN_ROW_STRIDE_MAX   ((1 << DMA_CHAN_ROW_STRIDE_WIDTH) - 1)
#define DMA_CHAN_ROW_STRIDE_MASK DMA_CHAN_ROW_STRIDE_MAX
#define DMA_CHAN_PLANE_STRIDE_LOW_SHIFT 16
#define DMA_CHAN_PLANE_STRIDE_LOW_WIDTH 16
#define DMA_CHAN_PLANE_STRIDE_LOW_M ((1 << DMA_CHAN_PLANE_STRIDE_LOW_WIDTH) - 1)
#define DMA_CHAN_PLANE_STRIDE_LOW_MASK (DMA_CHAN_PLANE_STRIDE_LOW_M <<         \
		DMA_CHAN_PLANE_STRIDE_LOW_SHIFT)

/* DMA_CHAN_IMG_LAYOUT_H Register Bits */
#define DMA_CHAN_PLANE_STRIDE_HIGH_WIDTH (DMA_CHAN_PLANE_STRIDE_WIDTH -        \
		DMA_CHAN_PLANE_STRIDE_LOW_WIDTH)
#define DMA_CHAN_PLANE_STRIDE_HIGH_M                                           \
		((1 << DMA_CHAN_PLANE_STRIDE_HIGH_WIDTH) - 1)
#define DMA_CHAN_PLANE_STRIDE_HIGH_MASK DMA_CHAN_PLANE_STRIDE_HIGH_M

/* DMA_CHAN_BIF_XFER Register Bits */
#define DMA_CHAN_STRIPE_HEIGHT_MASK 0xFFFF
#define DMA_CHAN_OUTSTANDING_M  0x3F
#define DMA_CHAN_OUTSTANDING_SHIFT 16
#define DMA_CHAN_OUTSTANDING_MASK (DMA_CHAN_OUTSTANDING_M <<                   \
		DMA_CHAN_OUTSTANDING_SHIFT)

#define DMA_CHAN_MAX_STRIPES     256

#define DMA_CHAN_OUTSTANDING_DEF  16
#define DMA_CHAN_STRIPE_HEIGHT_DEF 0x04

/* DMA_CHAN_VA_BDRY */
#define DMA_VA_BDRY_WIDTH 41
#define DMA_MAX_IMG_TRANSFER_LEN ((1ULL << DMA_VA_BDRY_WIDTH) - 1)

/* DMA_CHAN_NOC_XFER_L Register Bits */
#define DMA_CHAN_SHEET_WIDTH_MASK 0x1FF
#define DMA_CHAN_SHEET_HEIGHT_M 0x1F
#define DMA_CHAN_SHEET_HEIGHT_SHIFT 12
#define DMA_CHAN_SHEET_HEIGHT_MASK (DMA_CHAN_SHEET_HEIGHT_M <<                 \
		DMA_CHAN_SHEET_HEIGHT_SHIFT)
#define DMA_CHAN_NOC_OUTSTANDING_M 0xF
#define DMA_CHAN_NOC_OUTSTANDING_SHIFT 20
#define DMA_CHAN_NOC_OUTSTANDING_MASK (DMA_CHAN_NOC_OUTSTANDING_M <<           \
		DMA_CHAN_NOC_OUTSTANDING_SHIFT)

#define DMA_CHAN_MAX_SHEET_WIDTH 256
#define DMA_CHAN_MAX_SHEET_HEIGHT 16

#define DMA_CHAN_SHEET_WIDTH_DEF  64
#define DMA_CHAN_SHEET_HEIGHT_DEF 4

#define DMA_CHAN_NOC_OUTSTANDING_MIN 1
#define DMA_CHAN_NOC_OUTSTANDING_MAX 8
#define DMA_CHAN_NOC_OUTSTANDING_DEF 8

/* DMA_CHAN_NOC_XFER_H Register Bits */
#define DMA_CHAN_RETRY_INTERVAL_MASK 0x3FF

#define DMA_CHAN_RETRY_INTERVAL_MAX 0x3FF

#define DMA_CHAN_RETRY_INTERVAL_DEF 100

/* DMA_CHAN_NODE Register Bits */
#define DMA_CHAN_CORE_ID_MASK 0xFF
#define DMA_CHAN_LB_ID_WIDTH  0xFF
#define DMA_CHAN_LB_ID_SHIFT  8
#define DMA_CHAN_LB_ID_MASK   (DMA_CHAN_LB_ID_WIDTH << DMA_CHAN_LB_ID_SHIFT)
#define DMA_CHAN_RPTR_ID_WIDTH 0xFF
#define DMA_CHAN_RPTR_ID_SHIFT 16
#define DMA_CHAN_RPTR_ID_MASK   (DMA_CHAN_RPTR_ID_WIDTH <<                     \
		DMA_CHAN_RPTR_ID_SHIFT)

/* DMA_CHAN_IMR and DMA_CHAN_ISR Register Bits */
#define DMA_CHAN_INT_EOF      (1 << 0)
#define DMA_CHAN_INT_VA_ERR   (1 << 2)

/* DMA_STS_CTRL Register Bits */
#define DMA_STS_ENABLE        (1 << 0)
#define DMA_STS_CHAN_SEL_SHIFT 1
#define DMA_STS_CHAN_SEL_M     0xF
#define DMA_STS_CHAN_SEL_MASK  (DMA_STS_CHAN_SEL_M << DMA_STS_CHAN_SEL_SHIFT)
#define DMA_STS_DST_BIF_SEL_SSP (1 << 5)
#define DMA_STS_ADDR_DIR      (1 << 6)

/* DMA_STS_STATE Register Bits */
#define DMA_STS_SRC_MASK       0xF
#define DMA_STS_DST_SHIFT      4
#define DMA_STS_DST_M          0xF
#define DMA_STS_DST_MASK       (DMA_STS_DST_M << DMA_STS_DST_SHIFT)
#define DMA_STS_NOC_CREDITS_SHIFT 8
#define DMA_STS_NOC_CREDITS_M  0x0F
#define DMA_STS_NOC_CREDITS_MASK (DMA_STS_NOC_CREDITS_M <<                     \
		DMA_STS_NOC_CREDITS_SHIFT
#define DMA_STS_SRC_MODE       (1 << 12)
#define DMA_STS_DST_MODE       (1 << 13)
#define DMA_STS_NOC_CREDITS_MODE (1 << 14)

/* DMA_STS_PTR_L */
#define DMA_STS_X_MASK         0xFFFF
#define DMA_STS_Y_M            0xFFFF
#define DMA_STS_Y_SHIFT        16
#define DMA_STS_Y_MASK         (DMA_STS_Y_M << DMA_STS_Y_SHIFT)

/* DMS_STS_PTR_H */
#define DMA_STS_WIDTH_MASK     0x1FF
#define DMA_STS_HEIGHT_SHIFT   12
#define DMA_STS_HEIGHT_M       0x1F
#define DMA_STS_HEIGHT_MASK    (DMA_STS_HEIGHT_M << DMA_STS_HEIGHT_SHIFT)
#define DMA_STS_PTR_DIR        (1 << 31)

/* LBP Register Group Offsets */

/* Start of block for line buffer pool management registers */
#define LBP_SEL               0x00
#define LBP_CTRL_L            0x08
#define LBP_CTRL_H            0x0C
#define LBP_STAT              0x10
#define LBP_CAP0              0x18
#define LBP_CAP1              0x20
#define LBP_RAM_CTRL          0x28
#define LBP_RAM_DATA0         0x30
#define LBP_RAM_DATA0_H       0x34
#define LBP_RAM_DATA1         0x38
#define LBP_RAM_DATA1_H       0x3C
#define LBP_RAM_DATA2         0x40
#define LBP_RAM_DATA2_H       0x44
#define LBP_RAM_DATA3         0x48
#define LBP_RAM_DATA3_H       0x4C
#define LBP_PMON_CFG          0x50
#define LBP_PMON_CNT_0_CFG    0x58
#define LBP_PMON_CNT_0        0x60
#define LBP_PMON_CNT_0_STS    0x70
#define LBP_PMON_CNT_1_CFG    0x78
#define LBP_PMON_CNT_1        0x80
#define LBP_PMON_CNT_1_STS    0x90

/* Block length for line buffer pool management registers */
#define LBP_POOL_BLOCK_LEN    0x98
#define LBP_POOL_NUM_REGS     (LBP_POOL_BLOCK_LEN / IPU_REG_WIDTH)

/* Start of block for line buffer management registers */
#define LB_BLOCK_START        0xC0
#define LB_CTRL0_L            0xC0
#define LB_CTRL0_H            0xC4
#define LB_OFFSET_L           0xC8
#define LB_OFFSET_H           0xCC
#define LB_BDRY               0xD0
#define LB_IMG_SIZE           0xD8
#define LB_SB_SIZE            0xE0
#define LB_BASE               0xE8
#define LB_STAT               0xF0
#define LB_L_PARAM            0xF8
#define LBP_BLOCK_LEN         0x100
#define LBP_NUM_REGS          (LBP_BLOCK_LEN / IPU_REG_WIDTH)

/* Block length for just the line buffer registers */
#define LB_BLOCK_LEN          (LBP_BLOCK_LEN - LB_BLOCK_START)
#define LB_NUM_REGS           (LB_BLOCK_LEN / IPU_REG_WIDTH)

/* LBP_SEL Register Bits */
#define LBP_LBP_SEL_MASK      0x000000FF
#define LBP_LB_SEL_MASK       0x0000FF00
#define LBP_LB_SEL_SHIFT      8

#define LBP_LBP_SEL_DEF       0x0F
#define LBP_LB_SEL_DEF        0x0F

/* LBP_CTRL_L Register Bits */
#define LBP_LB_RESET_MASK     0xFFFF0000
#define LBP_LB_RESET_SHIFT    16
#define LBP_LBP_RESET         (1 << 4)
#define LBP_LB_ENA_MASK       0x0000000F

#define LBP_LB_ENA_DEF        8

/* LBP_CTRL_H Register Bits */
#define LBP_LB_INIT_MASK      0x000000FF

/* LBP_CAP0 Register Bits */
#define LBP_MAX_RPTR          0xF
#define LBP_MAX_RPTR_SHIFT    28
#define LBP_MAX_RPTR_MASK     (LBP_MAX_RPTR << LBP_MAX_RPTR_SHIFT)
#define LBP_MAX_FB_ROWS       0x0FFF
#define LBP_MAX_FB_ROWS_SHIFT 16
#define LBP_MAX_FB_ROWS_MASK  (LBP_MAX_FB_ROWS << LBP_MAX_FB_ROWS_SHIFT)
#define LBP_MAX_CHAN          0x1FF
#define LBP_MAX_CHAN_SHIFT    4
#define LBP_MAX_CHAN_MASK     (LBP_MAX_CHAN << LBP_MAX_CHAN_SHIFT)
#define LBP_MAX_LB            0xF
#define LBP_MAX_LB_MASK       LBP_MAX_LB

/* LBP_RAM_CTRL Register Bits */
#define LBP_RAM_ADDR          0xFFFF
#define LBP_RAM_ADDR_SHIFT    16
#define LBP_RAM_ADDR_MASK     (LBP_RAM_ADDR << LBP_RAM_ADDR_SHIFT)

#define LBP_RAM_WRITE         (1 << 1)
#define LBP_RAM_RUN           (1 << 0)

#define LBP_DATA_REG_COUNT    8

/* LB_CTRL0_L Register Bits */
#define LB_FB_ROWS_M          0xFFF
#define LB_FB_ROWS_SHIFT      16
#define LB_FB_ROWS_MASK       (LB_FB_ROWS_M << LB_FB_ROWS_SHIFT)
#define LB_NUM_CHAN_M         0x1FF
#define LB_NUM_CHAN_SHIFT     4
#define LB_NUM_CHAN_MASK      (LB_NUM_CHAN_M << LB_NUM_CHAN_SHIFT)
#define LB_NUM_RPTR_MASK      0xF

#define LB_FB_ROWS_DEF        40
#define LB_NUM_RPTR_DEF       1
#define LB_NUM_CHAN_DEF       2

/* LB_CTRL0_H Register Bits */
#define LB_REUSE_ROWS_MASK       0x1F
#define LB_REUSE_ROWS_MAX        0x1F

/* LB_OFFSET_L Register Bits */
#define LB_OFFSET_X_MASK      0x0000FFFF
#define LB_OFFSET_Y_M         0xFFFF
#define LB_OFFSET_Y_SHIFT     16
#define LB_OFFSET_Y_MASK      (LB_OFFSET_Y_M << LB_OFFSET_Y_SHIFT)

#define LB_OFFSET_MIN         -32767
#define LB_OFFSET_MAX         32767

/* LB_OFFSET_H Register Bits */
#define LB_OFFSET_CHAN_WIDTH  9
#define LB_OFFSET_CHAN_MASK   ((1 << LB_OFFSET_CHAN_WIDTH) - 1)
#define LB_FB_OFFSET_M        0xFF
#define LB_FB_OFFSET_SHIFT    16
#define LB_FB_OFFSET_MASK     (LB_FB_OFFSET_M << LB_FB_OFFSET_SHIFT)

#define LB_FB_OFFSET_MIN      -128
#define LB_FB_OFFSET_MAX      127

#define LB_OFFSET_CHAN_MAX    ((1 << LB_OFFSET_CHAN_WIDTH) - 1)

/* LB_BDRY Register Bits */
#define LB_BDRY_VAL_MASK      0xFFFF0000
#define LB_BDRY_VAL_SHIFT     16
#define LB_BDRY_MASK          0x00000003
#define LB_BDRY_CLAMP         0x00000000
#define LB_BDRY_REPEAT        0x00000001
#define LB_BDRY_REFLECT       0x00000002
#define LB_BDRY_RESERVED      0x00000003

/* LB_IMG_SIZE Register Bits */
#define LB_IMG_HEIGHT_MASK    0xFFFF0000
#define LB_IMG_HEIGHT_SHIFT   16
#define LB_IMG_WIDTH_MASK     0x0000FFFF

#define LB_IMG_HEIGHT_DEF     480
#define LB_IMG_WIDTH_DEF      640

/* LB_SB_SIZE Register Bits */
#define LB_SB_ROWS_MASK       0xFFFF0000
#define LB_SB_ROWS_SHIFT      16
#define LB_SB_COLS_MASK       0x0000FFFF

/* LB_BASE Register Bits */
#define LB_SB_BASE_ADDR_MASK  0xFFFF0000
#define LB_SB_BASE_ADDR_SHIFT 16
#define LB_FB_BASE_ADDR_MASK  0x0000FFFF
#define LB_ADDR_ALIGN_MASK    0x1F
#define LB_ADDR_ALIGN_SHIFT   5

/* LB_STAT Register Bits */
#define LB_STAT_FULL          (1 << 0)
#define LB_STAT_EMPTY0        (1 << 1)
#define LB_STAT_EMPTY1        (1 << 2)
#define LB_STAT_EMPTY2        (1 << 3)

/* LB_L_PARAM Register Bits */
#define LB_L_INC_MASK         0x00000FFF
#define LB_L_WIDTH_M          0x0FFF
#define LB_L_WIDTH_SHIFT      16
#define LB_L_WIDTH_MASK       (LB_L_WIDTH_M << LB_L_WIDTH_SHIFT)

#define LB_L_INC_WIDTH        12
#define LB_L_WIDTH_WIDTH      12
#define LB_L_INC_MAX          ((1 << LB_L_INC_WIDTH) - 1)
#define LB_L_WIDTH_MAX        ((1 << LB_L_WIDTH_WIDTH) - 1)

/* STP Register Group Offsets */
#define STP_SEL               0x00
#define STP_CTRL              0x08
#define STP_STAT_L            0x10
#define STP_STAT_H            0x14
#define STP_CAP_L             0x18
#define STP_CAP_H             0x1C
#define STP_RAM_CTRL          0x20
#define STP_RAM_DATA0_L       0x28
#define STP_RAM_DATA0_H       0x2C
#define STP_RAM_DATA1_L       0x30
#define STP_RAM_DATA1_H       0x34
#define STP_PMON_CFG          0x38
#define STP_PMON_CNT_0_CFG    0x40
#define STP_PMON_CNT_0        0x48
#define STP_PMON_CNT_0_STS    0x58
#define STP_PMON_CNT_1_CFG    0x60
#define STP_PMON_CNT_1        0x68
#define STP_PMON_CNT_1_STS    0x78
#define STP_BLOCK_LEN         0x80
#define STP_NUM_REGS          (STP_BLOCK_LEN / IPU_REG_WIDTH)

/* STP_SEL Register Bits */
#define STP_SEL_MASK          0x000000FF

#define STP_SEL_DEF           0x0F

/* STP_CTRL Register Bits */
#define STP_ENA               (1 << 0)
#define STP_RESET             (1 << 1)
#define STP_RESUME            (1 << 2)
#define STP_INT               (1 << 3)
#define STP_LBP_MASK          0xFFFF
#define STP_LBP_MASK_SHIFT    16
#define STP_LBP_MASK_MASK     (STP_LBP_MASK << STP_LBP_MASK_SHIFT)

#define STP_LBP_MASK_DEF      0xFFFF

/* STP_STAT_L Register Bits */
#define STP_INT_CODE          0xFFFF
#define STP_INT_CODE_SHIFT    16
#define STP_INT_CODE_MASK     (STP_INT_CODE << STP_INT_CODE_SHIFT)
#define STP_PC_MASK           0x7FF

/* STP_STAT_H Register Bits */
#define STP_STALLED           (1 << 0)

/* STP_CAP_L Register Bits */
#define STP_SCALAR_MEM_M      0xFFFF
#define STP_SCALAR_MEM_SHIFT  16
#define STP_SCALAR_MEM_MASK   (STP_SCALAR_MEM_M << STP_SCALAR_MEM_SHIFT)
#define STP_INST_MEM_MASK     0xFFFF

/* STP_CAP_H Register Bits */
#define STP_HALO_MEM_M        0xFF
#define STP_HALO_MEM_SHIFT    24
#define STP_HALO_MEM_MASK     (STP_HALO_MEM_M << STP_HALO_MEM_SHIFT)
#define STP_VECTOR_MEM_M      0xFF
#define STP_VECTOR_MEM_SHIFT  16
#define STP_VECTOR_MEM_MASK   (STP_VECTOR_MEM_M << STP_VECTOR_MEM_SHIFT)
#define STP_CONST_MEM_MASK    0xFFFF

/* STP_RAM_CTRL Register Bits */
#define STP_RAM_ADDR          0xFFFF
#define STP_RAM_ADDR_SHIFT    16
#define STP_RAM_ADDR_MASK     (STP_RAM_ADDR << STP_RAM_ADDR_SHIFT)

/* Vector Memory Address Bits */
#define STP_RAM_ADDR_ROW      0x0F
#define STP_RAM_ADDR_ROW_SHIFT 7
#define STP_RAM_ADDR_ROW_MASK (STP_RAM_ADDR_ROW << STP_RAM_ADDR_ROW_SHIFT)
#define STP_RAM_ADDR_OFFSET_MASK 0x7F

#define STP_RAM_TARG          0x0F
#define STP_RAM_TARG_SHIFT    8
#define STP_RAM_TARG_MASK     (STP_RAM_TARG << STP_RAM_TARG_SHIFT)
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

#define STP_RAM_WRITE         (1 << 1)
#define STP_RAM_RUN           (1 << 0)

#define STP_DATA_REG_COUNT    4

/* Common Data Register Offsets and Bits */
#define COMMON_RAM_CTRL       0
#define COMMON_RAM_DATA_START 8

#define COMMON_RAM_ADDR       0xFFFF
#define COMMON_RAM_ADDR_SHIFT 16
#define COMMON_RAM_ADDR_MASK  (COMMON_RAM_ADDR << COMMON_RAM_ADDR_SHIFT)

#define COMMON_RAM_WRITE      (1 << 1)
#define COMMON_RAM_RUN        (1 << 0)

#endif /* __PAINTBOX_REGS_H__ */
