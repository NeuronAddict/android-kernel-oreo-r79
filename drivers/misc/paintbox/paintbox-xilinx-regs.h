/*
 * Xilinx DMA Register Definitions
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

#ifndef __PAINTBOX_XILINX_REGS_H__
#define __PAINTBOX_XILINX_REGS_H__

/* Xilinx DMA Offsets */
#define MM2S_OFFSET           0x00
#define MM2S_DMACR            0x00
#define MM2S_DMASR            0x04
#define MM2S_SA               0x18
#define MM2S_SA_MSB           0x1C
#define MM2S_LENGTH           0x28
#define S2MM_OFFSET           0x30
#define S2MM_DMACR            0x30
#define S2MM_DMASR            0x34
#define S2MM_DA               0x48
#define S2MM_DA_MSB           0x4C
#define S2MM_LENGTH           0x58
#define XILINX_BLOCK_LEN      0x5C
#define XILINX_NUM_REGS       (XILINX_BLOCK_LEN / sizeof(uint32_t))

/* Xilinx DMA Register Bits */
#define DMACR_RS              (1 << 0)
#define DMACR_RESET           (1 << 2)
#define DMACR_KEYHOLE         (1 << 3)
#define DMACR_CYCLIC_BD_EN    (1 << 4)
#define DMACR_IOC_IRQ_EN      (1 << 12)
#define DMACR_DLY_IRQ_EN      (1 << 13)
#define DMACR_ERR_IRQ_EN      (1 << 14)
#define DMACR_IRQ_THRESH_MASK 0x00FF0000
#define DMACR_IRQ_THRESH_SHIFT 16
#define DMACR_IRQ_DELAY_MASK  0xFF000000
#define DMACR_IRQ_DELAY_SHIFT 23

#define DMASR_HALTED          (1 << 0)
#define DMASR_IDLE            (1 << 1)
#define DMASR_SG_INCLD        (1 << 3)
#define DMASR_DMA_INT_ERR     (1 << 4)
#define DMASR_DMA_SLV_ERR     (1 << 5)
#define DMASR_DMA_DEC_ERR     (1 << 6)
#define DMASR_SG_INT_ERR      (1 << 8)
#define DMASR_SG_SLV_ERR      (1 << 9)
#define DMASR_SG_DEC_ERR      (1 << 10)
#define DMASR_IOC_IRQ         (1 << 12)
#define DMASR_DLY_IRQ         (1 << 13)
#define DMASR_ERR_IRQ         (1 << 14)
#define DMASR_IRQ_THRESH_MASK 0x00FF0000
#define DMASR_IRQ_THRESH_SHIFT 16
#define DMASR_IRQ_DELAY_MASK  0xFF000000
#define DMASR_IRQ_DELAY_SHIFT 23

#define DMASR_ERR_MASK (DMASR_DMA_INT_ERR | \
			DMASR_DMA_SLV_ERR | \
			DMASR_DMA_DEC_ERR | \
			DMASR_SG_INT_ERR  | \
			DMASR_SG_SLV_ERR  | \
			DMASR_SG_DEC_ERR)

#define XILINX_MAX_DMA_TRANSFER_LEN 2048

#endif /* __PAINTBOX_XILINX_REGS_H__ */
