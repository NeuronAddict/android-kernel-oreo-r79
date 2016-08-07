/*
 * Paintbox programmable SRAM support
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

#ifndef __PAINTBOX_SRAM_H__
#define __PAINTBOX_SRAM_H__

#include <linux/io.h>

#include "paintbox-common.h"

#define RAM_DATA_MODE_NORMAL    0

/* TODO(ahampson):  Remove once b/30316979 is fixed.  RAM_DATA_MODE_SWAP is
 * needed because the assembler writes the instruction in reverse byte order
 * (due to an issue with the DV tools).
 */
#define RAM_DATA_MODE_SWAP      1

/* TODO(ahampson):  The conversion to the vector SRAM's column major lane
 * ordering needs to be moved to the runtime so it can be used for both DMA and
 * PIO.
 */
#define RAM_DATA_MODE_COL_MAJOR 2

#define VECTOR_SRAM_LANE_GROUP_SIMD_COLS 4
#define VECTOR_SRAM_LANE_GROUP_SIMD_ROWS 8
#define VECTOR_SRAM_LANE_GROUP_HALO_COLS 1
#define VECTOR_SRAM_LANE_GROUP_HALO_ROWS 2

#define VECTOR_LANE_WIDTH 2
#define VECTOR_LANE_GROUP_WIDTH 4 /* lanes */
#define VECTOR_LANE_GROUP_HEIGHT 2 /* lanes */

int sram_write_buffer(struct paintbox_data *pb, const uint8_t *buf,
		uint32_t sram_byte_addr, size_t len_bytes,
		uint32_t ram_ctrl_mask, void __iomem *reg_base,
		unsigned int reg_count);

int sram_write_user_buffer(struct paintbox_data *pb,
		const void __user *user_buf, uint32_t sram_byte_addr,
		size_t len_bytes, uint32_t ram_ctrl_mask,
		void __iomem *reg_base, unsigned int reg_count);

int sram_read_buffer(struct paintbox_data *pb, uint8_t *buf,
		uint32_t sram_byte_addr, size_t len_bytes,
		uint32_t ram_ctrl_mask, void __iomem *reg_base,
		unsigned int reg_count);

int sram_read_user_buffer(struct paintbox_data *pb, void __user *user_buf,
		uint32_t sram_byte_addr, size_t len_bytes,
		uint32_t ram_ctrl_mask, void __iomem *reg_base,
		unsigned int reg_count);

int sram_write_word(struct paintbox_data *pb, const uint8_t *buf,
		uint32_t ram_ctrl_mask, void __iomem *reg_base,
		size_t reg_count, unsigned int ram_data_mode);

int sram_write_word_partial(struct paintbox_data *pb, const uint8_t *buf,
		unsigned int byte_offset_in_word, size_t len_bytes,
		uint32_t ram_ctrl_mask, void __iomem *reg_base,
		size_t reg_count, unsigned int ram_data_mode);

int sram_read_word(struct paintbox_data *pb, uint8_t *buf,
		uint32_t ram_ctrl_mask, void __iomem *reg_base,
		size_t reg_count, unsigned int ram_data_mode);

int sram_read_word_partial(struct paintbox_data *pb, uint8_t *buf,
		unsigned int byte_offset_in_word, size_t len_bytes,
		uint32_t ram_ctrl_mask, void __iomem *reg_base,
		size_t reg_count, unsigned int ram_data_mode);

int alloc_and_copy_from_user(struct paintbox_data *pb, uint8_t **buf,
			const void __user *user_buf, size_t len_bytes);

#endif /* __PAINTBOX_SRAM_H__ */
