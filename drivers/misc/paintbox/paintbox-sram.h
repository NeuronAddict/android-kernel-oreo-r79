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

int poll_memory_transfer_complete(void __iomem *ram_ctrl_reg);

int alloc_and_copy_from_user(struct paintbox_data *pb, uint8_t **buf,
			const void __user *user_buf, size_t len_bytes);

void write_ram_data_registers(struct paintbox_data *pb, const uint8_t *buf,
		void __iomem *data_reg, unsigned int reg_count);
void write_ram_data_registers_swapped(struct paintbox_data *pb,
		const uint8_t *buf, void __iomem *data_reg,
		unsigned int reg_count);
void read_ram_data_registers(struct paintbox_data *pb, uint8_t *buf,
		void __iomem *data_reg, unsigned int reg_count);
void read_ram_data_registers_swapped(struct paintbox_data *pb, uint8_t *buf,
		void __iomem *data_reg, unsigned int reg_count);

#endif /* __PAINTBOX_SRAM_H__ */
