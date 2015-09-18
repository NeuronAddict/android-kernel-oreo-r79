/*
 * Paintbox programmable IPU Utilities
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

#ifndef __PAINTBOX_UTILS_H__
#define __PAINTBOX_UTILS_H__

#include <linux/io.h>

#include "paintbox-common.h"

#ifdef DEBUG
void dump_registers(struct paintbox_data *pb, void __iomem *reg_start,
		size_t reg_count, const char *msg);

#define DUMP_REGISTERS(pb, reg, reg_count, msg)	\
do {							\
	dump_registers(pb, reg, reg_count, msg);	\
} while (0)

#else
#define DUMP_REGISTERS(pb, reg, reg_count, msg)	\
do { } while (0)
#endif


int write_data_common(struct paintbox_data *pb, const void __user *user_buf,
		size_t len_bytes, uint16_t ram_addr, uint32_t ram_ctrl_mask,
		void __iomem *reg_base, unsigned int reg_count);

int read_data_common(struct paintbox_data *pb, void __user *user_buf,
		size_t len_bytes, uint16_t ram_addr, uint32_t ram_ctrl_mask,
		void __iomem *reg_base, unsigned int reg_count);

int poll_memory_transfer_complete(void __iomem *ram_ctrl_reg);

int alloc_and_copy_from_user(struct paintbox_data *pb, uint8_t **buf,
			const void __user *user_buf, size_t len_bytes);

#endif /* __PAINTBOX_UTILS_H__ */
