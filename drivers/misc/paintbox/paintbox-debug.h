/*
 * Debug support for the Paintbox programmable IPU
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

#ifndef __PAINTBOX_DEBUG_H__
#define __PAINTBOX_DEBUG_H__

#include <linux/io.h>
#include <linux/types.h>

#include "paintbox-common.h"


int dump_ipu_vprintf(struct paintbox_data *pb, char *buf, int *written,
		size_t len, const char *format, va_list args);

int dump_ipu_printf(struct paintbox_data *pb, char *buf, int *written,
		size_t len, const char *format, ...);

int dump_ipu_register(struct paintbox_data *pb, void __iomem *group_base,
		uint32_t reg_offset, const char *reg_name, char *buf,
		int *written, size_t len);

int dump_ipu_register64(struct paintbox_data *pb, void __iomem *group_base,
		uint32_t reg_offset, const char *reg_name, char *buf,
		int *written, size_t len);

int dump_ipu_register_with_value(struct paintbox_data *pb,
		void __iomem *group_base, uint32_t reg_offset,
		uint32_t reg_value, const char *reg_name, char *buf,
		int *written, size_t len);

int dump_ipu_register_with_value64(struct paintbox_data *pb,
		void __iomem *group_base, uint32_t reg_offset,
		uint64_t reg_value, const char *reg_name, char *buf,
		int *written, size_t len);

void paintbox_debug_create_entry(struct paintbox_data *pb,
		struct paintbox_debug *debug, struct dentry *debug_root,
		const char *name, unsigned int resource_id,
		register_dump_t register_dump, stats_dump_t stats_dump,
		void *arg);

int paintbox_debug_alloc_reg_entries(struct paintbox_data *pb,
		struct paintbox_debug *debug, size_t reg_count);
void paintbox_debug_free_reg_entries(struct paintbox_debug *debug);

int paintbox_debug_create_reg_entry(struct paintbox_data *pb,
		struct paintbox_debug *debug, unsigned int index,
		const char *reg_name, uint32_t reg_offset,
		register_write_t reg_write, register_read_t reg_read);

void paintbox_debug_create_reg_entries(struct paintbox_data *pb,
		struct paintbox_debug *debug, const char **reg_names,
		size_t reg_count, register_write_t reg_write,
		register_read_t reg_read);

#endif /* __PAINTBOX_DEBUG_H__ */
