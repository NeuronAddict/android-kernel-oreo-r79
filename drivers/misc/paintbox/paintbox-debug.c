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

#include <linux/debugfs.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/types.h>

#include "paintbox-common.h"
#include "paintbox-debug.h"
#include "paintbox-regs.h"


#define REG_NAME_COLUMN_NUMBER 8
#define REG_VALUE_COLUMN_NUMBER 44

int dump_ipu_vprintf(struct paintbox_data *pb, char *buf, int *written,
		size_t len, const char *format, va_list args)
{
	int ret;

	ret = vsnprintf(buf + *written, len - *written, format, args);

	if (ret < 0) {
		dev_err(&pb->pdev->dev, "%s: dump printf error, err = %d",
				__func__, ret);
		return ret;
	}

	*written += ret;

	return ret;
}

int dump_ipu_printf(struct paintbox_data *pb, char *buf, int *written,
		size_t len, const char *format, ...)
{
	va_list args;
	int ret;

	va_start(args, format);

	ret = dump_ipu_vprintf(pb, buf, written, len, format, args);

	va_end(args);

	if (ret < 0) {
		dev_err(&pb->pdev->dev, "%s: dump printf error, err = %d",
				__func__, ret);
		return ret;
	}

	return ret;
}

int dump_ipu_register(struct paintbox_data *pb, void __iomem *group_base,
		uint32_t reg_offset, const char *reg_name, char *buf,
		int *written, size_t len)
{
	return dump_ipu_printf(pb, buf, written, len, "0x%04lx: %-*s0x%08x\n",
			group_base - pb->reg_base + reg_offset,
			REG_VALUE_COLUMN_NUMBER - REG_NAME_COLUMN_NUMBER,
			reg_name ? reg_name : REG_UNUSED,
			readl(group_base + reg_offset));
}

#ifdef CONFIG_DEBUG_FS
static int debug_regs_show(struct seq_file *s, void *unused)
{
	struct paintbox_debug *debug = s->private;
	struct paintbox_data *pb = debug->pb;
	char *buf;
	size_t len;
	int ret, written;

	len = seq_get_buf(s, &buf);
	if (!buf)
		return -ENOMEM;

	mutex_lock(&pb->lock);

	if (debug->resource_id >= 0)
		ret = snprintf(buf, len, "%s%u:\n", debug->name,
				debug->resource_id);
	else
		ret = snprintf(buf, len, "%s:\n", debug->name);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		dev_err(&pb->pdev->dev, "%s: error dumping registers %d",
				__func__, ret);
		return ret;
	}

	written = ret;

	ret = debug->register_dump(debug, buf + written, len - written);

	mutex_unlock(&pb->lock);

	if (ret < 0)
		return ret;

	written += ret;

	seq_commit(s, written);

	return 0;
}

static int debug_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, debug_regs_show, inode->i_private);
}

static const struct file_operations debug_regs_fops = {
	.open = debug_regs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

void paintbox_debug_create_entry(struct paintbox_data *pb,
		struct paintbox_debug *debug, struct dentry *debug_root,
		const char *name, unsigned int resource_id,
		register_dump_t register_dump, void *arg)
{
	char resource_name[RESOURCE_NAME_LEN];
	int ret;

	debug->pb = pb;
	debug->name = name;
	debug->resource_id = resource_id;
	debug->register_dump = register_dump;

	if (debug->resource_id >= 0)
		ret = snprintf(resource_name, RESOURCE_NAME_LEN, "%s%u", name,
				resource_id);
	else
		ret = snprintf(resource_name, RESOURCE_NAME_LEN, "%s", name);
	if (ret < 0) {
		dev_err(&pb->pdev->dev, "%s: error creating debugs entry %d\b",
				__func__, ret);
		return;
	}

	debug->debug_dir = debugfs_create_dir(resource_name, debug_root);
	if (IS_ERR(debug->debug_dir)) {
		dev_err(&pb->pdev->dev, "%s: err = %ld", __func__,
				PTR_ERR(debug->debug_dir));
		return;
	}

	debug->reg_dump_dentry = debugfs_create_file("regs", S_IRUSR | S_IRGRP |
			S_IWUSR, debug->debug_dir, debug, &debug_regs_fops);
	if (IS_ERR(debug->reg_dump_dentry)) {
		dev_err(&pb->pdev->dev, "%s: err = %ld", __func__,
				PTR_ERR(debug->reg_dump_dentry));
		return;
	}
}

static int reg_entry_show(struct seq_file *s, void *p)
{
	struct paintbox_debug_reg_entry *reg_entry = s->private;
	seq_printf(s, "0x%08x\n", reg_entry->read(reg_entry));
	return 0;
}

static int reg_entry_open(struct inode *inode, struct file *file)
{
	return single_open(file, reg_entry_show, inode->i_private);
}

static ssize_t reg_entry_write(struct file *file, const char __user *user_buf,
		size_t count, loff_t *ppos)
{
	struct seq_file *s = (struct seq_file *)file->private_data;
	struct paintbox_debug_reg_entry *reg_entry = s->private;
	struct paintbox_debug *debug = reg_entry->debug;
	struct paintbox_data *pb = debug->pb;
	uint32_t val;
	int ret;

	ret = kstrtou32_from_user(user_buf, count, 0, &val);
	if (ret < 0) {
		dev_err(&pb->pdev->dev, "%s: invalid value, err = %d",
				__func__, ret);
		return ret;
	}

	reg_entry->write(reg_entry, val);

	return count;
}

static const struct file_operations reg_entry_fops = {
	.open = reg_entry_open,
	.write = reg_entry_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.owner = THIS_MODULE,
};

int paintbox_debug_alloc_reg_entries(struct paintbox_data *pb,
		struct paintbox_debug *debug, size_t reg_count)
{
	debug->reg_entries = kzalloc(sizeof(struct paintbox_debug_reg_entry) *
			reg_count, GFP_KERNEL);
	if (!debug->reg_entries) {
		dev_err(&pb->pdev->dev, "%s: allocation failure", __func__);
		debug->num_reg_entries = 0;
		return -ENOMEM;
	}

	debug->num_reg_entries = reg_count;

	return 0;
}

void paintbox_debug_free_reg_entries(struct paintbox_debug *debug)
{
	unsigned int i;

	for (i = 0; i < debug->num_reg_entries; i++)
		debugfs_remove(debug->reg_entries[i].debug_dentry);

	kfree(debug->reg_entries);

	debug->reg_entries = NULL;
	debug->num_reg_entries = 0;
}

int paintbox_debug_create_reg_entry(struct paintbox_data *pb,
		struct paintbox_debug *debug, unsigned int index,
		const char *reg_name, uint32_t reg_offset,
		register_write_t reg_write, register_read_t reg_read)
{
	struct paintbox_debug_reg_entry *reg_entry = &debug->reg_entries[index];

	reg_entry->debug_dentry = debugfs_create_file(reg_name,
			S_IRUSR | S_IRGRP | S_IWUSR, debug->debug_dir,
			reg_entry, &reg_entry_fops);
	if (IS_ERR(reg_entry->debug_dentry)) {
		dev_err(&pb->pdev->dev, "%s: err = %ld",__func__,
				PTR_ERR(reg_entry->debug_dentry));
		return PTR_ERR(reg_entry->debug_dentry);
	}

	reg_entry->reg_offset = reg_offset;
	reg_entry->write = reg_write;
	reg_entry->read = reg_read;
	reg_entry->debug = debug;

	return 0;
}

void paintbox_debug_create_reg_entries(struct paintbox_data *pb,
		struct paintbox_debug *debug, const char **reg_names,
		size_t reg_count, register_write_t reg_write,
		register_read_t reg_read)
{
	int i, ret;

	ret = paintbox_debug_alloc_reg_entries(pb, debug, reg_count);
	if (ret < 0)
		return;

	for (i = 0; i < reg_count; i++) {
		if (!reg_names[i])
			continue;

		ret = paintbox_debug_create_reg_entry(pb, debug, i,
				reg_names[i], i * IPU_REG_WIDTH, reg_write,
				reg_read);
		if (ret < 0) {
			paintbox_debug_free_reg_entries(debug);
			return;
		}
	}
}
#endif
