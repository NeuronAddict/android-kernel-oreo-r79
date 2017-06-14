/*
 * Copyright (c) 2014--2016 Intel Corporation.
 *
 * Author: Teemu Rytkonen <teemu.s.rytkonen@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __MIPI_DEV__
#define __MIPI_DEV__

#include <linux/cdev.h>
#include <linux/mipibridge.h>
#include "mipicsi_chardev.h"

struct mipi_dev {
	/** Platform device node */
	struct platform_device		*pdev;
	/** Device node */
	struct device			*dev;
	/** Device list */
	struct list_head		devlist;
	/** Spinlock */
	spinlock_t			slock;
	/** Mutex */
	struct mutex			mutex;
	/** Device Tree Information */
	void __iomem			*base_address;
	uint32_t			mem_size;
	uint32_t			irq_number;
	enum mipicsi_top_dev		device_id;
	struct mipi_chardev		chardev;
	void				*data;
};

#endif
