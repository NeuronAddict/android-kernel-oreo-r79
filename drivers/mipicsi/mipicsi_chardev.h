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

#ifndef __MIPICSI_CHARDEV__
#define __MIPICSI_CHARDEV__

#include <linux/cdev.h>
#include "mipicsi_top.h"

/*
 * Character device handling
 */

struct mipi_top_operations {
	int (*start)(struct mipicsi_top_cfg*);
	int (*stop)(enum mipicsi_top_dev);
	int (*set_mux)(struct mipicsi_top_mux*);
	int (*disable_mux)(struct mipicsi_top_mux*);
	void (*get_mux)(struct mipicsi_top_mux_data*);
	int (*get_mux_status)(struct mipicsi_top_mux*);
	int (*reset)(enum mipicsi_top_dev);
	int (*reset_all)(void);
	int (*writereg)(struct mipicsi_top_reg*);
	int (*readreg)(struct mipicsi_top_reg*);
	int (*vpg)(struct mipicsi_top_vpg*);
};

struct mipi_chardev {
	struct module *owner;
	const char *deviceName;
	int open;
	struct cdev cdev;
	struct class *chardevClass;
	struct device *chardev;
	struct mipicsi_top_device *mipidev;
	struct mipi_top_operations topOps;
};


/*
 * Create a character device
 */
int mipi_chardev_init(struct mipi_chardev *chardev);
void mipi_chardev_release(struct mipi_chardev *chardev);


#endif
