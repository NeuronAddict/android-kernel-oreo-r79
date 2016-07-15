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
struct mipi_host_operations {
	int (*poweron)(struct init_config *);
	int (*poweroff)(unsigned int);
	int (*writereg)(unsigned int, unsigned int);
	u32 (*readreg)(unsigned int);
};

struct mipi_device_operations {
	int (*poweron)(struct init_config *);
	int (*poweroff)(unsigned int);
	int (*writereg)(unsigned int, unsigned int);
	u32 (*readreg)(unsigned int);
};

struct mipi_top_operations {
	int (*set_rx_mux)(struct mipicsi_top_mux*);
	int (*get_rx_mux)(struct mipicsi_top_mux*);
	int (*set_tx_mux)(struct mipicsi_top_mux*);
	int (*get_tx_mux)(struct mipicsi_top_mux*);
	int (*writereg)(unsigned int, unsigned int, unsigned int);
	u32 (*readreg)(unsigned int, unsigned int);
};

struct mipi_chardev {
	struct module *owner;
	const char *deviceName;
	int open;
	struct cdev cdev;
	struct class *chardevClass;
	struct device *chardev;
	struct mipicsi_top_device *mipidev;
	struct mipi_host_operations hostOps;
	struct mipi_device_operations devOps;
	struct mipi_top_operations topOps;
};


/*
 * Create a character device
 */
int mipi_chardev_init(struct mipi_chardev *chardev);
void mipi_chardev_release(struct mipi_chardev *chardev);


#endif
