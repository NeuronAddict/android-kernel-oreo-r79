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

struct mipi_dev {
	/** Platform device node */
	struct platform_device	*pdev;
	/** Device node */
	struct device			*dev;
	/** Device list */
	struct list_head		devlist;
	/** Spinlock */
	spinlock_t				slock;
	/** Mutex */
	struct mutex			mutex;

	void __iomem			*base_address;
	uint32_t				mem_size;
	uint32_t				irq_number;
	struct mipi_chardev		chardev;
};


struct mipicsi_top_device {
	struct device	    *dev;     /* Device node */
	struct list_head    devlist;  /* Device list */
	spinlock_t          slock;    /* Spinlock */
	struct mutex        mutex;    /* Mutex */

	/** Device Tree Information */
	void __iomem      *base_address;
	uint32_t           mem_size;
	uint32_t           top_irq_number;
	uint32_t           top_irq;
	struct mipi_chardev		chardev;
	
};

#endif
