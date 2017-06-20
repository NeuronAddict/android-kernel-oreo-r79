/*
 * FPGA support for the Paintbox programmable IPU
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

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/types.h>

#include "paintbox-common.h"
#include "paintbox-fpga.h"

#define FPGA_SOFT_RESET             0x00
#define FPGA_SOFT_RESET_EN          (1 << 0)
#define FPGA_SOFT_RESET_HOLD_PERIOD 100 /* us */

/* This function must be called with pb->io.io_lock held. */
void paintbox_fpga_soft_reset(struct paintbox_data *pb)
{
#ifdef CONFIG_PAINTBOX_FPGA_SOFT_RESET
	writel(FPGA_SOFT_RESET_EN, pb->fpga_reg_base + FPGA_SOFT_RESET);

	udelay(FPGA_SOFT_RESET_HOLD_PERIOD);

	writel(0, pb->fpga_reg_base + FPGA_SOFT_RESET);
#endif
}

int paintbox_fpga_init(struct paintbox_data *pb)
{
	struct resource *r;

	r = platform_get_resource(pb->pdev, IORESOURCE_MEM, 1);
	if (r == NULL) {
		dev_err(&pb->pdev->dev, "platform_get_resource failed\n");
		return -ENODEV;
	}

	pb->fpga_reg_base = devm_ioremap(&pb->pdev->dev, r->start,
			resource_size(r));
	if (pb->fpga_reg_base == NULL) {
		dev_err(&pb->pdev->dev, "unable to remap MMIO\n");
		return -ENOMEM;
	}

	return 0;
}

void paintbox_fpga_remove(struct paintbox_data *pb)
{
	devm_iounmap(&pb->pdev->dev, pb->fpga_reg_base);
}
