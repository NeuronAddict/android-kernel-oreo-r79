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

#ifndef __PAINTBOX_FPGA_H__
#define __PAINTBOX_FPGA_H__

#include <linux/types.h>

#include "paintbox-common.h"

void paintbox_fpga_soft_reset(struct paintbox_data *pb);

int paintbox_fpga_init(struct paintbox_data *pb);
void paintbox_fpga_deinit(struct paintbox_data *pb);

#endif  /* __PAINTBOX_FPGA_H__ */
