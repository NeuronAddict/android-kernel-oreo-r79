/*
 * Copyright (c) 2014--2016 Intel Corporation.
 *
 * Author: Archana Vohra <archana.vohra@intel.com>
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

#ifndef MIPICSI_SYSFS_H_
#define MIPICSI_SYSFS_H_

int mipicsi_sysfs_init(struct device *mipicsi_top_device);
void mipicsi_sysfs_clean(struct device *mipicsi_top_device);

#endif
