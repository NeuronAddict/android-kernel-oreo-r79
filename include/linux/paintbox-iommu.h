/*
 * Paintbox virtual bus between IOMMU driver and core Paintbox driver.
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
#ifndef __LINUX_PAINTBOX_IOMMU_H__
#define __LINUX_PAINTBOX_IOMMU_H__

#include <linux/device.h>
#include <linux/types.h>


struct paintbox_iommu_config {
	unsigned long page_size_bitmap;
	unsigned int input_address_size;
	unsigned int output_address_size;
};

struct paintbox_mmu_ops {
	void (*tlb_flush_all)(void *priv);
	void (*tlb_invalidate_range_nosync)(void *priv, unsigned long iova,
			size_t size, bool leaf);
	void (*tlb_sync)(void *priv);

	void (*enable)(void *priv, uint64_t table_base_paddr);
	void (*disable)(void *priv);

	void *priv;
};

struct paintbox_iommu_pdata {
	struct paintbox_iommu_config config;
	struct paintbox_mmu_ops mmu_ops;
};

extern struct bus_type paintbox_bus_type;

#endif /* __LINUX_PAINTBOX_IOMMU_H__ */
