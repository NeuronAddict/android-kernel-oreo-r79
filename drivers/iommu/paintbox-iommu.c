/*
 * IOMMU Driver for the Paintbox programmable IPU
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

#include <linux/dma-iommu.h>
#include <linux/err.h>
#include <linux/iommu.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/paintbox-iommu.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/types.h>

#include "io-pgtable.h"

struct paintbox_iommu_data {
	struct paintbox_iommu_pdata *pdata;
	struct device *dev;
};

struct paintbox_domain {
	struct iommu_domain		domain;
	struct paintbox_iommu_data	*iommu_data;
	struct io_pgtable_ops		*pgtbl_ops;
	spinlock_t			pgtbl_lock;
	struct mutex			init_mutex;
};

static inline struct paintbox_iommu_data *to_iommu_data(struct device *dev)
{
	/* Normally an IOMMU driver can recover its driver data using
	 * dev->bus->iommu_ops->priv but the Paintbox IOMMU is internal to the
	 * IPU and does not sit on the same bus.  To workaround this the IPU
	 * device will store the IOMMU device in its platform data.
	 */
	return dev_get_drvdata(dev->platform_data);
}

static inline struct paintbox_domain *to_paintbox_domain(
		struct iommu_domain *dom)
{
	return container_of(dom, struct paintbox_domain, domain);
}

static inline struct paintbox_mmu_ops *get_mmu_ops(void *cookie)
{
	struct paintbox_iommu_data *iommu_data =
			(struct paintbox_iommu_data *)cookie;
	return &iommu_data->pdata->mmu_ops;
}

/* iommu_gather_ops tlb callback functions */
static void paintbox_iommu_tlb_flush_all(void *cookie)
{
	struct paintbox_mmu_ops *mmu_ops = get_mmu_ops(cookie);
	mmu_ops->tlb_flush_all(mmu_ops->priv);
}

static void paintbox_iommu_tlb_add_flush(unsigned long iova, size_t size,
		size_t granule, bool leaf, void *cookie)
{
	struct paintbox_mmu_ops *mmu_ops = get_mmu_ops(cookie);
	mmu_ops->tlb_invalidate_range_nosync(mmu_ops->priv, iova, size, leaf);
}

static void paintbox_iommu_tlb_sync(void *cookie)
{
	struct paintbox_mmu_ops *mmu_ops = get_mmu_ops(cookie);
	mmu_ops->tlb_sync(mmu_ops->priv);
}

static struct iommu_gather_ops paintbox_iommu_gather_ops = {
	.tlb_flush_all	= paintbox_iommu_tlb_flush_all,
	.tlb_add_flush	= paintbox_iommu_tlb_add_flush,
	.tlb_sync	= paintbox_iommu_tlb_sync
};

/* iommu_ops functions */
static struct iommu_domain *paintbox_iommu_domain_alloc(unsigned type)
{
	struct paintbox_domain *pb_domain;
	int ret;

	if (type != IOMMU_DOMAIN_DMA)
		return NULL;

	pb_domain = kzalloc(sizeof(*pb_domain), GFP_KERNEL);
	if (!pb_domain)
		return NULL;

	ret = iommu_get_dma_cookie(&pb_domain->domain);
	if (ret < 0) {
		kfree(pb_domain);
		return ERR_PTR(ret);
	}

	mutex_init(&pb_domain->init_mutex);
	spin_lock_init(&pb_domain->pgtbl_lock);

	return &pb_domain->domain;
}

static void paintbox_iommu_domain_free(struct iommu_domain *domain)
{
	struct paintbox_domain *pb_domain = to_paintbox_domain(domain);

	if (pb_domain->pgtbl_ops)
		free_io_pgtable_ops(pb_domain->pgtbl_ops);

	kfree(pb_domain);
}

static int paintbox_iommu_attach_dev(struct iommu_domain *domain,
		struct device *dev)
{
	struct paintbox_iommu_data *iommu_data = to_iommu_data(dev);
	struct paintbox_domain *pb_domain = to_paintbox_domain(domain);
	struct paintbox_mmu_ops *mmu_ops = &iommu_data->pdata->mmu_ops;
	struct io_pgtable_ops *pgtbl_ops;
	struct io_pgtable_cfg pgtbl_cfg;

	mutex_lock(&pb_domain->init_mutex);

	if (dev->archdata.iommu) {
		mutex_unlock(&pb_domain->init_mutex);
		dev_err(dev, "already attached to an IOMMU\n");
		return -EEXIST;
	}

	pgtbl_cfg = (struct io_pgtable_cfg) {
		.pgsize_bitmap	= iommu_data->pdata->config.page_size_bitmap,
		.ias		= iommu_data->pdata->config.input_address_size,
		.oas		= iommu_data->pdata->config.output_address_size,
		.tlb		= &paintbox_iommu_gather_ops,
		.iommu_dev	= iommu_data->dev,
	};

	/* Allocate a page table for the IOMMU client device.  The Paintbox
	 * IOMMU uses the ARM_64_LPAE_S2 format but only the first stage is
	 * used.
	 */
	pgtbl_ops = alloc_io_pgtable_ops(ARM_64_LPAE_S2, &pgtbl_cfg,
			iommu_data);
	if (!pgtbl_ops) {
		mutex_unlock(&pb_domain->init_mutex);
		dev_err(dev, "failed to allocate page table\n");
		return -ENOMEM;
	}

	pb_domain->iommu_data = iommu_data;
	pb_domain->pgtbl_ops = pgtbl_ops;

	dev->archdata.iommu = pb_domain;

	dev_dbg(iommu_data->dev, "page table base addr 0x%016llx\n",
			pgtbl_cfg.arm_lpae_s2_cfg.vttbr);

	mmu_ops->enable(mmu_ops->priv, pgtbl_cfg.arm_lpae_s2_cfg.vttbr);

	mutex_unlock(&pb_domain->init_mutex);

	dev_dbg(iommu_data->dev, "%s: %s attached to iommu\n", __func__,
			dev_name(dev));

	return 0;
}

static void paintbox_iommu_detach_dev(struct iommu_domain *domain,
		struct device *dev)
{
	struct paintbox_iommu_data *iommu_data = to_iommu_data(dev);
	struct paintbox_domain *pb_domain = to_paintbox_domain(domain);
	struct paintbox_mmu_ops *mmu_ops = &iommu_data->pdata->mmu_ops;

	mutex_lock(&pb_domain->init_mutex);

	mmu_ops->disable(mmu_ops->priv);

	dev->archdata.iommu = NULL;

	if (pb_domain->pgtbl_ops) {
		free_io_pgtable_ops(pb_domain->pgtbl_ops);
		pb_domain->pgtbl_ops = NULL;
	}

	pb_domain->iommu_data = NULL;

	mutex_unlock(&pb_domain->init_mutex);

	dev_dbg(iommu_data->dev, "%s: %s detached from iommu\n", __func__,
			dev_name(dev));
}

static int paintbox_iommu_map(struct iommu_domain *domain, unsigned long iova,
		phys_addr_t paddr, size_t size, int prot)
{
	struct paintbox_domain *pb_domain = to_paintbox_domain(domain);
	struct io_pgtable_ops *ops = pb_domain->pgtbl_ops;
	unsigned long flags;
	int ret;

	if (!ops)
		return -ENODEV;

	spin_lock_irqsave(&pb_domain->pgtbl_lock, flags);
	ret = ops->map(ops, iova, paddr, size, prot);
	spin_unlock_irqrestore(&pb_domain->pgtbl_lock, flags);

	dev_dbg(pb_domain->iommu_data->dev,
			"map pa 0x%pa -> iova 0x%016lx sz %zu prot %d ret %x\n",
			&paddr, iova, size, prot, ret);

	return ret;
}

static size_t paintbox_iommu_unmap(struct iommu_domain *domain,
		unsigned long iova, size_t size)
{
	struct paintbox_domain *pb_domain = to_paintbox_domain(domain);
	struct io_pgtable_ops *ops = pb_domain->pgtbl_ops;
	unsigned long flags;
	size_t ret;

	spin_lock_irqsave(&pb_domain->pgtbl_lock, flags);
	ret = ops->unmap(ops, iova, size);
	spin_unlock_irqrestore(&pb_domain->pgtbl_lock, flags);

	dev_dbg(pb_domain->iommu_data->dev,
			"unmap iova 0x%016lx sz %zu ret %zu\n", iova, size,
			ret);

	return ret;
}

static phys_addr_t paintbox_iommu_iova_to_phys(struct iommu_domain *domain,
		dma_addr_t iova)
{
	struct paintbox_domain *pb_domain = to_paintbox_domain(domain);
	struct io_pgtable_ops *ops = pb_domain->pgtbl_ops;
	unsigned long flags;
	phys_addr_t ret;

	spin_lock_irqsave(&pb_domain->pgtbl_lock, flags);
	ret = ops->iova_to_phys(ops, iova);
	spin_unlock_irqrestore(&pb_domain->pgtbl_lock, flags);

	dev_dbg(pb_domain->iommu_data->dev,
			"iova to phys iova %pad -> pa %pa\n", &iova, &ret);

	return ret;
}

static int paintbox_iommu_add_device(struct device *dev)
{
	struct paintbox_iommu_data *iommu_data = to_iommu_data(dev);
	struct iommu_group *group;

	/* If this is the IOMMU device then return -ENODEV to indicate that this
	 * device does not need IOMMU translation services.
	 */
	if (!iommu_data || iommu_data->dev == dev)
		return -ENODEV;

	group = iommu_group_get_for_dev(dev);
	if (IS_ERR(group))
		return PTR_ERR(group);

	return 0;
}

static void paintbox_iommu_remove_device(struct device *dev)
{
	iommu_group_remove_device(dev);
}

static struct iommu_group *paintbox_iommu_device_group(struct device *dev)
{
	struct paintbox_iommu_data *iommu_data = to_iommu_data(dev);
	struct iommu_group *group;

	group = generic_device_group(dev);
	if (IS_ERR(group))
		return group;

	iommu_group_set_iommudata(group, iommu_data, NULL);

	return group;
}

static struct iommu_ops paintbox_iommu_ops = {
	.domain_alloc		= paintbox_iommu_domain_alloc,
	.domain_free		= paintbox_iommu_domain_free,
	.attach_dev		= paintbox_iommu_attach_dev,
	.detach_dev		= paintbox_iommu_detach_dev,
	.map			= paintbox_iommu_map,
	.unmap			= paintbox_iommu_unmap,
	.map_sg			= default_iommu_map_sg,
	.iova_to_phys		= paintbox_iommu_iova_to_phys,
	.add_device		= paintbox_iommu_add_device,
	.remove_device		= paintbox_iommu_remove_device,
	.device_group		= paintbox_iommu_device_group,
};

static int paintbox_iommu_probe(struct device *dev)
{
	struct paintbox_iommu_data *iommu_data;
	int ret;

	iommu_data = devm_kzalloc(dev, sizeof(*iommu_data), GFP_KERNEL);
	if (!iommu_data) {
		dev_err(dev, "failed to allocate device data\n");
		return -ENOMEM;
	}

	dev_set_drvdata(dev, iommu_data);

	iommu_data->dev = dev;
	iommu_data->pdata = dev->platform_data;

	paintbox_iommu_ops.pgsize_bitmap =
			iommu_data->pdata->config.page_size_bitmap;

	ret = bus_set_iommu(&paintbox_bus_type, &paintbox_iommu_ops);
	if (ret < 0) {
		dev_err(dev, "failed to register iommu with bus\n");
		dev_set_drvdata(dev, NULL);
		kfree(iommu_data);
		return ret;
	}

	return 0;
}

static int paintbox_iommu_remove(struct device *dev)
{
	struct paintbox_iommu_data *iommu_data = dev_get_drvdata(dev);
	struct paintbox_mmu_ops *mmu_ops = &iommu_data->pdata->mmu_ops;

	mmu_ops->disable(mmu_ops->priv);

	dev_set_drvdata(dev, NULL);
	kfree(iommu_data);

	return 0;
}

static struct device_driver paintbox_iommu_driver = {
	.name	= "paintbox-iommu",
	.bus	= &paintbox_bus_type,
	.probe	= paintbox_iommu_probe,
	.remove	= paintbox_iommu_remove,
};

int paintbox_iommu_init(void)
{
	return driver_register(&paintbox_iommu_driver);
}

static void __exit paintbox_iommu_exit(void)
{
	driver_unregister(&paintbox_iommu_driver);
}

subsys_initcall(paintbox_iommu_init);
module_exit(paintbox_iommu_exit);
