/*
 * Easel Ion Driver
 *
 * Copyright (C) 2016 Google, Inc.
 *
 * Derived from HiSilicon Ion Driver:
 * Copyright (c) 2015 Hisilicon Limited.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#define pr_fmt(fmt) "ion: " fmt

#include <linux/err.h>
#include <linux/easel_ion.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/mm.h>
#include "../ion_priv.h"
#include "../ion.h"
#include "../ion_of.h"

struct easel_ion_dev {
	struct ion_heap	**heaps;
	struct ion_device *idev;
	struct ion_platform_data *data;
};

static struct ion_of_heap easel_heaps[] = {
	PLATFORM_HEAP("easel,sys_user", 0,
		      ION_HEAP_TYPE_SYSTEM, "sys_user"),
	PLATFORM_HEAP("easel,sys_contig", 1,
		      ION_HEAP_TYPE_SYSTEM_CONTIG, "sys_contig"),
	PLATFORM_HEAP("easel,cma", ION_HEAP_TYPE_DMA, ION_HEAP_TYPE_DMA,
		      "cma"),
	{}
};

static int easel_ion_probe(struct platform_device *pdev)
{
	struct easel_ion_dev *ipdev;
	int i;

	ipdev = devm_kzalloc(&pdev->dev, sizeof(*ipdev), GFP_KERNEL);
	if (!ipdev)
		return -ENOMEM;

	platform_set_drvdata(pdev, ipdev);

	ipdev->idev = ion_device_create(NULL);
	if (IS_ERR(ipdev->idev))
		return PTR_ERR(ipdev->idev);

	ipdev->data = ion_parse_dt(pdev, easel_heaps);
	if (IS_ERR(ipdev->data))
		return PTR_ERR(ipdev->data);

	ipdev->heaps = devm_kzalloc(&pdev->dev,
				sizeof(struct ion_heap) * ipdev->data->nr,
				GFP_KERNEL);
	if (!ipdev->heaps) {
		ion_destroy_platform_data(ipdev->data);
		return -ENOMEM;
	}

	for (i = 0; i < ipdev->data->nr; i++) {
		ipdev->heaps[i] = ion_heap_create(&ipdev->data->heaps[i]);
		if (!ipdev->heaps) {
			ion_destroy_platform_data(ipdev->data);
			return -ENOMEM;
		}
		ion_device_add_heap(ipdev->idev, ipdev->heaps[i]);
	}
	return 0;
}

static int easel_ion_remove(struct platform_device *pdev)
{
	struct easel_ion_dev *ipdev;
	int i;

	ipdev = platform_get_drvdata(pdev);

	for (i = 0; i < ipdev->data->nr; i++)
		ion_heap_destroy(ipdev->heaps[i]);

	ion_destroy_platform_data(ipdev->data);
	ion_device_destroy(ipdev->idev);

	return 0;
}

static const struct of_device_id easel_ion_match_table[] = {
	{.compatible = "easel,ion"},
	{},
};

static struct platform_driver easel_ion_driver = {
	.probe = easel_ion_probe,
	.remove = easel_ion_remove,
	.driver = {
		.name = "ion",
		.of_match_table = easel_ion_match_table,
	},
};

static int __init easel_ion_init(void)
{
	return platform_driver_register(&easel_ion_driver);
}

subsys_initcall(easel_ion_init);
