/*
 * Easel Ion Driver
 *
 * Copyright (C) 2016 Google, Inc.
 *
 * Derived from HiSilicon Ion Driver:
 * Copyright (c) 2011-2012, Code Aurora Forum. All rights reserved.
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
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/dma-mapping.h>
#include "../ion_priv.h"

struct easel_ion_name_id_table {
	const char *name;
	unsigned int id;
};

static struct easel_ion_name_id_table name_id_table[] = {
	{"gralloc-carveout", ION_GRALLOC_HEAP_ID},
	{"overlay", ION_OVERLAY_HEAP_ID},
	{"sys_user", ION_SYSTEM_HEAP_ID},
	{"sys_contig", ION_SYSTEM_CONTIG_HEAP_ID},
	{"cma", ION_HEAP_TYPE_DMA},
};

struct easel_ion_type_id_table {
	const char *name;
	enum ion_heap_type type;
};

static struct easel_ion_type_id_table type_id_table[] = {
	{"ion_system_contig", ION_HEAP_TYPE_SYSTEM_CONTIG},
	{"ion_system", ION_HEAP_TYPE_SYSTEM},
	{"ion_carveout", ION_HEAP_TYPE_CARVEOUT},
	{"ion_chunk", ION_HEAP_TYPE_CHUNK},
	{"ion_dma", ION_HEAP_TYPE_DMA},
	{"ion_custom", ION_HEAP_TYPE_CUSTOM},
	{"ion_cma", ION_HEAP_TYPE_DMA},
};

#define EASEL_ION_HEAP_NUM 16

static struct ion_platform_data easel_ion_platform_data = {0};
static struct ion_platform_heap easel_ion_platform_heap[EASEL_ION_HEAP_NUM] =
		{{0} };

static struct ion_device *easel_ion_device;
static struct ion_heap *easel_ion_heap[EASEL_ION_HEAP_NUM] = {NULL};

int easel_ion_get_heap_info(unsigned int id, struct ion_heap_info_data *data)
{
	int i;

	BUG_ON(!data);

	for (i = 0; i < easel_ion_platform_data.nr; i++) {
		if (easel_ion_platform_heap[i].id == id) {
			data->heap_phy  = easel_ion_platform_heap[i].base;
			data->heap_size = easel_ion_platform_heap[i].size;
			strncpy((void *)data->name, (void *)
					easel_ion_platform_heap[i].name,
					EASEL_ION_NAME_LEN);
			pr_debug("heap info : id %d name %s phy 0x%llx size %u"
					"\n", id, data->name, data->heap_phy,
					data->heap_size);
			return 0;
		}
	}
	pr_err("in %s please check the id %d\n", __func__, id);

	return -EINVAL;
}
EXPORT_SYMBOL(easel_ion_get_heap_info);

struct ion_device *get_ion_device(void)
{
	return easel_ion_device;
}
EXPORT_SYMBOL(get_ion_device);

static int __init get_id_by_name(const char *name, unsigned int *id)
{
	int i, n;

	n = sizeof(name_id_table)/sizeof(name_id_table[0]);
	for (i = 0; i < n; i++) {
		if (strncmp(name, name_id_table[i].name, EASEL_ION_NAME_LEN))
			continue;

		*id = name_id_table[i].id;
		return 0;
	}
	return -1;
}

static int __init get_type_by_name(const char *name, enum ion_heap_type *type)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(type_id_table); i++) {
		if (strncmp(name, type_id_table[i].name, EASEL_ION_NAME_LEN))
			continue;

		*type = type_id_table[i].type;
		return 0;
	}

	return -1;
}

static u64 easel_dmamask = DMA_BIT_MASK(32);

static struct platform_device ion_cma_device = {
	.name = "ion-cma-device",
	.id = -1,
	.dev = {
		.dma_mask = &easel_dmamask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	}
};

static int easel_ion_setup_platform_data(struct platform_device *pdev)
{
	struct device_node *node, *np;
	const char *heap_name;
	const char *type_name;
	unsigned int id;
	unsigned int range[2] = {0, 0};
	enum ion_heap_type type;
	int ret;
	int index = 0;

	node = pdev->dev.of_node;
	for_each_child_of_node(node, np) {
		ret = of_property_read_string(np, "heap-name", &heap_name);
		if (ret < 0) {
			pr_err("in node %s please check the name property of "
					"node %s\n", __func__, np->name);
			continue;
		}

		ret = get_id_by_name(heap_name, &id);
		if (ret < 0) {
			pr_err("in node %s please check the name %s\n",
					__func__, heap_name);
			continue;
		}

		ret = of_property_read_u32_array(np, "heap-range", range,
				ARRAY_SIZE(range));
		if (ret < 0) {
			pr_err("in node %s please check the range property of "
					"node %s\n", __func__, np->name);
			continue;
		}


		ret = of_property_read_string(np, "heap-type", &type_name);
		if (ret < 0) {
			pr_err("in node %s please check the type property of "
					"node %s\n", __func__, np->name);
			continue;
		}

		ret = get_type_by_name(type_name, &type);
		if (ret < 0) {
			pr_err("in node %s please check the type %s\n",
					__func__, type_name);
			continue;
		}

		easel_ion_platform_heap[index].name = heap_name;
		easel_ion_platform_heap[index].base = range[0];
		easel_ion_platform_heap[index].size = range[1];
		easel_ion_platform_heap[index].id = id;
		easel_ion_platform_heap[index].type = type;
		if (type == ION_HEAP_TYPE_DMA) {
			/* TODO(ahampson):  This looks partially implemented in
			 * the HiSilicon driver that this driver is based on.
			 * On Easel the dma_ops should probably be set to the
			 * Paintbox IOMMU dma_ops.  In the interim this will be
			 * be set to the dma_ops assigned to the ION device.
			 */
			ion_cma_device.dev.archdata.dma_ops =
					pdev->dev.archdata.dma_ops;
			easel_ion_platform_heap[index].priv =
				(void *)&ion_cma_device.dev;
		}
		index++;
	}

	easel_ion_platform_data.nr = index;
	easel_ion_platform_data.heaps = easel_ion_platform_heap;

	return 0;
}

static int __init easel_ion_probe(struct platform_device *pdev)
{
	int i, err;
	struct ion_heap *heap;
	struct ion_platform_heap *heap_data;

	if (easel_ion_setup_platform_data(pdev)) {
		pr_err("easel_ion_setup_platform_data failed\n");
		return -EINVAL;
	}

	easel_ion_device = ion_device_create(NULL);
	if (IS_ERR_OR_NULL(easel_ion_device))
		return PTR_ERR(easel_ion_device);
	/*
	 * create the heaps as specified in the board file
	 */
	for (i = 0; i < easel_ion_platform_data.nr; i++) {
		heap_data = &easel_ion_platform_data.heaps[i];
		heap = ion_heap_create(heap_data);
		if (IS_ERR_OR_NULL(heap)) {
			err = PTR_ERR(heap);
			goto out;
		}

		ion_device_add_heap(easel_ion_device, heap);
		easel_ion_heap[i] = heap;
	}

	platform_set_drvdata(pdev, easel_ion_device);

	return 0;

out:
	for (i = 0; i < EASEL_ION_HEAP_NUM; i++) {
		if (!easel_ion_heap[i])
			continue;
		ion_heap_destroy(easel_ion_heap[i]);
		easel_ion_heap[i] = NULL;
	}

	return err;
}

static int easel_ion_remove(struct platform_device *pdev)
{
	int i;

	ion_device_destroy(easel_ion_device);
	for (i = 0; i < EASEL_ION_HEAP_NUM; i++) {
		if (!easel_ion_heap[i])
			continue;
		ion_heap_destroy(easel_ion_heap[i]);
		easel_ion_heap[i] = NULL;
	}

	return 0;
}

static struct of_device_id easel_ion_match_table[] = {
	{ .compatible = "easel,ion", },
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

module_platform_driver(easel_ion_driver);
