/*
 * Copyright (c) 2017, Intel Corporation. All rights reserved.
 *
 * Author: Archana Vohra <archana.vohra@intel.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of Intel nor the names of its contributors may be used
 * to endorse or promote products derived from this software without specific
 * prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include <linux/of.h>

#include "mipicsi_top.h"

extern void * dev_addr_map[];

int mipi_emulation = -1;

void mipicsi_util_read_emulation(void)
{
	uint32_t val;
	struct device_node *node =
		of_find_node_by_name(NULL, "chosen");

	if (mipi_emulation != -1)
		return;

	if (node && !of_property_read_u32(node, "emulation", &val)) {
		mipi_emulation = true;
		if (val == 0)
			mipi_emulation = false;
	}

	pr_err("%s: DT Value %d\n", __func__, val);
}

bool mipicsi_util_is_emulation(void)
{
	return ((mipi_emulation == true) ? true:false);
}

uint16_t mipicsi_util_get_min_bitrate(void)
{
	return ((mipi_emulation == true) ? DC_MIN_BITRATE : G3_MIN_BITRATE);
}

uint16_t mipicsi_util_get_max_bitrate(void)
{
	return ((mipi_emulation == true) ? DC_MAX_BITRATE : G3_MAX_BITRATE);
}

void mipicsi_util_save_virt_addr(struct mipi_dev *dev)
{
	pr_err("Device %d Base Addr 0x%x\n", dev->device_id, dev->base_address);
	dev_addr_map[dev->device_id] = dev->base_address;
}

int mipicsi_util_write_top(uint16_t offset, uint32_t value)
{
	struct mipicsi_top_reg reg;

	reg.dev = MIPI_TOP;
	reg.offset = offset;
	reg.value = value;
	return mipicsi_top_write(&reg);
}
