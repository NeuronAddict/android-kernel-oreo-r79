/*
 * Copyright (c) 2016, Intel Corporation. All rights reserved.
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

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/string.h>
#include "mipicsi_device.h"
#include "mipicsi_util.h"
#include "mipicsi_debug.h"

int mipicsi_debug_vpg_preset(struct mipicsi_top_vpg *vpg,
			     enum mipicsi_debug_vpg_res res)
{
	int ret;
	/* NOTE Device must be populated in vpg struct before calling this */

	pr_info("%s: E\n", __func__);

	/* Frame and line numbers increment by 1 */
	vpg->pkt_cfg = (0x1<<11) | (0x01<<9) | CSI2_RAW10;
	pr_info("%s: VPG  pkt=0x%x", __func__, vpg->pkt_cfg);

	if (res == VPG_VGA) {
		/* VGA Settings */
		pr_info("%s: VGA ", __func__);
		vpg->mode_cfg = 1;
		vpg->pkt_size = 640;
		vpg->hsa_time = 120;
		vpg->hbp_time = 40;
		vpg->hline_time = 800;
		vpg->vsa_lines = 80;
		vpg->vbp_lines = 20;
		vpg->vfp_lines = 20;
		vpg->act_lines = 480;
		vpg->max_frame = 5;
		vpg->start_line = 0;
		vpg->step_line = 0;
	} else if (res == VPG_1080P) {
		/* 1080P Settings */
		pr_info("%s: 1080P", __func__);
		vpg->mode_cfg = 0;
		vpg->pkt_size = 1920;
		vpg->hsa_time = 53;
		vpg->hbp_time = 10;
		vpg->hline_time = 2200;
		vpg->vsa_lines = 80;
		vpg->vbp_lines = 20;
		vpg->vfp_lines = 20;
		vpg->act_lines = 1080;
		vpg->max_frame = 5;
		vpg->start_line = 0;
		vpg->step_line = 0;
	} else if (res == VPG_12MP) {
		/* 12MP Settings */
		pr_info("%s: 12MP", __func__);
		vpg->mode_cfg = 0;
		vpg->pkt_size = 4320;
		vpg->hsa_time = 50;
		vpg->hbp_time = 20;
		vpg->hline_time = 4600;
		vpg->vsa_lines = 80;
		vpg->vbp_lines = 20;
		vpg->vfp_lines = 20;
		vpg->act_lines = 2880;
		vpg->max_frame = 5;
		vpg->start_line = 0;
		vpg->step_line = 0;
	}
	ret = mipicsi_device_vpg(vpg);

	pr_info("%s: X\n", __func__);

	return ret;
}
