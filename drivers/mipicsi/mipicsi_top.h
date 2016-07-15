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

#ifndef MIPI_TOP_H_
#define MIPI_TOP_H_

#include <linux/mutex.h>
#include <linux/mipibridge.h>
 
#include "mipicsi_dc_dphy.h"

#define MIPICSI_TOP_MAX_LINKS 5

enum TX_BYP_SEL_VALUES {
	TX_POWER_OFF = 0,
	TX_BYPASS_RX0 = 1,
	TX_BYPASS_RX1 = 2,
	TX_BYPASS_RX2 = 3
};

#if 0
/* RX[x]_MODE */
#define RX_IPU_EN               (0x1 << 0)
#define RX_BYP_Tx0_EN           (0x1 << 1)
#define RX_BYP_Tx1_EN           (0x1 << 2)
#define RX_MODE_MASK            (0x7 << 3)
#define RX_FORCE_OFF            (0x1 << 6)
#define RX_VC0_EN               (0x1 << 7)
#define RX_VC1_EN               (0x1 << 8)
#define RX_VC2_EN               (0x1 << 9)
#define RX_VC3_EN               (0x1 << 10)
#endif // #if 0

/* ENABLE EMULATION SUPPORT */
#define MNH_EMULATION
#if 0
enum mipicsi_top_dev {
	MIPI_RX0,
	MIPI_RX1,
	MIPI_RX2,
	MIPI_TX0,
	MIPI_TX1,
	MIPI_TOP,
	MIPI_IPU,
	MIPI_MAX = MIPI_IPU
};
#endif

enum csi_data_type {
	CSI2_INVALID          = -1,
	CSI2_YUV420_8         = 0x18,
	CSI2_YUV420_10        = 0x19,
	CSI2_YUV420_8_LEG     = 0x1A,
	CSI2_YUV420_8_CSPS    = 0x1C,
	CSI2_YUV420_10_CSPS   = 0x1D,
	CSI2_YUV422_8         = 0x1E,
	CSI2_YUV422_10        = 0x1F,
	CSI2_RGB444           = 0x20,
	CSI2_RGB555           = 0x21,
	CSI2_RGB565           = 0x22,
	CSI2_RGB666           = 0x23,
	CSI2_RGB888           = 0x24,
	CSI2_RAW6             = 0x28,
	CSI2_RAW7             = 0x29,
	CSI2_RAW8             = 0x2A,
	CSI2_RAW10            = 0x2B,
	CSI2_RAW12            = 0x2C,
	CSI2_RAW14            = 0x2D
};

enum virt_chan {
	VC1 = 1,
	VC2,
	VC3,
	VC4,
	VC_MAX = VC4
};

struct csi2_vc_dt_pair {
	enum virt_chan      ch;
	enum csi_data_type  dt;
};

struct mipicsi_top_cfg {
	enum mipicsi_top_dev dev;         /* device */
	uint32_t       num_lanes;         /* number of lanes */
	uint32_t       mbps;              /* bitrate (per lane) */
	struct csi2_vc_dt_pair vc_dt[8];  /* virtchan/data type pairs */
};

struct mipicsi_top_mux {
	enum mipicsi_top_dev source;
	enum mipicsi_top_dev sink;
};

struct mipicsi_top_mux_data {
	uint8_t count;
	struct mipicsi_top_mux links[MIPICSI_TOP_MAX_LINKS];
};

struct mipicsi_top_reg {
	uint32_t    offset;
	uint32_t    value;
};


struct mipicsi_top_vpg {
	enum mipicsi_top_dev dev; /* device */
	uint32_t    mode_cfg;     /* orientation/mode */
	uint32_t    pkt_cfg;      /* packet configuration */
	uint32_t    pkt_size;     /* test packet size */
	uint32_t    hsa_time;     /* hsync active */
	uint32_t    hbp_time;     /* horiz back porch */
	uint32_t    hline_time;   /* horiz line */
	uint32_t    vsa_lines;    /* vsync active */
	uint32_t    vbp_lines;    /* vert back porch */
	uint32_t    vfp_lines;    /* vert front porch */
	uint32_t    act_lines;    /* vert active */
	uint32_t    max_frame;    /* max frame num */
	uint32_t    start_line;   /* start line num */
	uint32_t    step_line;    /* step line num */
};

int mipicsi_top_start(struct mipicsi_top_cfg *config);
int mipicsi_top_stop(enum mipicsi_top_dev dev);
int mipicsi_top_set_mux(struct mipicsi_top_mux *mux);
void mipicsi_top_get_mux(struct mipicsi_top_mux_data *mux_data);
int mipicsi_top_set_irq_mask(uint8_t mask);
int mipicsi_top_read(struct mipicsi_top_reg *reg);
int mipicsi_top_write(struct mipicsi_top_reg *reg);
int mipicsi_top_debug_bist_start(enum mipicsi_top_dev dev);
int mipicsi_top_debug_bist_status(enum mipicsi_top_dev dev);
int mipicsi_top_debug_vpg(struct mipicsi_top_vpg *vpg);

#endif /* MIPI_TOP_H_ */
