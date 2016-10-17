/*
 * Copyright (c) 2014--2016 Intel Corporation.
 *
 * Author: Teemu Rytkonen <teemu.s.rytkonen@intel.com>
 *         Archana Vohra <archana.vohra@intel.com>
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

#ifndef __MIPICISBRIDGE__
#define __MIPICISBRIDGE__

#define MIPICSI_TOP_MAX_LINKS 5

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

/* Interrupts */
/* Host int */
struct mipi_host_irq_st {
	enum mipicsi_top_dev dev;
	unsigned int main;
	unsigned int phy_fatal;
	unsigned int pkt_fatal;
	unsigned int frame_fatal;
	unsigned int phy;
	unsigned int pkt;
	unsigned int line;
};

struct mipi_host_irq_mask {
	enum mipicsi_top_dev dev;
	unsigned int   phy_fatal;
	unsigned int   pkt_fatal;
	unsigned int   frame_fatal;
	unsigned int   phy;
	unsigned int   pkt;
	unsigned int   line;
};

/* Device int */
struct device_int_mem {
	unsigned int if_fifo_owerflow : 1;
};

struct mipi_device_irq_st {
	enum mipicsi_top_dev dev;
	unsigned int main;
	unsigned int vpg;
	unsigned int idi;
	unsigned int phy;
};

struct mipi_device_irq_mask {
	enum mipicsi_top_dev dev;
	unsigned int vpg;
	unsigned int idi;
	unsigned int phy;
};

struct mipi_top_notification {
	unsigned int host_rx0_int : 1;
	unsigned int host_rx1_int : 1;
	unsigned int host_rx2_int : 1;
	unsigned int device_tx0_int : 1;
	unsigned int device_tx1_int : 1;
	unsigned int fifo_overflow_int : 1;
	unsigned int mux_ready_notif : 1;
};

enum CLK_SEL_VALUES {
	CLK_STOPPED = 0,
	CLK_GENERATION = 1,
	CLK_EXT_BUFFERED = 2,
	CLK_FORBIDDEN = 3
};

struct mipicsi_top_cfg {
	enum mipicsi_top_dev dev;    /* device */
	uint32_t       num_lanes;    /* number of lanes */
	uint32_t       mbps;         /* bitrate (per lane) */
};

struct mipicsi_top_mux {
	enum mipicsi_top_dev source;  /* Rx0, Rx1, Rx2, IPU */
	enum mipicsi_top_dev sink;    /* IPU, Tx0, Tx1 */
	uint8_t ss_vc_mask;           /* Safe switch for stream on (VC 3:0) */
	bool ss_stream_off;           /* Safe switch for stream off */
	bool active;                  /* If mux path is active */
};

struct mipicsi_top_mux_data {
	uint8_t count;
	struct mipicsi_top_mux links[MIPICSI_TOP_MAX_LINKS];
};

struct mipicsi_top_reg {
	enum mipicsi_top_dev dev;
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

#define MIPIBRIDGE_IOC_TOP_MAGIC  'T'
#define MIPI_TOP_MAX               18


#define MIPI_TOP_START     \
	_IOWR(MIPIBRIDGE_IOC_TOP_MAGIC,  1, struct mipicsi_top_cfg)
#define MIPI_TOP_STOP     \
	_IOWR(MIPIBRIDGE_IOC_TOP_MAGIC,  2, enum mipicsi_top_dev)
#define MIPI_TOP_S_MUX     \
	_IOWR(MIPIBRIDGE_IOC_TOP_MAGIC,  3, struct mipicsi_top_mux)
#define MIPI_TOP_G_MUX     \
	_IOWR(MIPIBRIDGE_IOC_TOP_MAGIC,  4, struct mipicsi_top_mux_data)
#define MIPI_TOP_G_MUX_STATUS     \
	_IOWR(MIPIBRIDGE_IOC_TOP_MAGIC,  5, struct mipicsi_top_mux)
#define MIPI_TOP_DIS_MUX     \
	_IOWR(MIPIBRIDGE_IOC_TOP_MAGIC,  6, struct mipicsi_top_mux)
#define MIPI_TOP_S_REG        \
	_IOW(MIPIBRIDGE_IOC_TOP_MAGIC,  7, struct mipicsi_top_reg)
#define MIPI_TOP_G_REG        \
	_IOWR(MIPIBRIDGE_IOC_TOP_MAGIC,  8, struct mipicsi_top_reg)
#define MIPI_TOP_VPG        \
	_IOWR(MIPIBRIDGE_IOC_TOP_MAGIC,  9, struct mipicsi_top_vpg)
#define MIPI_DEV_G_NOTIF     \
	_IOR(MIPIBRIDGE_IOC_TOP_MAGIC,  10, struct mipi_top_notification)
#define MIPI_TOP_RESET     \
	_IOW(MIPIBRIDGE_IOC_TOP_MAGIC,  11, enum mipicsi_top_dev)
#define MIPI_TOP_RESET_ALL     \
	_IO(MIPIBRIDGE_IOC_TOP_MAGIC,  12)
#define MIPI_DEV_G_INT_ST     \
	_IOR(MIPIBRIDGE_IOC_TOP_MAGIC,  13, struct mipi_device_irq_st)
#define MIPI_DEV_S_INT_MASK   \
	_IOW(MIPIBRIDGE_IOC_TOP_MAGIC,  14, struct mipi_device_irq_mask)
#define MIPI_DEV_S_INT_FORCE  \
	_IOW(MIPIBRIDGE_IOC_TOP_MAGIC,  15, struct mipi_device_irq_mask)
#define MIPI_HOST_G_INT_ST    \
	_IOR(MIPIBRIDGE_IOC_TOP_MAGIC,  16, struct mipi_host_irq_st)
#define MIPI_HOST_S_INT_MASK  \
	_IOW(MIPIBRIDGE_IOC_TOP_MAGIC,  17, struct mipi_host_irq_mask)
#define MIPI_HOST_S_INT_FORCE \
	_IOW(MIPIBRIDGE_IOC_TOP_MAGIC,  18, struct mipi_host_irq_mask)


#endif
