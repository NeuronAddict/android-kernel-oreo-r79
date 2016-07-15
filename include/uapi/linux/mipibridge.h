/*
 * Copyright (c) 2014--2016 Intel Corporation.
 *
 * Author: Teemu Rytkonen <teemu.s.rytkonen@intel.com>
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

/* for mipi host/dev start */
struct init_config {
	int test;
};

/* for mipi top pll configuring */
struct pll_config {
	int test;
};

enum host_mux_sel {
	host_rx0,
	host_rx1,
	host_rx2,
};

struct rx_mux_config {
	enum host_mux_sel sel;
	int mux_ipu : 1;
	int mux_tx0 : 1;
	int mux_tx1 : 1;
};

enum device_mux_sel {
	device_tx0,
	device_tx1,
};

enum device_mux_connect {
	mux_off,
	connect_ipu,
	connect_rx0,
	connect_rx1,
	connect_rx2
};

struct tx_mux_config {
	enum device_mux_sel sel;
	enum device_mux_connect state;
};

/* Interrupts */
/* Host int */
struct mipi_host_irq_st {
	unsigned int main;
	unsigned int phy_fatal;
	unsigned int pkt_fatal;
	unsigned int frame_fatal;
	unsigned int phy;
	unsigned int pkt;
	unsigned int line;
	unsigned int ipi;
};

struct mipi_host_irq_mask {
	unsigned int   phy_fatal;
	unsigned int   pkt_fatal;
	unsigned int   frame_fatal;
	unsigned int   phy;
	unsigned int   pkt;
	unsigned int   line;
	unsigned int   ipi;
};

/* Device int */
struct device_int_mem {
	unsigned int if_fifo_owerflow : 1;
};

struct mipi_device_irq_st {
	unsigned int main;
	unsigned int vpg;
	unsigned int idi;
	unsigned int mem;
};

struct mipi_device_irq_mask {
	unsigned int vpg;
	unsigned int idi;
	unsigned int mem;
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
#if 0
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
#endif

struct register_io {
	enum mipicsi_top_dev dev;
	unsigned int reg;
	unsigned int data;
};
#if 0
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
#endif

#define MIPIBRIDGE_IOC_HOST_MAGIC 'H'
#define MIPIBRIDGE_IOC_DEV_MAGIC  'D'
#define MIPIBRIDGE_IOC_TOP_MAGIC  'T'
#define MIPI_HOST_MAX              8
#define MIPI_DEV_MAX               8
#define MIPI_TOP_MAX               8
#define MIPI_HOST_START       \
	_IOWR(MIPIBRIDGE_IOC_HOST_MAGIC,  1, struct init_config)
#define MIPI_HOST_STOP        \
	_IOWR(MIPIBRIDGE_IOC_HOST_MAGIC,  2, int)
#define MIPI_HOST_S_REG       \
	_IOW(MIPIBRIDGE_IOC_HOST_MAGIC,   3, struct register_io)
#define MIPI_HOST_G_REG       \
	_IOWR(MIPIBRIDGE_IOC_HOST_MAGIC,  4, struct register_io)
#define MIPI_HOST_G_INT_ST    \
	_IOR(MIPIBRIDGE_IOC_HOST_MAGIC,  5, struct mipi_host_irq_st)
#define MIPI_HOST_S_INT_MASK  \
	_IOW(MIPIBRIDGE_IOC_HOST_MAGIC,  6, struct mipi_host_irq_mask)
#define MIPI_HOST_S_INT_FORCE \
	_IOW(MIPIBRIDGE_IOC_HOST_MAGIC,  7, struct mipi_host_irq_mask)
#define MIPI_DEV_START        \
	_IOWR(MIPIBRIDGE_IOC_DEV_MAGIC,  1, struct init_config)
#define MIPI_DEV_STOP         \
	_IOWR(MIPIBRIDGE_IOC_DEV_MAGIC,  2, int)
#define MIPI_DEV_S_REG        \
	_IOW(MIPIBRIDGE_IOC_DEV_MAGIC,   3, struct register_io)
#define MIPI_DEV_G_REG        \
	_IOWR(MIPIBRIDGE_IOC_DEV_MAGIC,  4, struct register_io)
#define MIPI_DEV_G_INT_ST     \
	_IOR(MIPIBRIDGE_IOC_DEV_MAGIC,  5, struct mipi_device_irq_st)
#define MIPI_DEV_S_INT_MASK   \
	_IOW(MIPIBRIDGE_IOC_DEV_MAGIC,  6, struct mipi_device_irq_mask)
#define MIPI_DEV_S_INT_FORCE  \
	_IOW(MIPIBRIDGE_IOC_DEV_MAGIC,  7, struct mipi_device_irq_mask)


#define MIPI_TOP_START     \
	_IOWR(MIPIBRIDGE_IOC_TOP_MAGIC,  1, struct mipicsi_top_cfg)
#define MIPI_TOP_STOP     \
	_IOWR(MIPIBRIDGE_IOC_TOP_MAGIC,  2, enum mipicsi_top_dev)
#define MIPI_TOP_S_MUX     \
	_IOWR(MIPIBRIDGE_IOC_TOP_MAGIC,  3, struct mipicsi_top_mux)
#define MIPI_TOP_G_MUX     \
	_IOWR(MIPIBRIDGE_IOC_TOP_MAGIC,  4, struct mipicsi_top_mux)
#define MIPI_TOP_S_REG        \
	_IOW(MIPIBRIDGE_IOC_TOP_MAGIC,  5, struct register_io)
#define MIPI_TOP_G_REG        \
	_IOWR(MIPIBRIDGE_IOC_TOP_MAGIC,  6, struct register_io)
#define MIPI_DEV_G_NOTIF     \
	_IOR(MIPIBRIDGE_IOC_TOP_MAGIC,  7, struct mipi_top_notification)

#endif
