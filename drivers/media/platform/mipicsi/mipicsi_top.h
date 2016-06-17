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
#include "mipicsi_dc_dphy.h"

#define MIPICSI_TOP_MAX_LINKS 5


/* MIPI TOP Configuration Registers */
#define TX0_MODE                 0x0000
#define TX0_BYPINT               0x0004
#define TX0_DPHY_PLL_CONFIG0     0x0008
#define TX0_DPHY_PLL_CONFIG1     0x000C
#define TX0_DPHY_PLL_CNTRL       0x0010
#define TX0_DPHY_PLL_STATUS      0x0014
#define TX0_DPHY_CONFIG          0x0018
#define TX0_DPHY_CONTROL         0x001C
#define TX0_DPHY_IOTEST          0x0020
#define TX0_DPHY_ESCAPE          0x0024

#define TX1_MODE                 0x0028
#define TX1_BYPINT               0x002C
#define TX1_DPHY_PLL_CONFIG0     0x0030
#define TX1_DPHY_PLL_CONFIG1     0x0034
#define TX1_DPHY_PLL_CNTRL       0x0038
#define TX1_DPHY_PLL_STATUS      0x003C
#define TX1_DPHY_CONFIG          0x0040
#define TX1_DPHY_CONTROL         0x0044
#define TX1_DPHY_IOTEST          0x0048
#define TX1_DPHY_ESCAPE          0x004C

#define RX0_MODE                 0x0050
#define RX0_DPHY_PLL_CONFIG0     0x0054
#define RX0_DPHY_PLL_CONFIG1     0x0058
#define RX0_DPHY_PLL_CNTRL       0x005C
#define RX0_DPHY_PLL_STATUS      0x0060
#define RX0_DPHY_CONFIG          0x0064
#define RX0_DPHY_CONTROL         0x0068
#define RX0_DPHY_IOTEST          0x006C
#define RX0_DPHY_ESCAPE          0x0070

#define RX1_MODE                 0x0074
#define RX1_DPHY_CONFIG          0x0078
#define RX1_DPHY_CONTROL         0x007C
#define RX1_DPHY_IOTEST          0x0080
#define RX1_DPHY_ESCAPE          0x0084

#define RX2_MODE                 0x0088
#define RX2_DPHY_CONFIG          0x008C
#define RX2_DPHY_CONTROL         0x0090
#define RX2_DPHY_IOTEST          0x0094
#define RX2_DPHY_ESCAPE          0x0098

#define IPU_TX0_CONFIG           0x009C
#define IPU_TX1_CONFIG           0x0100
#define CSI_CLK_CNTRL            0x0104

#define R_INVALID 0XFFFF

/* TX[x]_MODE */
#define TX_FUNC_EN              (0x1 << 0)
#define TX_BYP_SEL_MASK         (0x3 << 1)
#define TX_BYP_SEL_SET(x)       ((x) << 1)
#define TX_CMODE_MASK           (0x7 << 3)
#define TX_FORCE_OFF            (0x1 << 6)

enum TX_BYP_SEL_VALUES {
	TX_POWER_OFF = 0,
	TX_BYPASS_RX0 = 1,
	TX_BYPASS_RX1 = 2,
	TX_BYPASS_RX2 = 3
};


/* TX[x]_BYPINT */
#define TX_BYP_OF               (0x1 << 0)
#define TX_INT_EN               (0x1 << 1)

/* DPHY_PLL_CONFIG0 */
#define M_MASK                  (0x3FF << 0)
#define M_SET(x)                ((x) << 0)
#define N_MASK                  (0xF << 10)
#define N_SET(x)                ((x) << 10)
#define VCO_CNTRL_MASK          (0x3F << 14)
#define VCO_CNTRL_SET(x)        ((x) << 14)
#define CPBIAS_CNTRL_MASK       (0x7F << 20)
#define CPBIAS_CNTRl_SET(x)     ((x) << 20)
#define GMP_CNTRL_MASK          (0x3 << 27)
#define GMP_CNTRL_SET(x)        ((x) << 27)

/* DPHY_PLL_CONFIG1 */
#define INT_CNTRL_MASK          (0x3F << 0)
#define PROP_CNTRL_MASK         (0x3F << 6)

/* DPHY_PLL_CNTRL */
#define PLL_SHADOW_CONTROL_EN   (0x1 << 0)
#define PLL_SHADOW_CLEAR        (0x1 << 1)
#define UPDATEPLL               (0x1 << 2)
#define FORCEPLL                (0x1 << 3)
#define GP_CLK_EN               (0x1 << 4)
#define FORCE_LOCK              (0x1 << 5)
#define CLK_SEL_MASK            (0x3 << 6)
#define CLK_SEL_SET(x)          ((x) << 6)

enum CLK_SEL_VALUES {
	CLK_STOPPED = 0,
	CLK_GENERATION = 1,
	CLK_EXT_BUFFERED = 2,
	CLK_FORBIDDEN = 3
};


/* DPHY_PLL_STATUS */
#define PLL_LOCK                (0x1 << 0)
#define PLL_SHADOW_CONTROL_OBS  (0x1 << 1)

/* TX[x]_DPHY_CONFIG, RX[x]_DPHY_CONFIG */
#define HSFREQRANGE_MASK        (0x3F << 0)
#define CFGCLKFREQRANGE_MASK    (0xFF << 8)

/* TX[x]_DPHY_CONTROL */
#define SHUTDOWNZ_N             (0x1 << 0)
#define RSTZ_N                  (0x1 << 1)
#define PHY_SHADOW_CONTROL      (0x1 << 2)
#define SHADOW_CLEAR            (0x1 << 3)

/* TX[x]_DPHY_IOTEST, RX[x]_DPHY_IOTEST */
#define CONT_EN                 (0x1 << 0)
#define CONT_DATA_MASK          (0x7FF << 1)
#define BISTON                  (0x01 << 12)
#define BISTDONE                (0x01 << 13)
#define BISTOK                  (0x01 << 14)

/* TX[x]_DPHY_ESCAPE -- N/A */

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

/* RX[x]_DPHY_CONFIG -- see above */

/* RX[x]_DPHY_CONTROL -- defined above */
/* #define SHUTDOWNZ_N             (0x1 << 0) */
/* #define RSTZ_N                  (0x1 << 1) */

/* RX[x]_DPHY_IOTEST -- see above */

/* IPU_TX[x]_CONFIG */
#define TX_VC0_EN               (0x1 << 0)
#define TX_VC1_EN               (0x1 << 1)
#define TX_VC2_EN               (0x1 << 2)
#define TX_VC3_EN               (0x1 << 3)

/* CSI_CLK_CNTRL */
#define CSI_TX0_CG              (0x1 << 0)
#define CSI_TX1_CG              (0x1 << 1)
#define CSI_RX0_CG              (0x1 << 2)
#define CSI_RX1_CG              (0x1 << 3)
#define CSI_RX2_CG              (0x1 << 4)

/* ENABLE EMULATION SUPPORT */
#define MNH_EMULATION

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

struct mipicsi_top_device {
	struct device	    *dev;     /* Device node */
	struct list_head    devlist;  /* Device list */
	spinlock_t          slock;    /* Spinlock */
	struct mutex        mutex;    /* Mutex */

	/** Device Tree Information */
	void __iomem      *base_address;
	uint32_t           mem_size;
	uint32_t           top_irq_number;
	uint32_t           top_irq;
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
