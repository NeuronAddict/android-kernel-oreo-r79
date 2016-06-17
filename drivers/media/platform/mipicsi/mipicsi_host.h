#ifndef MIPICSI_HOST_H_
#define MIPICSI_HOST_H_

#include <linux/types.h>
#include "mipicsi_top.h"

enum register_addresses {
	R_CSI2_VERSION                   = 0x00,
	R_CSI2_N_LANES                   = 0x04,
	R_CSI2_CTRL_RESETN               = 0x08,
	R_CSI2_INTERRUPT                 = 0x0C,
	R_CSI2_DATA_IDS_1                = 0x10,
	R_CSI2_DATA_IDS_2                = 0x14,
	R_CSI2_DPHY_SHUTDOWNZ            = 0x40,
	R_CSI2_DPHY_RSTZ                 = 0x44,
	R_CSI2_DPHY_RX                   = 0x48,
	R_CSI2_DPHY_STOPSTATE            = 0x4C,
	R_CSI2_DPHY_TST_CTRL0            = 0x50,
	R_CSI2_DPHY_TST_CTRL1            = 0x54,
	R_CSI2_DPHY2_TST_CTRL0           = 0x58,
	R_CSI2_DPHY2_TST_CTRL1           = 0x5C,
	R_CSI2_IPI_MODE                  = 0x80,
	R_CSI2_IPI_VCID                  = 0x84,
	R_CSI2_IPI_DATA_TYPE             = 0x88,
	R_CSI2_IPI_MEM_FLUSH             = 0x8C,
	R_CSI2_IPI_HSA_TIME              = 0x90,
	R_CSI2_IPI_HBP_TIME              = 0x94,
	R_CSI2_IPI_HSD_TIME              = 0x98,
	R_CSI2_IPI_HLINE_TIME            = 0x9C,
	R_CSI2_IPI_VSA_LINES             = 0xB0,
	R_CSI2_IPI_VBP_LINES             = 0xB4,
	R_CSI2_IPI_VFP_LINES             = 0xB8,
	R_CSI2_IPI_VACTIVE_LINES         = 0xBC,
	R_CSI2_INT_PHY_FATAL             = 0xe0,
	R_CSI2_MASK_INT_PHY_FATAL        = 0xe4,
	R_CSI2_FORCE_INT_PHY_FATAL       = 0xe8,
	R_CSI2_INT_PKT_FATAL             = 0xf0,
	R_CSI2_MASK_INT_PKT_FATAL        = 0xf4,
	R_CSI2_FORCE_INT_PKT_FATAL       = 0xf8,
	R_CSI2_INT_FRAME_FATAL           = 0x100,
	R_CSI2_MASK_INT_FRAME_FATAL      = 0x104,
	R_CSI2_FORCE_INT_FRAME_FATAL     = 0x108,
	R_CSI2_INT_PHY                   = 0x110,
	R_CSI2_MASK_INT_PHY              = 0x114,
	R_CSI2_FORCE_INT_PHY             = 0x118,
	R_CSI2_INT_PKT                   = 0x120,
	R_CSI2_MASK_INT_PKT              = 0x124,
	R_CSI2_FORCE_INT_PKT             = 0x128,
	R_CSI2_INT_LINE                  = 0x130,
	R_CSI2_MASK_INT_LINE             = 0x134,
	R_CSI2_FORCE_INT_LINE            = 0x138,
	R_CSI2_INT_IPI                   = 0x140,
	R_CSI2_MASK_INT_IPI              = 0x144,
	R_CSI2_FORCE_INT_IPI             = 0x148
};

/* Masks and Values */
#define HOST_STOPSTATE     ((1<<0) | (1<<1) | (1<<2) | (1<<3) | (1<<16))

/* Bit Shift */
#define HOST_TESTCLK_SH     1  /* TST_CTRL0 */
#define HOST_TESTEN_SH      16 /* TST_CTRL1 */
#define HOST_TESTCLR_SH     0  /* TST_CTRL0 */
#define HOST_SHUTDOWNZ_SH   0
#define HOST_RSTZ_SH        0

struct mipicsi_host_dev {
	struct device	    *dev;     /* Device node */
	struct list_head    devlist;  /* Device list */
	spinlock_t          slock;    /* Spinlock */
	struct mutex        mutex;    /* Mutex */

	/** Device Tree Information */
	void __iomem      *base_address;
	uint32_t           mem_size;
	uint32_t           host_irq_number;
	uint32_t           host_irq;
};


void mipicsi_host_dphy_reset(enum mipicsi_top_dev dev);
int  mipicsi_host_start(struct mipicsi_top_cfg *config);
int mipicsi_host_hw_init(enum mipicsi_top_dev dev);

#endif
