
#ifndef MIPICSI_DEVICE_H_
#define MIPICSI_DEVICE_H_

#include "mipicsi_top.h"


/* Device controller registers */

/* This register contains the version of the CSI2 device controller. */
#define R_CSI2_DEV_VERSION                          0x0000

/* CSI-2 device controller reset. */
#define R_CSI2_DEV_CSI2_RESETN                      0x0004

/***************** Interrupt Related Register Address *****************/

/* Interrupt Main Status. */
#define R_CSI2_DEV_INT_ST_MAIN                      0x0020

/* Video pattern generator related errors.*/
#define R_CSI2_DEV_INT_ST_VPG                       0x0024

/* IDI interface related errors.*/
#define R_CSI2_DEV_INT_ST_IDI                       0x0028

/* FIFO Memory related errors.*/
#define R_CSI2_DEV_INT_ST_MEM                       0x002c

/* Mask int_st_vpg.*/
#define R_CSI2_DEV_INT_MASK_VPG                     0x0040

/* Force int_st_vpg.*/
#define R_CSI2_DEV_INT_FORCE_VPG                    0x0044

/* Mask int_st_idi.*/
#define R_CSI2_DEV_INT_MASK_IDI                     0x0048

/* Force int_st_idi.*/
#define R_CSI2_DEV_INT_FORCE_IDI                    0x004c

/* Mask int_st_mem.*/
#define R_CSI2_DEV_INT_MASK_MEM                     0x0050

/* Force int_st_mem.*/
#define R_CSI2_DEV_INT_FORCE_MEM                    0x0054

/***************** VPG Related Register Address *****************/


/* This register contains the enable signal of VPG.*/
#define R_CSI2_DEV_VPG_CTRL                         0x0080

/* This register contains the status of VPG.*/
#define R_CSI2_DEV_VPG_STATUS                       0x0084

/* Test Mode Configuration of VPG*/
#define R_CSI2_DEV_VPG_MODE_CFG                     0x0088

/* This register indicates the packet configuration of VPG */
#define R_CSI2_DEV_VPG_PKT_CFG                      0x008c

/* Test packet size.*/
#define R_CSI2_DEV_VPG_PKT_SIZE                     0x0090

/* Horizontal Synchronism ActiveHSA time.*/
#define R_CSI2_DEV_VPG_HSA_TIME                     0x0094

/* Horizontal Back PorchHBP time.*/
#define R_CSI2_DEV_VPG_HBP_TIME                     0x0098

/* Overall time for each video line.*/
#define R_CSI2_DEV_VPG_HLINE_TIME                   0x009c

/* Vertical Synchronism ActiveVSA period.*/
#define R_CSI2_DEV_VPG_VSA_LINES                    0x00a0

/* Vertical Back PorchVBP period.*/
#define R_CSI2_DEV_VPG_VBP_LINES                    0x00a4

/* Vertical Front PorchVFP period.*/
#define R_CSI2_DEV_VPG_VFP_LINES                    0x00a8

/* Vertical resolution of video pattern.*/
#define R_CSI2_DEV_VPG_ACT_LINES                    0x00ac

/* Maximum Frame Number.*/
#define R_CSI2_DEV_VPG_MAX_FRAME_NUM                0x00b0

/* Start Line Number.*/
#define R_CSI2_DEV_VPG_START_LINE_NUM               0x00b4

/* Step Line Number.*/
#define R_CSI2_DEV_VPG_STEP_LINE_NUM                0x00b8

/***************** PHY Related Register Address *****************/

/* This register controls resets and the PLL of the D-PHY */
#define R_CSI2_DEV_PHY_RSTZ                         0x00e0

/* This register configures the number of active lanes.*/
#define R_CSI2_DEV_PHY_IF_CFG                       0x00e4

/* This register configures the possibility for using non continuous clock
 * in the clock lane
*/
#define R_CSI2_DEV_LPCLK_CNTRL                      0x00e8

/* This register configures entering and leaving ULPS in the D-PHY.*/
#define R_CSI2_DEV_PHY_ULPS_CTRL                    0x00ec

/* This register configures the factor for internal dividers to divide
 * lanebyteclk for timeout...
 */
#define R_CSI2_DEV_CLKMGR_CFG                       0x00f0

/* This register configures the pins that activate triggers in the D-PHY.*/
#define R_CSI2_DEV_PHY_TX_TRIGGERS                  0x00f4

/* This register configures the calibration of D-PHY.*/
#define R_CSI2_DEV_PHY_CAL                          0x00f8

/* This register contains information about the status of the D-PHY.*/
#define R_CSI2_DEV_PHY_STATUS                       0x0110
/* This register controls clock and clear pins of the D-PHY0 vendor specific
 * interface.
 */
#define R_CSI2_DEV_PHY0_TST_CTRL0                   0x0114

/* This register controls data and enable pins of the D-PHY0 vendor specific
 * interface.
 */
#define R_CSI2_DEV_PHY0_TST_CTRL1                   0x0118

/* This register controls clock and clear pins of the D-PHY0 vendor specific
 * interface.
 */
#define R_CSI2_DEV_PHY1_TST_CTRL0                   0x011c

/* This register controls data and enable pins of the D-PHY0 vendor specific
 * interface.
 */
#define R_CSI2_DEV_PHY1_TST_CTRL1                   0x0120

#define R_CSI2_DEV_IPI_PKT_CFG                      0x0140

#define R_CSI2_DEV_IPI_PIXELS                       0x0144

#define R_CSI2_DEV_IPI_MAX_FRAME_NUM                0x0148

#define R_CSI2_DEV_IPI_START_LINE_NUM               0x014c

#define R_CSI2_DEV_IPI_STEP_LINE_NUM                0x0150

/* Masks and Values */
#define DEV_STOPSTATE     ((1<<4) | (1<<6) | (1<<8) | (1<<10) | (1<<12))

/* Bit Shift */
#define DEV_TESTCLK_SH       1    /* TST_CTRL0 */
#define DEV_TESTEN_SH        16   /* TST_CTRL1 */
#define DEV_TESTCLR_SH       0    /* TST_CTRL0 */
#define DEV_SHUTDOWNZ_SH     0
#define DEV_RSTZ_SH          1

struct mipicsi_device_dev {
	struct device	    *dev;     /* Device node */
	struct list_head    devlist;  /* Device list */
	spinlock_t          slock;    /* Spinlock */
	struct mutex        mutex;    /* Mutex */

	/** Device Tree Information */
	void __iomem      *base_address;
	uint32_t           mem_size;
	uint32_t           device_irq_number;
	uint32_t           device_irq;
};

struct mipicsi_pll {
	uint32_t  output_freq;          /* Active output frequency */
	uint32_t  input_freq;           /* Active input frequency */
	uint32_t  ref_freq;             /* Reference Frequency */
	uint32_t  hsfreq;               /* hsfreq */
	uint8_t   vco_range;            /* vcorange */
	uint8_t   cp_current;           /* icpctrl */
	uint8_t   lpf_resistor;         /* lpfctrl */
	uint32_t  loop_div;             /* (M) */
	uint32_t  input_div;            /* (N) */
	uint8_t	  output_div;           /* (P) */
};

void mipicsi_device_reset(enum mipicsi_top_dev dev);
void mipicsi_device_dphy_reset(enum mipicsi_top_dev dev);
int mipicsi_device_start(struct mipicsi_top_cfg *config);
int mipicsi_device_hw_init(enum mipicsi_top_dev dev);
int mipicsi_device_vpg(struct mipicsi_top_vpg *vpg);

#endif /* MIPICSI_DEVICE_H_ */
