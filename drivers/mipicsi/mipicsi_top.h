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

/*NOTE: This include file has structures used in MIPI Top APIs */
#include <linux/mipibridge.h>
 
#include "mipicsi_dc_dphy.h"
#include "mipi_dev.h"

/* External Clocks */
#define AHB_APB_CLK_MHZ       100
#define FPCLK_MHZ             100
#define REFCLK_MHZ            25
#define CFGCLK_MHZ            50
#define EXTCLK_MHZ            1000

/* PLL and Bitrates for Daughtercard and Gen3 DPhy*/
#define DC_PLL_MIN_MHZ     80
#define DC_PLL_MAX_MHZ     2000
#define DC_MIN_BITRATE     DC_PLL_MIN_MHZ
#define DC_MAX_BITRATE     DC_PLL_MAX_MHZ
#define G3_PLL_MIN_MHZ     40
#define G3_PLL_MAX_MHZ     1250
#define G3_MIN_BITRATE     (G3_PLL_MIN_MHZ*2)
#define G3_MAX_BITRATE     (G3_PLL_MAX_MHZ*2)

/* CSI Device Controller Parameters */
#define CSI2_DEVICE_DATAINTERFACE       IDI
#define CSI2_DEVICE_NUM_OF_LANES        4
#define CSI2_DEVICE_DFLT_F_SYNC_TYPE    2
#define CSI2_DEVICE_IDI_PLD_FIFO_DEPTH  4096
#define CSI2_DEVICE_IDI_PLD_RAM_DEPTH   4096
#define CSI2_DEVICE_IDI_HD_FIFO_DEPTH   8

/* CSI Host Controller Parameters */
#define CSI2_DATAINTERFACE              2
#define IDI_64_DATA_IF                  1
#define CSI2_HOST_NUM_OF_LANES          4
#define CSI2_HOST_DFLT_F_SYNC_TYPE      2
#define CSI2_HOST_N_DATA_IDS            1
#define CSI2_HOST_SNPS_PHY              1
#define CSI2_EXT_PPIP                   0
#define CSI2_PPI_PD                     2
#define CSI2_PCLK_FREE                  1

#define BIT0_MASK 0x0001
#define BIT1_MASK 0x0002
#define BIT2_MASK 0x0004
#define BIT3_MASK 0x0008
#define BIT4_MASK 0x0010
#define BIT5_MASK 0x0020
#define BIT6_MASK 0x0040
#define BIT7_MASK 0x0080
#define BIT8_MASK 0x0100
#define BIT9_MASK 0x0200
#define BIT10_MASK 0x0400
#define BIT11_MASK 0x0800
#define BIT12_MASK 0x1000
#define BIT13_MASK 0x2000
#define BIT14_MASK 0x4000
#define BIT15_MASK 0x8000

#define RX_MODE_IPU_EN_MASK        BIT3_MASK
#define RX_MODE_TX0_BYP_EN_MASK    BIT4_MASK
#define RX_MODE_TX1_BYP_EN_MASK    BIT5_MASK
#define TX_CMODE_IPU_EN_MASK       BIT3_MASK
#define TX_CMODE_RX0_BYP_EN_MASK   BIT4_MASK
#define TX_CMODE_RX1_BYP_EN_MASK   BIT5_MASK


enum TX_BYP_SEL_VALUES {
	TX_POWER_OFF = 0,
	TX_BYPASS_RX0 = 1,
	TX_BYPASS_RX1 = 2,
	TX_BYPASS_RX2 = 3
};

struct mipicsi_top_bist {
	enum mipicsi_top_dev dev;     /* Rx0, Rx1, Rx2, Tx0, Tx1 */
	bool done;
	bool ok;
};


int mipicsi_top_start(struct mipicsi_top_cfg *config);
int mipicsi_top_stop(enum mipicsi_top_dev dev);
int mipicsi_top_set_mux(struct mipicsi_top_mux *mux);
int mipicsi_top_disable_mux(struct mipicsi_top_mux *mux);
void mipicsi_top_get_mux(struct mipicsi_top_mux_data *mux_data);
int mipicsi_top_get_mux_status(struct mipicsi_top_mux *mux);
int mipicsi_top_reset(enum mipicsi_top_dev dev);
int mipicsi_top_reset_all(void);
int mipicsi_top_set_irq_mask(uint8_t mask);
int mipicsi_top_read(struct mipicsi_top_reg *reg);
int mipicsi_top_write(struct mipicsi_top_reg *reg);
int mipicsi_top_dphy_read(struct mipicsi_top_reg *reg);
int mipicsi_top_dphy_write(struct mipicsi_top_reg *reg);
int mipicsi_top_debug_bist_start(enum mipicsi_top_dev dev);
int mipicsi_top_debug_bist_status(struct mipicsi_top_bist *bist);
int mipicsi_top_debug_vpg(struct mipicsi_top_vpg *vpg);

void mipicsi_set_device(enum mipicsi_top_dev devid, struct mipi_dev *dev);
struct mipi_dev *mipicsi_get_device(enum mipicsi_top_dev devid);
enum mipicsi_top_dev get_device_id(const char *device_name);

#endif /* MIPI_TOP_H_ */
