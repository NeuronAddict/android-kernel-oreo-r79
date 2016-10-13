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

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/of_irq.h>
#include <linux/delay.h>
#include <linux/list.h>
#include <linux/kthread.h>
#include <linux/string.h>
#include <linux/of_device.h>
#include <linux/sysfs.h>
#include "mipicsi_chardev.h"
#include "mipicsi_top.h"
#include "mipicsi_host.h"
#include "mipicsi_device.h"
#include "mipicsi_sysfs.h"
#include "mipicsi_util.h"
#include "mipi_dev.h"

#include <linux/intel-hwio.h>
#include <soc/mnh/mnh-hwio-mipi-top.h>

void * dev_addr_map[MIPI_MAX] = { NULL };

void mipicsi_util_save_virt_addr(enum mipicsi_top_dev dev, void *base_addr)
{
  dev_addr_map[dev] = base_addr;
}

static struct device *mipicsi_top_device;

/* these macros assume a void * in scope named baddr */
#define TOP_IN(reg)             HW_IN(baddr,   MIPI_TOP, reg)
#define TOP_INf(reg, fld)       HW_INf(baddr,  MIPI_TOP, reg, fld)
#define TOP_OUT(reg, val)       HW_OUT(baddr,  MIPI_TOP, reg, val)
#define TOP_OUTf(reg, fld, val) HW_OUTf(baddr, MIPI_TOP, reg, fld, val)

#define TOP_MASK(reg, fld)      HWIO_MIPI_TOP_##reg##_##fld##_FLDMASK


#define DEVICE_NAME "mipicsitop"

/*
 * Linked list that contains the installed devices
 */
static LIST_HEAD(devlist_global);

/* External Clocks */
#define AHB_APB_CLK_MHZ       100
#define FPCLK_MHZ             100
#define REFCLK_MHZ            25
#define EXTCLK_MHZ            1000
#define CLKP_MIN_MHZ          40
#define CLKP_MAX_MHZ          1250

/* Internal Clocks */
#define RXCLK_MIN_MHZ         40
#define RXCLK_MAX_MHZ         1250
#define TXCLK_MIN_MHZ         RX_MIN_MHZ
#define TXCLK_MAX_MHZ         RX_MAX_MHZ
#define PLLCLK_MIN_MHZ        RX_MIN_MHZ
#define PLLCLK_MAX_MHZ        RX_MAX_MHZ

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

void top_dphy_reset(enum mipicsi_top_dev dev)
{

#ifdef MNH_EMULATION
	switch (dev) {
	case MIPI_RX0:
	case MIPI_RX1:
	case MIPI_RX2:
		mipicsi_host_dphy_reset(dev);
		break;

	case MIPI_TX0:
	case MIPI_TX1:
		mipicsi_device_dphy_reset(dev);
		break;

	default:
		break;
	}
#endif
	/* TO DO -  Reset via TOP Registers */
}

void top_dphy_write(enum mipicsi_top_dev dev,
		    uint8_t offset, uint8_t data)
{
	/* TO DO - Write for TOP Registers */
}

int mipicsi_top_write(struct mipicsi_top_reg *reg)
{
	pr_debug("writing dev %d reg 0x%x val 0x%x \n", reg->dev, reg->offset,
		 reg->value);
	writel(reg->value, (uint64_t)dev_addr_map[reg->dev] + reg->offset);

	return 0;
}

int mipicsi_top_read(struct mipicsi_top_reg *reg)
{
	pr_debug("reading dev %d reg 0x%x \n", reg->dev, reg->offset);
	
	reg->value = readl((uint64_t)dev_addr_map[reg->dev] + reg->offset);
	
	pr_debug("ret_val 0x%x \n", reg->value);

	return 0;
}

int top_set_pll(struct mipicsi_top_cfg *config)
{

	/* TO DO - Execute Gen 3 PLL write sequence on TOP registers */
#if 0
	struct mipi_pll pll;
	uint32_t pll_conf0 = 0;

	/* calculate_pll(&pll, config->mbps); */

	pll_conf0 = CPBIAS_CNTRL_SET(pll.vco_cntrol) |
		VCO_CNTRL_SET(pll.vco_cntrl) |
		N_SET(pll.n) |
		M_SET(pll.m);

	mipicsi_write(MIPI_TOP, top_reg_tbl[config->dev].pll_config0,
		       pll_conf0);
#endif
	return 0;
}

int top_start_rx(struct mipicsi_top_cfg *config)
{
	uint8_t i, vc, vc_en = 0;
	void * baddr = dev_addr_map[MIPI_TOP];
	if (!baddr) {
		pr_err("%s missing address for top\n", __func__);
		return -EINVAL;
	}

#ifdef MNH_EMULATION
	mipicsi_host_hw_init(config->dev);
	mipicsi_host_start(config);
#else /* MNH_EMULATION */
	if (config->mbps >= (RXCLK_MAX_MHZ*2))
		return -EINVAL;

	if (config->lanes >= CSI2_HOST_NUM_OF_LANES)
		return -EINVAL;

	/* TO DO Gen 3 sequence using TOP registers */
	/* TO DO Update MUX */

#endif /* MNH_EMULATION */
	/* Configure safe switching */
	for (i=0; i<MIPICSI_VC_DT_MAX_PAIRS; i++) {
		vc = config->vc_dt[i].vc;
		if ((vc > 0) && (vc <= VC_MAX))
			vc_en |= 1<<(vc-1);
	}

#if 0
	/* TO DO: Need to investigate why this causes Rx0->Tx0 bypass failure */
	if (config->dev == MIPI_RX0)
		TOP_OUTf(RX0_MODE, RX0_VC_EN, vc_en);
	else if (config->dev == MIPI_RX1)
		TOP_OUTf(RX1_MODE, RX1_VC_EN, vc_en);
	else
		TOP_OUTf(RX2_MODE, RX2_VC_EN, vc_en);
#endif
	return 0;
}

int top_start_tx(struct mipicsi_top_cfg *config)
{
	uint8_t i, vc, vc_en = 0;
	void * baddr = dev_addr_map[MIPI_TOP];
	if (!baddr) {
		pr_err("%s missing address for top\n", __func__);
		return -EINVAL;
	}

#ifdef MNH_EMULATION
	mipicsi_device_hw_init(config->dev);
	mipicsi_device_start(config);
#else /* MNH_EMULATION */
	uint32_t txdev = config->dev;
	struct mipi_csi_dev dev;

	if (config->mbps >= (TXCLK_MAX_MHZ*2))
		return -EINVAL;

	if (config->lanes >= CSI2_DEVICE_NUM_OF_LANES)
		return -EINVAL;

	top_dphy_reset(txdev);

	/* TO DO - Complete Phy and PLL Configuration with TOP registers */
	/* TO DO - Update MUX */

#endif /* MNH_EMULATION */

	/* Configure safe switching */
	for (i=0; i<MIPICSI_VC_DT_MAX_PAIRS; i++) {
		vc = config->vc_dt[i].vc;
		if ((vc > 0) && (vc <= VC_MAX))
			vc_en |= 1<<(vc-1);
	}

#if 0
	/* TO DO: Enable when Rx is resolved */
	if (config->dev == MIPI_TX0) {
		TOP_OUTf(TX0_IPU_VC_EN, TX0_IPU_VC_EN, vc_en);
		/* Enable TOP level interrupt */
		TOP_OUTf(TX0_BYPINT, TX0_INT_EN, 1);
	} else {
		TOP_OUTf(TX1_IPU_VC_EN, TX1_IPU_VC_EN, vc_en);
		/* Enable TOP level interrupt */
		TOP_OUTf(TX1_BYPINT, TX1_INT_EN, 1);
	}
#endif
	return 0;
}

int mipicsi_top_start(struct mipicsi_top_cfg *config)
{
	int ret = 0;

	pr_info("%s: E\n", __func__);

	switch (config->dev) {
	case MIPI_RX0:
	case MIPI_RX1:
	case MIPI_RX2:
		ret = top_start_rx(config);
		break;

	case MIPI_TX0:
	case MIPI_TX1:
		ret = top_start_tx(config);
		break;

	default:
		ret = -ENODEV;
	}
	/* TO DO - keep a list of configured devices */

	pr_info("%s: X\n", __func__);
	return ret;
}

static void find_tx_matching_rx_byp_and_pwr_off(uint32_t tx_bypass_sel)
{
	void * baddr = dev_addr_map[MIPI_TOP];
	uint32_t matches = 0;
	if (!baddr) {
		pr_err("%s missing address for top\n", __func__);
		return;
	}

	if (TOP_INf(TX0_MODE, TX0_BYP_SEL) == tx_bypass_sel) {
		TOP_OUTf(TX0_MODE, TX0_BYP_SEL, TX_POWER_OFF);
		matches++;
	}
	if (TOP_INf(TX1_MODE, TX1_BYP_SEL) == tx_bypass_sel) {
		TOP_OUTf(TX1_MODE, TX1_BYP_SEL, TX_POWER_OFF);
		matches++;
	}
	if (!matches) {
		pr_err("%s no match for %d\n", __func__, tx_bypass_sel);
	}
}

static void find_rx_matching_tx_and_disable(uint32_t rx_byp_tx_en_mask)
{
	void * baddr = dev_addr_map[MIPI_TOP];
	uint32_t value, matches;
	if (!baddr) {
		pr_err("%s missing address for top\n", __func__);
		return;
	}

	matches = 0;
	value = TOP_IN(RX0_MODE);
	if (value & rx_byp_tx_en_mask) {
		TOP_OUT(RX0_MODE, value & ~rx_byp_tx_en_mask);
		matches++;
	}
	value = TOP_IN(RX1_MODE);
	if (value & rx_byp_tx_en_mask) {
		TOP_OUT(RX1_MODE, value & ~rx_byp_tx_en_mask);
		matches++;
	}
	value = TOP_IN(RX2_MODE);
	if (value & rx_byp_tx_en_mask) {
		TOP_OUT(RX2_MODE, value & ~rx_byp_tx_en_mask);
		matches++;
	}
	if (!matches) {
		pr_err("%s no match for 0x%08X\n", __func__, rx_byp_tx_en_mask);
	}
}

int mipicsi_top_stop(enum mipicsi_top_dev dev)
{
	void * baddr = dev_addr_map[MIPI_TOP];
	pr_info("%s: %d E\n", __func__, dev);

	if (!baddr) {
		pr_err("%s missing address for top\n", __func__);
		return -EINVAL;
	}


	/* Turn off mux associated with the device */
	switch (dev) {

	case MIPI_RX0:
		mipicsi_host_stop(dev);
		TOP_OUT(RX0_MODE, 0);
		find_tx_matching_rx_byp_and_pwr_off(TX_BYPASS_RX0);
		break;

	case MIPI_RX1:
		mipicsi_host_stop(dev);
		TOP_OUT(RX1_MODE, 0);
		find_tx_matching_rx_byp_and_pwr_off(TX_BYPASS_RX1);
		break;

	case MIPI_RX2:
		mipicsi_host_stop(dev);
		TOP_OUT(RX2_MODE, 0);
		find_tx_matching_rx_byp_and_pwr_off(TX_BYPASS_RX2);
		break;

	case MIPI_TX0:
		mipicsi_device_stop(dev);
		TOP_OUT(TX0_MODE, 0);
		find_rx_matching_tx_and_disable(TOP_MASK(RX0_MODE, RX0_BYP_TX0_EN));
		break;

	case MIPI_TX1:
		mipicsi_device_stop(dev);
		TOP_OUT(TX1_MODE, 0);
		find_rx_matching_tx_and_disable(TOP_MASK(RX0_MODE, RX0_BYP_TX1_EN));
		break;

	default:
		return -EINVAL;
	}
	pr_info("%s: %d X\n", __func__, dev);
	return 0;
}

int mipicsi_top_set_mux(struct mipicsi_top_mux *mux)
{
	bool bypass = true;
	uint32_t tx_mode = 0, rx_mode = 0;
	void * baddr = dev_addr_map[MIPI_TOP];
	/* RX[x] masks for enabling IPU, TX0, TX1 match up.
	 * use rx0 ones for a mask to be applied to any RX[x] MODE register
	 */
	const uint32_t rxmode_en_masks =
	  TOP_MASK(RX0_MODE, RX0_IPU_EN) | 
	  TOP_MASK(RX0_MODE, RX0_BYP_TX0_EN) |
	  TOP_MASK(RX0_MODE, RX0_BYP_TX1_EN);

	pr_info("%s: E\n", __func__);
	if (!baddr) {
		return -EINVAL;
	}

	if ((mux->source == MIPI_IPU) || (mux->sink == MIPI_IPU))
		bypass = false;

	if (bypass) {
		if (mux->sink == MIPI_TX0)
			rx_mode = TOP_MASK(RX0_MODE, RX0_BYP_TX0_EN);
		else if (mux->sink == MIPI_TX1)
			rx_mode = TOP_MASK(RX0_MODE, RX0_BYP_TX1_EN);
		else
			return -EINVAL;
	}
	else {
		rx_mode = TOP_MASK(RX0_MODE, RX0_IPU_EN);
	}

	if (bypass) {
		if (mux->source == MIPI_RX0)
			tx_mode = TX_BYPASS_RX0;
		else if (mux->source == MIPI_RX1)
			tx_mode = TX_BYPASS_RX1;
		else if (mux->source == MIPI_RX2)
			tx_mode = TX_BYPASS_RX2;
		else
			return -EINVAL;
	}
	else {
		tx_mode = 0;
	}
	pr_info("%s bypass: %d, sink: %d source %d, rx_mode 0x%08X,\
                tx_mode 0x%08X\n",__func__, bypass, mux->sink, mux->source,
		rx_mode, tx_mode);

	switch (mux->source) {
	case MIPI_RX0:
		TOP_OUT(RX0_MODE,
			(TOP_IN(RX0_MODE) & ~rxmode_en_masks) | rx_mode);
		break;
	case MIPI_RX1:
		TOP_OUT(RX1_MODE,
			(TOP_IN(RX1_MODE) & ~rxmode_en_masks) | rx_mode);
		break;
	case MIPI_RX2:
		TOP_OUT(RX2_MODE,
			(TOP_IN(RX2_MODE) & ~rxmode_en_masks) | rx_mode);
		break;
	default:
		return -EINVAL;
	}

	switch (mux->sink) {
	case MIPI_TX0:
		TOP_OUTf(TX0_MODE, TX0_BYP_SEL, tx_mode);
		if (bypass) {
			TOP_OUTf(TX0_MODE, TX0_FUNC,    0);
		}
		else {
			TOP_OUTf(TX0_MODE, TX0_FUNC,    1);
		}
		break;

	case MIPI_TX1:
		TOP_OUTf(TX1_MODE, TX1_BYP_SEL, tx_mode);
		if (bypass) {
			TOP_OUTf(TX1_MODE, TX1_FUNC,    0);
		}
		else {
			TOP_OUTf(TX1_MODE, TX1_FUNC,    1);
		}
		break;

	default:
		return -EINVAL;
	}
	pr_info("%s: X\n", __func__);

	return 0;
}

void mipicsi_top_get_mux(struct mipicsi_top_mux_data *mux_data)
{
	void * baddr = dev_addr_map[MIPI_TOP];
	uint8_t i = 0;
	uint32_t reg_val;

	if (!baddr) {
		return;
	}

	/* Get all mux links requested */
	reg_val = TOP_IN(RX0_MODE);
	if (reg_val & (TOP_MASK(RX0_MODE, RX0_IPU_EN))) {
		mux_data->links[i].source = MIPI_RX0;
		mux_data->links[i].sink = MIPI_IPU;
		i++;
	}
	if (reg_val & (TOP_MASK(RX0_MODE, RX0_BYP_TX0_EN))) {
		mux_data->links[i].source = MIPI_RX0;
		mux_data->links[i].sink = MIPI_TX0;
		i++;
	}
	if (reg_val & (TOP_MASK(RX0_MODE, RX0_BYP_TX1_EN))) {
		mux_data->links[i].source = MIPI_RX0;
		mux_data->links[i].sink = MIPI_TX1;
		i++;
	}

	reg_val = TOP_IN(RX1_MODE);
	if (reg_val & (TOP_MASK(RX1_MODE, RX1_IPU_EN))) {
		mux_data->links[i].source = MIPI_RX1;
		mux_data->links[i].sink = MIPI_IPU;
		i++;
	}
	if (reg_val & (TOP_MASK(RX1_MODE, RX1_BYP_TX1_EN))) {
		mux_data->links[i].source = MIPI_RX1;
		mux_data->links[i].sink = MIPI_TX0;
		i++;
	}
	if (reg_val & (TOP_MASK(RX1_MODE, RX1_BYP_TX1_EN))) {
		mux_data->links[i].source = MIPI_RX1;
		mux_data->links[i].sink = MIPI_TX1;
		i++;
	}

	reg_val = TOP_IN(RX2_MODE);
	if (reg_val & (TOP_MASK(RX2_MODE, RX2_IPU_EN))) {
		mux_data->links[i].source = MIPI_RX2;
		mux_data->links[i].sink = MIPI_IPU;
		i++;
	}
	if (reg_val & (TOP_MASK(RX2_MODE, RX2_BYP_TX0_EN))) {
		mux_data->links[i].source = MIPI_RX2;
		mux_data->links[i].sink = MIPI_TX0;
		i++;
	}
	if (reg_val & (TOP_MASK(RX2_MODE, RX2_BYP_TX1_EN))) {
		mux_data->links[i].source = MIPI_RX2;
		mux_data->links[i].sink = MIPI_TX1;
		i++;
	}

	reg_val = TOP_IN(TX0_MODE);
	if (reg_val & (TOP_MASK(TX0_MODE, TX0_FUNC))) {
		mux_data->links[i].source = MIPI_IPU;
		mux_data->links[i].sink = MIPI_TX0;
		i++;
	}

	reg_val = TOP_IN(TX1_MODE);
	if (reg_val & (TOP_MASK(TX1_MODE, TX1_FUNC))) {
		mux_data->links[i].source = MIPI_IPU;
		mux_data->links[i].sink = MIPI_TX1;
		i++;
	}

	mux_data->count = i;
}

int mipicsi_top_get_mux_status(struct mipicsi_top_mux *mux)
{
	void * baddr = dev_addr_map[MIPI_TOP];
	int temp = 0;
	uint32_t reg_val;

	if (!baddr) {
		return -EINVAL;
	}

	/* Check if the specified mux link has taken effect */
	if (mux->source == MIPI_RX0) {
		reg_val = TOP_IN(RX0_MODE);
		if (mux->sink == MIPI_IPU)
			temp = (reg_val & RX_MODE_IPU_EN_MASK);
		else if (mux->sink == MIPI_TX0)
			temp = (reg_val & RX_MODE_TX0_BYP_EN_MASK);
		else if (mux->sink == MIPI_TX1)
			temp = (reg_val & RX_MODE_TX1_BYP_EN_MASK);
		else
			return -EINVAL;
	} else if (mux->source == MIPI_RX1) {
		reg_val = TOP_IN(RX1_MODE);
		if (mux->sink == MIPI_IPU)
			temp = (reg_val & RX_MODE_IPU_EN_MASK);
		else if (mux->sink == MIPI_TX0)
			temp = (reg_val & RX_MODE_TX0_BYP_EN_MASK);
		else if (mux->sink == MIPI_TX1)
			temp = (reg_val & RX_MODE_TX1_BYP_EN_MASK);
		else
			return -EINVAL;
	} else if (mux->source == MIPI_RX2) {
		reg_val = TOP_IN(RX2_MODE);
		if (mux->sink == MIPI_IPU)
			temp = (reg_val & RX_MODE_IPU_EN_MASK);
		else if (mux->sink == MIPI_TX0)
			temp = (reg_val & RX_MODE_TX0_BYP_EN_MASK);
		else if (mux->sink == MIPI_TX1)
			temp = (reg_val & RX_MODE_TX1_BYP_EN_MASK);
		else
			return -EINVAL;
	} else if (mux->source == MIPI_IPU) {
		if (mux->sink == MIPI_TX0)
			temp = TOP_IN(TX0_MODE) & TX_CMODE_IPU_EN_MASK;
		else if (mux->sink == MIPI_TX1)
			temp = TOP_IN(TX1_MODE) & TX_CMODE_IPU_EN_MASK;
		else
			return -EINVAL;
	} else {
		mux->active = false;
		return -EINVAL;
	}

	if (temp)
		mux->active = true;
	else
		mux->active = false;

	return 0;
}

int mipicsi_top_debug_vpg(struct mipicsi_top_vpg *vpg)
{
	enum mipicsi_top_dev dev = vpg->dev;

	if ((dev == MIPI_TX0) || (dev == MIPI_TX1)) {
		mipicsi_device_vpg(vpg);
		return 0;
	}
	return -EINVAL;
}

int mipicsi_top_hw_init(void)
{
#ifdef MNH_EMULATION
	/* Assuming two daughter cards are present and configuring
	 * Bypass mode from RX0 to TX0 by default
	 */

	struct mipicsi_top_cfg cfg;
	struct mipicsi_top_mux mux;

	cfg.dev = MIPI_RX0;
	cfg.num_lanes = 4;
	cfg.mbps = 640;
	memset(&cfg.vc_dt[0], 0, sizeof(cfg.vc_dt));

	mipicsi_top_start(&cfg);

	cfg.dev = MIPI_TX0;
	mipicsi_top_start(&cfg);

	mux.source = MIPI_RX0;
	mux.sink = MIPI_TX0;
	mipicsi_top_set_mux(&mux);

#endif
	return 0;
}

struct mipi_top_operations mipi_top_ioctl = {
	.start = mipicsi_top_start,
	.stop = mipicsi_top_stop,
	.set_mux = mipicsi_top_set_mux,
	.get_mux = mipicsi_top_get_mux,
	.get_mux_status = mipicsi_top_get_mux_status,
	.writereg = mipicsi_top_write,
	.readreg = mipicsi_top_read,
	.vpg = mipicsi_top_debug_vpg};

int mipicsi_top_init_chardev(struct mipicsi_top_device *mipidev)
{
        int ret;

        mipidev->chardev.mipidev = mipidev;
        mipidev->chardev.owner = THIS_MODULE;
        mipidev->chardev.deviceName = DEVICE_NAME;
        mipidev->chardev.topOps = mipi_top_ioctl;
        ret = mipi_chardev_init(&mipidev->chardev);
        if (ret < 0) {
                dev_err(mipidev->dev, "Could not init character device\n");
                return ret;
        }
        return 0;
}

static irqreturn_t mipicsi_top_irq(int irq, void *dev_id)
{

	struct mipicsi_top_device *dev = dev_id;
	u32 status, ind_status;
	void *baddr = dev->base_address;
	int ret = IRQ_NONE;

	spin_lock(&dev->slock);

	status = TOP_IN(TX0_BYPINT);
	if (status & TOP_MASK(TX0_BYPINT, TX0_BYP_OF)) {
		dev_info(dev->dev, "TX0_BYPINT BYP_OF occurred\n");
		/* TODO clear the interrupt*/
		ret = IRQ_HANDLED;
	}

	status = TOP_IN(TX1_BYPINT);
	if (status & TOP_MASK(TX1_BYPINT, TX1_BYP_OF)) {
		dev_info(dev->dev, "TX1_BYPINT BYP_OF occurred\n");
		/* TODO clear the interrupt*/
		ret = IRQ_HANDLED;
	}

	spin_unlock(&dev->slock);

	return ret;
}

int mipicsi_top_probe(struct platform_device *pdev)
{
	int ret = 0;
	int error = 0;
	struct mipicsi_top_device *dev;
	int irq_number = 0;
	struct resource *mem = NULL;

	dev_info(&pdev->dev, "Installing MIPI CSI-2 TOP module...\n");

	dev_info(&pdev->dev, "Device registration\n");
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		dev_err(&pdev->dev, "Could not allocated mipi_csi_top\n");
		return -ENOMEM;
	}

	/* Update the device node */
	dev->dev = &pdev->dev;

	/* Initialize dev */
	mipicsi_top_device = dev->dev;

	/* Device tree information: Base addresses & mapping */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dev->mem_size = resource_size(mem);
	if (mem == NULL) {
		dev_err(&pdev->dev, "Base address of the device is not set.\n"
			"See device tree.\n");
		error = -ENXIO;
		goto free_dev;
	}

	dev->base_address = ioremap(mem->start, resource_size(mem));

	if (!dev->base_address) {
		error = -ENOMEM;
		goto free_mem;
	}

	/* Initialize character device */
	ret = mipicsi_top_init_chardev(dev);

	dev_info(&pdev->dev, "MIPI TOP: ioremapped to %p\n",
		 dev->base_address);

	mipicsi_util_save_virt_addr(MIPI_TOP, dev->base_address);

	/* dev_info(&pdev->dev, "Intel Device at 0x%08x\n",
	 * (unsigned int)dev->base_address);
	 */

	/* Init locks */
	dev_info(&pdev->dev, "Init locks\n");
	spin_lock_init(&dev->slock);

	/* Init mutex */
	dev_info(&pdev->dev, "Init mutex\n");
	mutex_init(&dev->mutex);

	/* Device tree information: Get interrupts numbers */
	irq_number = platform_get_irq(pdev, 0);
	if (irq_number > 0) {
		/* Register interrupt */
		ret = request_irq(dev->top_irq_number, mipicsi_top_irq,
				IRQF_SHARED, dev_name(&pdev->dev), dev);
		if (ret)
			dev_err(&pdev->dev, "Could not register top interrupt\n");
	} else
		dev_err(&pdev->dev, "IRQ num not set. See device tree.\n");

	dev->top_irq_number = irq_number;


	/* Now that everything is fine, let's add it to device list */
	list_add_tail(&dev->devlist, &devlist_global);

	/* HW init */
	ret = mipicsi_top_hw_init();

	mipicsi_sysfs_init(mipicsi_top_device);

	return ret;
 free_mem:
	iounmap(dev->base_address);

 free_dev:
	return error;

}

/*
 * Exit routine - Exit point of the driver
 */
static int mipicsi_top_remove(struct platform_device *pdev)
{
	struct mipicsi_top_device *dev;
	struct list_head *list;

	dev_dbg(&pdev->dev, "Removing MIPI CSI-2 module\n");
	mipicsi_sysfs_clean(mipicsi_top_device);
	while (!list_empty(&devlist_global)) {
		list = devlist_global.next;
		list_del(list);
		dev = list_entry(list, struct mipicsi_top_device, devlist);

		devm_free_irq(&pdev->dev, dev->top_irq_number, dev);
		devm_free_irq(&pdev->dev, dev->top_irq, dev);

		iounmap(dev->base_address);
	}
	return 0;
}


/*
 * of_device_id structure
 */
static const struct of_device_id mipicsi_top[] = {
	{ .compatible = "intel,mipicsi_top" },
	{ }
};

MODULE_DEVICE_TABLE(of, mipicsi_top);
/*
 * Platform driver structure
 */
static struct platform_driver __refdata mipicsi_top_pdrv = {
	.remove = mipicsi_top_remove,
	.probe  = mipicsi_top_probe,
	.driver   = {
		.name   = "intel, mipicsi_top",
		.owner = THIS_MODULE,
		.of_match_table = mipicsi_top,
	},
};

module_platform_driver(mipicsi_top_pdrv);
