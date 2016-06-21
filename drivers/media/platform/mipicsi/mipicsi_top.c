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
#include "mipicsi_top.h"
#include "mipicsi_host.h"
#include "mipicsi_device.h"
#include "mipicsi_util.h"

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

struct top_regs {
	uint32_t mode;
	uint32_t bypint;
	uint32_t pll_config0;
	uint32_t pll_config1;
	uint32_t pll_cntrl;
	uint32_t pll_status;
	uint32_t dphy_config;
	uint32_t dphy_cntrl;
	uint32_t iotest;
};

struct top_regs top_reg_tbl[MIPI_MAX] = {
	[MIPI_RX0] = {RX0_MODE, R_INVALID, RX0_DPHY_PLL_CONFIG0,
		      RX0_DPHY_PLL_CONFIG1, RX0_DPHY_PLL_CNTRL,
		      RX0_DPHY_PLL_STATUS, RX0_DPHY_CONFIG,
		      RX0_DPHY_CONTROL, RX0_DPHY_IOTEST},
	[MIPI_RX1] = {RX1_MODE, R_INVALID, R_INVALID,
		      R_INVALID, R_INVALID,
		      R_INVALID, RX1_DPHY_CONFIG,
		      RX1_DPHY_CONTROL, RX1_DPHY_IOTEST},
	[MIPI_RX2] = {RX2_MODE, R_INVALID, R_INVALID,
		      R_INVALID, R_INVALID,
		      R_INVALID, RX2_DPHY_CONFIG,
		      RX2_DPHY_CONTROL, RX2_DPHY_IOTEST},
	[MIPI_TX0] = {TX0_MODE, TX0_BYPINT, TX0_DPHY_PLL_CONFIG0,
		      TX0_DPHY_PLL_CONFIG1, TX0_DPHY_PLL_CNTRL,
		      TX0_DPHY_PLL_STATUS, TX0_DPHY_CONFIG,
		      TX0_DPHY_CONTROL, TX0_DPHY_IOTEST},
	[MIPI_TX1] = {TX1_MODE, TX1_BYPINT, TX1_DPHY_PLL_CONFIG0,
		      TX1_DPHY_PLL_CONFIG1, TX1_DPHY_PLL_CNTRL,
		      TX1_DPHY_PLL_STATUS, TX1_DPHY_CONFIG,
		      TX1_DPHY_CONTROL, TX1_DPHY_IOTEST}
};

uint32_t top_read(enum mipicsi_top_dev dev, uint32_t offset)
{
	if (offset != R_INVALID)
		return mipicsi_read(dev, offset);
	else
		return -EINVAL;
}

uint32_t top_write(enum mipicsi_top_dev dev, uint32_t offset, uint32_t data)
{
	if (offset != R_INVALID) {
		mipicsi_write(dev, offset, data);
		return 0;
	} else {
		return -EINVAL;
	}
}


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


int32_t top_set_pll(struct mipicsi_top_cfg *config)
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


int32_t top_start_rx(struct mipicsi_top_cfg *config)
{
#ifdef MNH_EMULATION
	mipicsi_host_hw_init(config->dev);
	mipicsi_host_start(config);
#else /* MNH_EMULATION */
	if (config->mbps >= (RXCLK_MAX_MHZ*2))
		return -EINVAL;

	if (config->lanes >= CSI2_DEVICE_NUM_OF_LANES)
		return -EINVAL;

	/* TO DO Gen 3 sequence using TOP registers */
	/* TO DO Update MUX */

#endif /* MNH_EMULATION */
	return 0;
}

int32_t top_start_tx(struct mipicsi_top_cfg *config)
{
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
	return 0;
}

int32_t mipicsi_top_start(struct mipicsi_top_cfg *config)
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

int32_t mipicsi_top_stop(enum mipicsi_top_dev dev)
{
	uint32_t clr_tx_mode = 0, clr_rx_mode = 0, temp = 0;

	pr_info("%s: E\n", __func__);

	top_write(MIPI_TOP, top_reg_tbl[dev].dphy_cntrl, ~SHUTDOWNZ_N);

	/* Turn off mux associated with the device */
	switch (dev) {
	case MIPI_RX0:
		clr_tx_mode = TX_BYP_SEL_SET(TX_BYPASS_RX0);
		break;
	case MIPI_RX1:
		clr_tx_mode = TX_BYP_SEL_SET(TX_BYPASS_RX1);
		break;
	case MIPI_RX2:
		clr_tx_mode = TX_BYP_SEL_SET(TX_BYPASS_RX2);
		break;
	case MIPI_TX0:
		clr_rx_mode = RX_BYP_Tx0_EN;
		break;
	case MIPI_TX1:
		clr_rx_mode = RX_BYP_Tx1_EN;
		break;
	default:
		return -EINVAL;
	}

	/* Clear mode for current device */
	top_write(MIPI_TOP, top_reg_tbl[dev].mode, 0);

	/* Clear connections to other devices */
	if (clr_tx_mode != 0) {
		temp = top_read(MIPI_TOP, top_reg_tbl[MIPI_TX0].mode);
		if ((temp & TX_BYP_SEL_MASK) == clr_tx_mode) {
			temp &= ~TX_BYP_SEL_MASK;
			top_write(MIPI_TOP, top_reg_tbl[MIPI_TX0].mode, temp);
		}
		temp = top_read(MIPI_TOP, top_reg_tbl[MIPI_TX1].mode);
		if ((temp & TX_BYP_SEL_MASK) == clr_tx_mode) {
			temp &= ~TX_BYP_SEL_MASK;
			top_write(MIPI_TOP, top_reg_tbl[MIPI_TX1].mode, temp);
		}
	}
	if (clr_rx_mode != 0) {
		temp = top_read(MIPI_TOP, top_reg_tbl[MIPI_RX0].mode);
		temp &= ~clr_rx_mode;
		top_write(MIPI_TOP, top_reg_tbl[MIPI_RX0].mode, temp);

		temp = top_read(MIPI_TOP, top_reg_tbl[MIPI_RX1].mode);
		temp &= ~clr_rx_mode;
		top_write(MIPI_TOP, top_reg_tbl[MIPI_RX1].mode, temp);

		temp = top_read(MIPI_TOP, top_reg_tbl[MIPI_RX2].mode);
		temp &= ~clr_rx_mode;
		top_write(MIPI_TOP, top_reg_tbl[MIPI_RX2].mode, temp);
	}
	pr_info("%s: X\n", __func__);

	return 0;
}

int mipicsi_top_set_mux(struct mipicsi_top_mux *mux)
{
	bool bypass = true;
	uint32_t tx_mode = 0, rx_mode = 0, temp = 0;

	pr_info("%s: E\n", __func__);

	if ((mux->source == MIPI_IPU) || (mux->sink != MIPI_IPU))
		bypass = false;

	switch (mux->source) {
	case MIPI_RX0:
	case MIPI_RX1:
	case MIPI_RX2:
		if (bypass) {
			if (mux->sink == MIPI_TX0)
				rx_mode = RX_BYP_Tx0_EN;
			else if (mux->sink == MIPI_TX1)
				rx_mode = RX_BYP_Tx1_EN;
			else
				return -EINVAL;
		} else {
			rx_mode = RX_IPU_EN;
		}
		break;

	default:
		return -EINVAL;
	}

	switch (mux->sink) {
	case MIPI_TX0:
	case MIPI_TX1:
		if (bypass) {
			if (mux->source == MIPI_RX0)
				tx_mode = TX_BYP_SEL_SET(TX_BYPASS_RX0);
			else if (mux->source == MIPI_RX1)
				tx_mode = TX_BYP_SEL_SET(TX_BYPASS_RX1);
			else if (mux->source == MIPI_RX2)
				tx_mode = TX_BYP_SEL_SET(TX_BYPASS_RX2);
			else
				return -EINVAL;
		} else
			tx_mode = TX_FUNC_EN;
		break;
	default:
		return -EINVAL;
	}

	if (rx_mode != 0) {
		temp = top_read(MIPI_TOP, top_reg_tbl[mux->source].mode);
		rx_mode |= temp;
		top_write(MIPI_TOP, top_reg_tbl[mux->source].mode, rx_mode);
	}
	if (tx_mode != 0) {
		temp = top_read(MIPI_TOP, top_reg_tbl[mux->sink].mode);
		tx_mode |= temp;
		top_write(MIPI_TOP, top_reg_tbl[mux->sink].mode, tx_mode);
	}

	pr_info("%s: X\n", __func__);

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
	cfg.mbps = 1500;
	mipicsi_top_start(&cfg);

	cfg.dev = MIPI_TX0;
	mipicsi_top_start(&cfg);

	mux.source = MIPI_RX0;
	mux.sink = MIPI_TX0;
	mipicsi_top_set_mux(&mux);

#endif
	return 0;
}

void mipicsi_top_get_mux(struct mipicsi_top_mux_data *mux_data)
{
	/* TO DO */
}

int mipicsi_top_probe(struct platform_device *pdev)
{
	int ret = 0;
	int error = 0;
	struct resource *mem = NULL;
	struct mipicsi_top_device *dev;
	int irq_number = 0;
#ifdef JUNO_BRINGUP
	void *iomem;
#endif

	dev_info(&pdev->dev, "Installing MIPI CSI-2 TOP module...\n");

	dev_info(&pdev->dev, "Device registration\n");
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		dev_err(&pdev->dev, "Could not allocated mipi_csi_top\n");
		return -ENOMEM;
	}

	/* Update the device node */
	dev->dev = &pdev->dev;

#ifdef JUNO_BRINGUP
	dev_info(dev->dev, "Creating bogus memregion for PO\n");
	iomem = devm_kzalloc(dev->dev, (unsigned int)dev->mem_size,
		GFP_KERNEL);
	dev_info(dev->dev, "Allocated %p\n", iomem);
	dev->base_address = iomem;
	dev_info(dev->dev, "MIPI TOP at %p\n",
		 dev->base_address);
        pr_info("MIPI TOP: juno bringup %p\n", dev->base_address);
#else

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
#endif
	pr_info("MIPI TOP: ioremapped to %p\n", dev->base_address);
	mipicsi_util_save_virt_addr(MIPI_TOP, dev->base_address);

	/* dev_info(&pdev->dev, "Intel Device at 0x%08x\n",
	 * (unsigned int)dev->base_address);
	 */


	/* Device tree information: Get interrupts numbers */
	irq_number = platform_get_irq(pdev, 0);
#if 0
	if (irq_number <= 0) {
		dev_err(&pdev->dev, "IRQ num not set. See device tree.\n");
		error = -EINVAL;
		goto free_mem;
	}
#endif
	dev->top_irq_number = irq_number;

	/* Init locks */
	dev_info(&pdev->dev, "Init locks\n");
	spin_lock_init(&dev->slock);

	/* Init mutex */
	dev_info(&pdev->dev, "Init mutex\n");
	mutex_init(&dev->mutex);

	/* Now that everything is fine, let's add it to device list */
	list_add_tail(&dev->devlist, &devlist_global);

	/* HW init */
	ret = mipicsi_top_hw_init();
#if 0
	if (ret) {
		dev_err(&pdev->dev, "Could not init Intel MIPI TOP %d\n", ret);
		goto unreg_dev;
	}

	/* Register interrupt */
	ret = devm_request_irq(&pdev->dev, dev->top_irq,
				   mipicsi_top_irq1, IRQF_SHARED,
				   dev_name(&pdev->dev), dev);
	if (ret) {
		dev_err(&pdev->dev, "Couldn't register controller interrupt\n");
		goto unreg_dev;
	}

	irq_number = platform_get_irq(pdev, 1);
	if (irq_number <= 0) {
		dev_err(&pdev->dev, "IRQ number not set. See device tree.\n");
		error = -EINVAL;
		goto unreg_dev;
	}

	/* Device tree information  -- */
	/* TO DO  Read version of TOP to determine emulation vs silicon */
	if (of_property_read_u32(node, "version",
				 &dev->hw.version)) {
		dev_err(&pdev->dev, "Couldn't read version\n");
		goto unreg_dev;
	}
#endif
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
