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
#include <linux/list.h>
#include <linux/kthread.h>
#include <linux/string.h>
#include "mipicsi_device.h"
#include "mipicsi_util.h"
#include "mipicsi_dc_dphy.h"
#include "mipicsi_pll.h"

#include <linux/intel-hwio.h>
#include <soc/mnh/mnh-hwio-mipi-tx.h>
extern void * dev_addr_map[];
/*
 * Linked list that contains the installed devices
 */
static LIST_HEAD(devlist_global);

/* these macros assume a void * in scope named baddr */
#define TX_IN(reg)             HW_IN(baddr,   MIPI_TX, reg)
#define TX_OUT(reg, val)       HW_OUT(baddr,  MIPI_TX, reg, val)
#define TX_OUTf(reg, fld, val) HW_OUTf(baddr, MIPI_TX, reg, fld, val)

#define TX_MASK(reg, fld)      HWIO_MIPI_TX_##reg##_##fld##_FLDMASK

void mipicsi_dev_dphy_write(enum mipicsi_top_dev dev,
			    uint8_t command, uint8_t data)
{
	/* Consider passing in the base address
	 * rather than a lookup in a table.
	 */
	void * baddr = dev_addr_map[dev];

	if (!baddr) {
		pr_err("%s: no address for %d\n", __func__, dev);
		return;
	}

	pr_info("%s: dev=0x%x @ %p, command 0x%02X data=0x%02X\n",
		__func__, dev, baddr, command, data);

	TX_OUT(PHY_RSTZ, 0);
	TX_OUTf(PHY0_TST_CTRL0, PHY0_TESTCLR, 0);
	TX_OUTf(PHY0_TST_CTRL0, PHY0_TESTCLK, 1);
	TX_OUTf(PHY0_TST_CTRL1, PHY0_TESTDIN, command);
	TX_OUTf(PHY0_TST_CTRL1, PHY0_TESTEN,  1);
	TX_OUTf(PHY0_TST_CTRL0, PHY0_TESTCLK, 0);

	TX_OUTf(PHY0_TST_CTRL1, PHY0_TESTEN,  0);
	TX_OUTf(PHY0_TST_CTRL1, PHY0_TESTDIN, data);

	TX_OUTf(PHY0_TST_CTRL0, PHY0_TESTCLK, 1);
	/*
	  I thought we needed one more TESTCLK to low
	  but tested scripts don't have it.
	  TX_OUTf(PHY0_TST_CTRL0, PHY0_TESTCLK, 0);
	*/
	//pr_info("%s: X\n", __func__);

	udelay(1);
}

int mipicsi_dev_dphy_write_set(enum mipicsi_top_dev dev, uint32_t offset,
			       uint8_t data, uint8_t ps, uint8_t ps_width)
{
	uint32_t temp;

	/* Check if the incoming data is valid: data should have a maximum
	 * value of 8 bits minus the program selector width
	 * For example: if program selector is 1 bit, the data can be 7 bits
	 * ps can be a maximum of 2 bits
	 */
	if (data >= (1<<(8-ps_width)))
		return -EINVAL;

	if (ps > 3)
		return -EINVAL;

	if (ps_width > 2)
		return -EINVAL;

	/* Set the most significant bits to the target program selector
	 * and place the data in the remaining bits
	 */
	temp = ((ps << (8-ps_width)) | data);
	mipicsi_dev_dphy_write(dev, offset, temp);
	return 0;
}


void mipicsi_device_reset(enum mipicsi_top_dev dev)
{
	void * baddr = dev_addr_map[dev];
	if (!baddr) {
		pr_err("%s: no address for %d\n", __func__, dev);
		return;
	}
	pr_info("%s %d\n", __func__, dev);
	TX_OUT(CSI2_RESETN, 0);
	udelay(1000);
	TX_OUT(CSI2_RESETN, 1);
}


void mipicsi_device_dphy_reset(enum mipicsi_top_dev dev)
{
	void * baddr = dev_addr_map[dev];
	if (!baddr) {
		pr_err("%s: no address for %d\n", __func__, dev);
		return;
	}
	pr_info("%s %d\n", __func__, dev);
	TX_OUTf(PHY_RSTZ, PHY_RSTZ, 1);
	udelay(1000);
	TX_OUTf(PHY_RSTZ, PHY_RSTZ, 0);
}

int32_t mipicsi_device_set_pll(struct mipicsi_top_cfg *config)
{
	enum mipicsi_top_dev dev;
	struct mipicsi_pll pll;
	uint32_t val;

	dev = config->dev;

#ifdef MNH_EMULATION

	mipicsi_pll_calc(config->mbps, &pll);


	mipicsi_dev_dphy_write(dev, R_CSI2_DCPHY_HS_RX_CTRL_L0,
			       (pll.hsfreq << 1));

	mipicsi_dev_dphy_write(dev, R_CSI2_DCPHY_PLL_BIAS_FCC_VCO, (0x1 << 7) |
			       (pll.vco_range << 3) | 1);

	mipicsi_dev_dphy_write(dev, R_CSI2_DCPHY_PLL_LPF_CP_CTRL, (0x01 << 7) |
			       (0x01 << 6) | (pll.lpf_resistor << 0));

	mipicsi_dev_dphy_write(dev, R_CSI2_DCPHY_PLL_CP_LOCK_BYP_ULP,
			       (0x00 << 4) | (pll.cp_current << 0));

	/* Program log2(output divider) to register */
	val = ilog2(pll.output_div);
	mipicsi_dev_dphy_write(dev, R_CSI2_DCPHY_PLL_DIV_RAT_CTRL,
			       ((0x1<<5) | (0x1<<4) | (val<<0)));

	/* Program N-1 to register */
	mipicsi_dev_dphy_write(dev, R_CSI2_DCPHY_PLL_INPUT_DIV_RAT,
			       pll.input_div-1);

	/* Program M-2 to register */
	mipicsi_dev_dphy_write_set(dev, R_CSI2_DCPHY_PLL_LOOP_DIV_RAT,
				   (((pll.loop_div-2) >> 0) & 0x1F), 0, 1);
	mipicsi_dev_dphy_write_set(dev, R_CSI2_DCPHY_PLL_LOOP_DIV_RAT,
				   (((pll.loop_div-2) >> 5) & 0x1F), 1, 1);

#else

	if (config->mbps <= 800) {
		pr_info("%s: Setting bitrate to 800mbps",__func__);
		mipicsi_dev_dphy_write (dev, 0x179, 0x9E);
		mipicsi_dev_dphy_write (dev, 0x17A, 0x00);
		mipicsi_dev_dphy_write (dev, 0x17B, 0x9F);
		mipicsi_dev_dphy_write (dev, 0x178, 0xC8);
		mipicsi_dev_dphy_write (dev, 0x15E, 0x10);
		mipicsi_dev_dphy_write (dev, 0x162, 0x04);
		mipicsi_dev_dphy_write (dev, 0x16E, 0x0C);
	} else if (config->mbps <= 1000) {
		pr_info("%s: Setting bitrate to 1000mbps",__func__);
		mipicsi_dev_dphy_write (dev, 0x179, 0x4E);
		mipicsi_dev_dphy_write (dev, 0x17A, 0x00);
		mipicsi_dev_dphy_write (dev, 0x17B, 0x93);
		mipicsi_dev_dphy_write (dev, 0x178, 0x98);
		mipicsi_dev_dphy_write (dev, 0x15E, 0x10);
		mipicsi_dev_dphy_write (dev, 0x162, 0x04);
		mipicsi_dev_dphy_write (dev, 0x16E, 0x0C);
	} else if (config->mbps <= 1500) {
		pr_info("%s: Setting bitrate to 1500mbps",__func__);
		mipicsi_dev_dphy_write (dev, 0x179, 0x2A);
		mipicsi_dev_dphy_write (dev, 0x17A, 0x01);
		mipicsi_dev_dphy_write (dev, 0x17B, 0x87);
		mipicsi_dev_dphy_write (dev, 0x178, 0xC8);
		mipicsi_dev_dphy_write (dev, 0x15E, 0x10);
		mipicsi_dev_dphy_write (dev, 0x162, 0x04);
		mipicsi_dev_dphy_write (dev, 0x16E, 0x0C);
	} else if (config->mbps <= 2500) {
		pr_info("%s: Setting bitrate to 2500mbps",__func__);
		mipicsi_dev_dphy_write (dev, 0x179, 0xF2);
		mipicsi_dev_dphy_write (dev, 0x17A, 0x01);
		mipicsi_dev_dphy_write (dev, 0x17B, 0x83);
		mipicsi_dev_dphy_write (dev, 0x178, 0xC8);
		mipicsi_dev_dphy_write (dev, 0x15E, 0x10);
		mipicsi_dev_dphy_write (dev, 0x162, 0x04);
		mipicsi_dev_dphy_write (dev, 0x16E, 0x0C);
	}
#endif
	return 0;
}

int mipicsi_device_vpg(struct mipicsi_top_vpg *vpg)
{
	enum mipicsi_top_dev dev = vpg->dev;
	void * baddr = dev_addr_map[dev];
	if (!baddr) {
		pr_err("%s: no address for %d\n", __func__, dev);
		return -ENXIO;
	}

	TX_OUT(VPG_MODE_CFG, vpg->mode_cfg);
	TX_OUT(VPG_PKT_CFG, vpg->pkt_cfg);
	TX_OUT(VPG_PKT_SIZE, vpg->pkt_size);
	TX_OUT(VPG_HSA_TIME, vpg->hsa_time);
	TX_OUT(VPG_HBP_TIME, vpg->hbp_time);
	TX_OUT(VPG_HLINE_TIME, vpg->hline_time);
	TX_OUT(VPG_VSA_LINES, vpg->vsa_lines);
	TX_OUT(VPG_VBP_LINES, vpg->vbp_lines);
	TX_OUT(VPG_VFP_LINES, vpg->vfp_lines);
	TX_OUT(VPG_ACT_LINES, vpg->act_lines);
	TX_OUT(VPG_MAX_FRAME_NUM, vpg->max_frame);
	TX_OUT(VPG_START_LINE_NUM, vpg->start_line);
	TX_OUT(VPG_STEP_LINE_NUM, vpg->step_line);

	TX_OUTf(VPG_CTRL, VPG_EN, 1);
	return 0;
}


int mipicsi_device_start(struct mipicsi_top_cfg *config)
{
	uint32_t data = 0, val = 0;
	uint8_t counter = 0;
	enum mipicsi_top_dev dev = config->dev;
	void * baddr = dev_addr_map[dev];
	const uint32_t stop_mask =
		TX_MASK(PHY_STATUS, TXSTOPSTATE_CLK) |
		TX_MASK(PHY_STATUS, TXSTOPSTATE_L0) |
		TX_MASK(PHY_STATUS, TXSTOPSTATE_L1) | 
		TX_MASK(PHY_STATUS, TXSTOPSTATE_L2) |
		TX_MASK(PHY_STATUS, TXSTOPSTATE_L3);

	if (!baddr) {
		pr_err("%s: no address for %d\n", __func__, dev);
		return -ENXIO;
	}

	if ((dev != MIPI_TX0) && (dev != MIPI_TX1)) {
		pr_err("%s unexpected dev %d\n", __func__, dev);
		return -EINVAL;
	}
	pr_info("%s: dev: %d\n", __func__, dev);

#ifdef MNH_EMULATION
	TX_OUTf(CSI2_RESETN,    CSI2_RESETN_RW, 1);
	TX_OUT(PHY_RSTZ,        0);
	TX_OUTf(PHY_RSTZ,       PHY_ENABLECLK,  1);
	/* set TESTCLR to HIGH */
	TX_OUT(PHY0_TST_CTRL0,  1);

	/* Apply the appropriate frequency to the REFCLK signal; for correct
	 * values, refer to Table 6-1 on page 91
	 */
	/* Hardware controlled */

	/* Apply the appropriate frequency to the CFG_CLK signal; for correct
	 * values, refer to Table 12-5
	 */
	/* Hardware controlled */

       /* Set MASTERSLAVEZ = 1 for Master mode selection (1'b0 for Slave mode
        * selection).
        */
	mipicsi_dev_dphy_write(dev, R_CSI2_DCPHY_MASTER_SLAVEZ, 0x0E);

	/* Enable lanes */
	TX_OUTf(PHY_IF_CFG, LANE_EN_NUM, config->num_lanes-1);

	/* Set BASEDIR_N to the desired values */
	mipicsi_dev_dphy_write(dev, R_CSI2_DCPHY_BASEDIR_L0, DC_TX_BASEDIR_VAL);
	mipicsi_dev_dphy_write(dev, R_CSI2_DCPHY_BASEDIR_L1, DC_TX_BASEDIR_VAL);
	mipicsi_dev_dphy_write(dev, R_CSI2_DCPHY_BASEDIR_L2, DC_TX_BASEDIR_VAL);
	mipicsi_dev_dphy_write(dev, R_CSI2_DCPHY_BASEDIR_L3, DC_TX_BASEDIR_VAL);

	/* Wait for 15 ns */
	udelay(1);
	/* Configure the TESTCLR to LO */
	//TX_OUTf(PHY0_TST_CTRL0, PHY0_TESTCLR, 0);
	TX_OUT(PHY0_TST_CTRL0, 0);

	/* Wait for 35 ns */
	udelay(1);

	/* Configure PLL */
	mipicsi_device_set_pll(config);

	/* Wait 5 ns */
	udelay(1);
	TX_OUTf(LPCLK_CTRL, PHY_TXREQCLKHS_CON, 0);
	/* Configure Test Code 0x22
	 * Bitrate configuration
	 * Band Gap reference voltage
	 * BIASEXTR internal resistor control
	 * LPTX bias current control
	 */
	val = 0x04 | (1<<7) | (0x3<<11) | (0x6<<15);
	if (config->mbps > 1000)
		val |= (1<<10);

	mipicsi_dev_dphy_write_set(dev, R_CSI2_DCPHY_AFE_BYPASS_BANDGAP,
				   ((val >> 12) & 0x3F), 2, 2);
	mipicsi_dev_dphy_write_set(dev, R_CSI2_DCPHY_AFE_BYPASS_BANDGAP,
				   ((val >> 6) & 0x3F), 1, 2);
	mipicsi_dev_dphy_write_set(dev, R_CSI2_DCPHY_AFE_BYPASS_BANDGAP,
				   ((val >> 0) & 0x3F), 0, 2);

	//mipicsi_dev_dphy_write (dev, R_CSI2_DCPHY_LP_TX_PWR_CTRL_CLK, 0x0F);

	/* Configure clock and data lane timings */
	mipicsi_dev_dphy_write (dev, R_CSI2_DCPHY_HS_TX_CLK_TLP, 0x83);
	mipicsi_dev_dphy_write (dev, R_CSI2_DCPHY_HS_TX_CLK_TCLK_PREP, 0x85);
	mipicsi_dev_dphy_write (dev, R_CSI2_DCPHY_HS_TX_CLK_TCLK_ZERO, 0x93);
	mipicsi_dev_dphy_write (dev, R_CSI2_DCPHY_HS_TX_CLK_TCLK_TRAIL, 0x86);
	mipicsi_dev_dphy_write (dev, R_CSI2_DCPHY_HS_TX_CLK_THS_EXIT, 0x2B);
	mipicsi_dev_dphy_write (dev, R_CSI2_DCPHY_HS_TX_CLK_TCLK_POST, 0x2C);
	mipicsi_dev_dphy_write (dev, R_CSI2_DCPHY_HS_TX_DATA_TLP, 0x83);
	mipicsi_dev_dphy_write (dev, R_CSI2_DCPHY_HS_TX_DATA_THS_PREP, 0x84);
	mipicsi_dev_dphy_write (dev, R_CSI2_DCPHY_HS_TX_DATA_THS_ZERO, 0x89);
	mipicsi_dev_dphy_write (dev, R_CSI2_DCPHY_HS_TX_DATA_THS_TRAIL, 0x85);
	mipicsi_dev_dphy_write (dev, R_CSI2_DCPHY_HS_TX_DATA_THS_EXIT, 0x3F);

	TX_OUT(PHY_RSTZ, 0x07);
	udelay(1);

#else
	TX_OUTf(CSI2_RESETN,    CSI2_RESETN_RW, 1);
	/* Set rstz = 1'b0 */
	/* Set shutdownz= 1'b0 */
	TX_OUT(PHY_RSTZ,        0);
	TX_OUTf(PHY_RSTZ,       PHY_ENABLECLK,  1);
	/* Set testclr = 1'b1 */
	TX_OUT(PHY0_TST_CTRL0,  1);

	/* Wait for 15 ns */
	udelay(1);

	/* Set testclr to low; */
	TX_OUT(PHY0_TST_CTRL0, 0);

	/* Set hsfreqrange[6:0] = 7'b0001010 */
	/* Refer to table "Slew rate vs DDL oscilation target" on page 117 and
	 * configure test control registers with appropriate values for the
	 * specified rise/fall time. */
	mipicsi_dev_dphy_write (dev, 0x270, 0xE2);
	mipicsi_dev_dphy_write (dev, 0x271, 0x04);
	mipicsi_dev_dphy_write (dev, 0x272, 0x11);

	/* Set cfgclkfreqrange[7:0] = round[ (Fcfg_clk(MHz)-17)*4] = 8'b10000100
	   assuming cfg_clk = 50MHz; */

	/* Apply cfg_clk signal with 50Mhz frequency */
	/* Hardware controlled */

	/* Configure PLL operating frequency through D-PHY test control registers or through PLL
	 * SoC shadow registers interface as described in section "Initialization" on page 53
	 */
	mipicsi_device_set_pll(config);

	/* Set basedir_0 = 1'b0 */
	/* Hardware controlled */

	/* Set all requests inputs to zero; The purpose is to ensure that the following signals
	 * are set to low logic level: txrequesthsclk, txrequestdatahs_0/1/2/3,
	 * txrequestesc_0/1/2/3 and turnrequest_0;
	 */
	/* Hardware controlled */
	udelay(1);

	/* Wait for 15ns */
	/* Enable lanes */
	TX_OUTf(PHY_IF_CFG, LANE_EN_NUM, config->num_lanes-1);

	/* Enableclk=1'b1; Wait 5ns; Set shutdownz=1'b1;  Wait 5ns; Set rstz=1'b1; */
	TX_OUTf(PHY_RSTZ, PHY_ENABLECLK, 0x01);
	udelay(1);
	TX_OUTf(PHY_RSTZ, PHY_SHUTDOWNZ, 0x01);
	udelay(1);
	TX_OUTf(PHY_RSTZ, PHY_RSTZ, 0x01);

#endif
	/* Wait until the STOPSTATEDATA_N and STOPSTATECLK outputs are asserted.
	 * At this point, the PLL has already locked (for the Master) and the
	 * initialization of the analog drivers has completed. From this point,
	 * the REQUEST inputs can be set according to the desired transmission
	 * -- poll for 200 us
	 */
	do {
		data = TX_IN(PHY_STATUS);
		if ((data & stop_mask) == stop_mask) {
			pr_info("%s: X\n", __func__);
			return 0;
		}

		udelay(10);
		counter++;
	} while (counter < 20);
	pr_info("%s counter: %d 0x%08X\n",
		__func__, counter, data);

	return 0;
}

int mipicsi_device_stop(enum mipicsi_top_dev dev)
{
	void * baddr = dev_addr_map[dev];
	if (!baddr) {
		return -ENXIO;
	}
	TX_OUT(PHY_RSTZ, 0);
	TX_OUT(PHY0_TST_CTRL0, 1);

	return 0;
}

int mipicsi_device_hw_init(enum mipicsi_top_dev dev)
{
	void * baddr = dev_addr_map[dev];
	if (!baddr) {
		pr_err("%s: no address for %d\n", __func__, dev);
		return -ENXIO;
	}

	pr_info("%s %d version: 0x%08X\n",
		__func__, dev, TX_IN(VERSION));
	TX_OUTf(PHY_RSTZ, PHY_SHUTDOWNZ, 1);
	mipicsi_device_dphy_reset(dev);

	mipicsi_device_reset(dev);

	TX_OUT(INT_MASK_N_VPG, 0xFFFFFFFF);
	TX_OUT(INT_MASK_N_IDI, 0xFFFFFFFF);

	TX_OUTf(PHY_RSTZ, PHY_SHUTDOWNZ, 0);

	return 0;
}

static irqreturn_t mipicsi_device_irq(int irq, void *dev_id)
{

	struct mipicsi_device_dev *dev = dev_id;
	u32 int_status, ind_status;
	void *baddr = dev->base_address;
	int ret = IRQ_NONE;

	spin_lock(&dev->slock);

	int_status = TX_IN(INT_ST_MAIN);

	if (int_status & TX_MASK(INT_ST_MAIN, INT_ST_VPG)) {
		ind_status = TX_IN(INT_ST_VPG);
		dev_info(dev->dev, "CSI INT_ST_VPG: %x\n", ind_status);
		ret = IRQ_HANDLED;
	}

	if (int_status & TX_MASK(INT_ST_MAIN, INT_ST_IDI)) {
		ind_status = TX_IN(INT_ST_IDI);
		dev_info(dev->dev, "CSI INT_ST_IDI: %x\n", ind_status);
		ret = IRQ_HANDLED;
	}

	if (int_status & TX_MASK(INT_ST_MAIN, INT_ST_PHY)) {
		ind_status = TX_IN(INT_ST_PHY);
		dev_info(dev->dev, "CSI INT_ST_PHY: %x\n", ind_status);
		ret = IRQ_HANDLED;
	}

	spin_unlock(&dev->slock);

	return ret;
}

int mipicsi_device_probe(struct platform_device *pdev)
{
	int ret = 0;
	int error = 0;
	struct mipicsi_device_dev *dev;
	int irq_number = 0;
	struct resource *mem = NULL;

	dev_info(&pdev->dev, "Installing MIPI CSI-2 DEVICE module...\n");

	dev_info(&pdev->dev, "Device registration\n");
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		dev_err(&pdev->dev, "Could not allocated mipicsi_device\n");
		return -ENOMEM;
	}

	/* Update the device node */
	dev->dev = &pdev->dev;

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

	pr_info("MIPI DEV: ioremapped to %p\n", dev->base_address);
	mipicsi_util_save_virt_addr(MIPI_TX0, dev->base_address);

	/* dev_info(&pdev->dev, "SNPS Device at 0x%08x\n",
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
		dev->device_irq_number = irq_number;

		/* Register interrupt */
		ret = request_irq(dev->device_irq_number, mipicsi_device_irq,
				IRQF_SHARED, dev_name(&pdev->dev), dev);
		if (ret)
			dev_err(&pdev->dev, "Could not register controller interrupt\n");
	} else
		dev_err(&pdev->dev, "IRQ num not set. See device tree.\n");

	/* Now that everything is fine, let's add it to device list */
	list_add_tail(&dev->devlist, &devlist_global);

	return ret;
 free_mem:
	iounmap(dev->base_address);

 free_dev:
	return error;

}

/*
 * Exit routine - Exit point of the driver
 */
static int mipicsi_device_remove(struct platform_device *pdev)
{
	struct mipicsi_device_dev *dev;
	struct list_head *list;

	dev_dbg(&pdev->dev, "Removing MIPI CSI-2 module\n");
	while (!list_empty(&devlist_global)) {
		list = devlist_global.next;
		list_del(list);
		dev = list_entry(list, struct mipicsi_device_dev, devlist);

		devm_free_irq(&pdev->dev, dev->device_irq_number, dev);
		devm_free_irq(&pdev->dev, dev->device_irq, dev);

		iounmap(dev->base_address);
	}
	return 0;
}


/*
 * of_device_id structure
 */
static const struct of_device_id mipicsi_device[] = {
	{ .compatible = "snps,mipicsi_device" },
	{ }
};

MODULE_DEVICE_TABLE(of, mipicsi_device);
/*
 * Platform driver structure
 */
static struct platform_driver __refdata mipicsi_device_pdrv = {
	.remove = mipicsi_device_remove,
	.probe  = mipicsi_device_probe,
	.driver   = {
		.name   = "snps, mipicsi_device",
		.owner = THIS_MODULE,
		.of_match_table = mipicsi_device,
	},
};

module_platform_driver(mipicsi_device_pdrv);
