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

/*
 * Linked list that contains the installed devices
 */
static LIST_HEAD(devlist_global);

void mipicsi_dev_dphy_write(enum mipicsi_top_dev dev,
			    uint8_t offset, uint8_t data)
{
#ifdef MNH_EMULATION
	/* Gen 2 Daughtercard specific sequence */

	pr_info("%s: dev=0x%x, data=0x%x, offset=0x%x\n",
		__func__, dev, data, offset);

	/* set the TESTCLK input high in preparation to latch in the desired
	 * test mode
	 */
	mipicsi_write_part(dev, R_CSI2_DEV_PHY0_TST_CTRL0, 1, DEV_TESTCLK_SH,
			   1);

	/* set the desired test code in the input 8-bit bus TESTDIN[7:0] */
	mipicsi_write(dev, R_CSI2_DEV_PHY1_TST_CTRL1, offset);

	/* set TESTEN input high  */
	mipicsi_write_part(dev, R_CSI2_DEV_PHY0_TST_CTRL1, 1, DEV_TESTEN_SH, 1);

	/* drive the TESTCLK input low; the falling edge captures the chosen
	 *  test code into the transceiver
	 */
	mipicsi_write_part(dev, R_CSI2_DEV_PHY0_TST_CTRL0, 0, DEV_TESTCLK_SH,
			   1);

	/* set TESTEN input low to disable further test mode code latching  */
	mipicsi_write_part(dev, R_CSI2_DEV_PHY0_TST_CTRL1, 0, DEV_TESTEN_SH, 1);

	/* set TESTDIN[7:0] to the desired test data appropriate to the
	 * chosen test mode
	 */
	mipicsi_write(dev, R_CSI2_DEV_PHY0_TST_CTRL1, data);

	/* pulse TESTCLK high to capture this test data into the macrocell */
	mipicsi_write_part(dev, R_CSI2_DEV_PHY0_TST_CTRL0, 1, DEV_TESTCLK_SH,
			   1);

	pr_info("%s: X\n", __func__);
#endif /* MNH_EMULATION */
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
	mipicsi_write(dev, R_CSI2_DEV_CSI2_RESETN, 0);
	udelay(1000);
	mipicsi_write(dev, R_CSI2_DEV_CSI2_RESETN, 1);
}


void mipicsi_device_dphy_reset(enum mipicsi_top_dev dev)
{
	mipicsi_write_part(dev, R_CSI2_DEV_PHY_RSTZ, 1, DEV_RSTZ_SH, 1);
	udelay(1000);
	mipicsi_write_part(dev, R_CSI2_DEV_PHY_RSTZ, 0, DEV_RSTZ_SH, 1);
}

int32_t mipicsi_device_set_pll(struct mipicsi_top_cfg *config)
{
	enum mipicsi_top_dev dev;
	struct mipicsi_pll pll;

	dev = config->dev;

#ifdef MNH_EMULATION

	/* HARD CODE PLL for 1.5 GHzs, and input 12.5 Mhz */
	pll.hsfreq = 0x3c;
	pll.vco_range = 0x03;
	pll.cp_current = 0x06;
	pll.lpf_resistor = 0x08;
	pll.loop_div = 0x12A;
	pll.input_div = 0x04;
	pll.output_div = 0;

	/* Configure hsfreqrange according to table Frequency Ranges and
	 * Defaults in "PLL Locking Mode and AFE Initialization" on page 74
	 */
	mipicsi_dev_dphy_write(dev, R_CSI2_DCPHY_HS_RX_CTRL_L0,
			       (pll.hsfreq << 1));

	/* 11. Configure the PLL (if for Master mode), direct PLL SoC control:
	 * a. Set clk_sel[1:0] = 2'b01.
	 * b. Set pll_shadow_control = 0 for direct SoC control of the PLL
	 * configuration.
	 * c. Apply a shadow_clear pulse on the PLL interface to clear
	 * configuration shadow registers to default values. Apply PLL desired
	 * configurations:
	 * n, m, p, icpctrl, lpfctrl, vcorange, vcocap; the example
	 * below is for 2GHz operation -- for different operating frequencies,
	 * refer to Chapter 6,  "PLL Requirements"):
	 * ¦ VCO Control (vcorange and vcocap):
	 * ¦ vcorange[1:0] = 2'b11
	 * ¦ vcocap[1:0] = 2'b00
	 * ¦ PLL Control (icpctrl): icpctrl[3:0] = 4'b0111
	 * ¦ PLL Control (lpfctrl): lpfctrl[5:0] = 6'b000001
	 * ¦ PLL Input Divider Ratio (n): n[2:0] = 3'h2
	 * ¦ PLL Loop Divider Ratio (m): m[9:0] = 10'hF8
	 * ¦ PLL Output Frequency Division Ratio (p): p[1:0] = 2'b00
	 *
	 * e. Apply updatepll pulse to read-in PLL desired configurations
	 */

	mipicsi_dev_dphy_write(dev, R_CSI2_DCPHY_PLL_BYPASS, 0x01);

	mipicsi_dev_dphy_write(dev, R_CSI2_DCPHY_PLL_BIAS_FCC_VCO, (0x1 << 7) |
			       (pll.vco_range << 3) | 0);

	mipicsi_dev_dphy_write(dev, R_CSI2_DCPHY_PLL_CP_LOCK_BYP_ULP,
			       (0x00 << 4) | (pll.cp_current << 0));

	mipicsi_dev_dphy_write(dev, R_CSI2_DCPHY_PLL_LPF_CP_CTRL, (0x01 << 7) |
			       (0x01 << 6) | (pll.lpf_resistor << 0));

	mipicsi_dev_dphy_write(dev, R_CSI2_DCPHY_PLL_INPUT_DIV_RAT,
			       pll.input_div);

	mipicsi_dev_dphy_write_set(dev, R_CSI2_DCPHY_PLL_LOOP_DIV_RAT,
				   ((pll.loop_div >> 0) & 0x1F), 0, 1);
	mipicsi_dev_dphy_write_set(dev, R_CSI2_DCPHY_PLL_LOOP_DIV_RAT,
				   ((pll.loop_div >> 5) & 0x1F), 1, 1);

	mipicsi_dev_dphy_write(dev, R_CSI2_DCPHY_PLL_DIV_RAT_CTRL,
			       ((0x1<<5) | (0x1<<4) | (0x1<<2) |
				(pll.output_div<<0)));
#endif
	return 0;
}

int mipicsi_device_vpg(struct mipicsi_top_vpg *vpg)
{
	enum mipicsi_top_dev dev = vpg->dev;

	mipicsi_dev_dphy_write(dev, R_CSI2_DCPHY_HS_TX_PWR_CTRL_CLK, 0xB);

	mipicsi_write(dev, R_CSI2_DEV_VPG_MODE_CFG, vpg->mode_cfg);
	mipicsi_write(dev, R_CSI2_DEV_VPG_PKT_CFG, vpg->pkt_cfg);
	mipicsi_write(dev, R_CSI2_DEV_VPG_PKT_SIZE, vpg->pkt_size);
	mipicsi_write(dev, R_CSI2_DEV_VPG_HSA_TIME, vpg->hsa_time);
	mipicsi_write(dev, R_CSI2_DEV_VPG_HBP_TIME, vpg->hbp_time);
	mipicsi_write(dev, R_CSI2_DEV_VPG_HLINE_TIME, vpg->hline_time);
	mipicsi_write(dev, R_CSI2_DEV_VPG_VSA_LINES, vpg->vsa_lines);
	mipicsi_write(dev, R_CSI2_DEV_VPG_VBP_LINES, vpg->vbp_lines);
	mipicsi_write(dev, R_CSI2_DEV_VPG_VFP_LINES, vpg->vfp_lines);
	mipicsi_write(dev, R_CSI2_DEV_VPG_ACT_LINES, vpg->act_lines);
	mipicsi_write(dev, R_CSI2_DEV_VPG_MAX_FRAME_NUM, vpg->max_frame);
	mipicsi_write(dev, R_CSI2_DEV_VPG_START_LINE_NUM, vpg->start_line);
	mipicsi_write(dev, R_CSI2_DEV_VPG_STEP_LINE_NUM, vpg->step_line);

	/* Enable VPG */
	mipicsi_write(dev, R_CSI2_DEV_VPG_CTRL, 1);
	return 0;
}


int mipicsi_device_start(struct mipicsi_top_cfg *config)
{

#ifdef MNH_EMULATION
	uint32_t data = 0;
	uint8_t counter = 0;
	enum mipicsi_top_dev dev = config->dev;

	pr_info("%s: E\n", __func__);

	if ((dev != MIPI_TX0) && (dev != MIPI_TX1))
		return 0;

	/* Set SHUTDOWNZ logic low, */
	mipicsi_write_part(dev, R_CSI2_DEV_PHY_RSTZ, 0, DEV_SHUTDOWNZ_SH, 1);

	/* Set RSTZ signals to logic low */
	mipicsi_write_part(dev, R_CSI2_DEV_PHY_RSTZ, 0, DEV_RSTZ_SH, 1);

	/* TESTCLR to logic high */
	mipicsi_write_part(dev, R_CSI2_DEV_PHY0_TST_CTRL0, 1,
			    DEV_TESTCLR_SH, 1);

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
	mipicsi_dev_dphy_write(dev, R_CSI2_DCPHY_MASTER_SLAVEZ, DC_SLAVE_VAL);

	/* Set ENABLE_N = 1'b1 */
	mipicsi_write(dev, R_CSI2_DEV_PHY_IF_CFG, 0x03);

	/* Set BASEDIR_N to the desired values */
	mipicsi_dev_dphy_write(dev, R_CSI2_DCPHY_BASEDIR_L0, DC_TX_BASEDIR_VAL);
	mipicsi_dev_dphy_write(dev, R_CSI2_DCPHY_BASEDIR_L1, DC_TX_BASEDIR_VAL);
	mipicsi_dev_dphy_write(dev, R_CSI2_DCPHY_BASEDIR_L2, DC_TX_BASEDIR_VAL);
	mipicsi_dev_dphy_write(dev, R_CSI2_DCPHY_BASEDIR_L3, DC_TX_BASEDIR_VAL);

	/* Wait for 15 ns */
	udelay(1);

	/* Set TESTCLR to low */
	mipicsi_write_part(dev, R_CSI2_DEV_PHY0_TST_CTRL0, 0, DEV_TESTCLR_SH,
			   1);

	/* Wait for 35 ns */
	udelay(1);

	/* Configure PLL */
	mipicsi_device_set_pll(config);

	/* Wait 5 ns */
	udelay(1);

	/* Configure Test Code 0x22
	 * Bitrate configuration
	 * Band Gap reference voltage
	 * BIASEXTR internal resistor control
	 * LPTX bias current control
	 */
	mipicsi_dev_dphy_write(dev, R_CSI2_DCPHY_AFE_BYPASS_BANDGAP,
				0x04);
	mipicsi_dev_dphy_write(dev, R_CSI2_DCPHY_AFE_BYPASS_BANDGAP,
				0x50);

	/* Set SHUTDOWNZ signal to logic high */
	mipicsi_write_part(dev, R_CSI2_DEV_PHY_RSTZ, 1, DEV_SHUTDOWNZ_SH, 1);

	/* Wait 5 ns */
	udelay(1);

	/* Set RSTZ signal to logic high */
	mipicsi_write_part(dev, R_CSI2_DEV_PHY_RSTZ, 1, DEV_RSTZ_SH, 1);

	/* Wait until the STOPSTATEDATA_N and STOPSTATECLK outputs are asserted.
	 * At this point, the PLL has already locked (for the Master) and the
	 * initialization of the analog drivers has completed. From this point,
	 * the REQUEST inputs can be set according to the desired transmission
	 * -- poll for 200 us
	 */
	do {
		data = mipicsi_read(dev, R_CSI2_DEV_PHY_STATUS);

		if ((data & DEV_STOPSTATE) == DEV_STOPSTATE) {
			pr_info("%s: X\n", __func__);
			return 0;
		}

		udelay(10);
		counter++;
	} while (counter < 20);

	return -EINVAL;
#endif
	/* TO DO - initialize controller parameters only here for Gen 3 */
}


int mipicsi_device_hw_init(enum mipicsi_top_dev dev)
{
	mipicsi_write_part(dev, R_CSI2_DEV_PHY_RSTZ, 1, DEV_SHUTDOWNZ_SH, 1);

	mipicsi_device_dphy_reset(dev);

	mipicsi_device_reset(dev);

	mipicsi_write(dev, R_CSI2_DEV_INT_MASK_VPG, 0xFFFFFFFF);
	mipicsi_write(dev, R_CSI2_DEV_INT_MASK_IDI, 0xFFFFFFFF);
	mipicsi_write(dev, R_CSI2_DEV_INT_MASK_MEM, 0xFFFFFFFF);

	mipicsi_write_part(dev, R_CSI2_DEV_PHY_RSTZ, 0, DEV_SHUTDOWNZ_SH, 1);

	return 0;
}


int mipicsi_device_probe(struct platform_device *pdev)
{
	int ret = 0;
	int error = 0;
	struct resource *mem = NULL;
	struct mipicsi_device_dev *dev;
	int irq_number = 0;
#ifdef JUNO_BRINGUP
	void *iomem;
#endif

	dev_info(&pdev->dev, "Installing MIPI CSI-2 DEVICE module...\n");

	dev_info(&pdev->dev, "Device registration\n");
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		dev_err(&pdev->dev, "Could not allocated mipicsi_device\n");
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
	pr_info("MIPI DEVICE: juno bringup %p\n", dev->base_address);
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

	pr_info("MIPI DEV: ioremapped to %p\n", dev->base_address);
	mipicsi_util_save_virt_addr(MIPI_TX0, dev->base_address);

	/* dev_info(&pdev->dev, "SNPS Device at 0x%08x\n",
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
	dev->device_irq_number = irq_number;

	/* Init locks */
	dev_info(&pdev->dev, "Init locks\n");
	spin_lock_init(&dev->slock);

	/* Init mutex */
	dev_info(&pdev->dev, "Init mutex\n");
	mutex_init(&dev->mutex);

	/* Now that everything is fine, let's add it to device list */
	list_add_tail(&dev->devlist, &devlist_global);

	/* HW init */
#if 0
	/* TO DO - currently initialized from TOP */
	ret = mipicsi_device_hw_init();
	if (ret) {
		dev_err(&pdev->dev, "Could not init the SNPS MIPI %d\n", ret);
		goto unreg_dev;
	}

	/* Register interrupt */
	ret = devm_request_irq(&pdev->dev, dev->device_irq,
				   mipicsi_device_irq1, IRQF_SHARED,
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
	/* TO DO  Read version of DEVICE to determine emulation vs silicon */
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
