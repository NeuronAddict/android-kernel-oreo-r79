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
#include "mipicsi_host.h"
#include "mipicsi_util.h"
#include "mipicsi_dc_dphy.h"

#include <linux/intel-hwio.h>
#include <soc/mnh/mnh-hwio-mipi-rx.h>
extern void * dev_addr_map[];
/* these macros assume a void * in scope named baddr */
#define RX_IN(reg)            HW_IN(baddr,   MIPI_RX, reg)
#define RX_OUT(reg,val)       HW_OUT(baddr,  MIPI_RX, reg, val)
#define RX_OUTf(reg,fld,val)  HW_OUTf(baddr, MIPI_RX, reg, fld, val)

#define RX_MASK(reg, fld)     HWIO_MIPI_RX_##reg##_##fld##_FLDMASK


/*
 * Linked list that contains the installed devices
 */
static LIST_HEAD(devlist_global);

/* DPHY struct configuration */
struct dphy_freq_range {
	uint32_t freq;          /* upper margin of frequency range */
	uint32_t  hsrxthssettle; /* hsrxthssettle */
};


/* Daughter card frequencies */
struct dphy_freq_range dc_freq_tbl[] = {
	{ 110, 0x00}, { 120, 0x10}, { 130, 0x20},
	{ 140, 0x30}, { 150, 0x01}, { 160, 0x11},
	{ 170, 0x21}, { 180, 0x31}, { 190, 0x02},
	{ 200, 0x12}, { 215, 0x21}, { 225, 0x32},
	{ 240, 0x03}, { 255, 0x13}, { 270, 0x23},
	{ 275, 0x33}, { 300, 0x04}, { 325, 0x14},
	{ 350, 0x05}, { 390, 0x15}, { 440, 0x25},
	{ 495, 0x06}, { 550, 0x16}, { 600, 0x07},
	{ 650, 0x17}, { 700, 0x08}, { 760, 0x18},
	{ 810, 0x09}, { 865, 0x19}, { 915, 0x29},
	{ 970, 0x39}, {1020, 0x0A}, {1125, 0x1A},
	{1180, 0x3A}, {1230, 0x0B}, {1280, 0x1B},
	{1330, 0x2B}, {1390, 0x3B}, {1440, 0x1C},
	{1490, 0x1C}, {1540, 0x2C}, {1600, 0x3C},
	{1650, 0x0D}, {1700, 0x3D}, {1750, 0x2D},
	{1800, 0x0E}, {1860, 0x1E}, {1910, 0x2E},
	{1960, 0x3E}
};


void mipicsi_host_dphy_write(enum mipicsi_top_dev dev,
			     uint8_t command, uint8_t data)
{
	void * baddr = dev_addr_map[dev];
	if (baddr) {
		/* Gen 2 Daughtercard specific sequence */
		pr_info("%s: dev=%d, command 0x%02X data=0x%02X\n",
			__func__, dev, command, data);

		RX_OUTf(PHY_SHUTDOWNZ, PHY_SHUTDOWNZ, 0);
		RX_OUTf(DPHY_RSTZ,     DPHY_RSTZ,     0);
		RX_OUT(PHY_TEST_CTRL0, 0);
		RX_OUTf(PHY_TEST_CTRL0, PHY_TESTCLK, 1);
		/* set the desired test code in the input 8-bit bus TESTDIN[7:0] */
		RX_OUT(PHY_TEST_CTRL1, 0);
		RX_OUTf(PHY_TEST_CTRL1, PHY_TESTDIN, command);
		RX_OUTf(PHY_TEST_CTRL1, PHY_TESTEN, 1);

		RX_OUTf(PHY_TEST_CTRL0, PHY_TESTCLK, 0);

		RX_OUTf(PHY_TEST_CTRL1, PHY_TESTEN, 0);
		RX_OUTf(PHY_TEST_CTRL1, PHY_TESTDIN, data);

		RX_OUTf(PHY_TEST_CTRL0, PHY_TESTCLK, 1);

		//pr_info("%s: X\n", __func__);
	}
	else {
		pr_err("%s no address for %d\n", __func__, dev);
        }
}

#if 0
void mipicsi_host_reset(enum mipicsi_top_dev dev)
{
	void * baddr = dev_addr_map[dev];
	if (baddr) {
		RX_OUTf(CSI2_RESETN, CSI2_RESETN, 0);
		udelay(1000);
		RX_OUTf(CSI2_RESETN, CSI2_RESETN, 1);
	}
	else {
		pr_err("%s no address for %d\n", __func__, dev);
	}
}
#endif

void mipicsi_host_dphy_reset(enum mipicsi_top_dev dev)
{
	void * baddr = dev_addr_map[dev];
	if (baddr) {
		RX_OUTf(DPHY_RSTZ, DPHY_RSTZ, 1);
		udelay(1000);
		RX_OUTf(DPHY_RSTZ, DPHY_RSTZ, 0);
	}
	else {
		pr_err("%s no address for %d\n", __func__, dev);
        }
}

int mipicsi_host_start(struct mipicsi_top_cfg *config)
{

#ifdef MNH_EMULATION
	uint8_t counter = 0;
	uint32_t data;
	uint32_t index = 0;
	enum mipicsi_top_dev dev = config->dev;
	const uint32_t stop_mask =
	  RX_MASK(PHY_STOPSTATE, PHY_STOPSTATEDATA_0) |
	  RX_MASK(PHY_STOPSTATE, PHY_STOPSTATEDATA_1) |
	  RX_MASK(PHY_STOPSTATE, PHY_STOPSTATEDATA_2) |
	  RX_MASK(PHY_STOPSTATE, PHY_STOPSTATEDATA_3) |
	  RX_MASK(PHY_STOPSTATE, PHY_STOPSTATECLK);
	void * baddr = dev_addr_map[dev];

	if (baddr) {
		pr_info("%s: E\n", __func__);
		if ((dev != MIPI_RX0) && (dev != MIPI_RX1) && (dev != MIPI_RX2))
			return -EINVAL;

		/* Set SHUTDOWNZ to logic low*/
		RX_OUTf(PHY_SHUTDOWNZ, PHY_SHUTDOWNZ, 0);

		/* Set RSTZ to logic low */
		RX_OUTf(DPHY_RSTZ, DPHY_RSTZ, 0);


		/* Set TESTCLR to logic high*/
		//RX_OUTf(PHY_TEST_CTRL0, PHY_TESTCLR, 1);
		RX_OUT(PHY_TEST_CTRL0, 1);

		/* Apply CFG_CLK signal with the appropriate frequency; for
		 * correct values, refer to Table 12-5
		 */
		/* Note: Hardware controlled */

		/* Set MASTERSLAVEZ for SLAVE */
		mipicsi_host_dphy_write(dev, R_CSI2_DCPHY_MASTER_SLAVEZ,
					DC_SLAVE_VAL);

		/* Set BASEDIR_N to the desired values(lane direction) */
		mipicsi_host_dphy_write(dev, R_CSI2_DCPHY_BASEDIR_L0,
					DC_RX_BASEDIR_VAL);
		mipicsi_host_dphy_write(dev, R_CSI2_DCPHY_BASEDIR_L1,
					DC_RX_BASEDIR_VAL);
		mipicsi_host_dphy_write(dev, R_CSI2_DCPHY_BASEDIR_L2,
					DC_RX_BASEDIR_VAL);
		mipicsi_host_dphy_write(dev, R_CSI2_DCPHY_BASEDIR_L3,
					DC_RX_BASEDIR_VAL);

		/* Set all REQUEST inputs to zero */
		/* Hardware controls */

		/* Wait for 15 ns */
		udelay(1);

		/* Set TESTCLR to low */
		//RX_OUTf(PHY_TEST_CTRL0, PHY_TESTCLR, 0);
		RX_OUT(PHY_TEST_CTRL0, 0);
		/* Wait for 15 ns */
		udelay(1);

#if 0
		/* Configure hsfreqrange according to table Frequency Ranges
		 * and Defaults in "PLL Locking Mode and AFE Initialization"
		 * on page 74
		 */
		while ((index < (sizeof(dc_freq_tbl)/sizeof(struct dphy_freq_range))) &&
		       ((config->mbps/1000) > dc_freq_tbl[index].freq)) {
		  index++;
		}

		mipicsi_host_dphy_write(dev, R_CSI2_DCPHY_HS_RX_CTRL_L0,
					dc_freq_tbl[index].hsrxthssettle);
#else
		mipicsi_host_dphy_write(dev, R_CSI2_DCPHY_HS_RX_CTRL_L0, 0x10);
#endif // #if 0
		/* meant to write to RX0 I assume. won't hard code
		   on refactor work
		   mipicsi_write(MIPI_TX0, R_CSI2_N_LANES, config->num_lanes-1);
		*/
		RX_OUTf(N_LANES, N_LANES, (config->num_lanes-1));
		/* Wait 5ns */
		udelay(1);
		mipicsi_host_dphy_write(dev, R_CSI2_DCPHY_AFE_BYPASS_BANDGAP, 0x40);
		/* Configure Test Code 0x22
		 * BIASEXTR internal resistor control
		 * LPTX bias current control
		 */
		mipicsi_host_dphy_write(dev, R_CSI2_DCPHY_AFE_BYPASS_BANDGAP, 0x04);

		/* Set SHUTDOWNZ=1'b1 */
		RX_OUTf(PHY_SHUTDOWNZ, PHY_SHUTDOWNZ, 1);

		/* Wait 5ns */
		udelay(1);

		/* Set RSTZ=1'b1 */
		// MISMATCH PHY_SHUTDOWNZ doesn't have RSTZ field
		RX_OUTf(DPHY_RSTZ, DPHY_RSTZ, 1);

		RX_OUTf(CSI2_RESETN, CSI2_RESETN, 1);
		/* Wait until STOPSTATEDATA_N and STOPSTATECLK outputs are
		 * asserted -- poll for 200 us
		 */
		do {
			data = RX_IN(PHY_STOPSTATE);
			if ((data & stop_mask) == stop_mask) {
				pr_info("%s: X\n", __func__);
				return 0;
			}
			udelay(10);
			counter++;
		} while (counter < 20);
		pr_info("%s counter: %d 0x%08X\n",
			__func__, counter, data);
	}
	else {
		pr_err("%s no address for %d\n", __func__, dev);
	}
#endif
	/* TO DO - initialize controller parameters only here for Gen 3 */

	return 0;
}

int mipicsi_host_hw_init(enum mipicsi_top_dev dev)
{
	void * baddr = dev_addr_map[dev];
	if (baddr) {
		pr_info("%s: %d version: 0x%08X",
			__func__, dev, RX_IN(VERSION));
		/* set only one lane (lane 0) as active (ON) */
		RX_OUTf(N_LANES, N_LANES, 0);

		/* mipicsi_write(dev, PHY_SHUTDOWNZ, 1); */

		//mipicsi_host_dphy_reset(dev);

		//mipicsi_host_reset(dev);

		RX_OUT(INT_MSK_PHY_FATAL,   0xFFFFFFFF);
		RX_OUT(INT_MSK_PKT_FATAL,   0xFFFFFFFF);
		RX_OUT(INT_MSK_FRAME_FATAL, 0xFFFFFFFF);
		RX_OUT(INT_MSK_PHY,         0xFFFFFFFF);
		RX_OUT(INT_MSK_PKT,         0xFFFFFFFF);
		RX_OUT(INT_MSK_LINE,        0xFFFFFFFF);
		/* non existant register
		   mipicsi_write(dev, R_CSI2_MASK_INT_IPI, 0xFFFFFFFF);
		*/
		RX_OUTf(PHY_SHUTDOWNZ, PHY_SHUTDOWNZ, 0);

		//pr_info("%s: %d X\n", __func__, dev);
	}
	else {
		pr_err("%s no address for %d\n", __func__, dev);
		return -ENXIO;
        }

	return 0;
}

int mipicsi_host_probe(struct platform_device *pdev)
{
	int ret = 0;
	int error = 0;
	struct resource *mem = NULL;
	struct mipicsi_host_dev *dev;
	int irq_number = 0;
#ifdef JUNO_BRINGUP
	void *iomem;
#endif

	dev_info(&pdev->dev, "Installing MIPI CSI-2 HOST module...\n");

	dev_info(&pdev->dev, "Device registration\n");
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		dev_err(&pdev->dev, "Could not allocated mipi_csi_host\n");
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
	pr_info("MIPI HOST: juno bringup %p\n", dev->base_address);
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
	pr_info("MIPI HOST: ioremapped to %p\n", dev->base_address);
	mipicsi_util_save_virt_addr(MIPI_RX0, dev->base_address);

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
	dev->host_irq_number = irq_number;

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
	ret = mipicsi_host_hw_init();
	if (ret) {
		dev_err(&pdev->dev, "Could not init the SNPS MIPI %d\n", ret);
		goto unreg_dev;
	}

	/* Register interrupt */
	ret = devm_request_irq(&pdev->dev, dev->host_irq,
				   mipicsi_host_irq1, IRQF_SHARED,
				   dev_name(&pdev->dev), dev);
	if (ret) {
		dev_err(&pdev->dev, "Could not register controller interrupt\n");
		goto unreg_dev;
	}

	irq_number = platform_get_irq(pdev, 1);
	if (irq_number <= 0) {
		dev_err(&pdev->dev, "IRQ number not set. See device tree.\n");
		error = -EINVAL;
		goto unreg_dev;
	}

	/* Device tree information  -- */
	/* TO DO  Read version of HOST to determine emulation vs silicon */
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
static int mipicsi_host_remove(struct platform_device *pdev)
{
	struct mipicsi_host_dev *dev;
	struct list_head *list;

	dev_dbg(&pdev->dev, "Removing MIPI CSI-2 module\n");
	while (!list_empty(&devlist_global)) {
		list = devlist_global.next;
		list_del(list);
		dev = list_entry(list, struct mipicsi_host_dev, devlist);

		devm_free_irq(&pdev->dev, dev->host_irq_number, dev);
		devm_free_irq(&pdev->dev, dev->host_irq, dev);

		iounmap(dev->base_address);
	}
	return 0;
}


/*
 * of_device_id structure
 */
static const struct of_device_id mipicsi_host[] = {
	{ .compatible = "snps,mipicsi_host" },
	{ }
};

MODULE_DEVICE_TABLE(of, mipicsi_host);
/*
 * Platform driver structure
 */
static struct platform_driver __refdata mipicsi_host_pdrv = {
	.remove = mipicsi_host_remove,
	.probe  = mipicsi_host_probe,
	.driver   = {
		.name   = "snps, mipicsi_host",
		.owner = THIS_MODULE,
		.of_match_table = mipicsi_host,
	},
};

module_platform_driver(mipicsi_host_pdrv);
