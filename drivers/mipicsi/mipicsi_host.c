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
#include "mipi_dev.h"
#include "mipicsi_top.h"
#include "mipicsi_device.h"
#include "mipicsi_host.h"
#include "mipicsi_pll.h"
#include "mipicsi_util.h"
#include "mipicsi_dc_dphy.h"
#include "mipicsi_rx_dphy.h"

#include <linux/intel-hwio.h>
#include <soc/mnh/mnh-hwio-mipi-rx.h>
extern void * dev_addr_map[];


/* These macros assume a void * in scope named baddr */
/* TODO refactor to use dev->base_address at some point */
#define RX_IN(reg)             HW_IN(baddr,   MIPI_RX, reg)
#define RX_INf(reg, fld)       HW_INf(baddr,   MIPI_RX, reg, fld)
#define RX_OUT(reg, val)       HW_OUT(baddr,  MIPI_RX, reg, val)
#define RX_OUTf(reg, fld, val) HW_OUTf(baddr, MIPI_RX, reg, fld, val)

#define RX_MASK(reg, fld)      HWIO_MIPI_RX_##reg##_##fld##_FLDMASK


/*
 * Linked list that contains the installed devices
 */
static LIST_HEAD(devlist_global);

/* DPHY struct configuration */
struct dphy_freq_range {
	uint32_t freq;          /* upper margin of frequency range */
	uint32_t  hsrxthssettle; /* hsrxthssettle */
};


void mipicsi_host_dphy_write(enum mipicsi_top_dev dev,
			     uint16_t command, uint8_t data)
{
	void * baddr = dev_addr_map[dev];
	if (!baddr) {
		pr_err("%s: no address for %d\n", __func__, dev);
		return;
	}

	/* Gen 2 Daughtercard specific sequence */
	pr_info("%s: dev=%d, command 0x%02X data=0x%02X\n",
		__func__, dev, command, data);
	RX_OUTf(PHY_SHUTDOWNZ, PHY_SHUTDOWNZ, 0);
	RX_OUTf(DPHY_RSTZ,     DPHY_RSTZ,     0);
#ifdef MNH_EMULATION
	/* Set the desired testcode */
	RX_OUT(PHY_TEST_CTRL0, 0);
	RX_OUTf(PHY_TEST_CTRL0, PHY_TESTCLK, 1);
	RX_OUT(PHY_TEST_CTRL1, 0);
	RX_OUTf(PHY_TEST_CTRL1, PHY_TESTDIN, command);
	RX_OUTf(PHY_TEST_CTRL1, PHY_TESTEN, 1);
	RX_OUTf(PHY_TEST_CTRL0, PHY_TESTCLK, 0);

	/* Enter the test data */
	RX_OUTf(PHY_TEST_CTRL1, PHY_TESTEN, 0);
	RX_OUTf(PHY_TEST_CTRL1, PHY_TESTDIN, data);
	RX_OUTf(PHY_TEST_CTRL0, PHY_TESTCLK, 1);
#else
	/* Write 4-bit testcode MSB */
	RX_OUTf(PHY_TEST_CTRL0, PHY_TESTCLK, 0);
	RX_OUTf(PHY_TEST_CTRL1, PHY_TESTEN,  0);
	RX_OUTf(PHY_TEST_CTRL1, PHY_TESTEN , 1);
	RX_OUTf(PHY_TEST_CTRL0, PHY_TESTCLK, 1);
	RX_OUTf(PHY_TEST_CTRL1, PHY_TESTDIN, 0);
	RX_OUTf(PHY_TEST_CTRL0, PHY_TESTCLK, 0);
	RX_OUTf(PHY_TEST_CTRL1, PHY_TESTEN,  0);
	RX_OUTf(PHY_TEST_CTRL1, PHY_TESTDIN, ((command & 0xF00)>>8));
	RX_OUTf(PHY_TEST_CTRL0, PHY_TESTCLK, 1);

	/* Write 8-bit testcode LSB */
	RX_OUTf(PHY_TEST_CTRL0, PHY_TESTCLK, 0);
	RX_OUTf(PHY_TEST_CTRL1, PHY_TESTEN,  1);
	RX_OUTf(PHY_TEST_CTRL0, PHY_TESTCLK, 1);
	RX_OUTf(PHY_TEST_CTRL1, PHY_TESTDIN, (command & 0xFF));
	RX_OUTf(PHY_TEST_CTRL0, PHY_TESTCLK, 0);
	RX_OUTf(PHY_TEST_CTRL1, PHY_TESTEN,  0);

	/* Write the data */
	RX_OUTf(PHY_TEST_CTRL1, PHY_TESTDIN, data);
	RX_OUTf(PHY_TEST_CTRL0, PHY_TESTCLK, 1);
#endif
}

uint8_t mipicsi_host_dphy_read(enum mipicsi_top_dev dev, uint16_t command)
{
	void *baddr = dev_addr_map[dev];
	uint8_t data;

	if (!baddr) {
		pr_err("%s: no address for %d\n", __func__, dev);
		return 0;
	}

	RX_OUTf(PHY_SHUTDOWNZ, PHY_SHUTDOWNZ, 0);
	RX_OUTf(DPHY_RSTZ,     DPHY_RSTZ,     0);
	RX_OUTf(PHY_TEST_CTRL0, PHY_TESTCLR, 0);
	RX_OUTf(PHY_TEST_CTRL1, PHY_TESTDIN, command);
	RX_OUTf(PHY_TEST_CTRL1, PHY_TESTEN,  1);
	RX_OUTf(PHY_TEST_CTRL0, PHY_TESTCLK, 1);
	RX_OUTf(PHY_TEST_CTRL0, PHY_TESTCLK, 0);
	RX_OUTf(PHY_TEST_CTRL1, PHY_TESTEN,  0);
	data = (RX_IN(PHY_TEST_CTRL1))>>8;

	RX_OUTf(DPHY_RSTZ,     DPHY_RSTZ,     1);
	RX_OUTf(PHY_SHUTDOWNZ, PHY_SHUTDOWNZ, 1);

	pr_err("%s: Dev: %d Offset: 0x%x, Value: 0x%x\n", __func__, dev,
	       command, data);

	return data;
}


int mipicsi_host_dphy_write_set(enum mipicsi_top_dev dev, uint32_t offset,
			       uint8_t data, uint8_t ps, uint8_t ps_width)
{
	uint32_t temp;

	/* Check if the incoming data is valid: data should have a maximum
	 * value of 8 bits minus the program selector width
	 * For example: if program selector is 1 bit, the data can be 7 bits
	 * ps can be a maximum of 2 bits
	 */
	if (ps > 3)
		return -EINVAL;

	if (ps_width > 2)
		return -EINVAL;

	if (data >= (1<<(8-ps_width)))
		return -EINVAL;

	/* Set the most significant bits to the target program selector
	 * and place the data in the remaining bits
	 */
	temp = ((ps << (8-ps_width)) | data);
	mipicsi_host_dphy_write(dev, offset, temp);
	return 0;
}

void mipicsi_host_reset(enum mipicsi_top_dev dev)
{
	void * baddr = dev_addr_map[dev];
	if (!baddr) {
		pr_err("%s: no address for %d\n", __func__, dev);
		return;
	}

	RX_OUTf(CSI2_RESETN, CSI2_RESETN, 0);
	udelay(1000);
	RX_OUTf(CSI2_RESETN, CSI2_RESETN, 1);
}

void mipicsi_host_dphy_reset(enum mipicsi_top_dev dev)
{
	void * baddr = dev_addr_map[dev];
	if (!baddr) {
		pr_err("%s: no address for %d\n", __func__, dev);
		return;
	}

	RX_OUTf(DPHY_RSTZ, DPHY_RSTZ, 1);
	udelay(1000);
	RX_OUTf(DPHY_RSTZ, DPHY_RSTZ, 0);
}

int mipicsi_host_start(struct mipicsi_top_cfg *config)
{

	uint8_t counter = 0;
	uint32_t data, ui_ps, ths_setl_ns;
	uint8_t hsfreq, value;
	enum mipicsi_top_dev dev = config->dev;
	const uint32_t stop_mask =
	  RX_MASK(PHY_STOPSTATE, PHY_STOPSTATEDATA_0) |
	  RX_MASK(PHY_STOPSTATE, PHY_STOPSTATEDATA_1) |
	  RX_MASK(PHY_STOPSTATE, PHY_STOPSTATEDATA_2) |
	  RX_MASK(PHY_STOPSTATE, PHY_STOPSTATEDATA_3) |
	  RX_MASK(PHY_STOPSTATE, PHY_STOPSTATECLK);
	void * baddr = dev_addr_map[dev];

	if (!baddr) {
		pr_err("%s: no address for %d\n", __func__, dev);
		return -ENXIO;
	}

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

#ifdef MNH_EMULATION
	/* Apply CFG_CLK signal with the appropriate frequency; for
	 * correct values, refer to Table 12-5
	 */
	/* Note: Hardware controlled */

	/* Set MASTERSLAVEZ for SLAVE */
	mipicsi_host_dphy_write(dev, R_CSI2_DCPHY_MASTER_SLAVEZ,
				DC_SLAVE_VAL);

	/* Configure as RX per Synopsys feedback - registers not in databook */
	mipicsi_dev_dphy_write(dev, 0xB0, 0x1E);
	udelay (10);
	mipicsi_dev_dphy_write(dev, 0xAC, 0x03);

	/* Set all REQUEST inputs to zero */
	/* Hardware controls */

	/* Wait for 15 ns */
	udelay(1);

	/* Set TESTCLR to low */
	//RX_OUTf(PHY_TEST_CTRL0, PHY_TESTCLR, 0);
	RX_OUT(PHY_TEST_CTRL0, 0);
	/* Wait for 15 ns */
	udelay(1);

	/* Configure hsfreqrange according to table Frequency Ranges
	 * and Defaults in "PLL Locking Mode and AFE Initialization"
	 * on page 74
	 */
	if (mipicsi_pll_get_hsfreq(config->mbps, &hsfreq) != 0)
		return -EINVAL;

	mipicsi_host_dphy_write(dev, R_CSI2_DCPHY_HS_RX_CTRL_L0,
			       (hsfreq << 1));


	RX_OUTf(N_LANES, N_LANES, (config->num_lanes-1));

	/* Wait 5ns */
	udelay(1);

	/*
	 * RX THS Settle
	 * NOTE: THS target settle time is a little higher than MIPI spec due
	 * to Synopsys feedback
	 */
	ui_ps = 1000*1000/config->mbps;
	ths_setl_ns = MIN(PAD_TIME(115+6*ui_ps/1000), 145+10*ui_ps/1000);
	pr_info("THS : RX setl=%d\n",ths_setl_ns);
	value = ROUNDUP((ths_setl_ns-SETL_CONST_TIME),
		      (MIPI_DDR_CLOCK/(config->mbps/2)))-1;
	mipicsi_host_dphy_write(dev, R_CSI2_DCPHY_RX_THS_SETL, (1<<7) | value);

#else
	struct mipicsi_pll pll;

	/* Wait for 15ns */
	udelay (1);

	/* Set TESTCLR to logic low */
	RX_OUT(PHY_TEST_CTRL0, 0);

	/* TEMP - Hardcode 640 Settings */
	if (config->mbps == 640) {
		mipicsi_host_dphy_write(dev, 0x01, 0x20);
		mipicsi_host_dphy_write(dev, 0x02, 0x18);

		mipicsi_host_dphy_write(dev, 0xE2, 0xb6);
		mipicsi_host_dphy_write(dev, 0xE3, 0x1);
		mipicsi_host_dphy_write(dev, 0xE4, 0x1);

		mipicsi_host_dphy_write(dev, 0x08, 0x20);
		udelay (1);

		RX_OUTf(N_LANES, N_LANES, 3);
		udelay(1);

	} else if (config->mbps == 1296) {
		mipicsi_host_dphy_write(dev, 0x01, 0x20);
		mipicsi_host_dphy_write(dev, 0x02, 0x2B);

		mipicsi_host_dphy_write(dev, 0xE2, 0xB6);
		mipicsi_host_dphy_write(dev, 0xE3, 0x1);
		mipicsi_host_dphy_write(dev, 0xE4, 0x1);

		mipicsi_host_dphy_write(dev, 0x08, 0x20);
		udelay (1);

		RX_OUTf(N_LANES, N_LANES, 3);
		udelay(1);
	} else {  // TEMP

	/* Set hsfreqrange[6:0] */
	if (mipicsi_pll_calc(config->mbps, &pll) != 0)
		return -EINVAL;

	mipicsi_host_dphy_write(dev, R_DPHY_RDWR_RX_SYS_0, 1<<5);
	mipicsi_host_dphy_write(dev, R_DPHY_RDWR_RX_SYS_1, pll.hsfreq);

	/* Refer to "Frequency Ranges and Default" on page 141 and configure
	 * registers 0xe2, 0xe3 with the appropriate DDL target oscillation
	 * frequency. Enable override to configure the DDL target oscillation
	 * frequency on bit 0 of register 0xe4.
	 */
	if (pll.sr_osc_freq_tgt != 0){
		mipicsi_host_dphy_write(dev, R_DPHY_RDWR_RX_RX_STARTUP_OVR_2,
					pll.sr_osc_freq_tgt & 0xFF);
		mipicsi_host_dphy_write(dev, R_DPHY_RDWR_RX_RX_STARTUP_OVR_3,
					(pll.sr_osc_freq_tgt>>8) & 0xFF);
		mipicsi_host_dphy_write(dev, R_DPHY_RDWR_RX_RX_STARTUP_OVR_4,
					0x01);
	}

	/*  Configure register 0x8 to set deskew_polarity_rw signal
	 * (bit 5) to 1'b1
	 */
	mipicsi_host_dphy_write (dev, R_DPHY_RDWR_RX_SYS_7, 1<<5);

	/* Set cfgclkfreqrange[7:0] = round[ (Fcfg_clk(MHz)-17)*4]
	 * = 8'b10000100, assuming cfg_clk = 50MHz */
	/* Hardware controlled */

	/* Apply cfg_clk signal with 50Mhz frequency */
	/* Hardware controlled */

	/* Set basedir_0 = 1'b1; 12. Set all requests inputs to zero; The 
	 * purpose is to ensure that the following signals are set to low 
	 * logic level: txrequestesc_0 and turnrequest_0; */
	/* Hardware controlled */

	/*  Wait for 15 ns */
	udelay (1);

	/* Set enable_0/1/2/3, and enableclk=1'b1; 15. Wait 5ns */
	RX_OUTf(N_LANES, N_LANES, (config->num_lanes-1));
	udelay(1);
	} //TEMP

#endif

	/* Set SHUTDOWNZ=1'b1 */
	RX_OUTf(PHY_SHUTDOWNZ, PHY_SHUTDOWNZ, 1);

	/* Wait 5ns */
	udelay(1);

	/* Set RSTZ=1'b1 */
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
	pr_info("%s: Host not configured in 200us - 0x%0x\n", __func__, data);

	return 0;
}

int mipicsi_host_stop(enum mipicsi_top_dev dev)
{
	void * baddr = dev_addr_map[dev];
	if (!baddr) {
		return -ENXIO;
	}
	RX_OUTf(PHY_SHUTDOWNZ, PHY_SHUTDOWNZ, 0);
	RX_OUTf(DPHY_RSTZ, DPHY_RSTZ, 0);
	RX_OUT(PHY_TEST_CTRL0, 1);

	return 0;
}


int mipicsi_host_hw_init(enum mipicsi_top_dev dev)
{
	void * baddr = dev_addr_map[dev];
	if (!baddr) {
		pr_err("%s: no address for %d\n", __func__, dev);
		return -ENXIO;
	}
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

	RX_OUTf(PHY_SHUTDOWNZ, PHY_SHUTDOWNZ, 0);

	pr_info("%s: %d X\n", __func__, dev);

	return 0;
}

static irqreturn_t mipicsi_host_irq(int irq, void *device)
{

	struct mipi_dev *mipidev = device;
	int ret = IRQ_NONE;
	void *baddr = mipidev->base_address;
	/* latest read of interrupt status registers */
	struct mipi_host_irq_st *int_status =
		(struct mipi_host_irq_st *) mipidev->data;

	spin_lock(&mipidev->slock);

	int_status->main = RX_IN(INT_ST_MAIN);

	if (int_status->main & RX_MASK(INT_ST_MAIN, STATUS_INT_PHY_FATAL)) {
		int_status->phy_fatal = RX_IN(INT_ST_PHY_FATAL);
		dev_info(mipidev->dev, "CSI INT PHY FATAL: %x\n",
			 int_status->phy_fatal);
		ret = IRQ_HANDLED;
	}

	if (int_status->main & RX_MASK(INT_ST_MAIN, STATUS_INT_PKT_FATAL)) {
		int_status->pkt_fatal = RX_IN(INT_ST_PKT_FATAL);
		dev_info(mipidev->dev, "CSI INT PKT FATAL: %x\n",
			 int_status->pkt_fatal);
		ret = IRQ_HANDLED;
	}

	if (int_status->main & RX_MASK(INT_ST_MAIN, STATUS_INT_FRAME_FATAL)) {
		int_status->frame_fatal = RX_IN(INT_ST_FRAME_FATAL);
		dev_info(mipidev->dev, "CSI INT FRAME FATAL: %x\n",
			 int_status->frame_fatal);
		ret = IRQ_HANDLED;
	}

	if (int_status->main & RX_MASK(INT_ST_MAIN, STATUS_INT_PHY)) {
		int_status->phy = RX_IN(INT_ST_PHY);
		dev_info(mipidev->dev, "CSI INT PHY: %x\n", int_status->phy);
		ret = IRQ_HANDLED;
	}

	if (int_status->main & RX_MASK(INT_ST_MAIN, STATUS_INT_PKT)) {
		int_status->pkt = RX_IN(INT_ST_PKT);
		dev_info(mipidev->dev, "CSI INT PKT: %x\n", int_status->pkt);
		ret = IRQ_HANDLED;
	}

	if (int_status->main & RX_MASK(INT_ST_MAIN, STATUS_INT_LINE)) {
		int_status->line = RX_IN(INT_ST_LINE);
		dev_info(mipidev->dev, "CSI INT LINE: %x\n", int_status->line);
		ret = IRQ_HANDLED;
	}

	spin_unlock(&mipidev->slock);

	return ret;
}

int mipicsi_host_get_interrupt_status(enum mipicsi_top_dev devid,
				      struct mipi_host_irq_st *int_status)
{
	int ret;
	struct mipi_dev *mipidev;
	struct mipi_host_irq_st *cur_status;

	pr_debug("%s: dev %d\n", __func__, devid);
	if ((devid == MIPI_RX0) || (devid == MIPI_RX1) || (devid == MIPI_RX1)) {
		mipidev = mipicsi_get_device(devid);
		if (mipidev != NULL) {
			cur_status = (struct mipi_host_irq_st *) mipidev->data;
			dev_dbg(mipidev->dev, "mipidev 0x%x, int_status 0x%x\n",
				mipidev, cur_status);
			/* copy the values from current status
			* and reset the current status.
			*/
			int_status->main = cur_status->main;
			int_status->phy_fatal = cur_status->phy_fatal;
			int_status->pkt_fatal = cur_status->pkt_fatal;
			int_status->frame_fatal = cur_status->frame_fatal;
			int_status->phy = cur_status->phy;
			int_status->pkt = cur_status->pkt;
			int_status->line = cur_status->line;
			memset(cur_status, 0, sizeof(*cur_status));
			return ret;
		}
		pr_debug("%s: No mipi device found for dev %d\n",
			__func__, devid);
	}
	return -EINVAL;
}

int mipicsi_host_set_interrupt_mask(enum mipicsi_top_dev devid,
				      struct mipi_host_irq_mask *mask)
{
	int ret;
	struct mipi_dev *mipidev;
	void *baddr;

	pr_debug("%s: dev %d\n", __func__, devid);
	if ((devid == MIPI_RX0) || (devid == MIPI_RX1) || (devid == MIPI_RX1)) {
		mipidev = mipicsi_get_device(devid);
		if (mipidev != NULL) {
			baddr = mipidev->base_address;
			dev_dbg("%s Set masks\n", __func__);
			RX_OUT(INT_MSK_PHY_FATAL, mask->phy_fatal);
			RX_OUT(INT_MSK_PKT_FATAL, mask->pkt_fatal);
			RX_OUT(INT_MSK_FRAME_FATAL, mask->frame_fatal);
			RX_OUT(INT_MSK_PHY, mask->phy);
			RX_OUT(INT_MSK_PKT, mask->pkt);
			RX_OUT(INT_MSK_LINE, mask->line);
			return ret;
		}
		pr_debug("%s: No mipi device found for dev %d\n",
			__func__, devid);
	}
	return -EINVAL;
}

int mipicsi_host_force_interrupt(enum mipicsi_top_dev devid,
				 struct mipi_host_irq_mask *mask)
{
	int ret;
	struct mipi_dev *mipidev;
	void *baddr;

	pr_debug("%s: dev %d\n", __func__, devid);
	if ((devid == MIPI_RX0) || (devid == MIPI_RX1) || (devid == MIPI_RX1)) {
		mipidev = mipicsi_get_device(devid);
		if (mipidev != NULL) {
			baddr = mipidev->base_address;
			dev_dbg("%s Force interrupts\n", __func__);
			RX_OUT(INT_FORCE_PHY_FATAL, mask->phy_fatal);
			RX_OUT(INT_FORCE_PKT_FATAL, mask->pkt_fatal);
			RX_OUT(INT_FORCE_FRAME_FATAL, mask->frame_fatal);
			RX_OUT(INT_FORCE_PHY, mask->phy);
			RX_OUT(INT_FORCE_PKT, mask->pkt);
			RX_OUT(INT_FORCE_LINE, mask->line);
			return ret;
		}
		pr_debug("%s: No mipi device found for dev %d\n",
			__func__, devid);
	}
	return -EINVAL;
}

int mipicsi_host_probe(struct platform_device *pdev)
{
	int ret = 0;
	int error = 0;
	struct resource *mem = NULL;
	struct mipi_dev *dev;
	int irq_number = 0;
	struct mipi_host_irq_st *int_status = NULL;
	char *device_id_name;
	struct device_node *np = NULL;

	dev_info(&pdev->dev, "Installing MIPI CSI-2 HOST module...\n");

	dev_info(&pdev->dev, "Device registration\n");
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		dev_err(&pdev->dev, "Could not allocated mipi_csi_host\n");
		return -ENOMEM;
	}
	int_status = kzalloc(sizeof(struct mipi_host_irq_st), GFP_KERNEL);
	if (!int_status)
		return -ENOMEM;
	dev->data = int_status;

	/* Update the device node */
	dev->dev = &pdev->dev;
	np = pdev->dev.of_node;
	if (np == NULL)
		dev_err(&pdev->dev, "Could not find of device node!\n");
	else
		dev_info(&pdev->dev, "Device node 0x%x\n", np);


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

	dev_info(&pdev->dev, "MIPI HOST: ioremapped to %p\n",
		 dev->base_address);

	mipicsi_util_save_virt_addr(MIPI_RX0, dev->base_address);

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
		dev->irq_number = irq_number;

		/* Register interrupt */
		ret = request_irq(dev->irq_number, mipicsi_host_irq,
				IRQF_SHARED, dev_name(&pdev->dev), dev);
		if (ret)
			dev_err(&pdev->dev,
				"Could not register controller interrupt\n");
	} else
		dev_err(&pdev->dev, "IRQ num not set. See device tree.\n");

	if (of_property_read_string(np, "device-id", &device_id_name)) {
		dev_err(&pdev->dev, "Could not read device id!\n");
	} else {
		dev->device_id = get_device_id(device_id_name);
		mipicsi_set_device(dev->device_id, dev);
	}
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
static int mipicsi_host_remove(struct platform_device *pdev)
{
	struct mipi_dev *dev;
	struct list_head *list;

	dev_dbg(&pdev->dev, "Removing MIPI CSI-2 module\n");
	while (!list_empty(&devlist_global)) {
		list = devlist_global.next;
		list_del(list);
		dev = list_entry(list, struct mipi_dev, devlist);

		devm_free_irq(&pdev->dev, dev->irq_number, dev);

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
