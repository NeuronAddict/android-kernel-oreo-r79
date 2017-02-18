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
#include "mipicsi_device.h"
#include "mipicsi_util.h"
#include "mipicsi_dc_dphy.h"
#include "mipicsi_tx_dphy.h"
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

void config_clk_data_timing(enum mipicsi_top_dev dev, uint32_t mbps)
{
	uint32_t ui_ps, byteclk_ps;
	uint16_t tclk_lp_ns, tclk_prep_ns, tclk_zero_ns, tclk_trail_ns,
		tclk_exit_ns, tclk_post_ns;
	uint16_t ths_lp_ns, ths_prep_ns, ths_zero_ns, ths_trail_ns,
		ths_exit_ns;
	uint8_t value;

	void * baddr = dev_addr_map[dev];
	if (!baddr) {
		pr_err("%s: no address for %d\n", __func__, dev);
		return;
	}

	ui_ps = 1000*1000/mbps;
	byteclk_ps = ui_ps*8;

	/*
	 * Calculate the target clock/data lane timings. These are based on
	 * MIPI D-Phy 2.0 Spec minimum timing + % padding
	 * Note: Tclk-post is higher than spec due to Synopsys limitation
	 */
	tclk_lp_ns = PAD(50);
	tclk_prep_ns = MIN(PAD(38), TRIM(95));
	tclk_zero_ns = PAD(300-tclk_prep_ns);
	tclk_trail_ns = PAD(60);
	tclk_exit_ns = PAD(100);
	tclk_post_ns = PAD(60+52*ui_ps/1000);

	ths_lp_ns = PAD(50);
	ths_prep_ns = MIN(PAD(40+4*ui_ps/1000), TRIM(85+6*ui_ps/1000));
	ths_zero_ns = PAD(145+10*ui_ps/1000-ths_prep_ns);
	ths_trail_ns = PAD(MAX(8*ui_ps/1000, 60+4*ui_ps/1000));
	ths_exit_ns = PAD(100);

	pr_info("\n\nui_ps=%d, byteclk_ps=%d\n", ui_ps, byteclk_ps);

	pr_info("\nTarget Timings - Minimum + %dpct pad\n", PAD_PCT);
	pr_info("TCLK: lp=%d, prep=%d, zero=%d, trail=%d, exit=%d, post=%d\n",
		tclk_lp_ns, tclk_prep_ns, tclk_zero_ns, tclk_trail_ns,
		tclk_exit_ns, tclk_post_ns);

	pr_info("THS : lp=%d, prep=%d, zero=%d, trail=%d, exit=%d\n\n",
		ths_lp_ns, ths_prep_ns, ths_zero_ns, ths_trail_ns,
		ths_exit_ns);

	if (mipicsi_util_is_emulation()) {

		/* Calculate counters for clock and data lane timings */

		/* CLK LP */
		value = ROUNDUP((tclk_lp_ns-TLP_CONST_TIME)*1000, byteclk_ps);
		mipicsi_dev_dphy_write(dev, R_CSI2_DCPHY_TX_TCLK_LP,
				       (1<<7) | value);

		/* CLK Prepare */
		value = ROUNDUP((tclk_prep_ns-PREP_CONST_TIME)*1000,
				byteclk_ps)-1;
		mipicsi_dev_dphy_write(dev, R_CSI2_DCPHY_TX_TCLK_PREP,
				       (1<<7) | value);

		/* CLK Zero */
		value = ROUNDUP((tclk_zero_ns-ZERO_CONST_TIME)*1000,
				byteclk_ps)-1;
		mipicsi_dev_dphy_write(dev, R_CSI2_DCPHY_TX_TCLK_ZERO,
				       (1<<7) | value);

		/* CLK Trail */
		value = ROUNDUP((tclk_trail_ns-TRAIL_CONST_TIME)*1000,
				byteclk_ps)-1;
		mipicsi_dev_dphy_write(dev, R_CSI2_DCPHY_TX_TCLK_TRAIL,
				       (1<<7) | value);

		/* CLK Exit */
		value = ROUNDUP((tclk_exit_ns-EXIT_CONST_TIME)*1000,
				byteclk_ps)-1;
		mipicsi_dev_dphy_write(dev, R_CSI2_DCPHY_TX_TCLK_EXIT, (1<<5)
				       | value);

		/* CLK Post */
		value = ROUNDUP((tclk_post_ns-POST_CONST_TIME)*1000,
				byteclk_ps)-1;
		mipicsi_dev_dphy_write(dev, R_CSI2_DCPHY_TX_TCLK_POST, (1<<5)
				       | value);

		/* HS LP */
		value = ROUNDUP((ths_lp_ns-TLP_CONST_TIME)*1000, byteclk_ps)-1;
		mipicsi_dev_dphy_write(dev, R_CSI2_DCPHY_TX_THS_LP,
				       (1<<7) | value);

		/* HS Prepare */
		value = ROUNDUP((ths_prep_ns+PREP_CONST_TIME+ui_ps/2000)
				*1000, byteclk_ps)-1;;
		mipicsi_dev_dphy_write(dev, R_CSI2_DCPHY_TX_THS_PREP,
				       (1<<7) | value);

		/* HS Zero */
		value = ROUNDUP((ths_zero_ns-ZERO_CONST_TIME)*1000,
				byteclk_ps)-1;
		mipicsi_dev_dphy_write(dev, R_CSI2_DCPHY_TX_THS_ZERO,
				       (1<<7) | value);

		/* HS Trail */
		value = ROUNDUP((ths_trail_ns-TRAIL_CONST_TIME+ui_ps/2000)
				*1000, byteclk_ps) - 1;
		mipicsi_dev_dphy_write(dev, R_CSI2_DCPHY_TX_THS_TRAIL,
				       (1<<7) | value);

		/* HS Exit */
		value = ROUNDUP((ths_exit_ns-EXIT_CONST_TIME)*1000,
				byteclk_ps)+1;
		mipicsi_dev_dphy_write(dev, R_CSI2_DCPHY_TX_THS_EXIT,
				       (1<<5) | value);

		/* Phy Stop Wait time */
		mipicsi_pll_get_stop_wait (mbps, &value);
		value = ROUNDUP(value*1000, byteclk_ps);
		TX_OUTf(PHY_IF_CFG, PHY_STOP_WAIT_TIME, value);
	} else {
		/* TO DO: Revisit formulas/ANLG_FCTR after simulation results */

		/* Calculate and write clock lane timing values */
		/* CLK Post */
		value = ROUNDUP(tclk_post_ns*1000, byteclk_ps);
		mipicsi_dev_dphy_write(dev, R_DPHY_RDWR_TX_SYSTIMERS_12,
				       (1<<6) | value);
		pr_info("\t tclk_post_ns %d - %d\n", tclk_post_ns, value);

		/* CLK Exit*/
		value = ROUNDUP(tclk_exit_ns*1000, byteclk_ps);
		if (mbps >= 400)
			value -= 2;
		value = MAX(value, 1);
		mipicsi_dev_dphy_write(dev, R_DPHY_RDWR_TX_SYSTIMERS_13,
				       (1<<6) | value);
		pr_info("\t tclk_exit_ns %d - %d\n", tclk_exit_ns, value);

		/* CLK Prepare */
		value = ROUNDUP((tclk_prep_ns-ANLG_FCTR)*1000, byteclk_ps);
		value = MAX(value, 1);
		mipicsi_dev_dphy_write(dev, R_DPHY_RDWR_TX_SYSTIMERS_14,
				       (1<<7) | (1<<6) | value);
		pr_info("\t tclk_prep_ns %d - %d\n", tclk_prep_ns, value);

		/*
		 * CLK LP - Target time higher than spec due to power up
		 * requirements. TBD: tclk_lp_ns = PAD(500);
		 */
		value = ROUNDUP((tclk_lp_ns-(byteclk_ps/1000))*1000,
				byteclk_ps);
		if (mbps >= 400)
			value -= 1;
		value = MAX(value, 1);
		mipicsi_dev_dphy_write(dev, R_DPHY_RDWR_TX_SYSTIMERS_15, value);
		pr_info("\t tclk_lp_ns %d - %d\n", tclk_lp_ns, value);

		/* CLK Trail */
		value = ROUNDUP((tclk_trail_ns-ANLG_FCTR)*1000, byteclk_ps);
		if (mbps >= 400)
			value -= 1;
		value = MAX(value, 1);
		mipicsi_dev_dphy_write(dev, R_DPHY_RDWR_TX_SYSTIMERS_16,
			       (1<<6) | value);
		pr_info("\t tclk_trail_ns %d - %d\n", tclk_trail_ns, value);

		/* CLK Zero */
		value = ROUNDUP((tclk_zero_ns-ANLG_FCTR)*1000, byteclk_ps);
		if (mbps >= 400)
			value -= 3;
		value = MAX(value, 1);
		mipicsi_dev_dphy_write(dev, R_DPHY_RDWR_TX_SYSTIMERS_17,
				       (1<<7) | value);
		pr_info("\t tclk_zero_ns %d - %d\n", tclk_zero_ns, value);

		/* Calculate and write data lane timing values */
		/* HS Exit */
		value = ROUNDUP(ths_exit_ns*1000, byteclk_ps);
		if (mbps >= 400)
			value -= 2;
		value = MAX(value, 1);
		mipicsi_dev_dphy_write(dev, R_DPHY_RDWR_TX_SYSTIMERS_19,
				       (1<<6) | value);
		pr_info("\t ths_exit_ns %d - %d\n", ths_exit_ns, value);

		/* HS Prepare */
		value = ROUNDUP((ths_prep_ns-ANLG_FCTR)*1000, byteclk_ps);
		value = MAX(value, 1);
		mipicsi_dev_dphy_write(dev, R_DPHY_RDWR_TX_SYSTIMERS_20,
				       (1<<7) | (1<<6) | value);
		pr_info("\t ths_exit_ns %d - %d\n", ths_exit_ns, value);

		/*
		 * HS LP - Target time higher than spec due to power up
		 * requirements. TBD ths_lp_ns = PAD(500)
		 */
		value = ROUNDUP((ths_lp_ns-(byteclk_ps/1000))*1000,
				byteclk_ps);
		if (mbps >= 400)
			value -= 1;
		value = MAX(value, 1);
		mipicsi_dev_dphy_write(dev, R_DPHY_RDWR_TX_SYSTIMERS_21,
				       value);
		pr_info("\t ths_lp_ns %d - %d\n", ths_lp_ns, value);

		/* HS Trail */
		value = ROUNDUP((ths_trail_ns-ANLG_FCTR)*1000, byteclk_ps);
		if (mbps >= 400)
			value -= 1;
		value = MAX(value, 1);
		mipicsi_dev_dphy_write(dev, R_DPHY_RDWR_TX_SYSTIMERS_22,
				       (1<<6) | value);
		pr_info("\t ths_trail_ns %d - %d\n", ths_trail_ns, value);

		/* HS Zero */
		value = ROUNDUP((ths_zero_ns-ANLG_FCTR)*1000, byteclk_ps);
		if (mbps >= 400)
			value -= 3;
		value = MAX(value, 1);
		mipicsi_dev_dphy_write(dev, R_DPHY_RDWR_TX_SYSTIMERS_23,
				       (1<<7) | value);
		pr_info("\t ths_zero_ns %d - %d\n", ths_zero_ns, value);

		/* Phy Stop Wait time */
		mipicsi_pll_get_stop_wait(mbps, &value);
		value = ROUNDUP(value*1000, byteclk_ps);
		pr_info("%s: Phy stop wait = %d", __func__, value);
		TX_OUTf(PHY_IF_CFG, PHY_STOP_WAIT_TIME, value);
	}
}

void mipicsi_dev_dphy_write(enum mipicsi_top_dev dev,
			    uint16_t command, uint8_t data)
{
	void * baddr = dev_addr_map[dev];
	if (!baddr) {
		pr_err("%s: no address for %d\n", __func__, dev);
		return;
	}

	pr_info("%s: dev=0x%x, command 0x%02X data=0x%02X\n",
		__func__, dev, command, data);

	TX_OUT(PHY_RSTZ, 0);

	if (mipicsi_util_is_emulation()) {
		/* Set the desired testcode */
		TX_OUTf(PHY0_TST_CTRL0, PHY0_TESTCLR, 0);
		TX_OUTf(PHY0_TST_CTRL0, PHY0_TESTCLK, 1);
		TX_OUTf(PHY0_TST_CTRL1, PHY0_TESTDIN, command);
		TX_OUTf(PHY0_TST_CTRL1, PHY0_TESTEN,  1);
		TX_OUTf(PHY0_TST_CTRL0, PHY0_TESTCLK, 0);

		/* Enter the test data */
		TX_OUTf(PHY0_TST_CTRL1, PHY0_TESTEN,  0);
		TX_OUTf(PHY0_TST_CTRL1, PHY0_TESTDIN, data);
		TX_OUTf(PHY0_TST_CTRL0, PHY0_TESTCLK, 1);
	} else {
		/* Write 4-bit testcode MSB */
		TX_OUTf(PHY0_TST_CTRL0, PHY0_TESTCLK, 0);
		TX_OUTf(PHY0_TST_CTRL1, PHY0_TESTEN,  0);
		TX_OUTf(PHY0_TST_CTRL1, PHY0_TESTEN,  1);
		TX_OUTf(PHY0_TST_CTRL0, PHY0_TESTCLK, 1);
		TX_OUTf(PHY0_TST_CTRL1, PHY0_TESTDIN, 0);
		TX_OUTf(PHY0_TST_CTRL0, PHY0_TESTCLK, 0);
		TX_OUTf(PHY0_TST_CTRL1, PHY0_TESTEN,  0);
		TX_OUTf(PHY0_TST_CTRL1, PHY0_TESTDIN, ((command & 0xF00)>>8));
		TX_OUTf(PHY0_TST_CTRL0, PHY0_TESTCLK, 1);

		/* Write 8-bit testcode LSB */
		TX_OUTf(PHY0_TST_CTRL0, PHY0_TESTCLK, 0);
		TX_OUTf(PHY0_TST_CTRL1, PHY0_TESTEN,  1);
		TX_OUTf(PHY0_TST_CTRL0, PHY0_TESTCLK, 1);
		TX_OUTf(PHY0_TST_CTRL1, PHY0_TESTDIN, (command & 0xFF));
		TX_OUTf(PHY0_TST_CTRL0, PHY0_TESTCLK, 0);
		TX_OUTf(PHY0_TST_CTRL1, PHY0_TESTEN,  0);

		/* Write the data */
		TX_OUTf(PHY0_TST_CTRL1, PHY0_TESTDIN, data);
		TX_OUTf(PHY0_TST_CTRL0, PHY0_TESTCLK, 1);
	}
}

uint8_t mipicsi_dev_dphy_read(enum mipicsi_top_dev dev, uint16_t command)
{
	void *baddr = dev_addr_map[dev];
	uint8_t data;

	if (!baddr) {
		pr_err("%s: no address for %d\n", __func__, dev);
		return 0;
	}

	pr_info("%s: dev=0x%x @ %p, command 0x%02X\n",
		__func__, dev, baddr, command);

	if (mipicsi_util_is_emulation()) {
		TX_OUTf(PHY0_TST_CTRL0, PHY0_TESTCLR, 0);
		TX_OUTf(PHY0_TST_CTRL1, PHY0_TESTDIN, command);
		TX_OUTf(PHY0_TST_CTRL1, PHY0_TESTEN,  1);
		TX_OUTf(PHY0_TST_CTRL0, PHY0_TESTCLK, 1);
		TX_OUTf(PHY0_TST_CTRL0, PHY0_TESTCLK, 0);
		TX_OUTf(PHY0_TST_CTRL1, PHY0_TESTEN,  0);
		data = (TX_IN(PHY0_TST_CTRL1))>>8;
	} else {
		/* Write 4-bit testcode MSB */
		TX_OUTf(PHY0_TST_CTRL0, PHY0_TESTCLK, 0);
		TX_OUTf(PHY0_TST_CTRL1, PHY0_TESTEN,  0);
		TX_OUTf(PHY0_TST_CTRL1, PHY0_TESTEN,  1);
		TX_OUTf(PHY0_TST_CTRL0, PHY0_TESTCLK, 1);
		TX_OUTf(PHY0_TST_CTRL1, PHY0_TESTDIN, 0);
		TX_OUTf(PHY0_TST_CTRL0, PHY0_TESTCLK, 0);
		TX_OUTf(PHY0_TST_CTRL1, PHY0_TESTEN,  0);
		TX_OUTf(PHY0_TST_CTRL1, PHY0_TESTDIN, ((command & 0xF00)>>8));
		TX_OUTf(PHY0_TST_CTRL0, PHY0_TESTCLK, 1);

		/* Write 8-bit testcode LSB */
		TX_OUTf(PHY0_TST_CTRL0, PHY0_TESTCLK, 0);
		TX_OUTf(PHY0_TST_CTRL1, PHY0_TESTEN,  1);
		TX_OUTf(PHY0_TST_CTRL0, PHY0_TESTCLK, 1);
		TX_OUTf(PHY0_TST_CTRL1, PHY0_TESTDIN, (command & 0xFF));
		TX_OUTf(PHY0_TST_CTRL0, PHY0_TESTCLK, 0);
		TX_OUTf(PHY0_TST_CTRL1, PHY0_TESTEN,  0);

		/* Read the data */
		data = (TX_IN(PHY0_TST_CTRL1))>>8;
	}

	pr_err("%s: X  Offset: 0x%x, Value: 0x%x\n", __func__, command,
	       data);
	return data;
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

	dev = config->dev;

	if (mipicsi_pll_calc(config->mbps, &pll) != 0)
		return -EINVAL;

	if (mipicsi_util_is_emulation()) {
		mipicsi_dev_dphy_write(dev, R_CSI2_DCPHY_HS_RX_CTRL_L0,
				       (pll.hsfreq << 1));

		mipicsi_dev_dphy_write(dev, R_CSI2_DCPHY_PLL_BIAS_FCC_VCO, (0x1 << 7) |
				       (pll.vco_range << 3) | 1);

		mipicsi_dev_dphy_write(dev, R_CSI2_DCPHY_PLL_LPF_CP_CTRL, (0x01 << 7) |
				       (0x01 << 6) | (pll.lpf_resistor << 0));

		mipicsi_dev_dphy_write(dev, R_CSI2_DCPHY_PLL_CP_LOCK_BYP_ULP,
				       (0x00 << 4) | (pll.cp_current << 0));

#ifdef DC_BUG_FREE
		/* Program log2(output divider) to register */
		uint8_t val;
		val = ilog2(pll.output_div);
		mipicsi_dev_dphy_write(dev, R_CSI2_DCPHY_PLL_DIV_RAT_CTRL,
				       ((0x1<<5) | (0x1<<4) | (val<<0)));
#else
		/* Program output divider P=1 bits[1:0]=00; P=2 bits[1:0]=10 */
		/* NOTE: This is due to a bug in daughtercards */
		if (pll.output_div == 1)
			mipicsi_dev_dphy_write(dev, R_CSI2_DCPHY_PLL_DIV_RAT_CTRL,
					       ((0x1<<5) | (0x1<<4) | (0x01<<2)));
		else if (pll.output_div == 2)
		mipicsi_dev_dphy_write(dev, R_CSI2_DCPHY_PLL_DIV_RAT_CTRL,
				       ((0x1<<5) | (0x1<<4) | (0x01<<2)
					| (2<<0)));
		else
			mipicsi_dev_dphy_write(dev, R_CSI2_DCPHY_PLL_DIV_RAT_CTRL,
					       ((0x1<<5) | (0x1<<4)));
#endif

		/* Program N-1 to register */
		mipicsi_dev_dphy_write(dev, R_CSI2_DCPHY_PLL_INPUT_DIV_RAT,
				       pll.input_div-1);

		/* Program M-2 to register */
		mipicsi_dev_dphy_write_set(dev, R_CSI2_DCPHY_PLL_LOOP_DIV_RAT,
					   (((pll.loop_div-2) >> 0) & 0x1F), 0, 1);
		mipicsi_dev_dphy_write_set(dev, R_CSI2_DCPHY_PLL_LOOP_DIV_RAT,
					   (((pll.loop_div-2) >> 5) & 0x1F), 1, 1);
	} else {
		/* Set hsfreqrange[6:0]  */
		mipicsi_dev_dphy_write(dev, R_DPHY_RDWR_TX_SYS_1, pll.hsfreq);
		mipicsi_dev_dphy_write(dev, R_DPHY_RDWR_TX_SYS_0, 1<<5);

		/* Set cfgclkfreqrange[7:0] = round[ (Fcfg_clk(MHz)-17)*4] = 8'b10000100
		   assuming cfg_clk = 50MHz; */
		/* Hardware controlled*/

		/* Apply cfg_clk signal with 50Mhz frequency */
		/* Hardware controlled */

		/*
		 * Refer to table "Slew rate vs DDL oscilation target" on page
		 * 117 and configure test control registers with appropriate
		 * values for the
		 * specified rise/fall time.
		 */
		pr_err("sr_osc_freq_tgt 0x%x", pll.sr_osc_freq_tgt);
		if (pll.sr_osc_freq_tgt != 0){
			mipicsi_dev_dphy_write(dev, R_DPHY_RDWR_TX_SLEW_5,
					       pll.sr_osc_freq_tgt & 0xFF);
			mipicsi_dev_dphy_write(dev, R_DPHY_RDWR_TX_SLEW_6,
					       (pll.sr_osc_freq_tgt>>8) & 0xFF);
			mipicsi_dev_dphy_write(dev, R_DPHY_RDWR_TX_SLEW_7,
					       (pll.sr_range<<0) | (1<<4));
		}

		/* Configure PLL operating frequency through D-PHY test control
		 * registers or through PLL SoC shadow registers interface as
		 * described in section "Initialization" on page 53
		 */
		mipicsi_dev_dphy_write(dev, R_DPHY_RDWR_TX_PLL_28,
				       (pll.loop_div-2) & 0xFF);
		mipicsi_dev_dphy_write(dev, R_DPHY_RDWR_TX_PLL_29,
				       ((pll.loop_div-2)>>8) & 0xFF);
		mipicsi_dev_dphy_write(dev, R_DPHY_RDWR_TX_PLL_30,
				       (1<<7) | (pll.vco_cntrl<<1) | 1<<0);
		mipicsi_dev_dphy_write(dev, R_DPHY_RDWR_TX_PLL_27,
				       (1<<7) | ((pll.input_div-1)<<3));

		if (config->mbps <= 450)
			mipicsi_dev_dphy_write(dev, R_DPHY_RDWR_TX_CB_2, 1<<4);

		/* TO DO - these can come from fuse bits */
		mipicsi_dev_dphy_write(dev, R_DPHY_RDWR_TX_PLL_1, 0x10);
		mipicsi_dev_dphy_write(dev, R_DPHY_RDWR_TX_PLL_5, 0x04);
		mipicsi_dev_dphy_write(dev, R_DPHY_RDWR_TX_PLL_17, 0x0C);

		/* PLL phase error threshold */
		mipicsi_dev_dphy_write(dev, R_DPHY_RDWR_TX_PLL_22, 0x02);
		mipicsi_dev_dphy_write(dev, R_DPHY_RDWR_TX_PLL_23, 0x00);
		mipicsi_dev_dphy_write(dev, R_DPHY_RDWR_TX_PLL_24, 0x60);
		mipicsi_dev_dphy_write(dev, R_DPHY_RDWR_TX_PLL_25, 0x03);
	}
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
	uint32_t data = 0;
	uint8_t counter = 0, val;
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

	if (mipicsi_util_is_emulation()) {
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

		/* Configure as TX per Synopsys feedback - registers not in databook */
		mipicsi_dev_dphy_write(dev, 0xB0, 0x00);
		udelay (10);
		mipicsi_dev_dphy_write(dev, 0xAC, 0x03);

		/* Enable lanes */
		TX_OUTf(PHY_IF_CFG, LANE_EN_NUM, (config->num_lanes-1));

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

		/* Stay in high power so LP->Tx is faster */
		mipicsi_dev_dphy_write (dev, R_CSI2_DCPHY_LP_TX_PWR_CTRL_CLK, 0x03);
		mipicsi_dev_dphy_write (dev, R_CSI2_DCPHY_HS_TX_PWR_CTRL_CLK, 0x0C);
		mipicsi_dev_dphy_write (dev, R_CSI2_DCPHY_HS_TX_PWR_CTRL_L0, 0x0C);
		mipicsi_dev_dphy_write (dev, R_CSI2_DCPHY_HS_TX_PWR_CTRL_L1, 0x0C);
		mipicsi_dev_dphy_write (dev, R_CSI2_DCPHY_HS_TX_PWR_CTRL_L2, 0x0C);
		mipicsi_dev_dphy_write (dev, R_CSI2_DCPHY_HS_TX_PWR_CTRL_L3, 0x0C);

		config_clk_data_timing (config->dev, config->mbps);

		TX_OUT(PHY_RSTZ, 0x07);
		udelay(1);
	} else {
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

		mipicsi_device_set_pll(config);

		/* Set basedir_0 = 1'b0 */
		/* Hardware controlled */

		/* Set all requests inputs to zero; The purpose is to ensure that the
		 * following signals are set to low logic level: txrequesthsclk,
		 * txrequestdatahs_0/1/2/3, txrequestesc_0/1/2/3 and turnrequest_0;
		 */
		/* Hardware controlled */

		/* Wait for 15ns */
		/* Enable lanes */
		udelay(1);
		TX_OUTf(PHY_IF_CFG, LANE_EN_NUM, (config->num_lanes-1));

		config_clk_data_timing (config->dev, config->mbps);

		/*
		 * Enableclk=1'b1; Wait 5ns; Set shutdownz=1'b1;  Wait 5ns;
		 * Set rstz=1'b1;
		 */
		TX_OUTf(PHY_RSTZ, PHY_ENABLECLK, 0x01);
		udelay(1);
		TX_OUTf(PHY_RSTZ, PHY_SHUTDOWNZ, 0x01);
		udelay(1);
		TX_OUTf(PHY_RSTZ, PHY_RSTZ, 0x01);
	}
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
			break;
		}

		udelay(10);
		counter++;
	} while (counter < 30);

	if (counter >= 30)
		pr_info("%s: Device not configured in 200us - 0x%0x\n",
			__func__, data);

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

static irqreturn_t mipicsi_device_irq(int irq, void *device)
{

	struct mipi_dev *mipidev = device;
	void *baddr = mipidev->base_address;
	int ret = IRQ_NONE;
	/* latest read of interrupt status registers */
	struct mipi_device_irq_st *int_status =
		(struct mipi_device_irq_st *) mipidev->data;

	spin_lock(&mipidev->slock);

	int_status->main = TX_IN(INT_ST_MAIN);

	if (int_status->main & TX_MASK(INT_ST_MAIN, INT_ST_VPG)) {
		int_status->vpg = TX_IN(INT_ST_VPG);
		dev_info(mipidev->dev, "CSI INT_ST_VPG: %x\n", int_status->vpg);
		ret = IRQ_HANDLED;
	}

	if (int_status->main & TX_MASK(INT_ST_MAIN, INT_ST_IDI)) {
		int_status->idi = TX_IN(INT_ST_IDI);
		dev_info(mipidev->dev, "CSI INT_ST_IDI: %x\n", int_status->idi);
		ret = IRQ_HANDLED;
	}

	if (int_status->main & TX_MASK(INT_ST_MAIN, INT_ST_PHY)) {
		int_status->phy = TX_IN(INT_ST_PHY);
		dev_info(mipidev->dev, "CSI INT_ST_PHY: %x\n", int_status->phy);
		ret = IRQ_HANDLED;
	}

	spin_unlock(&mipidev->slock);

	return ret;
}

int mipicsi_device_get_interrupt_status(enum mipicsi_top_dev devid,
				      struct mipi_device_irq_st *int_status)
{
	int ret;
	struct mipi_dev *mipidev;
	struct mipi_device_irq_st *cur_status;

	pr_debug("%s: dev %d\n", __func__, devid);
	if ((devid == MIPI_RX0) || (devid == MIPI_RX1) || (devid == MIPI_RX1)) {
		mipidev = mipicsi_get_device(devid);
		if (mipidev != NULL) {
			cur_status = (struct mipi_device_irq_st *)mipidev->data;
			dev_dbg(mipidev->dev, "mipidev 0x%x, int_status 0x%x\n",
				mipidev, cur_status);
			/* copy the values from current status
			* and reset the current status.
			*/
			int_status->main = cur_status->main;
			int_status->vpg = cur_status->vpg;
			int_status->idi = cur_status->idi;
			int_status->phy = cur_status->phy;
			memset(cur_status, 0, sizeof(*cur_status));
			return ret;
		}
		pr_debug("%s: No mipi device found for dev %d\n",
			__func__, devid);
	}
	return -EINVAL;
}

int mipicsi_device_set_interrupt_mask(enum mipicsi_top_dev devid,
				     struct mipi_device_irq_mask *mask)
{
	int ret;
	struct mipi_dev *mipidev;
	void *baddr;

	pr_debug("%s: dev %d\n", __func__, devid);
	if ((devid == MIPI_TX0) || (devid == MIPI_TX1)) {
		mipidev = mipicsi_get_device(devid);
		if (mipidev != NULL) {
			baddr = mipidev->base_address;
			dev_dbg("%s Set masks\n", __func__);
			TX_OUT(INT_MASK_N_VPG, mask->vpg);
			TX_OUT(INT_MASK_N_IDI, mask->idi);
			TX_OUT(INT_MASK_N_PHY, mask->phy);
			return ret;
		}
		pr_debug("%s: No mipi device found for dev %d\n",
			__func__, devid);
	}
	return -EINVAL;
}

int mipicsi_device_force_interrupt(enum mipicsi_top_dev devid,
				  struct mipi_device_irq_mask *mask)
{
	int ret;
	struct mipi_dev *mipidev;
	void *baddr;

	pr_debug("%s: dev %d\n", __func__, devid);
	if ((devid == MIPI_TX0) || (devid == MIPI_TX1)) {
		mipidev = mipicsi_get_device(devid);
		if (mipidev != NULL) {
			baddr = mipidev->base_address;
			dev_dbg("%s Force interrupts\n", __func__);
			TX_OUT(INT_FORCE_VPG, mask->vpg);
			TX_OUT(INT_FORCE_IDI, mask->idi);
			TX_OUT(INT_FORCE_PHY, mask->phy);
			return ret;
		}
		pr_debug("%s: No mipi device found for dev %d\n",
			__func__, devid);
	}
	return -EINVAL;
}

int mipicsi_device_probe(struct platform_device *pdev)
{
	int ret = 0;
	int error = 0;
	struct mipi_dev *dev;
	int irq_number = 0;
	struct resource *mem = NULL;
	char *device_id_name;
	struct device_node *np = NULL;
	struct mipi_device_irq_st *int_status;

	dev_info(&pdev->dev, "Installing MIPI CSI-2 DEVICE module...\n");

	dev_info(&pdev->dev, "Device registration\n");
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		dev_err(&pdev->dev, "Could not allocated mipicsi_device\n");
		return -ENOMEM;
	}
	int_status = kzalloc(sizeof(struct mipi_device_irq_st), GFP_KERNEL);
	if (!int_status)
		return -ENOMEM;
	dev->data = int_status;

	/* Update the device node */
	dev->dev = &pdev->dev;
	np = pdev->dev.of_node;
	if (np == NULL)
		dev_err(&pdev->dev, "Could not find of device node!\n");

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

	/* Read emulation vs silicon setting */
	mipicsi_util_read_emulation ();

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
		ret = request_irq(dev->irq_number, mipicsi_device_irq,
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
		mipicsi_util_save_virt_addr(dev);
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
static int mipicsi_device_remove(struct platform_device *pdev)
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
