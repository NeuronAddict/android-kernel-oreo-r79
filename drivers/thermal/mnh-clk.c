/*
 *
 * MNH Clock Driver
 * Copyright (c) 2016, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */


#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/sysfs.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/intel-hwio.h>
#include <soc/mnh/mnh-hwio-scu.h>
#include <soc/mnh/mnh-hwio-ddr-ctl.h>
#include "mnh-clk.h"


#define PLL_UNLOCK 0x4CD9
#define REF_FREQ_SEL 0x8
#define MAX_STR_COPY 9
#define LP4_LPC_FREQ_SWITCH 0x8A
#define SYS200_CLK_KHZ 200000

#define DEVICE_NAME "mnh_freq_cooling"

#define LP_CMD_EXIT_LP 0x81
#define LP_CMD_SBIT 5

#define MNH_DDR_CTL_IN(reg) \
HW_IN(mnh_dev->ddraddr, DDR_CTL, reg)
#define MNH_DDR_CTL_INf(reg, fld) \
HW_INf(mnh_dev->ddraddr, DDR_CTL, reg, fld)
#define MNH_DDR_CTL_OUTf(reg, fld, val) \
HW_OUTf(mnh_dev->ddraddr, DDR_CTL, reg, fld, val)
#define MNH_DDR_CTL_OUT(reg, val) \
HW_OUT(mnh_dev->ddraddr, DDR_CTL, reg, val)

/* If IPU clock is driven by CPU_IPU PLL
 * calculate IPU divider based on CPU clk and divider
 * CPU CLK < 850: IPU_CLK_DIV = ((CPU_CLK_DIV+1)*2-1)
 * CPU CLK > 850: IPU_CLK_DIV = ((CPU_CLK_DIV+1)*2)
 */
#define IPU_DIV_BY_CPU(freq, div) \
	((freq < 850) ? ((div+1)*2-1):((div+1)*2))

enum mnh_refclk_type {
	REFCLK_KHZ_19200 = 0,
	REFCLK_KHZ_24000,
	REFCLK_KHZ_MAX
};

enum mnh_pll_type {
	CPU_CLK = 0,
	IPU_CLK
};

enum mnh_lpddr_lpc_rsp_type {
	LPC_CMD_NOERR = 0,
	LPC_CMD_ERR = 1
};

enum mnh_ipu_clk_src {
	CPU_IPU_PLL = 0,/*Both CPU and IPU Clock is derived from same PLL */
	IPU_PLL	/* IPU Clock is derived from IPU PLL */
};

struct freq_reg_table {
	char *freq_str;
	int fbdiv;
	int postdiv1;
	int postdiv2;
	int clk_div;
};

/* PLL reference clk table */
static uint32_t refclk_khz_tables[] = {
	19200,	/* 19.2 MHz */
	24000	/* 24.0 MHz */
};

/* SYS200 frequency calculation tables
 * SYS200  FBDIV FBDIV	POSTDIV1 POSTDIV2 FOUTPOSTDIV CLKDIV CLKFreq
 * CPU	 200	 104	 2	  1	   200.000	0	 200.000
 * IPU	 200	 104	 2	  1	   200.000	1	 100.000
 */
static const struct freq_reg_table sys200_reg_tables[] = {
	{"200", 104, 2, 1, 0},	/* CPU, 200 MHz */
	{"100", 104, 2, 1, 1}	/* IPU, 100 MHz */
};

/* CPU clock frequency calculation table */
static const struct freq_reg_table cpu_reg_tables[][CPU_FREQ_MAX+1] = {
	/* refclk = 19.2 MHz */
	{
		{"200", 125, 6, 2, 0},	/* 200 MHz */
		{"400", 125, 6, 1, 0},	/* 400 MHz */
		{"600", 125, 4, 1, 0},	/* 600 MHz */
		{"800", 125, 3, 1, 0},	/* 800 MHz */
		{"950", 99, 2, 1, 0}	/* 950 MHz */
	},
	/* refclk = 24.0 MHz */
	{
		{"200", 100, 6, 2, 0},	/* 200 MHz */
		{"400", 100, 6, 1, 0},	/* 400 MHz */
		{"600", 100, 4, 1, 0},	/* 600 MHz */
		{"800", 100, 3, 1, 0},	/* 800 MHz */
		{"950", 79, 2, 1, 0}	/* 948 MHz */
	}
};

/* IPU clock frequency calculation table */
static const struct freq_reg_table ipu_reg_tables[][IPU_FREQ_MAX+1] = {
	/* refclk = 19.2 MHz */
	{
		{"100", 125, 6, 2, 1},	/* 100 MHz */
		{"200", 125, 6, 1, 1},	/* 200 MHz */
		{"300", 125, 4, 1, 1},	/* 300 MHz */
		{"400", 125, 6, 1, 0},	/* 400 MHz */
		{"425", 133, 6, 1, 0}	/* 425 MHz */
	},
	/* refclk = 24.0 MHz */
	{
		{"100", 100, 6, 2, 1},	/* 100 MHz */
		{"200", 100, 6, 1, 1},	/* 200 MHz */
		{"300", 100, 4, 1, 1},	/* 300 MHz */
		{"400", 100, 6, 1, 0},	/* 400 MHz */
		{"425", 106, 6, 1, 0}	/* 424 MHz */
	}
};

struct mnh_freq_cooling_device {
	struct device *dev;
	void __iomem *regs;
	void __iomem *ddraddr;
	int ddr_irq;
	enum mnh_ipu_clk_src ipu_clk_src;
	enum mnh_cpu_freq_type cpu_freq;
	enum mnh_ipu_freq_type ipu_freq;
	enum mnh_lpddr_freq_type ddr_freq;
	enum mnh_refclk_type refclk;
	const struct freq_reg_table *cpu_pllcfg;
	const struct freq_reg_table *ipu_pllcfg;
};

static struct mnh_freq_cooling_device *mnh_dev;

/**
 * CPU clock controller
 * @index: int with frquency table index info.
 * Return: 0 on success, an error code otherwise.
 *
 * 1PLL(CPU_IPU PLL) for CPU/IPU clocks. Since CPU and IPU clocks are derived
 * from same PLL in this mode, there would be restriction on achedivable clock
 * frequencies for CPU and IPU clocks. IPU clock would be half of CPU clock. Any
 * frequency changes is achieved by changing FBDIV(integer feedback division) of
 * the PLL(PLL output = FBDIV * REFCLK frequency).
 * Default CPU_CLK_DIV : 1, IPU_CLK_DIV: 2
 * CLK = (REFCLK FREQ * FBDIV) / ((POSTDIV1 * POSTDIV2) * (CLK_DIV + 1))
 */
int mnh_cpu_freq_change(int index)
{
	int lock = 0;
	int ipu_div = 0;

	if (index < CPU_FREQ_MIN || index > CPU_FREQ_MAX)
		return -EINVAL;

	dev_dbg(mnh_dev->dev, "%s: %d\n", __func__, index);

	/* Unlock PLL access */
	HW_OUTf(mnh_dev->regs, SCU, PLL_PASSCODE, PASSCODE, PLL_UNLOCK);

	/* Switch to SYS200 mode */
	HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_CTL, CPU_IPU_SYS200_MODE, 0x1);

	/* Read current IPU clock source */
	mnh_dev->ipu_clk_src =
		HW_INf(mnh_dev->regs, SCU, CCU_CLK_CTL, IPU_CLK_SRC);

	/* Latch current settings going to PLL */
	HW_OUTf(mnh_dev->regs, SCU, CPU_IPU_PLL_CTRL, FRZ_PLL_IN, 1);

	/* Configure FBDIV first and set POSTDIV1, POSTDIV2
	* Compute dividers based on REF_FREQ_SEL hardware strap
	* Check FBDIV * REFCLK is witin VCO range (950-3800MHz)
	*/
	HW_OUTf(mnh_dev->regs, SCU, CPU_IPU_PLL_INTGR_DIV, FBDIV,
		mnh_dev->cpu_pllcfg[index].fbdiv);
	HW_OUTf(mnh_dev->regs, SCU, CPU_IPU_PLL_INTGR_DIV, POSTDIV1,
		mnh_dev->cpu_pllcfg[index].postdiv1);
	HW_OUTf(mnh_dev->regs, SCU, CPU_IPU_PLL_INTGR_DIV, POSTDIV2,
		mnh_dev->cpu_pllcfg[index].postdiv2);

	/* Set FOUTPOSTDIVPD = 1 to avoid glitches to output */
	HW_OUTf(mnh_dev->regs, SCU, CPU_IPU_PLL_CTRL, FOUTPOSTDIVPD, 1);

	/* Apply the updated PLL configurations  */
	HW_OUTf(mnh_dev->regs, SCU, CPU_IPU_PLL_CTRL, FRZ_PLL_IN, 0);

	/* Power up PLL */
	HW_OUTf(mnh_dev->regs, SCU, CPU_IPU_PLL_CTRL, PD, 0);

	/* Wait for minimum 128REFCLK, 6.7usec for 19.2MHz refclk
	* before checking PLL lock
	*/
	udelay(7);

	/* Check PLL is locked */
	do {
		lock = HW_INf(mnh_dev->regs,
			SCU, CPU_IPU_PLL_STS, LOCK);
	} while (lock != 1);

	/* Set FOUTPOSTDIVPD = 0 to ensure clk output is un-gated */
	HW_OUTf(mnh_dev->regs, SCU, CPU_IPU_PLL_CTRL, FOUTPOSTDIVPD, 0);

	/* Configure CPU_CLK_DIV */
	HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_DIV, CPU_CLK_DIV,
		mnh_dev->cpu_pllcfg[index].clk_div);

	/* If IPU clock is driven by CPU_IPU PLL,
	*  configure IPU divider based on CPU divider value
	*  to make sure IPU clock does not go over its limit
	*/
	if (mnh_dev->ipu_clk_src == CPU_IPU_PLL) {
		if (index > CPU_FREQ_800)
			ipu_div = IPU_DIV_BY_CPU(950,
				mnh_dev->cpu_pllcfg[index].clk_div);
		else
			ipu_div = IPU_DIV_BY_CPU(0,
				mnh_dev->cpu_pllcfg[index].clk_div);

		HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_DIV, IPU_CLK_DIV, ipu_div);
	}

	/* Go back to CPU_IPU PLL output */
	HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_CTL,
			CPU_IPU_SYS200_MODE, 0);

	mnh_dev->cpu_freq = index;
	if (mnh_dev->ipu_clk_src == CPU_IPU_PLL)
		mnh_dev->ipu_freq = index;

	/* Lock PLL access */
	HW_OUTf(mnh_dev->regs, SCU, PLL_PASSCODE, PASSCODE, 0);

	return 0;
}
EXPORT_SYMBOL(mnh_cpu_freq_change);

/**
 * IPU clock controller
 * @index: int with frquency table index info.
 * Return: 0 on success, an error code otherwise.
 *
 * Until IPU clock is configured, IPU clock is driven from PCIe or CPU_IPU PLL,
 * and once it is configured by driver, IPU PLL is used to control IPU clock.
 * To turn off IPU PLL, CPU frequency needs to be set to 200MHz to put
 * both CPU and IPU into SYS200 mode.
 */
int mnh_ipu_freq_change(int index)
{
	int lock = 0;

	if (index < IPU_FREQ_MIN || index > IPU_FREQ_MAX)
		return -EINVAL;

	dev_dbg(mnh_dev->dev, "%s: %d\n", __func__, index);

	/* Disable IPU_PLL clock output */
	HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_CTL, IPU_CLKEN, 0x0);

	/* Unlock PLL access */
	HW_OUTf(mnh_dev->regs, SCU, PLL_PASSCODE, PASSCODE, PLL_UNLOCK);

	/* Switch to stable clock before freq switch to avoid glitches */
	HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_CTL, IPU_CLK_SRC, CPU_IPU_PLL);

	/* Set FRZ_PLL_IN=1 to latch the current settings going to PLL */
	HW_OUTf(mnh_dev->regs, SCU, IPU_PLL_CTRL, FRZ_PLL_IN, 1);

	/* Configure FBDIV first and set POSTDIV1, POSTDIV2 */
	HW_OUTf(mnh_dev->regs, SCU, IPU_PLL_INTGR_DIV, FBDIV,
		mnh_dev->ipu_pllcfg[index].fbdiv);
	HW_OUTf(mnh_dev->regs, SCU, IPU_PLL_INTGR_DIV, POSTDIV1,
		mnh_dev->ipu_pllcfg[index].postdiv1);
	HW_OUTf(mnh_dev->regs, SCU, IPU_PLL_INTGR_DIV, POSTDIV2,
		mnh_dev->ipu_pllcfg[index].postdiv2);

	/* Set FOUTPOSTDIVPD = 1 to avoid glitches to output */
	HW_OUTf(mnh_dev->regs, SCU, IPU_PLL_CTRL, FOUTPOSTDIVPD, 1);

	/* Apply the updated PLL configurations  */
	HW_OUTf(mnh_dev->regs, SCU, IPU_PLL_CTRL, FRZ_PLL_IN, 0);

	/* Power up PLL */
	HW_OUTf(mnh_dev->regs, SCU, IPU_PLL_CTRL, PD, 0);

	/* Wait for minimum 128REFCLK, 6.7usec for 19.2MHz refclk
	* before checking PLL lock
	*/
	udelay(7);

	/* Check PLL is locked */
	do {
		lock = HW_INf(mnh_dev->regs, SCU, IPU_PLL_STS, LOCK);
	} while (lock != 1);

	/* Set FOUTPOSTDIVPD = 0 to ensure clk output is un-gated */
	HW_OUTf(mnh_dev->regs, SCU, IPU_PLL_CTRL, FOUTPOSTDIVPD, 0);

	/* Configure IPU_CLK_DIV */
	HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_DIV, IPU_CLK_DIV,
	mnh_dev->ipu_pllcfg[index].clk_div);

	/* Go back to IPU PLL output */
	HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_CTL, IPU_CLK_SRC, IPU_PLL);

	mnh_dev->ipu_freq = index;
	mnh_dev->ipu_clk_src =
		HW_INf(mnh_dev->regs, SCU, CCU_CLK_CTL, IPU_CLK_SRC);

	/* Lock PLL access */
	HW_OUTf(mnh_dev->regs, SCU, PLL_PASSCODE, PASSCODE, 0);

	/* Enable IPU_PLL clock output */
	HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_CTL, IPU_CLKEN, 0x1);

	return 0;
}
EXPORT_SYMBOL(mnh_ipu_freq_change);

/**
 * LPDDR clock control driver
 * @index: int with frquency table index info.
 * Return: 0 on success, an error code otherwise.
 *
 * LPDDR clock is controlled by LPC instead of using direct PLL configuration.
 * Precondition:
 * LPDDR refclk pll should be enabled at cold boot and resume
 * LPDDR FSPx registers should be configured at cold boot and resume
 */
int mnh_lpddr_freq_change(int index)
{
	int status = 0;

	dev_dbg(mnh_dev->dev, "%s: %d\n", __func__, index);

	if (index < LPDDR_FREQ_MIN || index > LPDDR_FREQ_MAX)
		return -EINVAL;

	/* Check the requested FSP is already in use */
	mnh_dev->ddr_freq = HW_INf(mnh_dev->regs, SCU, LPDDR4_LOW_POWER_STS,
		LPDDR4_CUR_FSP);
	if (mnh_dev->ddr_freq == index) {
		dev_info(mnh_dev->dev, "requested fsp%d is in use\n", index);
		return 0;
	}

	/* Disable LPC SW override */
	HW_OUTf(mnh_dev->regs, SCU, LPDDR4_LOW_POWER_CFG,
		LP4_FSP_SW_OVERRIDE, 0);

	/* Configure FSP index */
	HW_OUTf(mnh_dev->regs, SCU, LPDDR4_LOW_POWER_CFG,
		LPC_FREQ_CHG_COPY_NUM, index);

	/* Configure LPC cmd for frequency switch */
	HW_OUTf(mnh_dev->regs, SCU, LPDDR4_LOW_POWER_CFG,
		LPC_EXT_CMD, LP4_LPC_FREQ_SWITCH);

	/* Initiate LPC cmd to LPDDR controller */
	dev_info(mnh_dev->dev, "lpddr freq switching from fsp%d to fsp%d\n",
		mnh_dev->ddr_freq, index);
	HW_OUTf(mnh_dev->regs, SCU, LPDDR4_LOW_POWER_CFG, LPC_EXT_CMD_REQ, 1);

	/* Wait until LPC cmd process is done */
	do {
		status = HW_INf(mnh_dev->regs, SCU, LPDDR4_LOW_POWER_STS,
			LPC_CMD_DONE);
	} while (status != 1);

	/* Clear LPC cmd status */
	HW_OUTf(mnh_dev->regs, SCU, LPDDR4_LOW_POWER_STS, LPC_CMD_DONE, 1);

	/* Check LPC error status */
	if (HW_INf(mnh_dev->regs, SCU, LPDDR4_LOW_POWER_STS, LPC_CMD_RSP)
		== LPC_CMD_ERR) {
		/* Clear error status */
		HW_OUTf(mnh_dev->regs, SCU, LPDDR4_LOW_POWER_STS,
			LPC_CMD_RSP, 1);
		dev_err(mnh_dev->dev, "Failed to process lpc cmd:0x%x\n",
			LP4_LPC_FREQ_SWITCH);
		return -1;
	}

	/* Check FSPx switch status */
	if (HW_INf(mnh_dev->regs, SCU, LPDDR4_LOW_POWER_STS, LPDDR4_CUR_FSP)
		!= index) {
		dev_err(mnh_dev->dev, "Failed to switch to fsp%d\n", index);
		return -1;
	}

	mnh_dev->ddr_freq = index;

	return 0;
}
EXPORT_SYMBOL(mnh_lpddr_freq_change);

/**
 * LPDDR clock control driver
 * Return: 0 on success, an error code otherwise.
 *
 * LPDDR clock is derived from sys200 clk instead of separate lpddr clk
 */
int mnh_lpddr_sys200_mode(void)
{
	dev_dbg(mnh_dev->dev, "%s\n", __func__);
	/* Switch lpddr to SYS200 mode */
	HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_CTL, LP4_AXI_SYS200_MODE, 0x1);

	udelay(100);
	/* Unlock PLL access */
	HW_OUTf(mnh_dev->regs, SCU, PLL_PASSCODE, PASSCODE, PLL_UNLOCK);

	/* Power down LPDDR PLL */
	HW_OUTf(mnh_dev->regs, SCU, LPDDR4_REFCLK_PLL_CTRL, FRZ_PLL_IN, 1);
	HW_OUTf(mnh_dev->regs, SCU, LPDDR4_REFCLK_PLL_CTRL, PD, 1);
	HW_OUTf(mnh_dev->regs, SCU, LPDDR4_REFCLK_PLL_CTRL, FOUTPOSTDIVPD, 1);
	HW_OUTf(mnh_dev->regs, SCU, LPDDR4_REFCLK_PLL_CTRL, BYPASS, 1);
	HW_OUTf(mnh_dev->regs, SCU, LPDDR4_REFCLK_PLL_CTRL, FRZ_PLL_IN, 0);

	HW_OUTf(mnh_dev->regs, SCU, PLL_PASSCODE, PASSCODE, 0);

	return 0;
}
EXPORT_SYMBOL(mnh_lpddr_sys200_mode);

/**
 * CPU and IPU SYS200 clock control driver
 * Return: 0 on success, an error code otherwise.
 *
 * CPU and IPU clock is derived from sys200 clk instead of separate plls
 */
int mnh_cpu_ipu_sys200_mode(void)
{
	dev_dbg(mnh_dev->dev, "%s\n", __func__);

	/* Unlock PLL access */
	HW_OUTf(mnh_dev->regs, SCU, PLL_PASSCODE, PASSCODE, PLL_UNLOCK);

	/* Switch to SYS200 mode */
	HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_CTL, CPU_IPU_SYS200_MODE, 0x1);

	/* Read current IPU clock source */
	mnh_dev->ipu_clk_src =
		HW_INf(mnh_dev->regs, SCU, CCU_CLK_CTL, IPU_CLK_SRC);

	if (mnh_dev->ipu_clk_src == IPU_PLL) {
		/* Change clk source to CPU_IPU_PLL */
		HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_CTL,
			IPU_CLK_SRC, CPU_IPU_PLL);

		/* IPU: Latch current settings to go into PLL */
		HW_OUTf(mnh_dev->regs, SCU, IPU_PLL_CTRL, FRZ_PLL_IN, 1);

		/* IPU: Power down PLL */
		HW_OUTf(mnh_dev->regs, SCU, IPU_PLL_CTRL, PD, 1);
		HW_OUTf(mnh_dev->regs, SCU, IPU_PLL_CTRL,
			FOUTPOSTDIVPD, 1);

		/* IPU: Apply PLL configurations */
		HW_OUTf(mnh_dev->regs, SCU, IPU_PLL_CTRL, FRZ_PLL_IN, 0);
	}

	/* Configure IPU_CLK_DIV */
	HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_DIV, IPU_CLK_DIV,
	sys200_reg_tables[IPU_CLK].clk_div);

	/* Configure CPU_CLK_DIV */
	HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_DIV, CPU_CLK_DIV,
		sys200_reg_tables[CPU_CLK].clk_div);

	/* CPU_IPU: Latch current settings to go into PLL */
	HW_OUTf(mnh_dev->regs, SCU, CPU_IPU_PLL_CTRL, FRZ_PLL_IN, 1);

	/* CPU_IPU: Power down PLL */
	HW_OUTf(mnh_dev->regs, SCU, CPU_IPU_PLL_CTRL, PD, 1);
	HW_OUTf(mnh_dev->regs, SCU, CPU_IPU_PLL_CTRL, FOUTPOSTDIVPD, 1);

	/* CPU_IPU: Apply PLL configurations */
	HW_OUTf(mnh_dev->regs, SCU, CPU_IPU_PLL_CTRL, FRZ_PLL_IN, 0);

	mnh_dev->cpu_freq = 0;
	mnh_dev->ipu_freq = 0;

	/* Lock PLL access */
	HW_OUTf(mnh_dev->regs, SCU, PLL_PASSCODE, PASSCODE, 0);

	return 0;
}
EXPORT_SYMBOL(mnh_cpu_ipu_sys200_mode);

/* read entire int_status */
u64 mnh_ddr_int_status(void)
{
	u64 int_stat = ((u64)MNH_DDR_CTL_IN(228) << 32) | MNH_DDR_CTL_IN(227);
	return int_stat;
}
EXPORT_SYMBOL(mnh_ddr_int_status);

/* clear entire int_status */
int mnh_ddr_clr_int_status(void)
{
	u64 stat = 0;
	MNH_DDR_CTL_OUT(230, 0x0F);
	MNH_DDR_CTL_OUT(229, 0xFFFFFFFF);
	stat = mnh_ddr_int_status();
	if (stat) {
		pr_err("%s: int stat not all clear: %llx\n",
			__func__, stat);
		return -1;
	}
	return 0;
}
EXPORT_SYMBOL(mnh_ddr_clr_int_status);

/* read single bit in int_status */
static u32 mnh_ddr_int_status_bit(u8 sbit)
{
	u32 status = 0;
	u32 upper = 0;
	const u32 max_int_status_bit = 35;
	const u32 first_upper_bit = 32;
	if (sbit > max_int_status_bit)
		return -EINVAL;

	/*
	docs refer to int status by bit numbers 0-35,
	but we're only reading 32 bits at a time.
	*/
	upper = (sbit >= first_upper_bit) ? 1 : 0;
	sbit -= (upper) ? first_upper_bit : 0;

	status = (upper) ? MNH_DDR_CTL_IN(228) : MNH_DDR_CTL_IN(227);
	status &= (1 << sbit);
	return status;
}

/* clear single bit in int_status */
static int mnh_ddr_clr_int_status_bit(u8 sbit)
{
	const u32 max_int_status_bit = 35;
	const u32 first_upper_bit = 32;
	if (sbit > max_int_status_bit)
		return -EINVAL;

	if (sbit >= first_upper_bit)
		MNH_DDR_CTL_OUT(230, 1 << (sbit - first_upper_bit));
	else
		MNH_DDR_CTL_OUT(229, 1 << sbit);

	if (mnh_ddr_int_status_bit(sbit)) {
		pr_err("%s: bit %d is still set.\n",
			__func__, sbit);
		return -1;
	}
	return 0;
}
static int mnh_ddr_send_lp_cmd(u8 cmd)
{
	u32 timeout = 100000;
	pr_debug("%s sending cmd: 0x%x\n", __func__, cmd);
	MNH_DDR_CTL_OUTf(112, LP_CMD, cmd);

	while (!mnh_ddr_int_status_bit(LP_CMD_SBIT) && --timeout)
		udelay(1);

	if (mnh_ddr_int_status_bit(LP_CMD_SBIT)) {
		return mnh_ddr_clr_int_status_bit(LP_CMD_SBIT);
	} else {
		return -ETIMEDOUT;
	}
}

static void mnh_ddr_enable_lp(void)
{
	dev_info(mnh_dev->dev, "%s\n", __func__);
	MNH_DDR_CTL_OUTf(124, LP_AUTO_SR_MC_GATE_IDLE, 0xFF);
	MNH_DDR_CTL_OUTf(122, LP_AUTO_MEM_GATE_EN, 0x4);
	MNH_DDR_CTL_OUTf(122, LP_AUTO_ENTRY_EN, 0x4);
	MNH_DDR_CTL_OUTf(122, LP_AUTO_EXIT_EN, 0xF);
}

static void mnh_ddr_disable_lp(void)
{
	dev_info(mnh_dev->dev, "%s\n", __func__);
	MNH_DDR_CTL_OUTf(124, LP_AUTO_SR_MC_GATE_IDLE, 0x00);
	MNH_DDR_CTL_OUTf(122, LP_AUTO_MEM_GATE_EN, 0x0);
	MNH_DDR_CTL_OUTf(122, LP_AUTO_ENTRY_EN, 0x0);
	MNH_DDR_CTL_OUTf(122, LP_AUTO_EXIT_EN, 0x0);
	mnh_ddr_send_lp_cmd(LP_CMD_EXIT_LP);
}

/**
 * Set the SCU clock gating mode
 * Return: 0 on success, an error code otherwise.
 *
 * enable == 1 sets the SCU to enable clock gating in general when CPU enters
 * L2 WFI state.
 */
int mnh_clock_gating_mode(int enabled)
{
	dev_dbg(mnh_dev->dev, "%s\n", __func__);

	if (enabled != 1 && enabled != 0)
		return -EINVAL;

	/* Add periph clk gates */
	HW_OUTf(mnh_dev->regs, SCU, RSTC, TIMER_RST, enabled);
	HW_OUTf(mnh_dev->regs, SCU, RSTC, PERI_DMA_RST, enabled);
	HW_OUTf(mnh_dev->regs, SCU, RSTC, IPU_RST, enabled);
	HW_OUTf(mnh_dev->regs, SCU, PERIPH_CLK_CTRL, PVT_CLKEN,
		!enabled);
	HW_OUTf(mnh_dev->regs, SCU, PERIPH_CLK_CTRL, PERI_DMA_CLKEN_SW,
		!enabled);
	HW_OUTf(mnh_dev->regs, SCU, PERIPH_CLK_CTRL, TIMER_CLKEN_SW,
		!enabled);

	HW_OUTf(mnh_dev->regs, SCU, MEM_PWR_MGMNT, HALT_CPUMEM_PD_EN, enabled);
	HW_OUTf(mnh_dev->regs, SCU, MEM_PWR_MGMNT, HALT_LP4CMEM_PD_EN, enabled);
	HW_OUTf(mnh_dev->regs, SCU, MEM_PWR_MGMNT, HALT_BTROM_PD_EN, enabled);
	HW_OUTf(mnh_dev->regs, SCU, MEM_PWR_MGMNT, HALT_BTSRAM_PD_EN, enabled);
	HW_OUTf(mnh_dev->regs, SCU, MEM_PWR_MGMNT, BTROM_SLP, enabled);
	HW_OUTf(mnh_dev->regs, SCU, MEM_PWR_MGMNT, IPU_MEM_DS, enabled);

	HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_CTL, HALT_AXICG_EN, enabled);
	HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_CTL, HALT_CPUCG_EN, enabled);
	HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_CTL, HALT_BTSRAMCG_EN, enabled);
	HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_CTL, HALT_BTROMCG_EN, enabled);
	/* HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_CTL, HALT_LP4CG_EN, 1); */
	HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_CTL, HALT_LP4_PLL_BYPCLK_CG_EN,
		enabled);
	HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_CTL, LP4PHY_PLL_BYPASS_CLKEN,
		enabled);
	return 0;
}
EXPORT_SYMBOL(mnh_clock_gating_mode);


/* Frequency calculation by PLL configuration
 * @cfg: struct freq_reg_table with pll config information.
 * Return: freq MHz.
 *
 * This returns current frequency in MHz unit
 * freq = (refclk*fbdiv)/((postdiv1*postdiv2)*(clk_div+1))
 */
static int mnh_freq_get_by_pll(struct freq_reg_table cfg, int sys200)
{
	uint32_t freq_khz;

	if (sys200)
		freq_khz = (SYS200_CLK_KHZ)/(cfg.clk_div+1);
	else
		freq_khz = (refclk_khz_tables[mnh_dev->refclk]*cfg.fbdiv)/
			((cfg.postdiv1*cfg.postdiv2)*(cfg.clk_div+1));

	return (freq_khz/1000);
}

/* Current reference clock rate
 * Return: refclk index of mnh_refclk_type
 *
 * This returns current reference clk rate by index of mnh_refclk_type
 */
static int mnh_freq_check_refclk(uint32_t refclk_gpio)
{
	int ret, refclk = 0;

	if (gpio_is_valid(refclk_gpio)) {
		ret = gpio_request(refclk_gpio, "REFCLK");
		if (ret)
			goto end_gpio;
		ret = gpio_direction_input(refclk_gpio);
		if (ret)
			goto err_gpio;
		if (gpio_get_value(refclk_gpio))
			refclk = refclk | 0x1;

		pr_info("%s gpio:%d refclk:%d\n", __func__,
			refclk_gpio, refclk);
	}
err_gpio:
	if (gpio_is_valid(refclk_gpio))
		gpio_free(refclk_gpio);
end_gpio:
	return refclk;
}

static ssize_t cpu_freq_get(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct freq_reg_table pll_cfg;
	int sys200;

	/* Check CPU is in SYS200 mode */
	sys200 = HW_INf(mnh_dev->regs, SCU, CCU_CLK_CTL, CPU_IPU_SYS200_MODE);

	/* Calculate frequency by PLL configuration */
	pll_cfg.fbdiv =
	HW_INf(mnh_dev->regs, SCU, CPU_IPU_PLL_INTGR_DIV, FBDIV);
	pll_cfg.postdiv1 =
	HW_INf(mnh_dev->regs, SCU, CPU_IPU_PLL_INTGR_DIV, POSTDIV1);
	pll_cfg.postdiv2 =
	HW_INf(mnh_dev->regs, SCU, CPU_IPU_PLL_INTGR_DIV, POSTDIV2);
	pll_cfg.clk_div =
	HW_INf(mnh_dev->regs, SCU, CCU_CLK_DIV, CPU_CLK_DIV);

	return sprintf(buf, "%dMHz\n", mnh_freq_get_by_pll(pll_cfg, sys200));
}

static ssize_t cpu_freq_set(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf,
				  size_t count)
{
	int i, err;

	dev_dbg(mnh_dev->dev, "%s: %s\n", __func__, buf);
	for (i = 0; i < ARRAY_SIZE(cpu_reg_tables[0]); i++) {
		if (!strncmp(buf, mnh_dev->cpu_pllcfg[i].freq_str,
			strlen(mnh_dev->cpu_pllcfg[i].freq_str))) {
			err = mnh_cpu_freq_change(i);
			if (!err)
				return count;
			else
				return -EIO;
		}
	}

	dev_err(mnh_dev->dev, "invalid freq: %s\n", buf);
	return -EINVAL;

}

static const char * const ipu_freq_str[] = {"100", "200", "300", "400", "425"};
static ssize_t ipu_freq_get(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct freq_reg_table pll_cfg;
	int sys200, clk_src;

	/* Check IPU is in SYS200 mode */
	clk_src = HW_INf(mnh_dev->regs, SCU, CCU_CLK_CTL, IPU_CLK_SRC);
	sys200 = HW_INf(mnh_dev->regs, SCU, CCU_CLK_CTL, CPU_IPU_SYS200_MODE);
	if (sys200 && (clk_src == IPU_PLL))
		sys200 = 0;

	/* Calculate frequency by PLL configuration */
	if (clk_src == CPU_IPU_PLL) {
		pll_cfg.fbdiv = HW_INf(mnh_dev->regs, SCU,
				CPU_IPU_PLL_INTGR_DIV, FBDIV);
		pll_cfg.postdiv1 = HW_INf(mnh_dev->regs, SCU,
				CPU_IPU_PLL_INTGR_DIV, POSTDIV1);
		pll_cfg.postdiv2 = HW_INf(mnh_dev->regs, SCU,
				CPU_IPU_PLL_INTGR_DIV, POSTDIV2);
	} else {
		pll_cfg.fbdiv = HW_INf(mnh_dev->regs, SCU,
				IPU_PLL_INTGR_DIV, FBDIV);
		pll_cfg.postdiv1 = HW_INf(mnh_dev->regs, SCU,
				IPU_PLL_INTGR_DIV, POSTDIV1);
		pll_cfg.postdiv2 = HW_INf(mnh_dev->regs, SCU,
				IPU_PLL_INTGR_DIV, POSTDIV2);
	}
	pll_cfg.clk_div = HW_INf(mnh_dev->regs, SCU, CCU_CLK_DIV, IPU_CLK_DIV);

	return sprintf(buf, "%dMHz\n", mnh_freq_get_by_pll(pll_cfg, sys200));
}

static ssize_t ipu_freq_set(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf,
				  size_t count)
{
	int i, err;

	dev_dbg(mnh_dev->dev, "%s: %s\n", __func__, buf);
	for (i = 0; i < ARRAY_SIZE(ipu_reg_tables[0]); i++) {
		if (!strncmp(buf, mnh_dev->ipu_pllcfg[i].freq_str,
			strlen(mnh_dev->ipu_pllcfg[i].freq_str))) {
			err = mnh_ipu_freq_change(i);
			if (!err)
				return count;
			else
				return -EIO;
		}
	}

	dev_err(mnh_dev->dev, "invalid freq: %s\n", buf);
	return -EINVAL;
}

static ssize_t lpddr_freq_get(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	uint32_t var = HW_INf(mnh_dev->regs, SCU, LPDDR4_LOW_POWER_STS,
				LPDDR4_CUR_FSP);

	dev_dbg(mnh_dev->dev, "%s: %d\n", __func__, var);
	return sprintf(buf, "FSP%d\n", var);
}

static ssize_t lpddr_freq_set(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf,
				  size_t count)
{
	int var = 0;
	int ret;

	ret = kstrtoint(buf, 10, &var);
	if (ret < 0)
		return ret;

	dev_dbg(mnh_dev->dev, "%s: %d\n", __func__, var);
	if (!mnh_lpddr_freq_change(var))
		return count;
	else
		return -EIO;
}

static ssize_t ipu_clk_src_get(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	mnh_dev->ipu_clk_src =
		HW_INf(mnh_dev->regs, SCU, CCU_CLK_CTL, IPU_CLK_SRC);

	return scnprintf(buf, MAX_STR_COPY, "%s\n",
		(mnh_dev->ipu_clk_src == CPU_IPU_PLL) ? "CPU_IPU":"IPU");
}

static ssize_t sys200_freq_get(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	int sys200, clk_src;

	clk_src = HW_INf(mnh_dev->regs, SCU, CCU_CLK_CTL, IPU_CLK_SRC);
	sys200 = HW_INf(mnh_dev->regs, SCU, CCU_CLK_CTL, CPU_IPU_SYS200_MODE);
	if (sys200 && (clk_src == IPU_PLL))
		sys200 = 0;

	return sprintf(buf, "%d\n", sys200);
}

static ssize_t sys200_freq_set(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf,
				  size_t count)
{
	int var = 0;
	int ret;

	ret = kstrtoint(buf, 10, &var);
	if (ret < 0)
		return ret;

	if (var == 1) {
		dev_dbg(mnh_dev->dev, "%s: %d\n", __func__, var);
		if (!mnh_cpu_ipu_sys200_mode())
			return count;
	}
	return -EIO;
}

static ssize_t clock_gating_get(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	int clk_gated;

	clk_gated = HW_INf(mnh_dev->regs, SCU, CCU_CLK_CTL, HALT_CPUCG_EN);

	return sprintf(buf, "%d\n", clk_gated);
}

static ssize_t clock_gating_set(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf,
				  size_t count)
{
	int var = 0;
	int ret;

	ret = kstrtoint(buf, 10, &var);
	if (ret < 0)
		return ret;

	if (var == 1 || var == 0) {
		dev_dbg(mnh_dev->dev, "%s: %d\n", __func__, var);
		if (!mnh_clock_gating_mode(var))
			return count;
	}
	return -EIO;
}

static ssize_t lpddr_lp_get(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	return sprintf(buf, "%d\n",
		MNH_DDR_CTL_INf(122, LP_AUTO_ENTRY_EN));
}

static ssize_t lpddr_lp_set(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf,
				  size_t count)
{
	int var = 0;
	int ret;

	ret = kstrtoint(buf, 10, &var);
	if (ret < 0)
		return ret;

	dev_dbg(mnh_dev->dev, "%s: %d\n", __func__, var);
	if (var == 1)
		mnh_ddr_enable_lp();
	else if (var == 0)
		mnh_ddr_disable_lp();
	else
		return -EIO;

	return count;
}

static ssize_t lpddr_sys200_get(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	return sprintf(buf, "%d\n",
		HW_INf(mnh_dev->regs, SCU, CCU_CLK_CTL, LP4_AXI_SYS200_MODE));
}

static ssize_t lpddr_sys200_set(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf,
				  size_t count)
{
	int var = 0;
	int ret;

	ret = kstrtoint(buf, 10, &var);
	if (ret < 0)
		return ret;

	dev_dbg(mnh_dev->dev, "%s: %d\n", __func__, var);
	if (var == 1)
		mnh_lpddr_sys200_mode();
	else if (var == 0)
		mnh_lpddr_freq_change(0);
	else {

		dev_err(mnh_dev->dev, "Cannot disable sys200 mode. Use lpddr_freq.");
		return -EIO;
	}

	return count;
}

static DEVICE_ATTR(cpu_freq, S_IWUSR | S_IRUGO,
		cpu_freq_get, cpu_freq_set);
static DEVICE_ATTR(ipu_freq, S_IWUSR | S_IRUGO,
		ipu_freq_get, ipu_freq_set);
static DEVICE_ATTR(lpddr_freq, S_IWUSR | S_IRUGO,
		lpddr_freq_get, lpddr_freq_set);
static DEVICE_ATTR(ipu_clk_src, S_IRUGO,
		ipu_clk_src_get, NULL);
static DEVICE_ATTR(sys200, S_IWUSR | S_IRUGO,
		sys200_freq_get, sys200_freq_set);
static DEVICE_ATTR(clock_gating, S_IWUSR | S_IRUGO,
		clock_gating_get, clock_gating_set);
static DEVICE_ATTR(lpddr_lp, S_IWUSR | S_IRUGO,
		lpddr_lp_get, lpddr_lp_set);
static DEVICE_ATTR(lpddr_sys200, S_IWUSR | S_IRUGO,
		lpddr_sys200_get, lpddr_sys200_set);


static struct attribute *freq_dev_attributes[] = {
	&dev_attr_cpu_freq.attr,
	&dev_attr_ipu_freq.attr,
	&dev_attr_lpddr_freq.attr,
	&dev_attr_ipu_clk_src.attr,
	&dev_attr_sys200.attr,
	&dev_attr_clock_gating.attr,
	&dev_attr_lpddr_lp.attr,
	&dev_attr_lpddr_sys200.attr,
	NULL
};

static struct attribute_group mnh_freq_cooling_group = {
	.name = "mnh_freq_cool",
	.attrs = freq_dev_attributes
};



static int init_sysfs(struct device *dev, struct kobject *sysfs_kobj)
{
	int ret;

	ret = sysfs_create_group(sysfs_kobj, &mnh_freq_cooling_group);
	if (ret) {
		dev_err(dev, "Failed to create sysfs\n");
		return -EINVAL;
	}

	return 0;
}

static void clean_sysfs(void)
{
	sysfs_remove_group(kernel_kobj, &mnh_freq_cooling_group);
}

/**
 * Initial handler for ddr interrupts
 */
static irqreturn_t mnh_pm_handle_ddr_irq(int irq, void *dev_id)
{
	//spin_lock(&mnh_dev->irqlock);
	u64 status = mnh_ddr_int_status();
	/* Clear the bits */
	mnh_ddr_clr_int_status();

	//complete(&mnh_dev->ddrclk_complete);

	//spin_unlock(&mnh_dev->irqlock);

	dev_dbg(mnh_dev->dev, "%s status=0x%llx\n", __func__, status);
	/* return interrupt handled */
	return IRQ_HANDLED;
}
int mnh_clk_init(struct platform_device *pdev, void __iomem *baseadress)
{
	int ret = 0, err = 0;
	struct resource *res;
	uint32_t refclk_gpio;

	dev_info(&pdev->dev, "mnh_freq_cooling_init\n");

	mnh_dev = devm_kzalloc(&pdev->dev, sizeof(*mnh_dev),
			GFP_KERNEL);
	if (!mnh_dev)
		return -ENOMEM;

	/* Set baseadress for SCU */
	mnh_dev->regs = baseadress;
	mnh_dev->dev = &pdev->dev;
	// init_completion(&mnh_dev->ddrclk_complete);
	// spin_lock_init(&mnh_dev->irqlock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res) {
		dev_err(mnh_dev->dev, "cannot get platform resources\n");
		ret = -ENOENT;
		goto mnh_probe_err;
	}
	mnh_dev->ddraddr = ioremap_nocache(res->start, resource_size(res));
	if (!mnh_dev->ddraddr) {
		dev_err(mnh_dev->dev, "unable to remap resources\n");
		ret = -ENOMEM;
		goto mnh_probe_err;
	}

	mnh_dev->ddr_irq = platform_get_irq(pdev, 0);
	dev_dbg(mnh_dev->dev, "Allocate ddr irq %d\n", mnh_dev->ddr_irq);
	err = request_irq(mnh_dev->ddr_irq, mnh_pm_handle_ddr_irq,
	       IRQF_SHARED, DEVICE_NAME, mnh_dev->dev);
	if (err) {
		dev_err(mnh_dev->dev, "Could not allocated ddr irq\n");
		ret = -EINVAL;
		goto mnh_probe_err;
	}

	/* Check IPU_CLK src */
	mnh_dev->ipu_clk_src =
		HW_INf(mnh_dev->regs, SCU, CCU_CLK_CTL, IPU_CLK_SRC);

	/* Check refclk rate */
	err = device_property_read_u32(mnh_dev->dev, "refclk-gpio",
		&refclk_gpio);
	if (!err)
		mnh_dev->refclk = mnh_freq_check_refclk(refclk_gpio);
	else {
		pr_err("unable to read refclk-gpio\n");
		return -ENOMEM;
	}
	mnh_dev->cpu_pllcfg = &cpu_reg_tables[mnh_dev->refclk];
	mnh_dev->ipu_pllcfg = &ipu_reg_tables[mnh_dev->refclk];

	init_sysfs(mnh_dev->dev, kernel_kobj);

	mnh_clock_gating_mode(0);
	return 0;

mnh_probe_err:
	if (mnh_dev->ddraddr)
		iounmap(&mnh_dev->ddraddr);
	return ret;
}

void mnh_clk_clean(struct device *dev)
{
	clean_sysfs();
}
