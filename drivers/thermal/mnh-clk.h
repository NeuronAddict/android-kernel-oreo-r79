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

#ifndef __MNH_CLK
#define __MNH_CLK

#include <linux/platform_device.h>

enum mnh_cpu_freq_type {
	CPU_FREQ_MIN = 0,
	CPU_FREQ_200 = CPU_FREQ_MIN,
	CPU_FREQ_400,
	CPU_FREQ_600,
	CPU_FREQ_800,
	CPU_FREQ_950,
	CPU_FREQ_MAX = CPU_FREQ_950
};

enum mnh_ipu_freq_type {
	IPU_FREQ_MIN = 0,
	IPU_FREQ_100 = IPU_FREQ_MIN,
	IPU_FREQ_200,
	IPU_FREQ_300,
	IPU_FREQ_400,
	IPU_FREQ_425,
	IPU_FREQ_MAX = IPU_FREQ_425
};

enum mnh_lpddr_freq_type {
	LPDDR_FREQ_MIN = 0,
	LPDDR_FREQ_FSP0 = LPDDR_FREQ_MIN,
	LPDDR_FREQ_FSP1,
	LPDDR_FREQ_FSP2,
	LPDDR_FREQ_FSP3,
	LPDDR_FREQ_MAX = LPDDR_FREQ_FSP3,
	LPDDR_FREQ_NUM_FSPS,
	LPDDR_FREQ_SW = 7
};

enum mnh_clock_gating_mode {
	CLOCK_GATING_DISABLED = 0,
	CLOCK_GATING_ENABLED = 1
};


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
int mnh_cpu_freq_change(int index);

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
int mnh_ipu_freq_change(int index);

/**
 * CPU and IPU SYS200 clock control driver
 * Return: 0 on success, an error code otherwise.
 *
 * CPU and IPU clock is derived from sys200 clk instead of separate plls
 */
int mnh_cpu_ipu_sys200_mode(void);

/**
 * LPDDR clock control driver
 * Return: 0 on success, an error code otherwise.
 *
 * LPDDR clock is derived from sys200 clk instead of separate lpddr clk
 */
int mnh_lpddr_sys200_mode(bool enable);

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
int mnh_lpddr_freq_change(int index);


/**
 * @enabled: 0 or 1, to enable clock gating via SCU
 * Return: 0 on success, an error code otherwise.
 *
 */
int mnh_clock_init_gating(int enabled);


/**
 * Setup interface for controlling clocks
 * @dev: device structure.
 * Return 0 on success, errcode on failures.
 */
int mnh_clk_init(struct platform_device *pdev, void __iomem *baseadress);


/**
 * @enabled: 0 or 1, to enable clock gating via SCU
 * Return: 0 on success, an error code otherwise.
 *
 */
int mnh_bypass_clock_gating(int enabled);

/**
 * @enabled: 0 or 1, to enable pcie axi clock
 * Return: 0 on success, an error code otherwise.
 *
 */
int mnh_pcie_axi_clock_enable(int enabled);

/**
 * @enabled: 0 or 1, to enable clock gating via SCU
 * Return: 0 on success, an error code otherwise.
 *
 */
int mnh_axi_clock_gating(int enabled);


/**
 * Clean up the interface
 */
void mnh_clk_clean(struct device *dev);

int mnh_ddr_adjust_refresh_suspend(void);
int mnh_ddr_adjust_refresh_resume(void);

#endif /* __MNH_CLK */
