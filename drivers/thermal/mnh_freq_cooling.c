/*
 * Copyright (c) 2016, Intel Corporation. All rights reserved.
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
 *
 * Intel mnh clk driver.
 *
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>
#include <linux/intel-hwio.h>
#include <soc/mnh/mnh-hwio-scu.h>


#define DEVICE_NAME "mnh_freq_cooling"
#define PLL_UNLOCK 0x4CD9
#define REF_FREQ_SEL 0x8
#define MAX_STR_COPY 9
#define LP4_LPC_FREQ_SWITCH 0x8A

/* If IPU clock is driven by CPU_IPU PLL
 * calculate IPU divider based on CPU clk and divider
 * CPU CLK < 850: IPU_CLK_DIV = ((CPU_CLK_DIV+1)*2-1)
 * CPU CLK > 850: IPU_CLK_DIV = ((CPU_CLK_DIV+1)*2)
 */
#define IPU_DIV_BY_CPU(freq, div) \
	((freq < 850) ? ((div+1)*2-1):((div+1)*2))

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
	LPDDR_FREQ_SW = 7
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
	int fidiv;
	int postdiv1;
	int postdiv2;
	int clk_div;
};

/* CPU clock frequency calculation tables
 * Refclk  FBDIV VCO	POSTDIV1 POSTDIV2 FOUTPOSTDIV CLKDIV CLKFreq
 * SYS200                                                        200.000
 * 19.2	 125	 2400	 6	  1	   400.000	0	 400.000
 * 19.2	 125	 2400	 4	  1	   600.000	0	 600.000
 * 19.2	 125	 2400	 3	  1	   800.000	0	 800.000
 * 19.2	 99	 1900.8  2	  1	   950.400	0	 950.400
 */
static struct freq_reg_table cpu_reg_tables[] = {
	{0, 0, 0, 0},	/* 200 MHz in SYS200 */
	{125, 6, 1, 0},	/* 400 MHz */
	{125, 4, 1, 0},	/* 600 MHz */
	{125, 3, 1, 0},	/* 800 MHz */
	{99, 2, 1, 0}	/* 950 MHz */
};

/* IPU clock frequency calculation tables
 * Refclk  FBDIV  VCO	POSTDIV1 POSTDIV2 FOUTPOSTDIV CLKDIV CLKFreq
 * SYS200                                                        100.000
 * 19.2	 125	 2400	  6	    1	  400.000	1	 200.000
 * 19.2	 125	 2400	  4	    1	  600.000	1	 300.000
 * 19.2	 125	 2400	  6	    1	  400.000	0	 400.000
 * 19.2	 133	 2553.6   6	    1	  425.600	0	 425.600
 */
static struct freq_reg_table ipu_reg_tables[] = {
	{0, 0, 0, 0},	/* 100 MHz in SYS200 */
	{125, 6, 1, 1},	/* 200 MHz */
	{125, 4, 1, 1},	/* 300 MHz */
	{125, 6, 1, 0},	/* 400 MHz */
	{133, 6, 1, 0}	/* 425 MHz */
};


struct mnh_freq_cooling_device {
	struct device *dev;
	void __iomem *regs;
	enum mnh_ipu_clk_src ipu_clk_src;
	enum mnh_cpu_freq_type cpu_freq;
	enum mnh_ipu_freq_type ipu_freq;
	enum mnh_lpddr_freq_type ddr_freq;
};

static struct mnh_freq_cooling_device *mnh_dev;

/* CPU clock 200-1000MHZ (IPU clock 100-500MHz)
 * 1PLL(CPU_IPU PLL) for CPU/IPU clocks. Since CPU and IPU clocks are derived
 * from same PLL in this mode, there would be restriction on achedivable clock
 * frequencies for CPU and IPU clocks. IPU clock would be half of CPU clock. Any
 * frequency changes is achieved by changing FBDIV(integer feedback division) of
 * the PLL(PLL output = FBDIV * REFCLK frequency).
 * Default CPU_CLK_DIV : 1, IPU_CLK_DIV: 2
 * CLK = (REFCLK FREQ * FBDIV) / [ (POSTDIV1 * POSTDIV2) * [?_CLK_DIV[3:0] + 1])
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

	if (index == CPU_FREQ_200) {
		/* Power down CPU_IPU PLL */
		HW_OUTf(mnh_dev->regs, SCU, CPU_IPU_PLL_CTRL, PD, 1);
		HW_OUTf(mnh_dev->regs, SCU, CPU_IPU_PLL_CTRL, FOUTPOSTDIVPD, 1);

		goto pll_acc_lock;
	}

	/* Latch current settings going to PLL */
	HW_OUTf(mnh_dev->regs, SCU, CPU_IPU_PLL_CTRL, FRZ_PLL_IN, 1);

	/* Configure FBDIV first and set POSTDIV1, POSTDIV2
	* Compute dividers based on REF_FREQ_SEL hardware strap
	* Check FBDIV * REFCLK is witin VCO range (950-3800MHz)
	*/
	HW_OUTf(mnh_dev->regs, SCU, CPU_IPU_PLL_INTGR_DIV, FBDIV,
		cpu_reg_tables[index].fidiv);
	HW_OUTf(mnh_dev->regs, SCU, CPU_IPU_PLL_INTGR_DIV, POSTDIV1,
			cpu_reg_tables[index].postdiv1);
	HW_OUTf(mnh_dev->regs, SCU, CPU_IPU_PLL_INTGR_DIV, POSTDIV2,
		cpu_reg_tables[index].postdiv2);

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
		cpu_reg_tables[index].clk_div);

	/* If IPU clock is driven by CPU_IPU PLL,
	*  configure IPU divider based on CPU divider value
	*  to make sure IPU clock does not go over its limit
	*/
	if (mnh_dev->ipu_clk_src == CPU_IPU_PLL) {
		if (index > CPU_FREQ_800)
			ipu_div = IPU_DIV_BY_CPU(950,
				cpu_reg_tables[index].clk_div);
		else
			ipu_div = IPU_DIV_BY_CPU(0,
				cpu_reg_tables[index].clk_div);

		HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_DIV, IPU_CLK_DIV, ipu_div);
	}

	/* Go back to CPU_IPU PLL output */
	HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_CTL,
			CPU_IPU_SYS200_MODE, 0);

pll_acc_lock:
	mnh_dev->cpu_freq = index;
	if (mnh_dev->ipu_clk_src == CPU_IPU_PLL)
		mnh_dev->ipu_freq = index;

	/* Lock PLL access */
	HW_OUTf(mnh_dev->regs, SCU, PLL_PASSCODE, PASSCODE, 0);

	return 0;
}
EXPORT_SYMBOL(mnh_cpu_freq_change);

/* IPU clock 100-452 MHZ
 * 1PLL(IPU PLL) to be used only if IPU clock and CPU clocks are to be frequency
 * decoupled with no restriction of one over the other. By default IPU clock
 * is derived from CPU_IPU PLL
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

	if (index == IPU_FREQ_100) {
		HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_CTL,
			IPU_CLK_SRC, CPU_IPU_PLL);

		/* Power down PLL */
		HW_OUTf(mnh_dev->regs, SCU, IPU_PLL_CTRL, PD, 1);
		HW_OUTf(mnh_dev->regs, SCU, IPU_PLL_CTRL, FOUTPOSTDIVPD, 1);
		goto pll_acc_lock;
	}

	/* Switch to stable clock before freq switch to avoid glitches */
	HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_CTL, IPU_CLK_SRC, CPU_IPU_PLL);

	/* Set FRZ_PLL_IN=1 to latch the current settings going to PLL */
	HW_OUTf(mnh_dev->regs, SCU, IPU_PLL_CTRL, FRZ_PLL_IN, 1);

	/* Configure FBDIV first and set POSTDIV1, POSTDIV2 */
	HW_OUTf(mnh_dev->regs, SCU, IPU_PLL_INTGR_DIV, FBDIV,
		ipu_reg_tables[index].fidiv);
	HW_OUTf(mnh_dev->regs, SCU, IPU_PLL_INTGR_DIV, POSTDIV1,
		ipu_reg_tables[index].postdiv1);
	HW_OUTf(mnh_dev->regs, SCU, IPU_PLL_INTGR_DIV, POSTDIV2,
		ipu_reg_tables[index].postdiv2);

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
	ipu_reg_tables[index].clk_div);

	/* Go back to IPU PLL output */
	HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_CTL, IPU_CLK_SRC, IPU_PLL);

pll_acc_lock:
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

/* LPDDR clock 200/300/400/500/600MHz
 * 1PLL(LPDDR4 RefClk PLL) for LPDDR4 600MHz or lower clock
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


static const char * const cpu_freq_str[] = {"200", "400", "600", "800", "950"};
static ssize_t cpu_freq_get(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	if (mnh_dev->cpu_freq > CPU_FREQ_MAX ||
	    mnh_dev->cpu_freq < CPU_FREQ_MIN)
		return -EINVAL;

	dev_dbg(mnh_dev->dev, "%s: %d\n", __func__, mnh_dev->cpu_freq);
	return scnprintf(buf, MAX_STR_COPY, "%sMHz\n",
		cpu_freq_str[mnh_dev->cpu_freq]);
}

static ssize_t cpu_freq_set(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf,
				  size_t count)
{
	int i, err;

	dev_dbg(mnh_dev->dev, "%s: %s\n", __func__, buf);
	for (i = 0; i < ARRAY_SIZE(cpu_freq_str); i++) {
		if (!strncmp(buf, cpu_freq_str[i], strlen(cpu_freq_str[i]))) {
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
	if (mnh_dev->ipu_freq > IPU_FREQ_MAX ||
	    mnh_dev->ipu_freq < IPU_FREQ_MIN)
		return -EINVAL;

	dev_dbg(mnh_dev->dev, "%s: %d\n", __func__, mnh_dev->ipu_freq);
	return scnprintf(buf, MAX_STR_COPY, "%sMHz\n",
		ipu_freq_str[mnh_dev->ipu_freq]);
}

static ssize_t ipu_freq_set(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf,
				  size_t count)
{
	int i, err;

	dev_dbg(mnh_dev->dev, "%s: %s\n", __func__, buf);
	for (i = 0; i < ARRAY_SIZE(ipu_freq_str); i++) {
		if (!strncmp(buf, ipu_freq_str[i], strlen(ipu_freq_str[i]))) {
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
	unsigned long var = 0;
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

static DEVICE_ATTR(cpu_freq, S_IWUSR | S_IRUGO,
		cpu_freq_get, cpu_freq_set);
static DEVICE_ATTR(ipu_freq, S_IWUSR | S_IRUGO,
		ipu_freq_get, ipu_freq_set);
static DEVICE_ATTR(lpddr_freq, S_IWUSR | S_IRUGO,
		lpddr_freq_get, lpddr_freq_set);
static DEVICE_ATTR(ipu_clk_src, S_IRUGO,
		ipu_clk_src_get, NULL);


static struct attribute *freq_dev_attributes[] = {
	&dev_attr_cpu_freq.attr,
	&dev_attr_ipu_freq.attr,
	&dev_attr_lpddr_freq.attr,
	&dev_attr_ipu_clk_src.attr,
	NULL
};

static struct attribute_group mnh_freq_cooling_group = {
	.name = "mnh_freq_cool",
	.attrs = freq_dev_attributes
};



static int init_sysfs(struct platform_device *pdev)
{
	int ret;

	ret = sysfs_create_group(kernel_kobj, &mnh_freq_cooling_group);
	if (ret) {
		dev_err(mnh_dev->dev, "Failed to create sysfs\n");
		return -EINVAL;
	}

	return 0;
}

static void clean_sysfs(void)
{
	sysfs_remove_group(kernel_kobj, &mnh_freq_cooling_group);
}


static int mnh_freq_cooling_probe(struct platform_device *pdev)
{
	struct resource *res;

	dev_err(&pdev->dev, "mnh_freq_cooling_probe\n");

	mnh_dev = devm_kzalloc(&pdev->dev, sizeof(*mnh_dev),
			GFP_KERNEL);
	if (!mnh_dev)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "cannot get platform resources\n");
		return -ENOENT;
	}

	mnh_dev->regs = ioremap_nocache(res->start, resource_size(res));
	if (!mnh_dev->regs) {
		dev_err(&pdev->dev, "unable to remap resources\n");
		return -ENOMEM;
	}

	mnh_dev->dev = pdev;

	/* Check IPU_CLK src */
	mnh_dev->ipu_clk_src =
		HW_INf(mnh_dev->regs, SCU, CCU_CLK_CTL, IPU_CLK_SRC);


	/* TBD - Acquire current frequency */

	platform_set_drvdata(pdev, mnh_dev);

	init_sysfs(pdev);

	return 0;

}


static int mnh_freq_cooling_remove(struct platform_device *pdev)
{
	iounmap(&mnh_dev->regs);
	clean_sysfs();

	return 0;
}

/*
 * of_device_id structure
 */
static const struct of_device_id mnh_freq_cooling_of_match[] = {
	{ .compatible = "intel, mnh_freq_cooling" },
	{ }
};

MODULE_DEVICE_TABLE(of, mnh_freq_cooling_of_match);

/*
 * Platform driver structure
 */
static struct platform_driver mnh_freq_cooling_driver = {
	.probe = mnh_freq_cooling_probe,
	.remove = mnh_freq_cooling_remove,
	.driver = {
		.name = "intel, mnh_freq_cooling",
			.owner = THIS_MODULE,
			.of_match_table = mnh_freq_cooling_of_match,
	},
};
module_platform_driver(mnh_freq_cooling_driver);

MODULE_DESCRIPTION("Monette Hill Frequency Cooling Driver");
MODULE_LICENSE("GPL");
