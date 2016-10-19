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
#define MAX_STR_COPY 8
#define LP4_LPC_FREQ_SWITCH 0x8A

enum mnh_cpu_freq_type {
	CPU_FREQ_MIN = 0,
	CPU_FREQ_400 = CPU_FREQ_MIN,
	CPU_FREQ_600,
	CPU_FREQ_800,
	CPU_FREQ_950,
	CPU_FREQ_MAX = CPU_FREQ_950
};

enum mnh_ipu_freq_type {
	IPU_FREQ_MIN = 0,
	IPU_FREQ_200 = IPU_FREQ_MIN,
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
 * Refclk  FBDIV VCO	POSTDIV1 POSTDIV2 FOUTPOSTDIV CPU_CLKDIV CPUCLKFreq
 * 19.2	 99	 1900.8  2	  1	   950.400	0	 950.400
 * 19.2	 125	 2400	 3	  1	   800.000	0	 800.000
 * 19.2	 125	 2400	 4	  1	   600.000	0	 600.000
 * 19.2	 125	 2400	 6	  1	   400.000	0	 400.000
 */
static struct freq_reg_table cpu_reg_tables[] = {
	{99, 2, 1, 0},
	{125, 3, 1, 0},
	{125, 4, 1, 0},
	{125, 6, 1, 0}
};

/* IPU clock frequency calculation tables
 * Refclk  FBDIV  VCO	POSTDIV1 POSTDIV2 FOUTPOSTDIV IPU_CLKDIV CPUCLKFreq
 * 19.2	 133	 2553.6   6	    1	  425.600	0	 425.600
 * 19.2	 125	 2400	  6	    1	  400.000	0	 400.000
 * 19.2	 125	 2400	  4	    1	  600.000	1	 300.000
 * 19.2	 125	 2400	  6	    1	  400.000	1	 200.000
 */
static struct freq_reg_table ipu_reg_tables[] = {
	{133, 6, 1, 0},
	{125, 6, 1, 0},
	{125, 4, 1, 1},
	{125, 6, 1, 1}
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
	int ipu_clk_src = 0;
	int lock = 0;

	if (index < CPU_FREQ_MIN || index > CPU_FREQ_MAX)
		return -EINVAL;


	dev_dbg(mnh_dev->dev, "mnh_cpu_freq_change :%d\n", index);

	/* 1. Program PLL passcode */
	HW_OUTf(mnh_dev->regs, SCU, PLL_PASSCODE, PASSCODE, PLL_UNLOCK);

	/* 2. Power down CPU_IPU PLL.
	 *    Switch to SYS200_MODE (Set CPU_IPU_SYS200_MODE = 1)
	 */
	HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_CTL, CPU_IPU_SYS200_MODE, 0x1);

	/* 3. Change CPU_CLK_DIV and IPU_CLK_DIV values, so the get the
	 *    highest clock possible
	 */
	HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_DIV, CPU_CLK_DIV, 0);
	if (ipu_clk_src == CPU_IPU_PLL)
		HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_DIV, IPU_CLK_DIV, 4);

	/* 4. Set CPU_IPU_PLL_CTRL.FRZ_PLL_IN = 1 */
	HW_OUTf(mnh_dev->regs, SCU, CPU_IPU_PLL_CTRL, FRZ_PLL_IN, 1);

	/* 5. Change FBDIV to new frequency multiplier value.
	 * Compute it based on REF_FREQ_SEL hardware strap.  Change POSTDIV1
	 * and POSTDIV2 values if needed. FBDIV * REFCLK should be within
	 * VCO operating range of 950-3800MHz
	 */
	HW_OUTf(mnh_dev->regs, SCU, CPU_IPU_PLL_INTGR_DIV, FBDIV,
		cpu_reg_tables[index].fidiv);
	HW_OUTf(mnh_dev->regs, SCU, CPU_IPU_PLL_INTGR_DIV, POSTDIV1,
			cpu_reg_tables[index].postdiv1);
	HW_OUTf(mnh_dev->regs, SCU, CPU_IPU_PLL_INTGR_DIV, POSTDIV2,
		cpu_reg_tables[index].postdiv2);

	/* 6. Set CPU_IPU_PLL_CTRL.FOUTPOSTDIVPD = 1 */
	HW_OUTf(mnh_dev->regs, SCU, CPU_IPU_PLL_CTRL, FOUTPOSTDIVPD, 1);

	/* 7. Set CPU_IPU_PLL_CTRL.FRZ_PLL_IN = 0, Now the settings will go to
	 * PLL and associated clocking-unit
	 */
	HW_OUTf(mnh_dev->regs, SCU, CPU_IPU_PLL_CTRL, FRZ_PLL_IN, 0);

	/* 8. Enable the PLL. Set the CPU_IPU_PLL_CTRL.PD = 0 */
	HW_OUTf(mnh_dev->regs, SCU, CPU_IPU_PLL_CTRL, PD, 0);

	/* 9. Wait for minimum 128REFCLK cycles(6.7 usecs for 19.2MHz refclk)
	 * before checking for LOCK status from PLL.
	 */
	udelay(7);

	/* 10. Wait for PLL to lock, CPU_IPU_PLL_STAT.LOCK ==1 */
	do {
		lock = HW_INf(mnh_dev->regs, SCU, CPU_IPU_PLL_STS, LOCK);
	} while (lock != 1);

	/* 11. Set CPU_IPU_PLL_CTRL.FOUTPOSTDIVPD = 0 to ensure that the clock
	 * output from PLL is ungated.
	 */
	HW_OUTf(mnh_dev->regs, SCU, CPU_IPU_PLL_CTRL, FOUTPOSTDIVPD, 0);

	/* 12. Change CPU_CLK_DIV and IPU_CLK_DIV values if needed */
		HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_DIV, CPU_CLK_DIV,
	cpu_reg_tables[index].clk_div);

	/* 13. Move back to PLL output, CPU_IPU_SYS200_MODE = 0 */
	HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_CTL, CPU_IPU_SYS200_MODE, 0);

	/* 14. Program PLL passcode back to 0 to relock */
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

	/* Read if CPU and IPU PLL is derived from same PLL: 1-IPU, 0-CPU_IPU */
	if (mnh_dev->ipu_clk_src != IPU_PLL) {
		dev_err(mnh_dev->dev,
			"Operation not allowed! IPU_CLK SRC is CPU_IPU_PLL\n");
		return -EINVAL;
	}

	dev_dbg(mnh_dev->dev, "mnh_ipu_freq_change :%d\n", index);

	/* 1. Program PLL passcode */
	HW_OUTf(mnh_dev->regs, SCU, PLL_PASSCODE, PASSCODE, PLL_UNLOCK);

	/* 2. To prevent glitches in clock, set the IPU_CLK src to CPU_IPU PLL
	 * based stable clock output
	 */
	HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_CTL, IPU_CLK_SRC, CPU_IPU_PLL);

	/* 3. Change IPU_CLK_DIV values for highest frequency of operation of
	 * IPU clock.
	 * TBD: For now keep the DIV high so the IPU CLK is less than 452.
	 * Later on it should be adjusted based on the current CPU clk.
	 */
	HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_DIV, IPU_CLK_DIV, 4);

	/* 4. Set FRZ_PLL_IN=1 to latch the current settings going to PLL */
	HW_OUTf(mnh_dev->regs, SCU, IPU_PLL_CTRL, FRZ_PLL_IN, 1);

	/* 5. Change FBDIV to new value, change POSTDIV1, POSTDIV2 values */
	HW_OUTf(mnh_dev->regs, SCU, IPU_PLL_INTGR_DIV, FBDIV,
		ipu_reg_tables[index].fidiv);
	HW_OUTf(mnh_dev->regs, SCU, IPU_PLL_INTGR_DIV, POSTDIV1,
		ipu_reg_tables[index].postdiv1);
	HW_OUTf(mnh_dev->regs, SCU, IPU_PLL_INTGR_DIV, POSTDIV2,
		ipu_reg_tables[index].postdiv2);

	/* 6. Set IPU_PLL_CTRL.FOUTPOSTDIVPD = 1 to ensure there is no clock
	 * glitches at output till PLL locks.
	 */
	HW_OUTf(mnh_dev->regs, SCU, IPU_PLL_CTRL, FOUTPOSTDIVPD, 1);

	/* 7. Set IPU_PLL_CTRL.FRZ_PLL_IN =0 */
	HW_OUTf(mnh_dev->regs, SCU, IPU_PLL_CTRL, FRZ_PLL_IN, 1);

	/* 8. Enable the PLL. Set IPU_PLL_CTRL.PD = 0 */
	HW_OUTf(mnh_dev->regs, SCU, IPU_PLL_CTRL, PD, 0);

	/* 9. Wait for minimum 128REFCLK(6.7usec for 19.2MHz refclk)
	 * before checking LOCK from PLL
	 */
	udelay(7);

	/* 10. Wait for PLL to lock */
	do {
		lock = HW_INf(mnh_dev->regs, SCU, IPU_PLL_STS, LOCK);
	} while (lock != 1);

	/* 11. Set FOUTPOSTDIVPD = 0 to ensure the clock output from
	 * PLL is un-gated
	 */
	HW_OUTf(mnh_dev->regs, SCU, IPU_PLL_CTRL, FOUTPOSTDIVPD, 0);

	/* 12. Change IPU_CLK_DIV values */
	HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_DIV, IPU_CLK_DIV,
	ipu_reg_tables[index].clk_div);

	/* 13. Move back to PLL output (CCU_CLK_CTRL.IPU_CLK_SRC = 1) */
	HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_CTL, IPU_CLK_SRC, IPU_PLL);

	/* 14. Program PLL_PASSCODE.PASSCODE to 0 to relock */
	HW_OUTf(mnh_dev->regs, SCU, PLL_PASSCODE, PASSCODE, 0);


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

	dev_dbg(mnh_dev->dev, "mnh_lpddr_freq_change :%d\n", index);

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



static ssize_t ipu_clk_src_read(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	return scnprintf(buf, MAX_STR_COPY, "%s\n",
		(mnh_dev->ipu_clk_src == CPU_IPU_PLL)?"CPU_IPU":"IPU");
}

static ssize_t cpu_freq_read(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	static const char * const str[] = {"400", "600", "800", "950"};

	if (mnh_dev->cpu_freq > CPU_FREQ_MAX ||
	    mnh_dev->cpu_freq < CPU_FREQ_MIN)
		return -EINVAL;

	return scnprintf(buf, MAX_STR_COPY, "%sMHz\n", str[mnh_dev->cpu_freq]);

}
static ssize_t ipu_freq_read(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	static const char * const str[] = {"200", "300", "400", "425"};

	if (mnh_dev->ipu_freq > IPU_FREQ_MAX ||
	    mnh_dev->ipu_freq < IPU_FREQ_MIN)
		return -EINVAL;

	return scnprintf(buf, MAX_STR_COPY, "%sMHz\n", str[mnh_dev->ipu_freq]);

}

static ssize_t lpddr_freq_read(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	uint32_t var = HW_INf(mnh_dev->regs, SCU, LPDDR4_LOW_POWER_STS,
				LPDDR4_CUR_FSP);

	dev_info(mnh_dev->dev, "%s: index:0x%x\n", __func__, var);
	return sprintf(buf, "FSP%d\n", var);
}

static ssize_t cpu_freq_write(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf,
				  size_t count)
{
	static const char * const str[] = {"400", "600", "800", "950"};
	int i, err;

	for (i = 0; i < ARRAY_SIZE(str); i++) {
		if (strncmp(buf, str[i], sizeof(str[i])) == 0) {
			err = mnh_cpu_freq_change(i);
			if (!err)
				return count;
			else
				return -EIO;
		}
	}

	dev_err(mnh_dev->dev, "Invalid!, valid range:400,600,800,950(MHz)\n");

	return -EINVAL;

}

static ssize_t ipu_freq_write(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf,
				  size_t count)
{
	static const char * const str[] = {"200", "300", "400", "425"};
	int i, err;

	for (i = 0; i < ARRAY_SIZE(str); i++) {
		if (strncmp(buf, str[i], sizeof(str[i])) == 0) {
			err = mnh_ipu_freq_change(i);
			if (!err)
				return count;
			else
				return -EIO;
		}
	}

	dev_err(mnh_dev->dev, "Invalid!, valid range:200,300,400,425(MHz)\n");

	return -EINVAL;

}


static ssize_t lpddr_freq_write(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf,
				  size_t count)
{
	unsigned long var = 0;
	int ret;

	ret = kstrtoint(buf, 10, &var);
	if (ret < 0)
		return ret;

	dev_info(mnh_dev->dev, "%s: index:%d\n", __func__, var);
	if (!mnh_lpddr_freq_change(var))
		return count;
	else
		return -EIO;
}



static DEVICE_ATTR(cpu_freq, S_IWUSR | S_IRUGO,
		cpu_freq_read, cpu_freq_write);
static DEVICE_ATTR(ipu_freq, S_IWUSR | S_IRUGO,
		ipu_freq_read, ipu_freq_write);
static DEVICE_ATTR(lpddr_freq, S_IWUSR | S_IRUGO,
		lpddr_freq_read, lpddr_freq_write);
static DEVICE_ATTR(ipu_clk_src, S_IRUGO,
		ipu_clk_src_read, NULL);


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
