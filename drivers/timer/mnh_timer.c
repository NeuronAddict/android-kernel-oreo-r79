/*
 * Copyright (c) 2016-2017, Intel Corporation. All rights reserved.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/intel-hwio.h>
#include <soc/mnh/mnh-hwio-scu.h>
#include "mnh_timer.h"

/*
 * Global variables are declared as static, so are global within the file. 
 */
static void __iomem *scu;
static void __iomem *mnh_timer_base0;
static void __iomem *mnh_timer_base1;
static void __iomem *mnh_timer_base2;
static void __iomem *mnh_timer_base3;

char cmode0[] = "free-running\n";
char cmode1[] = "user-defined\n";

char enable_str[] = "1\n";
char disable_str[] = "0\n";

static void mnh_timer_init(void);
static int     mnh_timer_probe(struct platform_device *);
static int     mnh_timer_remove(struct platform_device *);

static inline u32 mnh_timer_readl(void __iomem *base, unsigned long offs)
{
	return readl(base + offs);
}
 
static inline void mnh_timer_writel(void __iomem *base, u32 val, unsigned long offs)
{
	writel(val, base + offs);
}

static inline u32 mnh_timer_readl_relaxed(void __iomem *base, unsigned long offs)
{
	return readl_relaxed(base + offs);
}

static inline void mnh_timer_writel_relaxed(void __iomem *base, u32 val, unsigned long offs)
{
	writel_relaxed(val, base + offs);
}

cycle_t mnh_timer_get_currentvalue(void __iomem *base)
{
	return (cycle_t)mnh_timer_readl(base, MNH_TIMER_CURRENTVALUE_OFFSET);
 }

cycle_t mnh_timer_get_loadcount(void __iomem *base)
{
	return (cycle_t)mnh_timer_readl(base,  MNH_TIMER_LOADCOUNT_OFFSET);
 }

static void mnh_timer_disable_int(void __iomem *base)
{
	u32 ctrl = mnh_timer_readl(base, MNH_TIMER_CONTROLREG_OFFSET);

	/* set 1 - disable interrupts, 0 - enable interrupts (reverse of synopsis spec  */
	ctrl |= MNH_TIMER_INTERRUPT_MASK;
	mnh_timer_writel(base, ctrl, MNH_TIMER_CONTROLREG_OFFSET);
}

void __iomem *get_timer_base(struct kobject *kobj)
{
	const char *name = kobject_name(kobj);

	pr_debug("%s : name = %s\n", __func__, name);

	if (strstr(name, "timer0"))
		return mnh_timer_base0;
	else if (strstr(name, "timer1"))
		return mnh_timer_base1;
	else if (strstr(name, "timer2"))
		return mnh_timer_base2;
	else if (strstr(name, "timer3"))
		return mnh_timer_base3;
	else
		return NULL;
}

static ssize_t mode_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
        u32 ctrl;
        int ret = 0;

        void __iomem *base = get_timer_base(kobj);
	if (!base)
		return -EIO;

	pr_debug("%s\n", __func__);
	ctrl = mnh_timer_readl(base, MNH_TIMER_CONTROLREG_OFFSET);

	pr_debug("ctrl = %u\n",ctrl);

	if (MNH_TIMER_MODE_USER_DEFINED & ctrl)	{
		ret =strlen(cmode1);
		strncpy(buf, cmode1, ret);
	} else {
		ret = strlen(cmode0);
		strncpy(buf, cmode0, ret);
	}
	return ret;
}

static ssize_t mode_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
        u32 ctrl, mode;
	void __iomem *base = get_timer_base(kobj);
	if (!base)
		return -EIO;

	pr_debug("%s\n", __func__);
	if (!strncmp(buf, cmode0, strlen(cmode0)))
		mode = MNH_TIMER_MODE_FREE_RUNNING;
	else if (!strncmp(buf, cmode1, strlen(cmode1)))
		mode = MNH_TIMER_MODE_USER_DEFINED;
	else {
		pr_err("only supported values = free-running, user-defined\n");
                pr_err("defaulting to free-running");
                mode = MNH_TIMER_MODE_FREE_RUNNING;
	}

        pr_debug("mode = %d\n", mode);
	ctrl = mnh_timer_readl(base, MNH_TIMER_CONTROLREG_OFFSET);
        if (mode)
		ctrl |= MNH_TIMER_MODE_USER_DEFINED;
	else
		ctrl &=~MNH_TIMER_MODE_USER_DEFINED;

	mnh_timer_writel(base, ctrl, MNH_TIMER_CONTROLREG_OFFSET);
	mnh_timer_writel(base, ctrl, MNH_TIMER_CONTROLREG_OFFSET);

	return count;
}

static ssize_t loadcounter_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	void __iomem *base = get_timer_base(kobj);
	if (!base)
		return -EIO;

	pr_debug("%s\n", __func__);
	return sprintf(buf, "%lu\n", mnh_timer_get_loadcount(base) );
}

static ssize_t loadcounter_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long var;
	void __iomem *base = get_timer_base(kobj);
	if (!base)
		return -EIO;

	pr_debug("%s\n", __func__);

	ret = kstrtoint(buf, 10, &var);
	if (ret < 0)
		return ret;

	pr_debug("%s Setting clock  period = %u   %d \n", __func__, var, var);

	mnh_timer_writel(base, var, MNH_TIMER_LOADCOUNT_OFFSET);

	return count;
}

static ssize_t currentvalue_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	void __iomem *base = get_timer_base(kobj);
	if (!base)
		return -EIO;

        pr_debug("%s\n", __func__);
	return sprintf(buf, "%lu\n", mnh_timer_get_currentvalue(base));
}


static ssize_t status_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	u32 ctrl;
	int ret;
	void __iomem *base = get_timer_base(kobj);
	if (!base)
		return -EIO;

	pr_debug("%s\n", __func__);
	ctrl = mnh_timer_readl(base, MNH_TIMER_CONTROLREG_OFFSET);
	pr_debug("ctrl = %u\n", ctrl);
	if (MNH_TIMER_ENABLE & ctrl)
	{
		ret = strlen(enable_str);
		strncpy(buf, enable_str, ret);
	}
	else
	{
		ret = strlen(disable_str);
		strncpy(buf, disable_str, ret);
	}

	return ret;
}

static ssize_t status_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;
        u32 ctrl;
	unsigned long var;
        void __iomem *base = get_timer_base(kobj);
	if (!base)
		return -EIO;

	pr_debug("%s\n", __func__);

	ret = kstrtoint(buf, 10, &var);
	if (ret < 0)
		return ret;


	ctrl = mnh_timer_readl(base, MNH_TIMER_CONTROLREG_OFFSET);

	if (var & MNH_TIMER_ENABLE)
	{
		pr_debug("timer enable\n");
		ctrl |= MNH_TIMER_ENABLE;
	}
	else 
        {
		pr_debug("timer disable\n");	
		ctrl &= ~MNH_TIMER_ENABLE;
	}

	mnh_timer_writel(base, ctrl, MNH_TIMER_CONTROLREG_OFFSET);

	return count;
}


static struct kobject *mnh_timer_kobject, *timer0_kobject, *timer1_kobject, *timer2_kobject, *timer3_kobject;

static struct kobj_attribute mode_attribute = 	__ATTR(mode, 0664, mode_show, mode_store);
static struct kobj_attribute loadcounter_attribute = 	__ATTR(loadcounter, 0664, loadcounter_show, loadcounter_store);
static struct kobj_attribute currentvalue_attribute =  	__ATTR(currentvalue, 0664, currentvalue_show,NULL);
static struct kobj_attribute status_attribute =  	__ATTR(status, 0664,status_show, status_store);

static struct attribute *attrs[] = {
	&mode_attribute.attr,
	&loadcounter_attribute.attr,
	&currentvalue_attribute.attr,
	&status_attribute.attr,
	NULL,	/* need to NULL terminate the list of attributes */
};

static struct attribute_group attr_group = {
	.attrs = attrs,
};


int mnh_timer_sysfs_init(void)
{
	int ret =0 ;
	pr_debug("%s ....\n", __func__);

	mnh_timer_kobject = kobject_create_and_add("mnh-timer",NULL);
	if (!mnh_timer_kobject)
	{
		pr_err("%s failed to create mnh_timer kobj\n", __func__);
		return -ENOMEM;
	}

	timer0_kobject = kobject_create_and_add("timer0", mnh_timer_kobject);
	if (!timer0_kobject)
	{
		pr_err("%s failed to create timer0 kobj\n", __func__);
		kobject_put(mnh_timer_kobject);
		return -ENOMEM;
	}

	timer1_kobject = kobject_create_and_add("timer1", mnh_timer_kobject);
	if (!timer1_kobject)
	{
		pr_err("%s failed to create timer1 kobj\n", __func__);
		kobject_put(timer0_kobject);
		kobject_put(mnh_timer_kobject);
		return -ENOMEM;
	}

	timer2_kobject = kobject_create_and_add("timer2", mnh_timer_kobject);
	if (!timer2_kobject)
	{
		pr_err("%s failed to create timer2 kobj\n", __func__);
		kobject_put(timer0_kobject);
		kobject_put(timer1_kobject);
		kobject_put(mnh_timer_kobject);
		return -ENOMEM;
	}

	timer3_kobject = kobject_create_and_add("timer3", mnh_timer_kobject);
	if (!timer3_kobject)
	{
		pr_err("%s failed to create timer3 kobj\n", __func__);
		kobject_put(timer0_kobject);
		kobject_put(timer1_kobject);
		kobject_put(timer2_kobject);
		kobject_put(mnh_timer_kobject);
		return -ENOMEM;
	}

	/* Create the files associated with this kobject */
	ret = sysfs_create_group(timer0_kobject, &attr_group);
	if (ret)
	{
		pr_err("%s failed to create attributes for timer0", __func__);
		kobject_put(timer0_kobject);
		kobject_put(timer1_kobject);
		kobject_put(timer2_kobject);
		kobject_put(timer3_kobject);
		kobject_put(mnh_timer_kobject);
		return ret;
	}

	ret = sysfs_create_group(timer1_kobject, &attr_group);
	if (ret)
	{
		pr_err("%s failed to create attributes for timer1", __func__);
		sysfs_remove_group(timer0_kobject, &attr_group);
		kobject_put(timer0_kobject);
		kobject_put(timer1_kobject);
		kobject_put(timer2_kobject);
		kobject_put(timer3_kobject);
		kobject_put(mnh_timer_kobject);
		return ret;
	}

	ret = sysfs_create_group(timer2_kobject, &attr_group);
	if (ret)
	{
		pr_err("%s failed to create attributes for timer2", __func__);
		sysfs_remove_group(timer0_kobject, &attr_group);
		sysfs_remove_group(timer1_kobject, &attr_group);
		kobject_put(timer0_kobject);
		kobject_put(timer1_kobject);
		kobject_put(timer2_kobject);
		kobject_put(timer3_kobject);
		kobject_put(mnh_timer_kobject);
		return ret;
	}

	ret = sysfs_create_group(timer3_kobject, &attr_group);
	if (ret)
	{
		pr_err("%s failed to create attributes for timer3", __func__);
		sysfs_remove_group(timer0_kobject, &attr_group);
		sysfs_remove_group(timer1_kobject, &attr_group);
		sysfs_remove_group(timer2_kobject, &attr_group);
		kobject_put(timer0_kobject);
		kobject_put(timer1_kobject);
		kobject_put(timer2_kobject);
		kobject_put(timer3_kobject);
		kobject_put(mnh_timer_kobject);
		return ret;
	}

	pr_debug("%s created mnh-timer groups successfully", __func__);

	return ret;
}


void mnh_timer_sysfs_clean(void)
{
	sysfs_remove_group(timer0_kobject, &attr_group);
	sysfs_remove_group(timer1_kobject, &attr_group);
	sysfs_remove_group(timer2_kobject, &attr_group);
	sysfs_remove_group(timer3_kobject, &attr_group);
	kobject_put(timer0_kobject);
	kobject_put(timer1_kobject);
	kobject_put(timer2_kobject);
	kobject_put(timer3_kobject);
	kobject_put(mnh_timer_kobject);
}

static void mnh_timer_init()
{
	pr_debug("mnh_timer_init\n");
	HW_OUTf(scu, SCU, PERIPH_CLK_CTRL, TIMER_CLKEN_SW, 1);
	HW_OUTf(scu, SCU, RSTC, TIMER_RST, 0);
	if (!mnh_timer_sysfs_init())
	{
		//disable interrupts
		mnh_timer_disable_int(mnh_timer_base0);
		mnh_timer_disable_int(mnh_timer_base1);
		mnh_timer_disable_int(mnh_timer_base2);
		mnh_timer_disable_int(mnh_timer_base3);
	}
}

static int mnh_timer_suspend(struct platform_device *pdev)
{
#if 0
	HW_OUTf(scu, SCU, RSTC, TIMER_RST, 1);
	HW_OUTf(scu, SCU, PERIPH_CLK_CTRL, TIMER_CLKEN_SW, 0);
#endif
	return 0;
}

static int mnh_timer_resume(struct platform_device *pdev)
{
	HW_OUTf(scu, SCU, PERIPH_CLK_CTRL, TIMER_CLKEN_SW, 1);
	HW_OUTf(scu, SCU, RSTC, TIMER_RST, 0);
	return 0;
}

static int mnh_timer_probe(struct platform_device *pdev)
{
	int error = 0;
	struct resource *scu_temp = NULL;
	struct resource *mem = NULL;

	/* Device tree information: Base addresses & mapping */
	scu_temp = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (scu_temp == NULL) {
		pr_err("Base address of scu is not set.\n");
		error = -ENXIO;
		return error;
	}
	scu = ioremap_nocache(scu_temp->start, resource_size(scu_temp));
	if (!scu) {
		pr_err("%s scu ioremap failure.\n", __func__);
		return -ENOMEM;
	}

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (mem == NULL) {
		pr_err("Base address of the device is not set.\nSee device tree.\n");
		error = -ENXIO;
		return error;
	}

	mnh_timer_base0 = ioremap_nocache(mem->start, resource_size(mem));
	if (!mnh_timer_base0) {
		iounmap(scu);
		error = -ENOMEM;
		return error;
	}

	pr_debug("ioremap mnh_timer_base0 successfully\n");
	mnh_timer_base1 = mnh_timer_base0 + MNH_TIMER_REG_LENGTH;
	mnh_timer_base2 = mnh_timer_base1 + MNH_TIMER_REG_LENGTH;
	mnh_timer_base3 = mnh_timer_base2 + MNH_TIMER_REG_LENGTH;

	mnh_timer_init();

	pr_info("mnh_timer: probe\n");

	return error;
}

static int mnh_timer_remove(struct platform_device *pdev)
{
	mnh_timer_sysfs_clean();
	iounmap(scu);
	iounmap(mnh_timer_base0);
	return 0;
}

static const struct of_device_id mnh_timer_dev[] = {
	{ .compatible = "mnh-timer" },
};

MODULE_DEVICE_TABLE(of, mnh_timer_dev);

static struct platform_driver mnh_timer_pldriver = {
	.probe		= mnh_timer_probe,
	.remove		= mnh_timer_remove,
	.suspend	= mnh_timer_suspend,
	.resume		= mnh_timer_resume,
	.driver		= {
		.name  = "mnh-timer",
		.owner = THIS_MODULE,
		.of_match_table = mnh_timer_dev,
	},
};

module_platform_driver(mnh_timer_pldriver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Device driver for Timer");
MODULE_AUTHOR("Intel");
