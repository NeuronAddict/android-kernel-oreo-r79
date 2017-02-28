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
 *
 * Intel MonetteHill PM driver
 *
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/suspend.h>
#include <linux/pm.h>
#include <linux/arm-smccc.h>
#include <asm/suspend.h>

/*
 * Driver definitions
 */
#define DEVICE_NAME "mnh_pm"
#define MNH_PM_SUSPEND_AARCH64	0xC300FF04

/* PM debug messages definition */
#if CONFIG_MNH_PM_DEBUG
#define mnh_debug pr_err
#else
#define mnh_debug pr_debug
#endif

/* TODO: temporary test code. needs to be removed later.
 * PM IO interrupt temporary code
 */
#define MNH_PM_IO_INTR_TMP 1
#if MNH_PM_IO_INTR_TMP
#define MNH_PM_SUSPEND_IO	0x9
#define MNH_PM_SUSPEND_VAL	(1<<MNH_PM_SUSPEND_IO)
#endif /* MNH_PM_IO_INTR_TMP */

/*
 * Variables
 */
struct mnh_pm_device {
	struct device *dev;
	void __iomem *regs;
	int irq;
	int status;
};

static struct mnh_pm_device *mnh_pm;
struct work_struct mnh_pm_intr_work;

static unsigned long invoke_mnh_fn_smc(unsigned long function_id,
			unsigned long arg0, unsigned long arg1,
			unsigned long arg2)
{
	struct arm_smccc_res res;

	arm_smccc_smc(function_id, arg0, arg1, arg2, 0, 0, 0, 0, &res);
	return res.a0;
}

static int mnh_pm_suspend(unsigned long resume_addr)
{
	return invoke_mnh_fn_smc(MNH_PM_SUSPEND_AARCH64, resume_addr, 0, 0);
}

int mnh_pm_suspend_enter(suspend_state_t state)
{
	phys_addr_t phy;

	phy = virt_to_phys(cpu_resume);
	mnh_debug("%s state:%d, addr:0x%llx\n", __func__, state, phy);

	return cpu_suspend(phy, mnh_pm_suspend);
}

void mnh_pm_suspend_wake(void)
{
	mnh_debug("%s mnh_resume_start\n", __func__);
}

static void mnh_pm_intr_proc(struct work_struct *work)
{
	int error;

	mnh_debug("%s mnh_suspend_start\n", __func__);

	error = pm_suspend(PM_SUSPEND_MEM);
	if (error)
		pr_err("%s: suspend error. err=%d\n", __func__, error);

	mnh_debug("%s mnh_resume_complete\n", __func__);
}

static irqreturn_t mnh_pm_handle_irq(int irq, void *dev_id)
{
	int pm_irq = ((struct mnh_pm_device *)dev_id)->irq;
	void __iomem *regs = ((struct mnh_pm_device *)dev_id)->regs;

	if (irq == pm_irq) {
#if MNH_PM_IO_INTR_TMP
		/* clear interrupt status */
		writel(MNH_PM_SUSPEND_VAL, regs + 0x4C);
#endif /* MNH_PM_IO_INTR_TMP */

		schedule_work(&mnh_pm_intr_work);

		/* return interrupt handled */
		return IRQ_HANDLED;
	} else
		return IRQ_NONE;
}

static const struct platform_suspend_ops mnh_pm_ops = {
	.enter		= mnh_pm_suspend_enter,
	.wake		= mnh_pm_suspend_wake,
	.valid		= suspend_valid_only_mem,
};

static int mnh_pm_probe(struct platform_device *pdev)
{
	int err, ret;
	struct resource *res;

	dev_info(&pdev->dev, "init\n");

	if (!pdev)
		return -EINVAL;

	mnh_pm = devm_kzalloc(&pdev->dev, sizeof(*mnh_pm),
			GFP_KERNEL);
	if (!mnh_pm)
		return -ENOMEM;

	mnh_pm->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "cannot get platform resources\n");
		ret = -ENOENT;
		goto mnh_pm_probe_err;
	}

	mnh_pm->regs = ioremap_nocache(res->start, resource_size(res));
	if (!mnh_pm->regs) {
		dev_err(&pdev->dev, "unable to remap resources\n");
		ret = -ENOMEM;
		goto mnh_pm_probe_err;
	}

	/* Set up IRQ handler */
	mnh_pm->irq = platform_get_irq(pdev, 0);
	mnh_pm->status = 0;
	err = request_irq(mnh_pm->irq, mnh_pm_handle_irq,
			IRQF_SHARED, DEVICE_NAME, mnh_pm);
	if (err) {
		dev_err(&pdev->dev, "Could not allocated irq\n");
		ret = -EINVAL;
		goto mnh_pm_probe_err;
	}
	INIT_WORK(&mnh_pm_intr_work, &mnh_pm_intr_proc);

#if MNH_PM_IO_INTR_TMP
	/* enable debounce to avoid glitches */
	writel(MNH_PM_SUSPEND_VAL, mnh_pm->regs + 0x48);
	/* configure type */
	writel(MNH_PM_SUSPEND_VAL, mnh_pm->regs + 0x38);
	/* configure polarity */
	writel(MNH_PM_SUSPEND_VAL, mnh_pm->regs + 0x3C);
	/* enable interrupt */
	writel(MNH_PM_SUSPEND_VAL, mnh_pm->regs + 0x30);
#endif /* MNH_PM_IO_INTR_TMP */

	/* Set up suspend operation table */
	suspend_set_ops(&mnh_pm_ops);

	platform_set_drvdata(pdev, mnh_pm);
	mnh_debug("%s done\n", __func__);

	return 0;

mnh_pm_probe_err:
	suspend_set_ops(NULL);
	if (mnh_pm->regs)
		iounmap(&mnh_pm->regs);

	return ret;
}

static int mnh_pm_remove(struct platform_device *pdev)
{
	suspend_set_ops(NULL);

	iounmap(&mnh_pm->regs);

	return 0;
}

/*
 * of_device_id structure
 */
static const struct of_device_id mnh_pm_of_match[] = {
	{ .compatible = "intel, mnh_pm" },
	{ }
};

MODULE_DEVICE_TABLE(of, mnh_pm_of_match);

static struct platform_driver mnh_pm_driver = {
	.probe		= mnh_pm_probe,
	.remove		= mnh_pm_remove,
	.driver		= {
		.name		= DEVICE_NAME,
		.owner		= THIS_MODULE,
		.of_match_table = mnh_pm_of_match,
	},
};

static int mnh_pm_init(void)
{
	pr_info("%s\n", __func__);
	return platform_driver_register(&mnh_pm_driver);
}

static void mnh_pm_exit(void)
{
	pr_info("%s\n", __func__);
	platform_driver_unregister(&mnh_pm_driver);
}

late_initcall(mnh_pm_init);
module_exit(mnh_pm_exit);

MODULE_DESCRIPTION("MonetteHill PM Driver");
MODULE_LICENSE("GPL");

