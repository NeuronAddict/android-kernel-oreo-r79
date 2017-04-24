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
#include <linux/intel-hwio.h>
#include "mnh-clk.h"


#define DEVICE_NAME "mnh_freq_cooling"
static void __iomem *baseaddr;
static struct device *dev;


static int mnh_freq_cooling_probe(struct platform_device *pdev)
{
	struct resource *res;

	dev = &pdev->dev;
	dev_info(&pdev->dev, "mnh_freq_cooling_probe\n");

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "cannot get platform resources\n");
		return -ENOENT;
	}

	baseaddr = ioremap_nocache(res->start, resource_size(res));
	if (!baseaddr) {
		dev_err(&pdev->dev, "unable to remap resources\n");
		return -ENOMEM;
	}

	mnh_clk_init(pdev, baseaddr);

	return 0;
}


static int mnh_freq_cooling_remove(struct platform_device *pdev)
{
	iounmap(&baseaddr);
	mnh_clk_clean(&pdev->dev);
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
