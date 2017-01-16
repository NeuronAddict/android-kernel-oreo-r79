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

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/of_device.h>
#include <linux/sysfs.h>
#include <linux/string.h>
#include "mipicsi_debug.h"
#include "mipicsi_device.h"
#include "mipicsi_top.h"
#include "mipicsi_util.h"
#include "mipicsi_sysfs.h"
#include <soc/mnh/mnh-hwio-mipi-rx.h>
#include <soc/mnh/mnh-hwio-mipi-tx.h>

#define MAX_STR_COPY	100


struct devs{
	char name[4];
	enum mipicsi_top_dev device;
};

struct devs dev_tbl[MIPI_MAX+1] = {
	{"Rx0", MIPI_RX0},
	{"Rx1", MIPI_RX1},
	{"Rx2", MIPI_RX2},
	{"Tx0", MIPI_TX0},
	{"Tx1", MIPI_TX1},
	{"TOP", MIPI_TOP},
	{"IPU", MIPI_IPU}
};

static uint32_t read_data;
static char string_data[MAX_STR_COPY];

#define SHOW_FMT_NA(name)                                        \
static ssize_t name##_show(struct device *dev,                   \
                           struct device_attribute *attr,        \
                           char *buf)                            \
{                                                                \
	return snprintf((char *)buf, MAX_STR_COPY, "cat %s not supported\n", \
			#name);					 \
}


int find_device(const char *str, enum mipicsi_top_dev *device)
{
	uint8_t i;
	bool found = false;

	for (i=0; i < sizeof (dev_tbl)/sizeof(struct devs); i++) {
		if (!strncmp(str, dev_tbl[i].name, strlen(dev_tbl[i].name))) {
		    found = true;
		    break;
		}
	}

	if (!found) {
		pr_err("Device not found\n");
		return -EINVAL;
	}

	*device = dev_tbl[i].device;
	return 0;
}

SHOW_FMT_NA(start_dev);

static ssize_t start_dev_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf,
			       size_t count)
{
	struct mipicsi_top_cfg cfg;
	enum mipicsi_top_dev device;
	uint8_t *token;
	const char *delim = ";";

	token = strsep((char **)&buf, delim);
	if ((token) && (find_device(token, &device) >= 0)) {
		cfg.dev = device;
		token = strsep((char **)&buf, delim);
		if (!(token) || (kstrtou32(token, 0, &cfg.mbps)))
			cfg.mbps = 640;
		token = strsep((char **)&buf, delim);
		if (!(token) || (kstrtou32(token, 0, &cfg.num_lanes)))
			cfg.num_lanes = 4;
		mipicsi_top_start(&cfg);
		return count;
	}
	pr_err("Usage: echo\"<dev>;<bitrate>;<lanes>\">start_dev\n");
	pr_err("dev=Rx0,Rx1,Rx2,Tx0,Tx1\n");
	return -EINVAL;
}

static DEVICE_ATTR(start_dev, S_IRUGO | S_IWUSR | S_IWGRP,
		   start_dev_show, start_dev_store);

SHOW_FMT_NA(stop_dev);

static ssize_t stop_dev_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf,
			      size_t count)
{
	enum mipicsi_top_dev device;

	if (find_device(buf, &device) >= 0) {
		mipicsi_top_stop(device);
		return count;
	}
	pr_err("Usage: echo\"<dev>\">stop_dev\n");
	pr_err("dev=Rx0,Rx1,Rx2,Tx0,Tx1\n");
	return -EINVAL;
}

static DEVICE_ATTR(stop_dev, S_IRUGO | S_IWUSR | S_IWGRP,
		   stop_dev_show, stop_dev_store);

SHOW_FMT_NA(reset_dev);

static ssize_t reset_dev_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf,
			       size_t count)
{
	enum mipicsi_top_dev device;

	if (!strncmp(buf,"ALL",3)) {
		mipicsi_top_reset_all();
		return count;
	} else if (find_device(buf, &device) >= 0) {
		mipicsi_top_reset(device);
		return count;
	}
	pr_err("Usage: echo\"<dev>\">reset_dev\n");
	pr_err("dev=ALL,Rx0,Rx1,Rx2,Tx0,Tx1\n");
	return -EINVAL;
}

static DEVICE_ATTR(reset_dev, S_IRUGO | S_IWUSR | S_IWGRP,
		   reset_dev_show, reset_dev_store);

SHOW_FMT_NA(set_mux);

static ssize_t set_mux_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf,
			     size_t count)
{
	enum mipicsi_top_dev device;
        uint8_t *token;
        const char *delim = ";";
	struct mipicsi_top_mux mux;

        token = strsep((char **)&buf, delim);
	if ((token) && (find_device(token, &device) >= 0)) {
		mux.source = device;
		token = strsep((char **)&buf, delim);
		if ((token) && (find_device(token, &device) >= 0)) {
			mux.sink = device;
			mux.ss_vc_mask = 0x0F;
			mux.ss_stream_off = true;
			mipicsi_top_set_mux(&mux);
			return count;
		}
	}
	pr_err("Usage: echo\"<source>;<sink>\">set_mux\n");
	pr_err("source=Rx0,Rx1,Rx2,IPU sink=Tx0,Tx1,IPU\n");
	return -EINVAL;
}

static DEVICE_ATTR(set_mux, S_IRUGO | S_IWUSR | S_IWGRP,
		   set_mux_show, set_mux_store);

SHOW_FMT_NA(disable_mux);

static ssize_t disable_mux_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf,
				 size_t count)
{
	enum mipicsi_top_dev device;
        uint8_t *token;
        const char *delim = ";";
	struct mipicsi_top_mux mux;

	token = strsep((char **)&buf, delim);
	if ((token) && (find_device(token, &device) >= 0)) {
		mux.source = device;
		token = strsep((char **)&buf, delim);
		if ((token) && (find_device(token, &device) >= 0)) {
			mux.sink = device;
			mux.ss_stream_off = true;
			mipicsi_top_disable_mux(&mux);
			return count;
		}
	}
	pr_err("Usage: echo\"<source>;<sink>\">disable_mux\n");
	pr_err("source=Rx0,Rx1,Rx2,IPU sink=Tx0,Tx1,IPU\n");
	return -EINVAL;
}

static DEVICE_ATTR(disable_mux, S_IRUGO | S_IWUSR | S_IWGRP,
		   disable_mux_show, disable_mux_store);

static ssize_t get_mux_status_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
        ssize_t strlen = 0;
	strlen = snprintf((char *)buf, MAX_STR_COPY, "%s\n", string_data);
	string_data[0] = '\0';

	return strlen;
}

static ssize_t get_mux_status_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf,
				    const size_t count)
{
	enum mipicsi_top_dev device;
        uint8_t *token;
        const char *delim = ";";
	struct mipicsi_top_mux mux;
	
        token = strsep((char **)&buf, delim);
	if ((token) && (find_device(token, &device) >= 0)) {
		mux.source = device;
		token = strsep((char **)&buf, delim);
		if ((token) && (find_device(token, &device) >= 0)) {
			mux.sink = device;
			mipicsi_top_get_mux_status(&mux);

			if (mux.active == true)
				snprintf(string_data, MAX_STR_COPY,
					 "%s\n", "Mux path active");
			else if (mux.active == false)
				snprintf(string_data, MAX_STR_COPY,
					 "%s\n", "Mux path inactive");
			return count;
		}
	}
	pr_err("Usage: echo\"<source>;<sink>;\">get_mux_status\n");
	pr_err("source=Rx0,Rx1,Rx2,IPU sink=Tx0,Tx1,IPU\n");
	return -EINVAL;
}

static DEVICE_ATTR(get_mux_status, S_IRUGO | S_IWUSR | S_IWGRP,
		   get_mux_status_show, get_mux_status_store);

SHOW_FMT_NA(vpg_preset);

static ssize_t vpg_preset_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				const size_t count)
{
        enum mipicsi_top_dev device;
        uint8_t *token;
        const char *delim = ";";
	struct mipicsi_top_vpg vpg;
	
        token = strsep((char **)&buf, delim);
	if ((token) && (find_device(token, &device) >= 0)) {
		vpg.dev = device;
		token = strsep((char **)&buf, delim);
		if (token) {
			if (!strncmp (token, "VGA", 3)) {
				mipicsi_debug_vpg_preset(&vpg, VPG_VGA);
				return count;
			}
			else if (!strncmp (token, "1080P", 5)) {
				mipicsi_debug_vpg_preset(&vpg, VPG_1080P);
				return count;
			}
			else if (!strncmp (token, "12MP", 4)) {
				mipicsi_debug_vpg_preset(&vpg, VPG_12MP);
				return count;
			}
		}
	}
	pr_err("Usage: echo\"<dev>;<resolution>\">vpg_preset\n");
	pr_err("dev=Tx0,Tx1, resolution=VGA,1080P,12MP\n");
	return -EINVAL;
}

static DEVICE_ATTR(vpg_preset, S_IRUGO | S_IWUSR | S_IWGRP,
		   vpg_preset_show, vpg_preset_store);

SHOW_FMT_NA(bist_start);

static ssize_t bist_start_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	enum mipicsi_top_dev device;

	if (find_device(buf, &device) >= 0) {
		mipicsi_top_debug_bist_start(device);
		return count;
	}
	pr_err("Usage: echo\"<dev>\">bist_start\n");
	pr_err("dev=Rx0,Rx1,Rx2,Tx0,Tx1\n");
	return -EINVAL;
}

static DEVICE_ATTR(bist_start, S_IRUGO | S_IWUSR | S_IWGRP,
		   bist_start_show, bist_start_store);

static ssize_t bist_status_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
        ssize_t strlen = 0;
	strlen = snprintf((char *)buf, MAX_STR_COPY, "%s\n", string_data);
	string_data[0] = '\0';

	return strlen;
}

static ssize_t bist_status_store(struct device *dev,
                                struct device_attribute *attr,
                                const char *buf,
                                const size_t count)
{
	enum mipicsi_top_dev device;
	struct mipicsi_top_bist bist;

	if (find_device(buf, &device) >= 0) {
		mipicsi_top_debug_bist_status(&bist);
		if (bist.done == true) {
			if (bist.ok == true)
				snprintf(string_data, MAX_STR_COPY,
					 "%s\n", "BIST OK");
			else
				snprintf(string_data, MAX_STR_COPY,
					 "%s\n", "BIST Failed");
		} else {
			snprintf(string_data, MAX_STR_COPY,
				 "%s\n", "BIST In Progress");
		}
		return count;
	}
	pr_err("Usage: echo\"<dev>\">bist_status\n");
	pr_err("dev=Rx0,Rx1,Rx2,Tx0,Tx1\n");
	return -EINVAL;
}

static DEVICE_ATTR(bist_status, S_IRUGO | S_IWUSR | S_IWGRP,
		   bist_status_show, bist_status_store);

static ssize_t reg_read_show(struct device *dev,
			     struct device_attribute *attr,
			     char *buf)
{
        ssize_t strlen = 0;
	strlen = snprintf(buf, MAX_STR_COPY, "0x%lx\n",
				(unsigned long) read_data);
	read_data = 0;

	return strlen;
}

static ssize_t reg_read_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf,
			      size_t count)
{
        uint32_t val;
	enum mipicsi_top_dev device;
        uint8_t *token;
        const char *delim = ";";
	struct mipicsi_top_reg reg;

        token = strsep((char **)&buf, delim);
        if ((token) && (find_device(token, &device) >= 0)) {
		reg.dev = device;
		token = strsep((char **)&buf, delim);
		if ((token) && (!(kstrtou32(token, 0, &val)))
		    && (val >= 0) && (val <= 0xFFFF)) {
			reg.offset = val;
			mipicsi_top_read(&reg);
			read_data = reg.value;
			pr_err("Reg Read: Offset %d, Value %d\n", reg.offset,
			       reg.value);
			return count;
		}
	}
	pr_err("Usage: echo\"<dev>;<offset>;\">reg_read\n");
	pr_err("dev=Rx0,Rx1,Rx2,Tx0,Tx1\n");
	return -EINVAL;
}

static DEVICE_ATTR(reg_read, S_IRUGO | S_IWUSR | S_IWGRP,
		   reg_read_show, reg_read_store);

SHOW_FMT_NA(reg_write);

static ssize_t reg_write_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf,
			       size_t count)
{
        uint32_t val;
	enum mipicsi_top_dev device;
        uint8_t *token;
        const char *delim = ";";
	struct mipicsi_top_reg reg;

        token = strsep((char **)&buf, delim);
        if ((token) && (find_device(token, &device) >= 0)) {
		reg.dev = device;
		token = strsep((char **)&buf, delim);

		if ((token) && (!(kstrtou32(token, 0, &val)))
		    && (val >= 0) && (val <= 0xFFFF)) {
			reg.offset = val;
			token = strsep((char **)&buf, delim);
			if ((token) && (!(kstrtou32(token, 0, &val)))) {
				reg.value = val;
				mipicsi_top_write(&reg);
				return count;
			}
		}
	}
	pr_err("Usage: echo\"<dev>;<offset>;<value>;\">reg_write\n");
	pr_err("dev=Rx0,Rx1,Rx2,Tx0,Tx1\n");
	return -EINVAL;
}

static DEVICE_ATTR(reg_write, S_IRUGO | S_IWUSR | S_IWGRP,
		   reg_write_show, reg_write_store);

static ssize_t dphy_read_show(struct device *dev,
			      struct device_attribute *attr,
			      char *buf)
{
	ssize_t strlen = 0;

	strlen = snprintf(buf, MAX_STR_COPY, "0x%lx\n",
			  (unsigned long)read_data);
	read_data = 0;

	return strlen;
}

static ssize_t dphy_read_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf,
			       size_t count)
{
	uint32_t val;
	enum mipicsi_top_dev device;
	uint8_t *token;
	const char *delim = ";";
	struct mipicsi_top_reg reg;

	token = strsep((char **)&buf, delim);
	if ((token) && (find_device(token, &device) >= 0)) {
		reg.dev = device;
		token = strsep((char **)&buf, delim);
		if ((token) && (!(kstrtou32(token, 0, &val)))
		    && (val >= 0) && (val <= 0xFFFF)) {
			reg.offset = val;
			mipicsi_top_dphy_read(&reg);
			read_data = reg.value;
			pr_err("DPhy Read: Offset 0x%x, Val 0x%x\n", reg.offset,
			       reg.value);
			/* Get the device out of shutdown */
			if ((reg.dev == MIPI_TX0) ||
			    (reg.dev == MIPI_TX1)) {
				reg.offset = HWIO_MIPI_TX_PHY_RSTZ_REGOFF;
				reg.value = 0x07;
				mipicsi_top_write(&reg);
			} else {
				reg.offset = HWIO_MIPI_RX_PHY_SHUTDOWNZ_REGOFF;
				reg.value = 1;
				mipicsi_top_write(&reg);
				reg.offset = HWIO_MIPI_RX_DPHY_RSTZ_REGOFF;
				reg.value = 1;
				mipicsi_top_write(&reg);
			}
			return count;
		}
	}
	pr_err("Usage: echo\"<dev>;<offset>;\">dphy_read\n");
	pr_err("dev=Rx0,Rx1,Rx2,Tx0,Tx1\n");
	return -EINVAL;
}

static DEVICE_ATTR(dphy_read, S_IRUGO | S_IWUSR | S_IWGRP,
		   dphy_read_show, dphy_read_store);

SHOW_FMT_NA(dphy_write);

static ssize_t dphy_write_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	uint32_t val;
	enum mipicsi_top_dev device;
	uint8_t *token;
	const char *delim = ";";
	struct mipicsi_top_reg reg;

	token = strsep((char **)&buf, delim);
	if ((token) && (find_device(token, &device) >= 0)) {
		reg.dev = device;
		token = strsep((char **)&buf, delim);

		if ((token) && (!(kstrtou32(token, 0, &val)))
		    && (val >= 0) && (val <= 0xFFFF)) {
			reg.offset = val;
			token = strsep((char **)&buf, delim);
			if ((token) && (!(kstrtou32(token, 0, &val)))) {
				reg.value = val;
				mipicsi_top_dphy_write(&reg);
				/* Take the device out of shutdown */
				if ((reg.dev == MIPI_TX0) || (reg.dev == MIPI_TX1)) {
					reg.offset = HWIO_MIPI_TX_PHY_RSTZ_REGOFF;
					reg.value = 0x07;
					mipicsi_top_write(&reg);
				} else {
					reg.offset = HWIO_MIPI_RX_PHY_SHUTDOWNZ_REGOFF;
					reg.value = 1;
					mipicsi_top_write(&reg);
					reg.offset = HWIO_MIPI_RX_DPHY_RSTZ_REGOFF;
					reg.value = 1;
					mipicsi_top_write(&reg);
				}
				return count;
			}
		}
	}
	pr_err("Usage: echo\"<dev>;<offset>;<value>;\">dphy_write\n");
	pr_err("dev=Rx0,Rx1,Rx2,Tx0,Tx1\n");
	return -EINVAL;
}

static DEVICE_ATTR(dphy_write, S_IRUGO | S_IWUSR | S_IWGRP,
		   dphy_write_show, dphy_write_store);


SHOW_FMT_NA(dump_regs);

static ssize_t dump_regs_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf,
			       size_t count)
{
	enum mipicsi_top_dev device;

	if (find_device(buf, &device) >= 0) {
		mipicsi_debug_dump(device);
		return count;
	}
	pr_err("Usage: echo\"<dev>\">dump_regs\n");
	pr_err("dev=Rx0,Rx1,Rx2,Tx0,Tx1\n");
	return -EINVAL;
}

static DEVICE_ATTR(dump_regs, S_IRUGO | S_IWUSR | S_IWGRP,
		   dump_regs_show, dump_regs_store);
int mipicsi_sysfs_init(struct device *mipicsi_top_device)
{
	int ret;

	pr_err("MIPI TOP: init_sysfs\n");
	ret = device_create_file(mipicsi_top_device,
				 &dev_attr_start_dev);
	if (ret) {
		dev_err(mipicsi_top_device,
			"Failed to create sysfs: start_dev\n");
		return -EINVAL;
	}

	ret = device_create_file(mipicsi_top_device,
				 &dev_attr_stop_dev);
	if (ret) {
		dev_err(mipicsi_top_device,
			"Failed to create sysfs: stop_dev\n");
		return -EINVAL;
	}

	ret = device_create_file(mipicsi_top_device,
				 &dev_attr_reset_dev);
	if (ret) {
		dev_err(mipicsi_top_device,
			"Failed to create sysfs: reset_dev\n");
		return -EINVAL;
	}

	ret = device_create_file(mipicsi_top_device,
				 &dev_attr_set_mux);
	if (ret) {
		dev_err(mipicsi_top_device,
			"Failed to create sysfs: set_mux\n");
		return -EINVAL;
	}

	ret = device_create_file(mipicsi_top_device,
				 &dev_attr_disable_mux);
	if (ret) {
		dev_err(mipicsi_top_device,
			"Failed to create sysfs: disable_mux\n");
		return -EINVAL;
	}

	ret = device_create_file(mipicsi_top_device,
				 &dev_attr_get_mux_status);
	if (ret) {
		dev_err(mipicsi_top_device,
			"Failed to create sysfs: get_mux_status\n");
		return -EINVAL;
	}

	ret = device_create_file(mipicsi_top_device,
				 &dev_attr_vpg_preset);
	if (ret) {
		dev_err(mipicsi_top_device,
			"Failed to create sysfs: vpg_preset\n");
		return -EINVAL;
	}

        ret = device_create_file(mipicsi_top_device,
				 &dev_attr_bist_start);
	if (ret) {
		dev_err(mipicsi_top_device,
			"Failed to create sysfs: bist_start\n");
		return -EINVAL;
	}

	ret = device_create_file(mipicsi_top_device,
				 &dev_attr_bist_status);
	if (ret) {
		dev_err(mipicsi_top_device,
			"Failed to create sysfs: bist_status\n");
		return -EINVAL;
	}

	ret = device_create_file(mipicsi_top_device,
				 &dev_attr_reg_read);
	if (ret) {
		dev_err(mipicsi_top_device,
			"Failed to create sysfs: reg_read\n");
		return -EINVAL;
	}

	ret = device_create_file(mipicsi_top_device,
				 &dev_attr_reg_write);
	if (ret) {
		dev_err(mipicsi_top_device,
			"Failed to create sysfs: reg_write\n");
		return -EINVAL;
	}

	ret = device_create_file(mipicsi_top_device,
				 &dev_attr_dphy_read);
	if (ret) {
		dev_err(mipicsi_top_device,
			"Failed to create sysfs: dphy_read\n");
		return -EINVAL;
	}

	ret = device_create_file(mipicsi_top_device,
				 &dev_attr_dphy_write);
	if (ret) {
		dev_err(mipicsi_top_device,
			"Failed to create sysfs: dphy_write\n");
		return -EINVAL;
	}

	ret = device_create_file(mipicsi_top_device,
				 &dev_attr_dump_regs);
	if (ret) {
		dev_err(mipicsi_top_device,
			"Failed to create sysfs: dump_regs\n");
		return -EINVAL;
	}

	return 0;
}

void mipicsi_sysfs_clean(struct device *mipicsi_top_device)
{
	device_remove_file(mipicsi_top_device,
			   &dev_attr_start_dev);

	device_remove_file(mipicsi_top_device,
			   &dev_attr_stop_dev);

	device_remove_file(mipicsi_top_device,
			   &dev_attr_reset_dev);

	device_remove_file(mipicsi_top_device,
			   &dev_attr_set_mux);

	device_remove_file(mipicsi_top_device,
			   &dev_attr_disable_mux);

	device_remove_file(mipicsi_top_device,
			   &dev_attr_get_mux_status);

	device_remove_file(mipicsi_top_device,
			   &dev_attr_reg_read);

	device_remove_file(mipicsi_top_device,
			   &dev_attr_reg_write);

	device_remove_file(mipicsi_top_device,
			   &dev_attr_vpg_preset);

	device_remove_file(mipicsi_top_device,
			   &dev_attr_dump_regs);
}
