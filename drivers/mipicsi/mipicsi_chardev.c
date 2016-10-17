/*
 * Copyright (c) 2014--2016 Intel Corporation.
 *
 * Author: Teemu Rytkonen <teemu.s.rytkonen@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/semaphore.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/version.h>
#include <linux/ioctl.h>
#include <linux/uaccess.h>

#include <linux/mipibridge.h>
#include "mipicsi_chardev.h"
#include "mipi_dev.h"

/*
 *	Active char devices
 */
#define MAX_CHAR_DEVICES 128
#define CLASS_NAME "mipi"

static unsigned int char_major_dev;
static unsigned int char_minor_device = 1;
static struct mipi_chardev *mipi_char_devices[MAX_CHAR_DEVICES];
static DEFINE_MUTEX(chardev_lock);

struct mipi_ioctl {
	unsigned int cmd;
	int (*func)(unsigned long arg, struct mipi_chardev* chardev);
};

static int get_available_index(struct mipi_chardev **mipi_char_devices)
{
	int i;

	for (i = 0; i < MAX_CHAR_DEVICES-1; i++) {
		if (mipi_char_devices[i] == NULL)
			return i;
	}
	return -1;
}

int mipi_chardev_open(struct inode *inode, struct file *filp)
{
	struct mipi_chardev *dev; /* device information */

	pr_debug("opening mipi\n");
	dev = container_of(inode->i_cdev, struct mipi_chardev, cdev);
	filp->private_data = dev; /* for other methods */
	pr_debug("opening mipi done dev->open=%d\n", dev->open);
	if (dev->open)
		return -EBUSY;
	dev->open++;
	return 0;
}

int mipi_chardev_close(struct inode *inode, struct file *filp)
{
	struct mipi_chardev *dev; /* device information */

	pr_debug("Inside close\n");
	dev = container_of(inode->i_cdev, struct mipi_chardev, cdev);
	filp->private_data = dev; /* for other methods */
	dev->open--;
	pr_debug("closing mipi done\n");
	return 0;
}

int mipi_chardev_read_reg_ioctl( unsigned long arg, struct mipi_chardev* chardev )
{
	int err = 0;
	struct mipicsi_top_reg regio;

	/* copy from user */
	err = copy_from_user(&regio, (void __user *)arg, sizeof(regio));
	pr_debug("mipi_chardev_ioctl: writing dev %d reg 0x%x val 0x%x",
		 regio.dev, regio.offset, regio.value);
	if (err == 0)
	{
		chardev->topOps.readreg(&regio);
								
		err = copy_to_user((void __user *)arg, &regio, sizeof(regio));
					
		if( err != 0 )
			pr_err("Could not copy data to user. err=%d", err);
	}
	else
		pr_err("Could not copy data from user. err=%d", err);
		
	return err;			
}

int mipi_chardev_write_reg_ioctl( unsigned long arg, struct mipi_chardev* chardev )
{
	int err = 0;
	struct mipicsi_top_reg regio;
	/* copy from user */
	err = copy_from_user(&regio, (void __user *)arg, sizeof(regio));
	
	pr_debug("mipi_chardev_ioctl: writing dev %d reg 0x%x val 0x%x",
		 regio.dev, regio.offset, regio.value);
		
	if (err == 0)
		chardev->topOps.writereg(&regio);
	else
		pr_err("Could not copy data from user. err=%d", err);

	return err;
}

int mipi_chardev_start_ioctl( unsigned long arg, struct mipi_chardev* chardev )
{
	int err = 0;
	struct mipicsi_top_cfg config;
	/* copy from user */
	err = copy_from_user(&config, (void __user *)arg, sizeof(config));

	pr_debug("mipi_chardev_ioctl: starting dev %d", config.dev);

	if (err == 0)
		chardev->topOps.start(&config);
	else
		pr_err("Could not copy data from user. err=%d", err);

	return err;
}

int mipi_chardev_stop_ioctl( unsigned long arg, struct mipi_chardev* chardev )
{
	int err = 0;
	enum mipicsi_top_dev dev;
	/* copy from user */
	err = copy_from_user(&dev, (void __user *)arg, sizeof(dev));

	pr_debug("mipi_chardev_ioctl: stopping dev %d", dev);

	if (err == 0)
		chardev->topOps.stop(dev);
	else
		pr_err("Could not copy data from user. err=%d", err);

	return err;
}

int mipi_chardev_set_mux_ioctl( unsigned long arg, struct mipi_chardev* chardev )
{
	int err = 0;
	struct mipicsi_top_mux mux;
	/* copy from user */
	err = copy_from_user(&mux, (void __user *)arg, sizeof(mux));

	pr_debug("mipi_chardev_ioctl: setting mux %d to %d", mux.source, mux.sink);

	if (err == 0)
		chardev->topOps.set_mux(&mux);
	else
		pr_err("Could not copy data from user. err=%d", err);

	return err;
}

int mipi_chardev_disable_mux_ioctl( unsigned long arg,
				    struct mipi_chardev* chardev )
{
	int err = 0;
	struct mipicsi_top_mux mux;
	/* copy from user */
	err = copy_from_user(&mux, (void __user *)arg, sizeof(mux));

	pr_debug("mipi_chardev_ioctl: disabling mux %d to %d", mux.source, mux.sink);

	if (err == 0)
		chardev->topOps.disable_mux(&mux);
	else
		pr_err("Could not copy data from user. err=%d", err);

	return err;
}

int mipi_chardev_get_mux_ioctl( unsigned long arg, struct mipi_chardev* chardev )
{
	int err = 0;
	struct mipicsi_top_mux_data mux_data;

	/* copy from user */
	err = copy_from_user(&mux_data, (void __user *)arg, sizeof(mux_data));
	pr_debug("mipi_chardev_ioctl: getting mux settings");
	if (err == 0)
	{
		chardev->topOps.get_mux(&mux_data);

		err = copy_to_user((void __user *)arg, &mux_data,
				   sizeof(mux_data));

		if( err != 0 )
			pr_err("Could not copy data to user. err=%d", err);
	}
	else
		pr_err("Could not copy data from user. err=%d", err);

	return err;
}

int mipi_chardev_get_mux_status_ioctl( unsigned long arg, 
				       struct mipi_chardev* chardev )
{
	int err = 0;
	struct mipicsi_top_mux mux;

	/* copy from user */
	err = copy_from_user(&mux, (void __user *)arg, sizeof(mux));
	pr_debug("mipi_chardev_ioctl: getting mux settings");
	if (err == 0)
	{
		chardev->topOps.get_mux_status(&mux);

		err = copy_to_user((void __user *)arg, &mux,
				   sizeof(mux));
	}
	else
		pr_err("Could not copy data from user. err=%d", err);

	return err;
}

int mipi_chardev_vpg_ioctl( unsigned long arg, struct mipi_chardev* chardev )
{
	int err = 0;
	struct mipicsi_top_vpg vpg;
	/* copy from user */
	err = copy_from_user(&vpg, (void __user *)arg, sizeof(vpg));

	pr_debug("mipi_chardev_ioctl: starting vpg %d", vpg.dev);

	if (err == 0)
		chardev->topOps.vpg(&vpg);
	else
		pr_err("Could not copy data from user. err=%d", err);

	return err;
}

int mipi_chardev_reset_ioctl( unsigned long arg, struct mipi_chardev* chardev )
{
	int err = 0;
	enum mipicsi_top_dev dev;
	/* copy from user */
	err = copy_from_user(&dev, (void __user *)arg, sizeof(dev));

	pr_debug("mipi_chardev_ioctl: resetting dev %d", dev);

	if (err == 0)
		chardev->topOps.reset(dev);
	else
		pr_err("Could not copy data from user. err=%d", err);

	return err;
}


int mipi_chardev_reset_all_ioctl (unsigned long arg,
				  struct mipi_chardev* chardev)
{
	pr_debug("mipi_chardev_ioctl: resetting all devices");

	chardev->topOps.reset_all();

	return 0;
}

int mipi_chardev_get_irq_st_ioctl(unsigned long arg,
				  struct mipi_chardev *chardev)
{
	int err = 0;
	struct mipi_device_irq_st *pdev_status;
	struct mipi_device_irq_st dev_status;
	struct mipi_host_irq_st *phost_status;
	struct mipi_host_irq_st host_status;

	err = copy_from_user(&dev_status, (void __user *)arg,
			     sizeof(dev_status));
	err = copy_from_user(&host_status, (void __user *)arg,
			     sizeof(host_status));

	pr_debug("mipi_chardev_ioctl: %s dev_status=%d\n", __func__,
		dev_status.dev);
	pr_debug("mipi_chardev_ioctl: %s host_status=%d\n", __func__,
		host_status.dev);
	if (err == 0) {
		switch (dev_status.dev) {
		case MIPI_TX0:
		case MIPI_TX1:
			err = chardev->topOps.get_device_irq_status(
				dev_status.dev,
				&dev_status);
			if (err != 0)
				err = copy_to_user((void __user *)arg,
						   &dev_status,
						   sizeof(dev_status));
			break;
		case MIPI_RX0:
		case MIPI_RX1:
		case MIPI_RX2:
			err = chardev->topOps.get_host_irq_status(
				dev_status.dev,
				&host_status);
			if (err != 0)
				err = copy_to_user((void __user *)arg,
						   &host_status,
						   sizeof(host_status));
			break;
		default:
			pr_err("Invalid mipi device given for irq status\n");
			return -EINVAL;
		}
	} else
		pr_err("Could not copy data from user. err=%d", err);
	return err;
}

int mipi_chardev_set_irq_mask_ioctl(unsigned long arg,
					 struct mipi_chardev *chardev)
{
	int err = 0;
	struct mipi_device_irq_mask dev_mask;
	struct mipi_host_irq_mask host_mask;
	/* copy from user */
	err = copy_from_user(&dev_mask, (void __user *)arg,
			     sizeof(dev_mask));
	err = copy_from_user(&host_mask, (void __user *)arg,
			     sizeof(host_mask));

	pr_debug("mipi_chardev_ioctl: %s dev=%d", __func__, dev_mask.dev);
	if (err == 0) {
		switch (dev_mask.dev) {
		case MIPI_TX0:
		case MIPI_TX1:
			err = chardev->topOps.set_device_irq_mask(
				dev_mask.dev,
				&dev_mask);
			break;
		case MIPI_RX0:
		case MIPI_RX1:
		case MIPI_RX2:
			err = chardev->topOps.set_host_irq_mask(
				host_mask.dev,
				&host_mask);
			break;
		default:
			pr_err("Invalid mipi device given for irq status\n");
			return -EINVAL;
		}
	} else
		pr_err("Could not copy data from user. err=%d", err);
	return err;
}

int mipi_chardev_force_irq_ioctl(unsigned long arg,
				 struct mipi_chardev *chardev)
{
	int err = 0;
	struct mipi_device_irq_mask dev_mask;
	struct mipi_host_irq_mask host_mask;
	/* copy from user */
	err = copy_from_user(&dev_mask, (void __user *)arg,
			     sizeof(dev_mask));
	err = copy_from_user(&host_mask, (void __user *)arg,
			     sizeof(host_mask));

	pr_debug("mipi_chardev_ioctl: %s dev=%d", __func__, dev_mask.dev);
	if (err == 0) {
		switch (dev_mask.dev) {
		case MIPI_TX0:
		case MIPI_TX1:
			err = chardev->topOps.force_device_irq(
				dev_mask.dev,
				&dev_mask);
			break;
		case MIPI_RX0:
		case MIPI_RX1:
		case MIPI_RX2:
			err = chardev->topOps.force_host_irq(
				host_mask.dev,
				&host_mask);
			break;
		default:
			pr_err("Invalid mipi device given for irq status\n");
			return -EINVAL;
		}
	} else
		pr_err("Could not copy data from user. err=%d", err);
	return err;
}

const struct mipi_ioctl mipi_ioctl_tbl[] = {
	{MIPI_TOP_START, mipi_chardev_start_ioctl},
	{MIPI_TOP_STOP, mipi_chardev_stop_ioctl},
	{MIPI_TOP_S_MUX, mipi_chardev_set_mux_ioctl},
	{MIPI_TOP_DIS_MUX, mipi_chardev_disable_mux_ioctl},
	{MIPI_TOP_G_MUX, mipi_chardev_get_mux_ioctl},
	{MIPI_TOP_G_MUX_STATUS, mipi_chardev_get_mux_status_ioctl},
	{MIPI_TOP_VPG, mipi_chardev_vpg_ioctl},
	{MIPI_TOP_S_REG, mipi_chardev_write_reg_ioctl},
	{MIPI_TOP_G_REG, mipi_chardev_read_reg_ioctl},
	{MIPI_TOP_RESET, mipi_chardev_reset_ioctl},
	{MIPI_TOP_RESET_ALL, mipi_chardev_reset_all_ioctl},
	{MIPI_DEV_G_INT_ST, mipi_chardev_get_irq_st_ioctl},
	{MIPI_DEV_S_INT_MASK, mipi_chardev_set_irq_mask_ioctl},
	{MIPI_DEV_S_INT_FORCE, mipi_chardev_force_irq_ioctl},
	{MIPI_HOST_G_INT_ST, mipi_chardev_get_irq_st_ioctl},
	{MIPI_HOST_S_INT_MASK, mipi_chardev_set_irq_mask_ioctl},
	{MIPI_HOST_S_INT_FORCE, mipi_chardev_force_irq_ioctl},
};


long mipi_chardev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int err = 0, i;
	bool found = false;
	struct mipi_chardev* chardev;
	
	pr_debug("Inside mipi_chardev_ioctl\n");
	/*
	 * the direction is a bitmask, and VERIFY_WRITE catches R/W
	 * transfers. `Type' is user-oriented, while
	 * access_ok is kernel-oriented, so the concept of "read" and
	 * "write" is reversed
	 */
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg,
				 _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err =  !access_ok(VERIFY_READ, (void __user *)arg,
				  _IOC_SIZE(cmd));
	if (err) {
		pr_err("mipi_chardev_ioctl access error!\n");
		return -EFAULT;
	}

	/*
	 * extract the type and number bitfields, and don't decode
	 * wrong cmds: return ENOTTY (inappropriate ioctl) before access_ok(  )
	 */
	if (_IOC_TYPE(cmd) == MIPIBRIDGE_IOC_TOP_MAGIC &&
	    _IOC_NR(cmd) <= MIPI_TOP_MAX) {
		pr_debug("mipi_chardev_ioctl top call\n");
		chardev = mipi_char_devices[0];
		for (i = 0;
		     i < sizeof(mipi_ioctl_tbl)/sizeof(struct mipi_ioctl); i++){
			if (mipi_ioctl_tbl[i].cmd == cmd) {
				err = mipi_ioctl_tbl[i].func(arg, chardev);
				found = true;
				break;
			}
		}
		if (!found)
			pr_warn("Unrecognized mipi ioctl");
	} else {
		pr_warn("mipibridge_ioctl: unknown ioctl %c, dir=%d, #%d (0x%08x)\n",
			_IOC_TYPE(cmd), _IOC_DIR(cmd), _IOC_NR(cmd), cmd);
		return -ENOTTY;
	}

	return err;
}

const struct file_operations mipi_chardev_base = {
	.open = mipi_chardev_open,
	.unlocked_ioctl = mipi_chardev_ioctl,
	.release = mipi_chardev_close
};

int mipi_chardev_init(struct mipi_chardev *chardev)
{
	int err, index = get_available_index(mipi_char_devices);
	dev_t devnode = MKDEV(char_major_dev, char_minor_device + index);
	struct device *dev = chardev->mipidev->dev;
	int major;

	dev_info(dev, "Initialize chardev %s index %d\n",
		 chardev->deviceName, index);

	if (alloc_chrdev_region(&devnode,
			char_minor_device,
			1, chardev->deviceName)) {
		dev_info(dev, "Failed to alloc_chrdev_region %d\n",
			 err);
		return -ENODEV;
	}
	cdev_init(&chardev->cdev, &mipi_chardev_base);
	chardev->cdev.owner = chardev->owner;
	chardev->cdev.ops = &mipi_chardev_base;
	err = cdev_add(&chardev->cdev, devnode, 1);
	major = MAJOR(devnode);
	/* Fail gracefully if need be */
	if (err)
		dev_err(dev, "Error %d adding mipi char dev\n",
			 err);
	/* Register the device class */
	chardev->chardevClass = class_create(chardev->owner, CLASS_NAME);
	if (IS_ERR(chardev->chardevClass)) {
		unregister_chrdev(major, chardev->deviceName);
		dev_err(dev, "Failed to register device class\n");
		return PTR_ERR(chardev->chardevClass);
	}

	/* Register the device driver and create device node*/
	chardev->chardev = device_create(chardev->chardevClass, NULL,
					  devnode, NULL,
					  chardev->deviceName);
	if (IS_ERR(chardev->chardev)) {
		class_destroy(chardev->chardevClass);
		unregister_chrdev(major, chardev->deviceName);
		dev_err(dev, "mipi_chardev: Failed to create the device\n");
		return PTR_ERR(chardev->chardev);
	}
	mipi_char_devices[index] = chardev;
	return err;
}

void mipi_chardev_release(struct mipi_chardev *chardev)
{
	/* remove the device */
	device_destroy(chardev->chardevClass, chardev->cdev.dev);
	class_unregister(chardev->chardevClass);
	class_destroy(chardev->chardevClass);
	unregister_chrdev_region(chardev->cdev.dev, 1);
	cdev_del(&chardev->cdev);
}

MODULE_LICENSE("GPL");
