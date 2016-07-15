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
	struct register_io regio;
	unsigned int reg_data = 0;

	/* copy from user */
	err = copy_from_user(&regio, (void __user *)arg, sizeof(regio));
	pr_debug("mipi_chardev_ioctl: writing dev %d reg 0x%x val 0x%x", regio.dev, regio.reg, regio.data);
	if (err == 0)
	{
		regio.data = chardev->topOps.readreg(
						regio.dev,
						regio.reg);
								
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
	struct register_io regio;
	/* copy from user */
	err = copy_from_user(&regio, (void __user *)arg, sizeof(regio));
	
	pr_debug("mipi_chardev_ioctl: writing dev %d reg 0x%x val 0x%x", regio.dev, regio.reg, regio.data);
		
	if (err == 0)
		chardev->topOps.writereg(
				regio.dev,
				regio.reg,
				regio.data
				);
	else
		pr_err("Could not copy data from user. err=%d", err);

	return err;
}

long mipi_chardev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int err = 0;
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
	if (_IOC_TYPE(cmd) == MIPIBRIDGE_IOC_HOST_MAGIC &&
	    _IOC_NR(cmd) < MIPI_HOST_MAX) {
		pr_debug("mipi_chardev_ioctl host call\n");

	} else if (_IOC_TYPE(cmd) == MIPIBRIDGE_IOC_DEV_MAGIC &&
		   _IOC_NR(cmd) < MIPI_DEV_MAX) {
		pr_debug("mipi_chardev_ioctl device call\n");

	} else if (_IOC_TYPE(cmd) == MIPIBRIDGE_IOC_TOP_MAGIC &&
		   _IOC_NR(cmd) < MIPI_TOP_MAX) {
		pr_debug("mipi_chardev_ioctl top call\n");
		chardev = mipi_char_devices[0];
		switch (cmd) {
			case MIPI_TOP_S_REG:
				err = mipi_chardev_write_reg_ioctl(arg, chardev);
				break;
			case MIPI_TOP_G_REG:
				err = mipi_chardev_read_reg_ioctl(arg, chardev);
				break;
			default:
				pr_warn("Unrecognized mipi ioctl");
		}
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
