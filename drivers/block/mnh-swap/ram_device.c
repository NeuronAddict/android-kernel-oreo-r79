/*
 * ram_device.c - 
 * 
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program;
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Author: 
 */

#include <linux/types.h>
#include <linux/vmalloc.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/io.h>
#include "ram_device.h"
#include "partition.h"

#define RB_DEVICE_SIZE 524288 /* sectors */


static u8 *dev_data;

int ramdevice_init(void)
{
	dev_data = ioremap_nocache(0x80000000, RB_DEVICE_SIZE * RB_SECTOR_SIZE);
	if (dev_data == NULL)
        {
		printk("mnh_blk: %s ioremap failed \n", __func__);		
		return -ENOMEM;
	}

	/* Setup its partition table */
	copy_mbr(dev_data);

	return RB_DEVICE_SIZE;
}

void ramdevice_cleanup(void)
{
	iounmap(dev_data);
}

void ramdevice_write(sector_t sector_off, u8 *buffer, unsigned int sectors)
{
	memcpy(dev_data + sector_off * RB_SECTOR_SIZE, buffer,
		sectors * RB_SECTOR_SIZE);
}
void ramdevice_read(sector_t sector_off, u8 *buffer, unsigned int sectors)
{
	memcpy(buffer, dev_data + sector_off * RB_SECTOR_SIZE,
		sectors * RB_SECTOR_SIZE);
}
