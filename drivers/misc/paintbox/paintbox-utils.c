/*
 * Paintbox programmable IPU Utilities
 *
 * Copyright (C) 2016 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include "paintbox-common.h"
#include "paintbox-regs.h"
#include "paintbox-utils.h"


#define MIN_RAM_ACCESS_SLEEP       10  /* us */
#define MAX_RAM_ACCESS_SLEEP       100 /* us */
#define MAX_MEMORY_ACCESS_ATTEMPTS 5

/* The maximum SRAM transfer width is width of the LBP SRAM transfer width. */
#define MAX_SRAM_TRANSFER_WIDTH    (LBP_DATA_REG_COUNT * sizeof(uint32_t))

void dump_registers(struct paintbox_data *pb, void __iomem *reg_start,
		size_t reg_count, const char *msg) {
	unsigned int i;

	dev_info(&pb->pdev->dev, "%s: reg addr %p len %lu\n", msg, reg_start,
			reg_count);
	for (i = 0; i < reg_count; i++)
		dev_info(&pb->pdev->dev, "0x%08x\n", readl(reg_start + i *
				sizeof(uint32_t)));
}

int poll_memory_transfer_complete(void __iomem *ram_ctrl_reg)
{
	uint32_t ram_ctrl;
	unsigned int attempts = 0;

	do {
		ram_ctrl = readl(ram_ctrl_reg);

		/* This works because the RUN bit is the same in the LBP and STP
		 * ram control register.
		 */
		if ((ram_ctrl & STP_RAM_RUN) == 0)
			break;

		usleep_range(MIN_RAM_ACCESS_SLEEP, MAX_RAM_ACCESS_SLEEP);

		if (attempts++ >= MAX_MEMORY_ACCESS_ATTEMPTS)
			return -ETIMEDOUT;
	} while (1);

	return 0;
}

int alloc_and_copy_from_user(struct paintbox_data *pb, uint8_t **buf,
			const void __user *user_buf, size_t len_bytes)
{
	*buf = kmalloc(len_bytes, GFP_KERNEL);
	if (!*buf) {
		dev_err(&pb->pdev->dev, "%s: allocation failure\n", __func__);
		return -ENOMEM;
	}

	if (copy_from_user(*buf, user_buf, len_bytes)) {
		kfree(*buf);
		return -EFAULT;
	}

	return 0;
}

static void write_ram_data(struct paintbox_data *pb, const uint8_t *buf,
		void __iomem *data_reg, unsigned int reg_count)
{
	unsigned int i, j;

	for (i = 0, j = reg_count - 1; i < reg_count * sizeof(uint32_t);
			i += sizeof(uint32_t), j--) {
		uint32_t data;

		/* Swap the byte order when loading the registers */
		data = ((uint32_t)buf[i]) << 24;
		data |= ((uint32_t)buf[i + 1]) << 16;
		data |= ((uint32_t)buf[i + 2]) << 8;
		data |= (uint32_t)buf[i + 3];

		writel(data, data_reg + j * sizeof(uint32_t));
	}

	DUMP_REGISTERS(pb, data_reg, reg_count, __func__);
}

static void read_ram_data(struct paintbox_data *pb, uint8_t *buf,
		void __iomem *data_reg, unsigned int reg_count)
{
	unsigned int i, j;

	DUMP_REGISTERS(pb, data_reg, reg_count, __func__);

	for (i = 0, j = reg_count - 1; i < reg_count * sizeof(uint32_t);
			i += sizeof(uint32_t), j--) {
		uint32_t data = readl(data_reg + j * sizeof(uint32_t));

		/* Swap the byte order when copying from the registers */
		buf[i] = (uint8_t)((data & 0xFF000000) >> 24);
		buf[i + 1] = (uint8_t)((data & 0x00FF0000) >> 16);
		buf[i + 2] = (uint8_t)((data & 0x0000FF00) >> 8);
		buf[i + 3] = (uint8_t)(data & 0x000000FF);
	}
}

int read_data_tail(struct paintbox_data *pb, uint8_t *buf, size_t len_bytes,
		uint16_t sram_transfer_addr, uint32_t ram_ctrl_mask,
		void __iomem *reg_base, size_t reg_count)
{
	uint8_t transfer_buf[MAX_SRAM_TRANSFER_WIDTH];
	uint32_t ram_ctrl;
	int ret;

	/* Read out one full sram transfer width of RAM data into the transfer
	 * buffer.  Later we will copy out the portion we are interested in.
	 */
	ram_ctrl = COMMON_RAM_RUN | ram_ctrl_mask |
			((sram_transfer_addr & COMMON_RAM_ADDR) <<
			COMMON_RAM_ADDR_SHIFT);

	writel(ram_ctrl, reg_base + COMMON_RAM_CTRL);

	ret = poll_memory_transfer_complete(reg_base + COMMON_RAM_CTRL);
	if (ret < 0) {
		dev_err(&pb->pdev->dev, "%s: read timeout\n", __func__);
		return ret;
	}

	read_ram_data(pb, transfer_buf, reg_base + COMMON_RAM_DATA_START,
			reg_count);

	/* Copy the the portion of the transfer buffer that we are interested in
	 * to the user buffer.
	 */
	memcpy(buf, transfer_buf, len_bytes);

	return ret;
}

int write_data_tail(struct paintbox_data *pb, const uint8_t *buf,
		size_t len_bytes, uint16_t sram_transfer_addr,
		uint32_t ram_ctrl_mask, void __iomem *reg_base,
		size_t reg_count)
{
	uint8_t transfer_buf[MAX_SRAM_TRANSFER_WIDTH];
	uint32_t ram_ctrl;
	int ret;

	/* Read out one full sram transfer width of RAM data into the transfer
	 * buffer.  Later we will overwrite the portion that we are interested
	 * in with data from the user buffer.
	 */
	ram_ctrl = COMMON_RAM_RUN | ram_ctrl_mask |
			((sram_transfer_addr & COMMON_RAM_ADDR) <<
			COMMON_RAM_ADDR_SHIFT);

	writel(ram_ctrl, reg_base + COMMON_RAM_CTRL);

	ret = poll_memory_transfer_complete(reg_base + COMMON_RAM_CTRL);
	if (ret < 0) {
		dev_err(&pb->pdev->dev, "%s: read timeout\n", __func__);
		return ret;
	}

	read_ram_data(pb, transfer_buf, reg_base + COMMON_RAM_DATA_START,
			reg_count);

	/* Copy the tail bytes of the user buffer into the transfer buffer */
	memcpy(transfer_buf, buf, len_bytes);

	ram_ctrl = COMMON_RAM_RUN | COMMON_RAM_WRITE | ram_ctrl_mask |
			((sram_transfer_addr & COMMON_RAM_ADDR) <<
			COMMON_RAM_ADDR_SHIFT);

	/* Write the merged buffer into the SRAM */
	write_ram_data(pb, transfer_buf, reg_base + COMMON_RAM_DATA_START,
			reg_count);

	writel(ram_ctrl, reg_base + COMMON_RAM_CTRL);

	ret = poll_memory_transfer_complete(reg_base + COMMON_RAM_CTRL);
	if (ret < 0)
		dev_err(&pb->pdev->dev, "%s: write timeout\n", __func__);

	return ret;
}

int write_data_common(struct paintbox_data *pb, const void __user *user_buf,
		size_t len_bytes, uint16_t sram_transfer_addr,
		uint32_t ram_ctrl_mask, void __iomem *reg_base,
		unsigned int reg_count)
{
	size_t sram_transfer_width = reg_count * sizeof(uint32_t);
	uint16_t sram_transfer_offset = 0;
	size_t bytes_transferred = 0;
	uint32_t ram_ctrl;
	uint8_t *buf;
	int ret = 0;

	ret = alloc_and_copy_from_user(pb, &buf, user_buf, len_bytes);
	if (ret < 0)
		return ret;

	/* TODO(ahampson): Add support for unaligned writes. */

	/* If the transfer data length is greater than or equal to the SRAM
	 * transfer width then transfer the data in full width transfers.
	 */
	while (bytes_transferred < len_bytes &&
			len_bytes - bytes_transferred >= sram_transfer_width) {
		ram_ctrl = COMMON_RAM_RUN | COMMON_RAM_WRITE | ram_ctrl_mask |
				(((sram_transfer_addr + sram_transfer_offset) &
				COMMON_RAM_ADDR) << COMMON_RAM_ADDR_SHIFT);

		/* Write one SRAM transfer width of the transfer buffer. */
		write_ram_data(pb, buf + bytes_transferred,
				reg_base + COMMON_RAM_DATA_START, reg_count);
		bytes_transferred += sram_transfer_width;
		sram_transfer_offset++;

		dev_dbg(&pb->pdev->dev, "%s: ram_ctrl 0x%08x\n", __func__,
				readl(reg_base + COMMON_RAM_CTRL));

		writel(ram_ctrl, reg_base + COMMON_RAM_CTRL);

		ret = poll_memory_transfer_complete(reg_base + COMMON_RAM_CTRL);
		if (ret < 0) {
			dev_err(&pb->pdev->dev, "%s: write timeout\n",
					__func__);
			goto exit;
		}
	}

	/* Handle any remaining bytes that are shorter than the SRAM transfer
	 * width.
	 */
	if (len_bytes - bytes_transferred > 0) {
		ret = write_data_tail(pb, buf + bytes_transferred,
				len_bytes - bytes_transferred,
				sram_transfer_addr + sram_transfer_offset,
				ram_ctrl_mask, reg_base, reg_count);
	}

exit:
	kfree(buf);

	return ret;
}

int read_data_common(struct paintbox_data *pb, void __user *user_buf,
		size_t len_bytes, uint16_t sram_transfer_addr,
		uint32_t ram_ctrl_mask, void __iomem *reg_base,
			unsigned int reg_count)
{
	size_t sram_transfer_width = reg_count * sizeof(uint32_t);
	uint16_t sram_transfer_offset = 0;
	size_t bytes_transferred = 0;
	uint32_t ram_ctrl;
	uint8_t *buf;
	int ret = 0;

	buf = kmalloc(len_bytes, GFP_KERNEL);
	if (!buf) {
		dev_err(&pb->pdev->dev, "%s: allocation failure\n", __func__);
		return -ENOMEM;
	}

	/* TODO(ahampson): Add support for unaligned writes. */

	/* If the transfer data length is greater than or equal to the SRAM
	 * transfer width then transfer the data in full width transfers.
	 */
	while (bytes_transferred < len_bytes &&
			len_bytes - bytes_transferred >= sram_transfer_width) {
		ram_ctrl = COMMON_RAM_RUN | ram_ctrl_mask |
				(((sram_transfer_addr + sram_transfer_offset) &
				COMMON_RAM_ADDR) << COMMON_RAM_ADDR_SHIFT);

		dev_dbg(&pb->pdev->dev, "%s: ram_ctrl 0x%08x\n", __func__,
				readl(reg_base + COMMON_RAM_CTRL));

		writel(ram_ctrl, reg_base + COMMON_RAM_CTRL);

		ret = poll_memory_transfer_complete(reg_base + COMMON_RAM_CTRL);
		if (ret < 0)
			goto exit;

		read_ram_data(pb, buf + bytes_transferred,
				reg_base + COMMON_RAM_DATA_START, reg_count);

		bytes_transferred += sram_transfer_width;
		sram_transfer_offset++;
	}

	/* Handle any remaining bytes that are shorter than the SRAM transfer
	 * width.
	 */
	if (len_bytes - bytes_transferred > 0) {
		ret = read_data_tail(pb, buf + bytes_transferred,
				len_bytes - bytes_transferred,
				sram_transfer_addr + sram_transfer_offset,
				ram_ctrl_mask, reg_base, reg_count);
		if (ret < 0)
			goto exit;
	}

	if (copy_to_user((void __user *)user_buf, buf, len_bytes))
		ret = -EFAULT;

exit:
	kfree(buf);

	return ret;
}
