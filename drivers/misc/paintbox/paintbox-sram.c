/*
 * Paintbox programmable IPU SRAM Support
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
#include "paintbox-sram.h"


#define MIN_RAM_ACCESS_SLEEP       10  /* us */
#define MAX_RAM_ACCESS_SLEEP       100 /* us */
#define MAX_MEMORY_ACCESS_ATTEMPTS 5

/* The maximum SRAM transfer width is width of the LBP SRAM transfer width. */
#define MAX_SRAM_TRANSFER_WIDTH    (LBP_DATA_REG_COUNT * IPU_REG_WIDTH)

#ifdef DEBUG
void dump_registers(struct paintbox_data *pb, void __iomem *reg_start,
		size_t reg_count, const char *msg) {
	unsigned int i;

	dev_info(&pb->pdev->dev, "%s: reg addr %p len %lu\n", msg, reg_start,
			reg_count);
	for (i = 0; i < reg_count; i++)
		dev_info(&pb->pdev->dev, "0x%08x\n", readl(reg_start + i *
				IPU_REG_WIDTH));
}

#define DUMP_REGISTERS(pb, reg, reg_count, msg)	\
do {							\
	dump_registers(pb, reg, reg_count, msg);	\
} while (0)
#else
#define DUMP_REGISTERS(pb, reg, reg_count, msg)	\
do { } while (0)
#endif


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

void write_ram_data_registers(struct paintbox_data *pb, const uint8_t *buf,
		void __iomem *data_reg, unsigned int reg_count)
{
	unsigned int i;

	for (i = 0; i < reg_count * IPU_REG_WIDTH; i += IPU_REG_WIDTH) {
		uint32_t data;

		data = ((uint32_t)buf[i + 0]) << 0;
		data |= ((uint32_t)buf[i + 1]) << 8;
		data |= ((uint32_t)buf[i + 2]) << 16;
		data |= ((uint32_t)buf[i + 3]) << 24;

		writel(data, data_reg + i);
	}

	DUMP_REGISTERS(pb, data_reg, reg_count, __func__);
}

void write_ram_data_registers_swapped(struct paintbox_data *pb,
		const uint8_t *buf, void __iomem *data_reg,
		unsigned int reg_count)
{
	unsigned int i, j;

	for (i = 0, j = reg_count - 1; i < reg_count * IPU_REG_WIDTH;
			i += IPU_REG_WIDTH, j--) {
		uint32_t data;

		/* Swap the byte order when loading the registers */
		data = ((uint32_t)buf[i + 0]) << 24;
		data |= ((uint32_t)buf[i + 1]) << 16;
		data |= ((uint32_t)buf[i + 2]) << 8;
		data |= ((uint32_t)buf[i + 3]) << 0;

		writel(data, data_reg + j * IPU_REG_WIDTH);
	}

	DUMP_REGISTERS(pb, data_reg, reg_count, __func__);
}

void read_ram_data_registers(struct paintbox_data *pb, uint8_t *buf,
		void __iomem *data_reg, unsigned int reg_count)
{
	unsigned int i;

	DUMP_REGISTERS(pb, data_reg, reg_count, __func__);

	for (i = 0; i < reg_count * IPU_REG_WIDTH; i += IPU_REG_WIDTH) {
		uint32_t data = readl(data_reg + i);

		buf[i + 0] = (uint8_t)((data >> 0) & 0xff);
		buf[i + 1] = (uint8_t)((data >> 8) & 0xff);
		buf[i + 2] = (uint8_t)((data >> 16) & 0xff);
		buf[i + 3] = (uint8_t)((data >> 24) & 0xff);
	}
}

void read_ram_data_registers_swapped(struct paintbox_data *pb, uint8_t *buf,
		void __iomem *data_reg, unsigned int reg_count)
{
	unsigned int i, j;

	DUMP_REGISTERS(pb, data_reg, reg_count, __func__);

	for (i = 0, j = reg_count - 1; i < reg_count * IPU_REG_WIDTH;
			i += IPU_REG_WIDTH, j--) {
		uint32_t data = readl(data_reg + j * IPU_REG_WIDTH);

		/* Swap the byte order when copying from the registers */
		buf[i + 0] = (uint8_t)((data >> 24) & 0xff);
		buf[i + 1] = (uint8_t)((data >> 16) & 0xff);
		buf[i + 2] = (uint8_t)((data >> 8) & 0xff);
		buf[i + 3] = (uint8_t)((data >> 0) & 0xff);
	}
}

static int sram_read_word_partial(struct paintbox_data *pb, uint8_t *buf,
		uint32_t sram_word_addr, unsigned int byte_offset_in_word,
		size_t len_bytes, uint32_t ram_ctrl_mask,
		void __iomem *reg_base, size_t reg_count, bool swap_data)
{
	uint8_t transfer_buf[MAX_SRAM_TRANSFER_WIDTH];
	uint32_t ram_ctrl;
	int ret;

	/* Read out one full sram transfer width of RAM data into the transfer
	 * buffer.  Later we will copy out the portion we are interested in.
	 */
	ram_ctrl = COMMON_RAM_RUN | ram_ctrl_mask |
			((sram_word_addr & COMMON_RAM_ADDR) <<
			COMMON_RAM_ADDR_SHIFT);

	writel(ram_ctrl, reg_base + COMMON_RAM_CTRL);

	ret = poll_memory_transfer_complete(reg_base + COMMON_RAM_CTRL);
	if (ret < 0) {
		dev_err(&pb->pdev->dev, "%s: read timeout\n", __func__);
		return ret;
	}

	if (swap_data)
		read_ram_data_registers_swapped(pb, transfer_buf, reg_base +
				COMMON_RAM_DATA_START, reg_count);
	else
		read_ram_data_registers(pb, transfer_buf, reg_base +
				COMMON_RAM_DATA_START, reg_count);

	/* The transfer buffer contains a full word of data but we are only
	 * interested in len_bytes starting at byte_offset_in_word.
	 */
	memcpy(buf, transfer_buf + byte_offset_in_word, len_bytes);

	return ret;
}

static int sram_write_word_partial(struct paintbox_data *pb, const uint8_t *buf,
		uint32_t sram_word_addr, unsigned int byte_offset_in_word,
		size_t len_bytes, uint32_t ram_ctrl_mask,
		void __iomem *reg_base, size_t reg_count, bool swap_data)
{
	uint8_t transfer_buf[MAX_SRAM_TRANSFER_WIDTH];
	uint32_t ram_ctrl;
	int ret;

	/* Read out one word of data from the SRAM into the transfer buffer.
	 * Later we will overwrite the portion that we are interested in with
	 * data from the user buffer.
	 */
	ram_ctrl = COMMON_RAM_RUN | ram_ctrl_mask |
			((sram_word_addr & COMMON_RAM_ADDR) <<
			COMMON_RAM_ADDR_SHIFT);

	writel(ram_ctrl, reg_base + COMMON_RAM_CTRL);

	ret = poll_memory_transfer_complete(reg_base + COMMON_RAM_CTRL);
	if (ret < 0) {
		dev_err(&pb->pdev->dev, "%s: read timeout\n", __func__);
		return ret;
	}

	if (swap_data)
		read_ram_data_registers_swapped(pb, transfer_buf, reg_base +
				COMMON_RAM_DATA_START, reg_count);
	else
		read_ram_data_registers(pb, transfer_buf, reg_base +
				COMMON_RAM_DATA_START, reg_count);

	/* Merge the bytes from the user buffer into the transfer buffer. */
	memcpy(transfer_buf + byte_offset_in_word, buf, len_bytes);

	ram_ctrl = COMMON_RAM_RUN | COMMON_RAM_WRITE | ram_ctrl_mask |
			((sram_word_addr & COMMON_RAM_ADDR) <<
			COMMON_RAM_ADDR_SHIFT);

	/* Write the merged buffer into the SRAM */
	if (swap_data)
		write_ram_data_registers_swapped(pb, transfer_buf, reg_base +
				COMMON_RAM_DATA_START, reg_count);
	else
		write_ram_data_registers(pb, transfer_buf, reg_base +
				COMMON_RAM_DATA_START, reg_count);

	writel(ram_ctrl, reg_base + COMMON_RAM_CTRL);

	ret = poll_memory_transfer_complete(reg_base + COMMON_RAM_CTRL);
	if (ret < 0)
		dev_err(&pb->pdev->dev, "%s: write timeout\n", __func__);

	return ret;
}

static int sram_write_word(struct paintbox_data *pb, const uint8_t *buf,
		uint32_t sram_word_addr, uint32_t ram_ctrl_mask,
		void __iomem *reg_base, size_t reg_count, bool swap_data)
{
	uint32_t ram_ctrl;
	int ret;

	ram_ctrl = COMMON_RAM_RUN | COMMON_RAM_WRITE | ram_ctrl_mask |
			((sram_word_addr & COMMON_RAM_ADDR) <<
			COMMON_RAM_ADDR_SHIFT);

	if (swap_data)
		write_ram_data_registers_swapped(pb, buf, reg_base +
				COMMON_RAM_DATA_START, reg_count);
	else
		write_ram_data_registers(pb, buf, reg_base +
				COMMON_RAM_DATA_START, reg_count);

	dev_dbg(&pb->pdev->dev, "%s: ram_ctrl 0x%08x\n", __func__,
			readl(reg_base + COMMON_RAM_CTRL));

	writel(ram_ctrl, reg_base + COMMON_RAM_CTRL);

	ret = poll_memory_transfer_complete(reg_base + COMMON_RAM_CTRL);
	if (ret < 0) {
		dev_err(&pb->pdev->dev, "%s: write timeout\n", __func__);
		return ret;
	}

	return 0;
}

static int sram_read_word(struct paintbox_data *pb, uint8_t *buf,
		uint32_t sram_word_addr, uint32_t ram_ctrl_mask,
		void __iomem *reg_base, size_t reg_count, bool swap_data)
{
	uint32_t ram_ctrl;
	int ret;

	ram_ctrl = COMMON_RAM_RUN | ram_ctrl_mask | ((sram_word_addr &
			COMMON_RAM_ADDR) << COMMON_RAM_ADDR_SHIFT);

	dev_dbg(&pb->pdev->dev, "%s: ram_ctrl 0x%08x\n", __func__,
			readl(reg_base + COMMON_RAM_CTRL));

	writel(ram_ctrl, reg_base + COMMON_RAM_CTRL);

	ret = poll_memory_transfer_complete(reg_base + COMMON_RAM_CTRL);
	if (ret < 0)
		return ret;

	if (swap_data)
		read_ram_data_registers_swapped(pb, buf, reg_base +
				COMMON_RAM_DATA_START, reg_count);
	else
		read_ram_data_registers(pb, buf, reg_base +
				COMMON_RAM_DATA_START, reg_count);

	return 0;
}

int sram_write_buffer(struct paintbox_data *pb, const uint8_t *buf,
		uint32_t sram_byte_addr, size_t len_bytes,
		uint32_t ram_ctrl_mask, void __iomem *reg_base,
		unsigned int reg_count)
{
	size_t sram_word_bytes = reg_count * IPU_REG_WIDTH;
	size_t bytes_remaining = len_bytes;
	size_t bytes_written = 0;
	unsigned int byte_offset_in_word;
	uint32_t sram_word_addr;
	bool swap_data;
	int ret;

	/* TODO(ahampson):  The assembler generates the pISA in the byte order
	 * expected by the DV.  In order for the pISA to be used by the hardware
	 * the instruction buffer needs to be byte swapped.  This will
	 * eventually be fixed in the assembler.  b/30316979
	 */
	swap_data = ((ram_ctrl_mask & STP_RAM_TARG_MASK) >>
			STP_RAM_TARG_SHIFT) == STP_RAM_TARG_INST_RAM;

	sram_word_addr = sram_byte_addr / sram_word_bytes;
	byte_offset_in_word = sram_byte_addr % sram_word_bytes;

	/* If the transfer does not start at the beginning of a word then it
	 * will require special handling before we can get to the main body of
	 * the transfer.
	 */
	if (byte_offset_in_word > 0) {
		size_t short_write_len = min(sram_word_bytes -
				byte_offset_in_word, len_bytes);
		ret = sram_write_word_partial(pb, buf, sram_word_addr,
				byte_offset_in_word, short_write_len,
				ram_ctrl_mask, reg_base, reg_count, swap_data);
		if (ret < 0)
			return ret;

		bytes_remaining -= short_write_len;
		bytes_written += short_write_len;
		sram_word_addr++;
	}

	/* If the transfer data length is greater than or equal to the SRAM
	 * word then transfer the data in full word transfers.
	 */
	while (bytes_remaining && bytes_remaining >= sram_word_bytes) {
		ret = sram_write_word(pb, buf + bytes_written, sram_word_addr,
				ram_ctrl_mask, reg_base, reg_count, swap_data);
		if (ret < 0)
			return ret;

		bytes_remaining -= sram_word_bytes;
		bytes_written += sram_word_bytes;
		sram_word_addr++;
	}

	/* Handle any remaining bytes that are shorter than an SRAM word. */
	if (bytes_remaining > 0) {
		ret = sram_write_word_partial(pb, buf, sram_word_addr, 0,
				bytes_remaining, ram_ctrl_mask, reg_base,
				reg_count, swap_data);
		if (ret < 0)
			return ret;
	}

	return 0;
}

int sram_write_user_buffer(struct paintbox_data *pb,
		const void __user *user_buf, uint32_t sram_byte_addr,
		size_t len_bytes, uint32_t ram_ctrl_mask,
		void __iomem *reg_base, unsigned int reg_count)
{
	uint8_t *buf;
	int ret;

	ret = alloc_and_copy_from_user(pb, &buf, user_buf, len_bytes);
	if (ret < 0)
		return ret;

	ret = sram_write_buffer(pb, buf, sram_byte_addr, len_bytes,
			ram_ctrl_mask, reg_base, reg_count);

	kfree(buf);

	return ret;
}

int sram_read_buffer(struct paintbox_data *pb, uint8_t *buf,
		uint32_t sram_byte_addr, size_t len_bytes,
		uint32_t ram_ctrl_mask, void __iomem *reg_base,
		unsigned int reg_count)
{
	size_t sram_word_bytes = reg_count * IPU_REG_WIDTH;
	size_t bytes_remaining = len_bytes;
	size_t bytes_read = 0;
	unsigned int byte_offset_in_word;
	uint32_t sram_word_addr;
	bool swap_data;
	int ret = 0;

	/* TODO(ahampson):  The assembler generates the pISA in the byte order
	 * expected by the DV.  In order for the pISA to be used by the hardware
	 * the instruction buffer needs to be byte swapped.  This will
	 * eventually be fixed in the assembler.  b/30316979
	 */
	swap_data = ((ram_ctrl_mask & STP_RAM_TARG_MASK) >>
			STP_RAM_TARG_SHIFT) == STP_RAM_TARG_INST_RAM;

	sram_word_addr = sram_byte_addr / sram_word_bytes;
	byte_offset_in_word = sram_byte_addr % sram_word_bytes;

	/* If the transfer does not start at the beginning of a word then it
	 * will require special handling before we can get to the main body of
	 * the transfer.
	 */
	if (byte_offset_in_word > 0) {
		size_t short_read_len = min(sram_word_bytes -
				byte_offset_in_word, len_bytes);
		ret = sram_read_word_partial(pb, buf, sram_word_addr,
				byte_offset_in_word, short_read_len,
				ram_ctrl_mask, reg_base, reg_count, swap_data);
		if (ret < 0)
			return ret;

		bytes_remaining -= short_read_len;
		bytes_read += short_read_len;
		sram_word_addr++;
	}

	/* If the transfer data length is greater than or equal to an SRAM word
	* then transfer the data in full word transfers.
	 */
	while (bytes_remaining && bytes_remaining >= sram_word_bytes) {
		ret = sram_read_word(pb, buf + bytes_read, sram_word_addr,
				ram_ctrl_mask, reg_base, reg_count, swap_data);
		if (ret < 0)
			return ret;

		bytes_remaining -= sram_word_bytes;
		bytes_read += sram_word_bytes;
		sram_word_addr++;
	}

	/* Handle any remaining bytes that are shorter than an SRAM word */
	if (bytes_remaining > 0) {
		ret = sram_read_word_partial(pb, buf, sram_word_addr, 0,
				bytes_remaining, ram_ctrl_mask, reg_base,
				reg_count, swap_data);
		if (ret < 0)
			return ret;
	}

	return 0;
}

int sram_read_user_buffer(struct paintbox_data *pb, void __user *user_buf,
		uint32_t sram_byte_addr, size_t len_bytes,
		uint32_t ram_ctrl_mask, void __iomem *reg_base,
		unsigned int reg_count)
{
	uint8_t *buf;
	int ret;

	buf = kmalloc(len_bytes, GFP_KERNEL);
	if (!buf) {
		dev_err(&pb->pdev->dev, "%s: allocation failure\n", __func__);
		return -ENOMEM;
	}

	ret = sram_read_buffer(pb, buf, sram_byte_addr, len_bytes,
		ram_ctrl_mask, reg_base, reg_count);
	if (ret < 0)
		goto exit;

	if (copy_to_user((void __user *)user_buf, buf, len_bytes))
		ret = -EFAULT;

exit:
	kfree(buf);

	return ret;
}
