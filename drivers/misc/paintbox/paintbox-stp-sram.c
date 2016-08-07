/*
 * STP SRAM support for the Paintbox programmable IPU
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

#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include "paintbox-common.h"
#include "paintbox-regs.h"
#include "paintbox-sram.h"
#include "paintbox-stp.h"
#include "paintbox-stp-sram.h"


static int validate_ram_transfer(struct paintbox_data *pb,
		struct paintbox_stp *stp, uint16_t sram_addr_bytes,
		uint8_t sram_target, size_t len_bytes)
{
	size_t sram_len_words;
	size_t sram_len_bytes;

	switch (sram_target) {
	case STP_RAM_TARGET_INSTRUCTION_RAM:
		sram_len_words = stp->inst_mem_size_in_words;
		break;
	case STP_RAM_TARGET_CONSTANT_RAM:
		sram_len_words = stp->const_mem_size_in_words;
		break;
	case STP_RAM_TARGET_SCALAR_RAM:
		sram_len_words = stp->scalar_mem_size_in_words;
		break;
	default:
		dev_err(&pb->pdev->dev,
				"%s: stp%u: invalid ram transfer type\n",
						__func__, stp->stp_id);
		return -EINVAL;
	};

	sram_len_bytes = sram_len_words * STP_WORD_WIDTH_BYTES;

	if (sram_addr_bytes + len_bytes > sram_len_bytes) {
		dev_err(&pb->pdev->dev,
				"%s: stp%u: memory transfer out of range %lu > "
				"%lu bytes\n", __func__, stp->stp_id, len_bytes,
				sram_len_bytes);
		return -ERANGE;
	}

	return 0;
}

static int set_ram_target(uint32_t *ram_ctrl, uint8_t ram_target)
{
	switch (ram_target) {
	case STP_RAM_TARGET_INSTRUCTION_RAM:
		*ram_ctrl |= (STP_RAM_TARG_INST_RAM << STP_RAM_TARG_SHIFT);
		break;
	case STP_RAM_TARGET_CONSTANT_RAM:
		*ram_ctrl |= (STP_RAM_TARG_CNST_RAM << STP_RAM_TARG_SHIFT);
		break;
	case STP_RAM_TARGET_SCALAR_RAM:
		*ram_ctrl |= (STP_RAM_TARG_DATA_RAM << STP_RAM_TARG_SHIFT);
		break;
	default:
		return -EINVAL;
	};

	return 0;
}

static inline uint32_t generate_vector_address(uint32_t lane_group_x,
		uint32_t lane_group_y, uint32_t sheet_slot, bool alu_registers)
{
	uint32_t target_base = alu_registers ? STP_RAM_TARG_ALU_IO_RF_0 :
			STP_RAM_TARG_ALU_IO_RAM_0;
	uint32_t ram_ctrl;

	/* RAM_ADDR field is composed of the bank y coordinate and the bank
	 * index
	 */
	ram_ctrl = ((lane_group_y << STP_RAM_ADDR_ROW_SHIFT) | sheet_slot) <<
			STP_RAM_ADDR_SHIFT;
	ram_ctrl |= (lane_group_x + target_base) << STP_RAM_TARG_SHIFT;

	return ram_ctrl;
}

static int validate_stp_vector_write_replicate_parameters(
		struct paintbox_data *pb, struct paintbox_stp *stp,
		struct sram_vector_replicate_write *req) {

	size_t sram_len_bytes, sram_start_bytes, num_sheet_slots;

	if (!req->buf) {
		dev_err(&pb->pdev->dev, "%s: invalid user buffer\n", __func__);
		return -EINVAL;
	}

	num_sheet_slots = req->write_halo_lanes ? stp->halo_mem_size_in_words :
			stp->vector_mem_size_in_words;

	sram_len_bytes = num_sheet_slots * STP_WORD_WIDTH_BYTES;
	sram_start_bytes = req->sheet_slot * STP_WORD_WIDTH_BYTES;

	if (req->len_bytes  > sram_len_bytes - sram_start_bytes) {
		dev_err(&pb->pdev->dev, "%s: write length too long %lu > %lu\n",
				__func__, req->len_bytes, sram_len_bytes -
				sram_start_bytes);
		return -ERANGE;
	}

	if (req->sheet_slot > num_sheet_slots) {
		dev_err(&pb->pdev->dev,
				"%s: sheet slot out of range %u > %lu\n",
				__func__, req->sheet_slot, num_sheet_slots);
		return -ERANGE;
	}

	if (req->byte_offset_in_lane_group > STP_WORD_WIDTH_BYTES) {
		dev_err(&pb->pdev->dev,
				"%s: byte offset in lange group exceeds the "
				"size of a lange group %u > %u\n", __func__,
				req->byte_offset_in_lane_group,
				STP_WORD_WIDTH_BYTES);
		return -EINVAL;
	}

	return 0;
}

static int validate_stp_vector_write_coordinate_parameters(
		struct paintbox_data *pb, struct paintbox_stp *stp,
		struct sram_vector_coordinate_write *req) {
	size_t sram_start_bytes, sram_len_bytes, num_sheet_slots, max_rows,
			max_cols;
	bool is_halo_write = false;

	if (!req->buf) {
		dev_err(&pb->pdev->dev, "%s: invalid user buffer\n", __func__);
		return -EINVAL;
	}

	is_halo_write = (req->lane_group_x >=
			VECTOR_SRAM_LANE_GROUP_SIMD_COLS || req->lane_group_y >=
			VECTOR_SRAM_LANE_GROUP_SIMD_ROWS);

	num_sheet_slots = is_halo_write ? stp->halo_mem_size_in_words :
			stp->vector_mem_size_in_words;

	sram_len_bytes = num_sheet_slots * STP_WORD_WIDTH_BYTES;
	sram_start_bytes = req->sheet_slot * STP_WORD_WIDTH_BYTES;

	max_rows = VECTOR_SRAM_LANE_GROUP_SIMD_ROWS;
	max_cols = VECTOR_SRAM_LANE_GROUP_SIMD_COLS;

	if (is_halo_write) {
		max_rows += VECTOR_SRAM_LANE_GROUP_HALO_ROWS;
		max_cols += VECTOR_SRAM_LANE_GROUP_HALO_COLS;
	}

	if (req->len_bytes > sram_len_bytes - sram_start_bytes) {
		dev_err(&pb->pdev->dev, "%s: write length too long %lu > %lu\n",
				__func__, req->len_bytes, sram_len_bytes -
				sram_start_bytes);
		return -ERANGE;
	}

	if (req->sheet_slot > num_sheet_slots) {
		dev_err(&pb->pdev->dev,
				"%s: sheet slot out of range %u > %lu\n",
				__func__, req->sheet_slot, num_sheet_slots);
		return -ERANGE;
	}

	if (req->byte_offset_in_lane_group > STP_WORD_WIDTH_BYTES) {
		dev_err(&pb->pdev->dev,
				"%s: byte offset in lange group exceeds the "
				"size of a lange group %u > %u\n", __func__,
				req->byte_offset_in_lane_group,
				STP_WORD_WIDTH_BYTES);
		return -EINVAL;
	}

	if (req->lane_group_x >= max_cols) {
		dev_err(&pb->pdev->dev,
				"%s: lane group x out of range %u >= %lu\n",
				__func__, req->lane_group_x, max_cols);
		return -ERANGE;
	}

	if (req->lane_group_y >= max_rows) {
		dev_err(&pb->pdev->dev,
				"%s: lane group y out of range %u >= %lu\n",
				__func__, req->lane_group_y, max_rows);
		return -ERANGE;
	}

	return 0;
}

static int validate_stp_vector_read_coordinate_parameters(
		struct paintbox_data *pb, struct paintbox_stp *stp,
		struct sram_vector_coordinate_read *req) {
	size_t sram_start_bytes, sram_len_bytes, num_sheet_slots, max_rows,
			max_cols;
	bool is_halo_read = false;

	if (!req->buf) {
		dev_err(&pb->pdev->dev, "%s: invalid user buffer\n", __func__);
		return -EINVAL;
	}

	is_halo_read = (req->lane_group_x >= VECTOR_SRAM_LANE_GROUP_SIMD_COLS ||
			req->lane_group_y >= VECTOR_SRAM_LANE_GROUP_SIMD_ROWS);

	num_sheet_slots = is_halo_read ? stp->halo_mem_size_in_words :
			stp->vector_mem_size_in_words;

	sram_len_bytes = num_sheet_slots * STP_WORD_WIDTH_BYTES;
	sram_start_bytes = req->sheet_slot * STP_WORD_WIDTH_BYTES;

	max_rows = VECTOR_SRAM_LANE_GROUP_SIMD_ROWS;
	max_cols = VECTOR_SRAM_LANE_GROUP_SIMD_COLS;

	if (is_halo_read) {
		max_rows += VECTOR_SRAM_LANE_GROUP_HALO_ROWS;
		max_cols += VECTOR_SRAM_LANE_GROUP_HALO_COLS;
	}

	if (req->len_bytes > sram_len_bytes - sram_start_bytes) {
		dev_err(&pb->pdev->dev, "%s: read length too long %lu > %lu\n",
				__func__, req->len_bytes, sram_len_bytes -
				sram_start_bytes);
		return -ERANGE;
	}

	if (req->sheet_slot > num_sheet_slots) {
		dev_err(&pb->pdev->dev,
				"%s: sheet slot out of range %u > %lu\n",
				__func__, req->sheet_slot, num_sheet_slots);
		return -ERANGE;
	}

	if (req->byte_offset_in_lane_group > STP_WORD_WIDTH_BYTES) {
		dev_err(&pb->pdev->dev,
				"%s: byte offset in lange group exceeds the "
				"size of a lange group %u > %u\n", __func__,
				req->byte_offset_in_lane_group,
				STP_WORD_WIDTH_BYTES);
		return -EINVAL;
	}

	if (req->lane_group_x >= max_cols) {
		dev_err(&pb->pdev->dev,
				"%s: lane group x out of range %u >= %lu\n",
				__func__, req->lane_group_x, max_cols);
		return -ERANGE;
	}

	if (req->lane_group_y >= max_rows) {
		dev_err(&pb->pdev->dev,
				"%s: lane group y out of range %u >= %lu\n",
				__func__, req->lane_group_y, max_rows);
		return -ERANGE;
	}

	return 0;
}

int write_stp_scalar_sram_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct ipu_sram_write __user *user_req;
	struct ipu_sram_write req;
	struct paintbox_stp *stp;
	uint32_t ram_ctrl_mask = 0;
	int ret;

	user_req = (struct ipu_sram_write __user *)arg;
	if (copy_from_user(&req, user_req, sizeof(req)))
		return -EFAULT;

	if (!req.buf) {
		dev_err(&pb->pdev->dev, "%s: invalid user buffer\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&pb->lock);
	stp = get_stp(pb, session, req.id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	ret = validate_ram_transfer(pb, stp, req.sram_byte_addr, req.ram_target,
			req.len_bytes);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	writel(stp->stp_id, pb->stp_base + STP_SEL);

	ret = set_ram_target(&ram_ctrl_mask, req.ram_target);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		dev_err(&pb->pdev->dev,
				"%s: stp%u: invalid ram target: 0x%04x "
				"err = %d\n", __func__, req.id, req.ram_target,
				ret);
		return ret;
	}

	ret = sram_write_user_buffer(pb, req.buf, req.sram_byte_addr,
			req.len_bytes, ram_ctrl_mask,
			pb->stp_base + STP_RAM_CTRL, STP_DATA_REG_COUNT);
	if (ret < 0)
		dev_err(&pb->pdev->dev,
				"%s: stp%u: write error addr: 0x%04x "
				"type: 0x%02x ram_ctrl 0x%08x err = %d\n",
				__func__, req.id, req.sram_byte_addr,
				req.ram_target,
				readl(pb->stp_base + STP_RAM_CTRL), ret);

	mutex_unlock(&pb->lock);

	return ret;
}

int read_stp_scalar_sram_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct ipu_sram_read __user *user_req;
	struct ipu_sram_read req;
	struct paintbox_stp *stp;
	uint32_t ram_ctrl_mask = 0;
	int ret;

	user_req = (struct ipu_sram_read __user *)arg;
	if (copy_from_user(&req, user_req, sizeof(req)))
		return -EFAULT;

	if (!req.buf) {
		dev_err(&pb->pdev->dev, "%s: invalid user buffer\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&pb->lock);
	stp = get_stp(pb, session, req.id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	ret = validate_ram_transfer(pb, stp, req.sram_byte_addr, req.ram_target,
			req.len_bytes);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	writel(stp->stp_id, pb->stp_base + STP_SEL);

	ret = set_ram_target(&ram_ctrl_mask, req.ram_target);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		dev_err(&pb->pdev->dev,
				"%s: stp%u: invalid ram target: 0x%04x "
				"err = %d\n", __func__, req.id, req.ram_target,
				ret);
		return ret;
	}

	ret = sram_read_user_buffer(pb, req.buf, req.sram_byte_addr,
			req.len_bytes, ram_ctrl_mask,
			pb->stp_base + STP_RAM_CTRL, STP_DATA_REG_COUNT);
	if (ret < 0)
		dev_err(&pb->pdev->dev,
				"%s: stp%u: read error addr: 0x%04x "
				"type: 0x%02x ram_ctrl 0x%08x err = %d\n",
				__func__, req.id, req.sram_byte_addr,
				req.ram_target,
				readl(pb->stp_base + STP_RAM_CTRL), ret);

	mutex_unlock(&pb->lock);

	return ret;
}

static int sram_read_vector_buffer(struct paintbox_data *pb, uint8_t *buf,
		uint32_t lane_group_x, uint32_t lane_group_y,
		uint32_t sheet_slot_start, uint32_t byte_offest_in_lane_group,
		size_t len_bytes, bool read_alu_registers)
{
	size_t bytes_remaining = len_bytes;
	size_t bytes_read = 0;
	uint32_t sheet_slot = sheet_slot_start, ram_ctrl;
	int ret;

	/* If the transfer does not start at the beginning of a word then it
	 * will require special handling before we can get to the main body of
	 * the transfer.
	 */
	if (byte_offest_in_lane_group > 0) {
		size_t short_read_len = STP_WORD_WIDTH_BYTES -
				byte_offest_in_lane_group;
		ram_ctrl = generate_vector_address(lane_group_x, lane_group_y,
				sheet_slot, read_alu_registers);

		ret = sram_read_word_partial(pb, buf, byte_offest_in_lane_group,
				short_read_len, ram_ctrl,
				pb->stp_base + STP_RAM_CTRL, STP_DATA_REG_COUNT,
				RAM_DATA_MODE_COL_MAJOR);
		if (ret < 0)
			return ret;

		bytes_remaining -= short_read_len;
		bytes_read += short_read_len;
		sheet_slot++;
	}

	/* If the transfer data length is greater than or equal to an SRAM word
	* then transfer the data in full word transfers.
	 */
	while (bytes_remaining && bytes_remaining >= STP_WORD_WIDTH_BYTES) {
		ram_ctrl = generate_vector_address(lane_group_x, lane_group_y,
				sheet_slot, read_alu_registers);

		ret = sram_read_word(pb, buf + bytes_read, ram_ctrl,
				pb->stp_base + STP_RAM_CTRL, STP_DATA_REG_COUNT,
				RAM_DATA_MODE_COL_MAJOR);
		if (ret < 0)
			return ret;

		bytes_remaining -= STP_WORD_WIDTH_BYTES;
		bytes_read += STP_WORD_WIDTH_BYTES;
		sheet_slot++;
	}

	/* Handle any remaining bytes that are shorter than an SRAM word */
	if (bytes_remaining > 0) {
		ram_ctrl = generate_vector_address(lane_group_x, lane_group_y,
				sheet_slot, read_alu_registers);

		ret = sram_read_word_partial(pb, buf, 0, bytes_remaining,
				ram_ctrl, pb->stp_base + STP_RAM_CTRL,
				STP_DATA_REG_COUNT, RAM_DATA_MODE_COL_MAJOR);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int sram_write_vector_buffer(struct paintbox_data *pb,
		const uint8_t *buf, uint32_t lane_group_x,
		uint32_t lane_group_y, uint32_t sheet_slot_start,
		uint32_t byte_offset_in_lane_group, size_t len_bytes,
		bool write_alu_registers)
{
	size_t bytes_remaining = len_bytes;
	size_t bytes_written = 0;
	uint32_t sheet_slot = sheet_slot_start, ram_ctrl;
	int ret;

	/* If the transfer does not start at the beginning of a word then it
	 * will require special handling before we can get to the main body of
	 * the transfer.
	 */
	if (byte_offset_in_lane_group > 0) {
		size_t short_write_len = STP_WORD_WIDTH_BYTES -
				byte_offset_in_lane_group;

		ram_ctrl = generate_vector_address(lane_group_x, lane_group_y,
				sheet_slot, write_alu_registers);

		ret = sram_write_word_partial(pb, buf,
				byte_offset_in_lane_group, short_write_len,
				ram_ctrl, pb->stp_base + STP_RAM_CTRL,
				STP_DATA_REG_COUNT, RAM_DATA_MODE_COL_MAJOR);
		if (ret < 0)
			return ret;

		bytes_remaining -= short_write_len;
		bytes_written += short_write_len;
		sheet_slot++;
	}

	/* If the transfer data length is greater than or equal to the SRAM
	 * word then transfer the data in full word transfers.
	 */
	while (bytes_remaining && bytes_remaining >= STP_WORD_WIDTH_BYTES) {
		ram_ctrl = generate_vector_address(lane_group_x, lane_group_y,
				sheet_slot, write_alu_registers);

		ret = sram_write_word(pb, buf + bytes_written, ram_ctrl,
				pb->stp_base + STP_RAM_CTRL, STP_DATA_REG_COUNT,
				RAM_DATA_MODE_COL_MAJOR);
		if (ret < 0)
			return ret;

		bytes_remaining -= STP_WORD_WIDTH_BYTES;
		bytes_written += STP_WORD_WIDTH_BYTES;
		sheet_slot++;
	}

	/* Handle any remaining bytes that are shorter than an SRAM word. */
	if (bytes_remaining > 0) {
		ram_ctrl = generate_vector_address(lane_group_x, lane_group_y,
				sheet_slot, write_alu_registers);

		ret = sram_write_word_partial(pb, buf + bytes_written, 0,
				bytes_remaining, ram_ctrl,
				pb->stp_base + STP_RAM_CTRL, STP_DATA_REG_COUNT,
				RAM_DATA_MODE_COL_MAJOR);
		if (ret < 0)
			return ret;
	}

	return 0;
}

/* Vector memory on the IPU is organized as a torus.  The lane group addresses
 * in the RAM access registers are physical addresses.  The following arrays map
 * the various lane groups of vector memory from their physical addresses to
 * their logical addresses.
 *
 * TODO(ahampson):  Conversion between logical and physical addressing for the
 * lane groups may be moved to the runtime.  If that is the case then this code
 * can be remmoved.
 */
static const unsigned int logical_to_phys_row_map[10] = {
	7, 6, 3, 2, 0, 1, 4, 5, 8, 9
};

static const unsigned int logical_to_phys_col_map[5] = {
	1, 3, 4, 2, 0
};

int write_stp_vector_sram_coordinates_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct sram_vector_coordinate_write __user *user_req;
	struct sram_vector_coordinate_write req;
	struct paintbox_stp *stp;
	uint8_t *buf = NULL;
	int ret;

	user_req = (struct sram_vector_coordinate_write __user *)arg;
	if (copy_from_user(&req, user_req, sizeof(req)))
		return -EFAULT;

	mutex_lock(&pb->lock);
	stp = get_stp(pb, session, req.id, &ret);
	if (ret < 0)
		goto exit;

	ret = validate_stp_vector_write_coordinate_parameters(pb, stp, &req);
	if (ret < 0)
		goto exit;

	ret = alloc_and_copy_from_user(pb, &buf, req.buf, req.len_bytes);
	if (ret < 0)
		goto exit;

	writel(stp->stp_id, pb->stp_base + STP_SEL);

	ret = sram_write_vector_buffer(pb, buf,
			logical_to_phys_col_map[req.lane_group_x],
			logical_to_phys_row_map[req.lane_group_y],
			req.sheet_slot, req.byte_offset_in_lane_group,
			req.len_bytes, req.write_alu_registers);

exit:
	mutex_unlock(&pb->lock);

	kfree(buf);

	return ret;
}

int write_stp_vector_sram_replicate_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct sram_vector_replicate_write __user *user_req;
	struct sram_vector_replicate_write req;
	struct paintbox_stp *stp;
	unsigned int num_lane_group_cols, num_lane_group_rows;
	unsigned int lane_group_col, lane_group_row;
	unsigned int phys_col, phys_row;
	uint8_t *buf = NULL;
	int ret;

	user_req = (struct sram_vector_replicate_write __user *)arg;
	if (copy_from_user(&req, user_req, sizeof(req)))
		return -EFAULT;

	mutex_lock(&pb->lock);
	stp = get_stp(pb, session, req.id, &ret);
	if (ret < 0)
		goto exit;

	ret = validate_stp_vector_write_replicate_parameters(pb, stp, &req);
	if (ret < 0)
		goto exit;

	num_lane_group_rows = VECTOR_SRAM_LANE_GROUP_SIMD_ROWS;
	num_lane_group_cols = VECTOR_SRAM_LANE_GROUP_SIMD_COLS;

	if (req.write_halo_lanes) {
		num_lane_group_rows += VECTOR_SRAM_LANE_GROUP_HALO_ROWS;
		num_lane_group_cols += VECTOR_SRAM_LANE_GROUP_HALO_COLS;
	}

	ret = alloc_and_copy_from_user(pb, &buf, req.buf, req.len_bytes);
	if (ret < 0)
		goto exit;

	writel(stp->stp_id, pb->stp_base + STP_SEL);

	/* Iterate over all lane groups and write the supplied user buffer to
	 * those groups.
	 */
	for (lane_group_col = 0; lane_group_col < num_lane_group_cols;
			lane_group_col++) {
		phys_col = logical_to_phys_col_map[lane_group_col];

		for (lane_group_row = 0; lane_group_row < num_lane_group_rows;
				lane_group_row++) {
			phys_row = logical_to_phys_row_map[lane_group_row];

			ret = sram_write_vector_buffer(pb, buf, phys_col,
					phys_row, req.sheet_slot,
					req.byte_offset_in_lane_group,
					req.len_bytes, req.write_alu_registers);
			if (ret < 0)
				goto exit;
		}
	}

exit:
	mutex_unlock(&pb->lock);

	kfree(buf);

	return ret;
}

int read_stp_vector_sram_coordinates_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct sram_vector_coordinate_read __user *user_req;
	struct sram_vector_coordinate_read req;
	struct paintbox_stp *stp;
	uint8_t *buf = NULL;
	int ret;

	user_req = (struct sram_vector_coordinate_read __user *)arg;
	if (copy_from_user(&req, user_req, sizeof(req)))
		return -EFAULT;

	mutex_lock(&pb->lock);
	stp = get_stp(pb, session, req.id, &ret);
	if (ret < 0)
		goto exit;

	ret = validate_stp_vector_read_coordinate_parameters(pb, stp, &req);
	if (ret < 0)
		goto exit;

	buf = kmalloc(req.len_bytes, GFP_KERNEL);
	if (!buf) {
		dev_err(&pb->pdev->dev, "%s: allocation failure\n", __func__);
		ret = -ENOMEM;
		goto exit;
	}

	writel(stp->stp_id, pb->stp_base + STP_SEL);

	ret = sram_read_vector_buffer(pb, buf,
			logical_to_phys_col_map[req.lane_group_x],
			logical_to_phys_row_map[req.lane_group_y],
			req.sheet_slot, req.byte_offset_in_lane_group,
			req.len_bytes, req.read_alu_registers);
	if (ret < 0)
		goto exit;

	if (copy_to_user((void __user *)req.buf, buf, req.len_bytes))
		ret = -EFAULT;

exit:
	mutex_unlock(&pb->lock);

	kfree(buf);

	return ret;
}
