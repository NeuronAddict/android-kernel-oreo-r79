/*
 * Driver interface for the Paintbox Image Processing Unit
 *
 * Copyright (C) 2015 Google, Inc.
 * Author: Adam Hampson <ahampson@google.com>
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

#ifndef __PAINTBOX_H__
#define __PAINTBOX_H__

#include <linux/compiler.h>
#include <linux/ioctl.h>

/* TODO(ahampson): There are several enumerations and types that are used in
 * C and C++ code across the kernel, QEMU, and Simulator.  A unified header file
 * hierarchy should be created to avoid type duplication.  This also has build
 * system implications as the components using these types are spread out over
 * multiple git projects and build systems.
 *
 * In the interim the following enums are specified here and must be kept in
 * sync with theer equivalent definitions elsewhere.
 *
 * This is being tracked by http://b/23560812
 */

/* Note this enum must be kept in sync with PaddingMethod in ipu_types.h */
enum ipu_padding_method {
	IPU_PADDING_DO_NOT_PAD = 0,
	IPU_PADDING_CONSTANT,
	IPU_PADDING_PERIODIC,
	IPU_PADDING_SYMMETRIC,
	IPU_PADDING_METHOD_SIZE
};

/* This flag will be set in the ipu_capabilities version field when the IPU the
 * driver is operating on is a simulator and not real hardware.
 */
#define IPU_CAPS_VERSION_SIMULATOR_FLAG (1 << 31)

struct ipu_capabilities {
	uint32_t version;
	uint32_t mem_size;
	uint32_t max_fb_rows;
	uint32_t num_stps;
	uint32_t num_interrupts;
	uint32_t num_lbps;
	uint32_t num_dma_channels;
	uint32_t max_line_buffers;
	uint32_t max_read_ptrs;
	uint32_t max_channels;
};

enum dma_transfer_type {
	DMA_DRAM_TO_LBP = 0,
	DMA_DRAM_TO_STP,
	DMA_LBP_TO_DRAM,
	DMA_STP_TO_DRAM,
	DMA_MIPI_TO_LBP,
	DMA_SRC_DST_PAIRS
};

struct dma_dram_config {
	void __user *host_vaddr;
	uint64_t len_bytes;
};

struct dma_lbp_config {
	uint32_t lbp_id;
	uint32_t lb_id;
	uint32_t read_ptr_id;
	uint32_t start_x_pixels;
	uint32_t start_y_pixels;
	bool gather;
};

enum dma_swizzle_mode {
	SWIZZLE_MODE_DISABLED = 0,
	SWIZZLE_MODE_BIG_ENDIAN,
	SWIZZLE_MODE_NEIGHBOR,
	SWIZZLE_MODE_COUNT
};

enum dma_alpha_mode {
	ALPHA_MODE_DISABLED = 0,
	ALPHA_MODE_RGBA,
	ALPHA_MODE_ARGB,
	ALPHA_MODE_COUNT
};

struct dma_image_config {
	uint64_t plane_stride_bytes;
	uint32_t width_pixels;
	uint32_t height_pixels;
	uint32_t start_x_pixels;
	uint32_t start_y_pixels;
	uint32_t row_stride_bytes;
	enum dma_swizzle_mode swizzle_mode;
	enum dma_alpha_mode alpha_mode;
	uint8_t bit_depth;
	uint8_t planes;
	uint8_t components;
	bool block4x4;
};

struct dma_transfer_config {
	uint32_t channel_id;
	enum dma_transfer_type transfer_type;
	struct dma_image_config img;
	union {
		/* TODO(ahampson):  Add STP config support. b/28341158 */
		/* TDOO(ahampson):  Add MIPI config support. b/28340987 */
		struct dma_dram_config dram;
		struct dma_lbp_config lbp;
	} src;
	union {
		/* TODO(ahampson):  Add STP config support. b/28341158 */
		/* TDOO(ahampson):  Add MIPI config support. b/28340987 */
		struct dma_dram_config dram;
		struct dma_lbp_config lbp;
	} dst;
	uint32_t sheet_width;
	uint32_t sheet_height;
	uint32_t stripe_height;
};

/* TODO(ahampson):  We can remove this when the IOMMU based transfers are
 * finished.
 */
struct dma_transfer_read {
	uint32_t channel_id;
	void __user *host_vaddr;
	size_t len_bytes;
};

struct padding_params {
	enum ipu_padding_method method;
	uint16_t value_or_period;
};

struct line_buffer_config {
	uint8_t lb_pool_id;
	uint8_t lb_id;
	uint8_t num_read_ptrs;
	uint8_t num_reuse_rows;
	uint8_t num_channels;
	uint8_t fb_offset_pixels;
	uint8_t x_offset_pixels;
	uint8_t y_offset_pixels;
	uint8_t chan_offset_pixels;
	uint16_t fb_rows;
	uint16_t width_pixels;
	uint16_t height_pixels;
	uint16_t sb_rows;
	uint16_t sb_cols;
	uint32_t ipu_fb_base_addr;
	uint32_t ipu_sb_base_addr;
	struct padding_params padding;
};

struct interrupt_config {
	uint8_t channel_id;
	uint8_t interrupt_id;
};

struct interrupt_wait {
	unsigned int interrupt_id;
	int64_t timeout_ns;
};

struct stp_config {
	unsigned int processor_id;
	const void __user *buf;
	size_t len;
};

struct stp_program_state {
	uint32_t program_counter;
	uint8_t stp_id;
};

#define STP_RAM_TARGET_INSTRUCTION_RAM 0
#define STP_RAM_TARGET_CONSTANT_RAM    1
#define STP_RAM_TARGET_SCALAR_RAM      2
#define STP_RAM_TARGET_VECTOR_RAM      3

struct ipu_sram_write {
	const void __user *buf;
	size_t len_bytes;
	uint32_t sram_addr; /* Byte aligned SRAM address */
	uint8_t id;
	uint8_t ram_target;
	uint8_t priority;
};

struct ipu_sram_read {
	void __user *buf;
	size_t len_bytes;
	uint32_t sram_addr; /* Byte aligned SRAM address */
	uint8_t id;
	uint8_t ram_target;
	uint8_t priority;
};

struct ipu_sram_vector_write {
	const void __user *buf;
	size_t len_bytes;
	/* Logical Lane Coordinates */
	uint8_t lane_x_start;
	uint8_t lane_x_end;
	uint8_t lane_y_start;
	uint8_t lane_y_end;
	uint8_t offset_in_bank;
	uint8_t priority;
	uint8_t id;
};

struct ipu_sram_vector_read {
	void __user *buf;
	size_t len_bytes;
	/* Logical Lane Coordinates */
	uint8_t lane_x;
	uint8_t lane_y;
	uint8_t offset_in_bank;
	uint8_t priority;
	uint8_t id;
};


/* Ioctl interface to IPU driver
 *
 * The following ioctls will return these error codes on error conditions:
 *
 * -EACCES: Resource access error
 * -EEXIST: Resource exists or is already in use
 * -EFAULT: Buffer transfer error (rare)
 * -EINVAL: Invalid Parameter
 * -ENOENT: No entry
 * -ENOMEN: Out of memory
 * -ENOSYS: Unimplemented Functionality
 *
 */
#define PB_GET_IPU_CAPABILITIES      _IOR('p', 1, struct ipu_capabilities)
#define PB_ALLOCATE_DMA_CHANNEL      _IOW('p', 2, unsigned int)
#define PB_SETUP_DMA_TRANSFER        _IOW('q', 3, struct dma_transfer_config)
#define PB_START_DMA_TRANSFER        _IOW('p', 4, unsigned int)
#define PB_BIND_DMA_INTERRUPT        _IOW('p', 5, struct interrupt_config)
#define PB_UNBIND_DMA_INTERRUPT      _IOW('p', 6, unsigned int)
#define PB_RELEASE_DMA_CHANNEL       _IOW('p', 7, unsigned int)
#define PB_ALLOCATE_INTERRUPT        _IOW('p', 8, unsigned int)
#define PB_WAIT_FOR_INTERRUPT        _IOW('p', 11, struct interrupt_wait)
#define PB_RELEASE_INTERRUPT         _IOW('p', 12, unsigned int)
#define PB_ALLOCATE_LINE_BUFFER_POOL _IOW('p', 13, unsigned int)
#define PB_SETUP_LINE_BUFFER         _IOW('p', 14, struct line_buffer_config)
#define PB_RELEASE_LINE_BUFFER_POOL  _IOW('p', 15, unsigned int)
#define PB_ALLOCATE_PROCESSOR        _IOW('p', 16, unsigned int)
#define PB_SETUP_PROCESSOR           _IOW('p', 17, struct stp_config)
#define PB_START_PROCESSOR           _IOW('p', 18, unsigned int)
#define PB_RELEASE_PROCESSOR         _IOW('p', 19, unsigned int)

/* The STP will be nearly idle when it sends the DMA completion interrupt.
 * Following the interrupt there will be a small amount of cleanup work.
 * PB_GET_PROCESSOR_IDLE and PB_WAIT_FOR_ALL_PROCESSOR_IDLE can be used to
 * determine when the post interrupt cleanup work has completed.
 */

/* Returns 1 if idle, 0 not idle, < 0 error */
#define PB_GET_PROCESSOR_IDLE        _IOW('p', 20, unsigned int)
#define PB_WAIT_FOR_ALL_PROCESSOR_IDLE _IO('p', 21)

#define PB_WRITE_LBP_MEMORY          _IOW('p', 22, struct ipu_sram_write)
#define PB_READ_LBP_MEMORY          _IOWR('p', 23, struct ipu_sram_read)
#define PB_WRITE_STP_MEMORY          _IOW('p', 24, struct ipu_sram_write)
#define PB_READ_STP_MEMORY          _IOWR('p', 25, struct ipu_sram_read)
#define PB_STOP_PROCESSOR            _IOW('p', 26, unsigned int)
#define PB_RESUME_PROCESSOR          _IOW('p', 27, unsigned int)
#define PB_RESET_PROCESSOR           _IOW('p', 28, unsigned int)
#define PB_GET_PROGRAM_STATE        _IOWR('p', 29, struct stp_program_state)
#define PB_WRITE_STP_VECTOR_MEMORY   _IOW('p', 30, struct ipu_sram_vector_write)
#define PB_READ_STP_VECTOR_MEMORY   _IOWR('p', 31, struct ipu_sram_vector_read)
#define PB_READ_DMA_TRANSFER         _IOW('p', 32, struct dma_transfer_read )

#endif /* __PAINTBOX_H__ */
