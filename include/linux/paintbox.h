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

struct ipu_capabilities {
	uint32_t version_major;
	uint32_t version_minor;
	uint32_t version_build;
	uint32_t hardware_id;
	uint32_t max_fb_rows;
	uint32_t num_stps;
	uint32_t num_interrupts;
	uint32_t num_lbps;
	uint32_t num_dma_channels;
	uint32_t max_line_buffers;
	uint32_t max_read_ptrs;
	uint32_t max_channels;
	bool is_simulator;
	bool is_fpga;
};

enum sram_target_type {
	SRAM_TARGET_LBP = 0,
	SRAM_TARGET_STP_INSTRUCTION_RAM,
	SRAM_TARGET_STP_CONSTANT_RAM,
	SRAM_TARGET_STP_SCALAR_RAM,
	SRAM_TARGET_STP_VECTOR_RAM
};

enum dma_transfer_type {
	DMA_DRAM_TO_LBP = 0,
	DMA_DRAM_TO_STP,
	DMA_LBP_TO_DRAM,
	DMA_MIPI_TO_LBP,
	DMA_LBP_TO_MIPI,
	DMA_MIPI_TO_DRAM,
	DMA_SRC_DST_PAIRS
};

enum dma_dram_buffer_type {
	DMA_DRAM_BUFFER_UNUSED = 0,
	DMA_DRAM_BUFFER_USER,
	DMA_DRAM_BUFFER_DMA_BUF
};

struct dma_dram_config {
	enum dma_dram_buffer_type buffer_type;
	union {
		void __user *host_vaddr;
		int dma_buf_fd;
	};
	uint64_t len_bytes;
};

struct dma_lbp_config {
	uint32_t lbp_id;
	uint32_t lb_id;
	uint32_t read_ptr_id;
	int32_t start_x_pixels;
	int32_t start_y_pixels;
	bool gather;
};

struct dma_stp_config {
	enum sram_target_type sram_target;
	uint32_t stp_id;
	uint32_t sram_addr;
	bool include_halo;
};

enum dma_rgba_format {
	RGBA_FORMAT_DISABLED = 0,
	RGBA_FORMAT_RGBA,
	RGBA_FORMAT_ARGB,
	RGBA_FORMAT_COUNT
};

struct dma_image_config {
	uint64_t plane_stride_bytes;
	uint32_t width_pixels;
	uint32_t height_pixels;
	int32_t start_x_pixels;
	int32_t start_y_pixels;
	uint32_t row_stride_bytes;
	enum dma_rgba_format rgba_format;
	uint8_t bit_depth;
	uint8_t planes;
	uint8_t components;
	bool block4x4;
	bool mipi_raw_format;
};

struct dma_transfer_config {
	uint32_t channel_id;
	enum dma_transfer_type transfer_type;
	struct dma_image_config img;
	union {
		/* MIPI transfers do not require any additional settings */
		struct dma_stp_config stp;
		struct dma_dram_config dram;
		struct dma_lbp_config lbp;
	} src;
	union {
		/* MIPI transfers do not require any additional settings */
		struct dma_stp_config stp;
		struct dma_dram_config dram;
		struct dma_lbp_config lbp;
	} dst;
	uint32_t sheet_width;
	uint32_t sheet_height;
	uint32_t stripe_height;
	uint32_t noc_outstanding;
	uint32_t retry_interval;

	/* Set to true when the runtime will be waiting for a completion
	 * notification.
	 */
	bool notify_on_completion;

	/* Set to true when the transfer should be auto-loaded once the
	 * preceding transfer has completed without invoking the
	 * PB_START_DMA_TRANSFER ioctl.
	 */
	bool auto_load_transfer;
};

/* TODO(ahampson):  We can remove this when the IOMMU based transfers are
 * finished.
 */
struct dma_transfer_read {
	uint32_t channel_id;
	void __user *host_vaddr;
	size_t len_bytes;
};

struct dma_transfer_flush {
	uint32_t channel_id;
	bool flush_pending;
	bool flush_active;
	bool flush_completed;
};

struct padding_params {
	enum ipu_padding_method method;
	uint16_t value_or_period;
};

struct line_buffer_config {
	int32_t x_offset_pixels;
	int32_t y_offset_pixels;
	int32_t fb_offset_pixels;
	uint8_t lb_pool_id;
	uint8_t lb_id;
	uint8_t num_read_ptrs;
	uint8_t num_reuse_rows;
	uint8_t num_channels;
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

struct line_buffer_reset {
	uint32_t lbp_id;
	uint32_t lb_id;
};

struct dma_interrupt_config {
	uint32_t channel_id;
	uint32_t interrupt_id;
};

struct interrupt_wait {
	/* inputs */
	uint64_t interrupt_mask_all;
	uint64_t interrupt_mask_any;
	int64_t timeout_ns;

	/* outputs */
	uint64_t interrupt_mask_fired;
	int32_t interrupt_id_error;
	int32_t error;

	/* actual size of array depends on number of stps */
	uint16_t interrupt_code[1];

	/* interrupt_code MUST BE LAST MEMBER */
};

struct stp_config {
	unsigned int processor_id;
	const void __user *buf;
	size_t len;
};

struct stp_program_state {
	int32_t program_counter;
	uint8_t stp_id;
	bool enabled;
	bool stalled;
};

struct stp_interrupt_config {
	uint32_t stp_id;
	uint32_t interrupt_id;
};

struct ipu_sram_write {
	const void __user *buf;
	size_t len_bytes;
	uint32_t sram_byte_addr;
	uint32_t id;
	enum sram_target_type sram_target;
	bool swap_data;

	/* When set the driver will pad unaligned or short writes with zeros
	 * instead of doing a read-modify-write.  This feature is used for
	 * test and debugging and should not normally be set.
	 */
	bool pad_to_align;
};

struct ipu_sram_read {
	void __user *buf;
	size_t len_bytes;
	uint32_t sram_byte_addr;
	uint32_t id;
	enum sram_target_type sram_target;
	bool swap_data;
};

struct sram_vector_coordinate_write {
	const void __user *buf;
	size_t len_bytes;
	uint32_t lane_group_x;
	uint32_t lane_group_y;
	uint32_t sheet_slot;
	uint32_t byte_offset_in_lane_group;
	uint32_t id;
	bool write_alu_registers;
};

struct sram_vector_replicate_write {
	const void __user *buf;
	size_t len_bytes;
	uint32_t sheet_slot;
	uint32_t byte_offset_in_lane_group;
	uint32_t id;
	bool write_alu_registers;
	bool write_halo_lanes;
};

struct sram_vector_coordinate_read {
	void __user *buf;
	size_t len_bytes;
	uint32_t lane_group_x;
	uint32_t lane_group_y;
	uint32_t sheet_slot;
	uint32_t byte_offset_in_lane_group;
	uint32_t id;
	bool read_alu_registers;
};

struct mipi_input_stream_setup {
	uint32_t seg_start;
	uint32_t seg_words_per_row;
};

struct mipi_output_stream_setup {
	bool enable_row_sync;
};

struct mipi_stream_setup {
	uint32_t stream_id;
	uint32_t virtual_channel;
	/* Received data type */
	uint32_t data_type;

	/* Unpacked data type */
	uint32_t unpacked_data_type;
	uint32_t img_width;
	uint32_t img_height;
	uint32_t seg_end;
	uint32_t segs_per_row;
	uint32_t stripe_height;

	union {
		struct mipi_input_stream_setup input;
		struct mipi_output_stream_setup output;
	};

	/* If enable_on_setup is true then the stream will be enabled after the
	 * configuration is written, otherwise if false then the client will
	 * have to enable the stream.
	 */
	bool enable_on_setup;

	/* If enable_on_setup is true and free_running is true then the stream
	 * will run with this configuration until the client disables it.
	 */
	bool free_running;

	/* If enable_on_setup is true and free_running is false then the stream
	 * will run for the specified number of frames.
	 */
	int32_t frame_count;
};

struct mipi_stream_enable {
	uint32_t stream_id;

	/* If free_running is true then the stream will run with this
	 * configuration until the client disables it.
	 */
	bool free_running;

	/* If free_running is false then the stream will run for the specified
	 * number of frames.
	 */
	int32_t frame_count;

	/* enable_row_sync is only used for output streams. */
	struct {
		bool enable_row_sync;
	} output;
};

struct mipi_interrupt_config {
	uint32_t stream_id;
	uint32_t interrupt_id;
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
#define PB_BIND_DMA_INTERRUPT        _IOW('p', 5, struct dma_interrupt_config)
#define PB_UNBIND_DMA_INTERRUPT      _IOW('p', 6, unsigned int)
#define PB_RELEASE_DMA_CHANNEL       _IOW('p', 7, unsigned int)
#define PB_ALLOCATE_INTERRUPT        _IOW('p', 8, unsigned int)
#define PB_WAIT_FOR_INTERRUPT        _IOWR('p', 11, struct interrupt_wait)
#define PB_RELEASE_INTERRUPT         _IOW('p', 12, unsigned int)
#define PB_ALLOCATE_LINE_BUFFER_POOL _IOW('p', 13, unsigned int)
#define PB_SETUP_LINE_BUFFER         _IOW('p', 14, struct line_buffer_config)
#define PB_RESET_LINE_BUFFER_POOL    _IOW('p', 15, unsigned int)
#define PB_RESET_LINE_BUFFER         _IOW('p', 16, struct line_buffer_reset)
#define PB_RELEASE_LINE_BUFFER_POOL  _IOW('p', 17, unsigned int)
#define PB_ALLOCATE_PROCESSOR        _IOW('p', 18, unsigned int)
#define PB_SETUP_PROCESSOR           _IOW('p', 19, struct stp_config)
#define PB_START_PROCESSOR           _IOW('p', 20, unsigned int)
#define PB_RELEASE_PROCESSOR         _IOW('p', 21, unsigned int)

/* The STP will be nearly idle when it sends the DMA completion interrupt.
 * Following the interrupt there will be a small amount of cleanup work.
 * PB_GET_PROCESSOR_IDLE and PB_WAIT_FOR_ALL_PROCESSOR_IDLE can be used to
 * determine when the post interrupt cleanup work has completed.
 */

/* Returns 1 if idle, 0 not idle, < 0 error */
#define PB_GET_PROCESSOR_IDLE        _IOW('p', 22, unsigned int)
#define PB_WAIT_FOR_ALL_PROCESSOR_IDLE _IO('p', 23)

#define PB_WRITE_LBP_MEMORY          _IOW('p', 24, struct ipu_sram_write)
#define PB_READ_LBP_MEMORY          _IOWR('p', 25, struct ipu_sram_read)
#define PB_WRITE_STP_MEMORY          _IOW('p', 26, struct ipu_sram_write)
#define PB_READ_STP_MEMORY          _IOWR('p', 27, struct ipu_sram_read)
#define PB_STOP_PROCESSOR            _IOW('p', 28, unsigned int)
#define PB_RESUME_PROCESSOR          _IOW('p', 29, unsigned int)
#define PB_RESET_PROCESSOR           _IOW('p', 30, unsigned int)
#define PB_GET_PROGRAM_STATE        _IOWR('p', 31, struct stp_program_state)
#define PB_WRITE_VECTOR_SRAM_COORDINATES _IOW('p', 32, \
		struct sram_vector_coordinate_write)
#define PB_WRITE_VECTOR_SRAM_REPLICATE _IOW('p', 33,   \
		struct sram_vector_replicate_write)
#define PB_READ_VECTOR_SRAM_COORDINATES _IOW('p', 34, \
		struct sram_vector_coordinate_read)
#define PB_READ_DMA_TRANSFER         _IOW('p', 35, struct dma_transfer_read)
#define PB_ALLOCATE_MIPI_IN_STREAM   _IOW('p', 36, unsigned int)
#define PB_RELEASE_MIPI_IN_STREAM    _IOW('p', 37, unsigned int)
#define PB_SETUP_MIPI_IN_STREAM      _IOW('p', 38, struct mipi_stream_setup)
#define PB_ENABLE_MIPI_IN_STREAM     _IOW('p', 39, unsigned int)
#define PB_DISABLE_MIPI_IN_STREAM    _IOW('p', 40, unsigned int)

/* Returns frame number, < 0 error */
#define PB_GET_MIPI_IN_FRAME_NUMBER  _IOW('p', 41, unsigned int)

#define PB_CLEANUP_MIPI_IN_STREAM    _IOW('p', 42, unsigned int)
#define PB_ALLOCATE_MIPI_OUT_STREAM  _IOW('p', 45, unsigned int)
#define PB_RELEASE_MIPI_OUT_STREAM   _IOW('p', 46, unsigned int)
#define PB_SETUP_MIPI_OUT_STREAM     _IOW('p', 47, struct mipi_stream_setup)
#define PB_ENABLE_MIPI_OUT_STREAM    _IOW('p', 48, unsigned int)
#define PB_DISABLE_MIPI_OUT_STREAM   _IOW('p', 49, unsigned int)
#define PB_CLEANUP_MIPI_OUT_STREAM   _IOW('p', 51, unsigned int)
#define PB_BIND_MIPI_IN_INTERRUPT    _IOW('p', 54, struct mipi_interrupt_config)
#define PB_UNBIND_MIPI_IN_INTERRUPT  _IOW('p', 55, unsigned int)
#define PB_BIND_MIPI_OUT_INTERRUPT   _IOW('p', 56, struct mipi_interrupt_config)
#define PB_UNBIND_MIPI_OUT_INTERRUPT _IOW('p', 57, unsigned int)

/* Returns the number of transfers that have completed and are ready to be
 * read back into the userspace buffer.
 */
#define PB_GET_COMPLETED_UNREAD_COUNT _IOW('p', 58, unsigned int)
#define PB_INIT_PROCESSOR             _IOW('p', 59, unsigned int)
#define PB_ENABLE_STP_INTERRUPT       _IOW('p', 60, unsigned int)
#define PB_DISABLE_STP_INTERRUPT      _IOW('p', 61, unsigned int)
#define PB_BIND_STP_INTERRUPT         _IOW('p', 62, struct stp_interrupt_config)
#define PB_UNBIND_STP_INTERRUPT       _IOW('p', 63, unsigned int)
#define PB_STOP_DMA_TRANSFER          _IOW('p', 64, unsigned int)
#define PB_FLUSH_DMA_TRANSFERS        _IOW('p', 65, struct dma_transfer_flush)

/* Test ioctls
 * The following ioctls are for testing and are not to be used for normal
 * operation.  Whether or not the implementation of these ioctls is included in
 * the driver is governed by the PAINTBOX_TEST_SUPPORT kconfig.
 */
#define PB_TEST_DMA_RESET              _IO('t', 1)
#define PB_TEST_DMA_CHANNEL_RESET     _IOW('t', 2, unsigned int)
#define PB_TEST_MIPI_IN_RESET_STREAM  _IOW('t', 3, unsigned int)
#define PB_TEST_MIPI_OUT_RESET_STREAM _IOW('t', 4, unsigned int)

#endif /* __PAINTBOX_H__ */
