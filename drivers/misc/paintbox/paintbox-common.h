/*
 * Paintbox IPU Common Header
 *
 * Copyright (C) 2015 Google, Inc.
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

#ifndef __PAINTBOX_COMMON_H__
#define __PAINTBOX_COMMON_H__

#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/miscdevice.h>
#include <linux/paintbox.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>

#define IRQ_NO_DMA_CHANNEL   0xFF
#define DMA_NO_INTERRUPT     0xFF

#define RESOURCE_NAME_LEN    16

/* This timeout is the minimum wait time for a MIPI stream cleanup to
 * complete.
 */
#define MIPI_CLEANUP_TIMEOUT_US 200

/* This timeout is the minimum wait time for a DMA stop operation to complete
 * before resetting the channel.
 */
#define DMA_STOP_TIMEOUT_US 500

struct paintbox_data;

/* Data structure for all information related to a paintbox session.  A session
 * will be allocated on open() and deleted on release().
 */
struct paintbox_session {
	struct paintbox_data *dev;
	struct list_head irq_list;
	struct list_head dma_list;
	struct list_head stp_list;
	struct list_head lbp_list;
	struct list_head mipi_input_list;
	struct list_head mipi_output_list;
	struct list_head wait_list;
};

struct paintbox_debug_reg_entry;
struct paintbox_debug;

typedef int (*register_dump_t)(struct paintbox_debug *debug, char *buf,
		size_t len);
typedef int (*stats_dump_t)(struct paintbox_debug *debug, char *buf,
		size_t len);

typedef void (*register_write_t)(struct paintbox_debug_reg_entry *reg_entry,
		uint64_t val);
typedef uint64_t (*register_read_t)(struct paintbox_debug_reg_entry *reg_entry);


struct paintbox_debug_reg_entry {
	struct paintbox_debug *debug;
	struct dentry *debug_dentry;
	unsigned int reg_offset;
	register_write_t write;
	register_read_t read;
};

struct paintbox_debug {
	struct paintbox_data *pb;
	struct dentry *debug_dir;

	/* Debug FS entry used for dumping all registers in a block (STP, LBP,
	 * etc.) including field details.
	 */
	struct dentry *reg_dump_dentry;

	/* Debug FS entry used for dumping statistics in a block. */
	struct dentry *stats_dump_dentry;

	/* Array of Debug FS entries sized to number of registers in the block.
	 * (STP, LBP, etc.).  Each entry is used for read and write access
	 * to an individual register in the block.
	 */
	struct paintbox_debug_reg_entry *reg_entries;
	size_t num_reg_entries;
	const char *name;
	int resource_id;
	register_dump_t register_dump;
	stats_dump_t stats_dump;
};

struct paintbox_power {
	struct paintbox_debug debug;
	unsigned int active_core_count;
};

struct paintbox_io {
	struct paintbox_debug axi_debug;
	struct paintbox_debug apb_debug;
	void __iomem *axi_base;
	void __iomem *apb_base;
	unsigned int ipu_interrupts;
	unsigned int irq_activations;
	int irq;
	unsigned int stp_start;
	unsigned int bif_start;
	unsigned int mmu_start;
	unsigned int mipi_input_start;
	unsigned int mipi_output_start;
	uint32_t dma_mask;
	uint32_t stp_mask;
	uint32_t bif_mask;
	uint32_t mmu_mask;
	uint32_t mipi_input_mask;
	uint32_t mipi_output_mask;

	/* io_lock is used to protect the interrupt control registers */
	spinlock_t io_lock;
};

struct paintbox_mipi_interface {
	unsigned int interface_id;
	unsigned int num_streams;
	unsigned int inf_interrupts;

	/* protected by pb->io_ipu.mipi_lock */
	unsigned int active_stream_mask;

	struct paintbox_mipi_stream **streams;
};

struct paintbox_mipi_stream {
	/* Session list entry, A MIPI stream may be allocated to a session or
	 * released using the PB_ALLOCATE_MIPI_IN_STREAM,
	 * PB_RELEASE_MIPI_IN_STREAM, PB_ALLOCATE_MIPI_OUT_STREAM, and
	 * PB_RELEASE_MIPI_OUT_STREAM ioctls.
	 */
	struct list_head session_entry;

	struct paintbox_debug debug;
	struct paintbox_session *session;
	struct paintbox_mipi_interface *interface;
	struct paintbox_irq *irq;
	struct paintbox_data *pb;

	/* protected by pb->lock and pb->io_ipu.mipi_lock */
	struct paintbox_dma_channel *dma_channel;

	/* workqueue enqueue and cancel operations must be done with
	 * pb->io_ipu.mipi_lock held.
	 */
	struct delayed_work cleanup_work;

	unsigned int stream_id;
	union {
		struct {
			/* protected by pb->io_ipu.mipi_lock */
			unsigned int missed_sof_interrupt;
			unsigned int missed_ovf_interrupt;
			int32_t last_frame_number;
			bool frame_in_progress;

			struct {
				unsigned int sof_interrupts;
				unsigned int ovf_interrupts;
				unsigned int missed_sof_interrupts;
				unsigned int missed_ovf_interrupts;
			} stats;
		} input;
		struct {
			/* protected by pb->io_ipu.mipi_lock */
			unsigned int missed_eof_interrupt;

			struct {
				unsigned int eof_interrupts;
				unsigned int missed_eof_interrupts;
			} stats;
		} output;
	};
	uint32_t ctrl_offset;
	uint32_t select_offset;
	int32_t frame_count;
	int error;
	bool is_input;

	/* protected by pb->io_ipu.mipi_lock */
	bool cleanup_in_progress;
	bool free_running;
	bool last_frame;
	bool enabled;
	bool is_clean;
};

struct paintbox_io_ipu {
	struct paintbox_debug debug;
	void __iomem *ipu_base;
	struct paintbox_mipi_stream *mipi_input_streams;
	struct paintbox_mipi_stream *mipi_output_streams;
	struct paintbox_mipi_interface *mipi_input_interfaces;
	struct paintbox_mipi_interface *mipi_output_interfaces;
	unsigned int num_mipi_input_streams;
	unsigned int num_mipi_output_streams;
	unsigned int num_mipi_input_interfaces;
	unsigned int num_mipi_output_interfaces;

	/* mipi_lock is used to protect the mipi registers */
	spinlock_t mipi_lock;
};

enum paintbox_irq_src {
	IRQ_SRC_NONE,
	IRQ_SRC_DMA_CHANNEL,
	IRQ_SRC_STP,
	IRQ_SRC_MIPI_IN_STREAM,
	IRQ_SRC_MIPI_OUT_STREAM
};

/* It is possible that multiple interrupts for the an IRQ wait object may occur
 * during the wait period.  This could happen with MIPI overflow interrupts
 * where the MIPI interrupt may be triggered for the SOF and for the overflow
 * event.  The value for this define should be sufficient to hold the number of
 * interrupts that can occur between the first interrupt and when the runtime
 * thread can be woken up.  If too may interrupts occur in the wait period then
 * a kernel warning will be generated.
 */
#define IRQ_MAX_PER_WAIT_PERIOD 10

/* Data structure for information specific to an interrupt.
 * One entry will be allocated for each interrupt in the IPU's interrupt mask.
 *
 * Note that the interrupt id is stored with the paintbox_irq as a convenience
 * to avoid having to recover the interrupt id from the pb->irqs array when a
 * function only has the paintbox_irq object.
 *
 * All fields are protected by pb->irq_lock except the following:
 * interrupt_id - Only set at init.
 * entry, session - protected by pb->lock
 */
struct paintbox_irq {
	/* Session list entry, An IRQ waiter may be allocated to a session or
	 * released using the PB_ALLOCATE_INTERRUPT and PB_RELEASE_INTERRUPT
	 * ioctls.
	 */
	struct list_head session_entry;

	struct completion completion;
	struct paintbox_session *session;
	enum paintbox_irq_src source;
	unsigned int interrupt_id;
	union {
		struct paintbox_dma_channel *dma_channel;
		struct paintbox_stp *stp;
		struct paintbox_mipi_stream *mipi_stream;
	};

	/* The fields below are protected by pb->irq_lock */
	unsigned int data_count;
	int data[IRQ_MAX_PER_WAIT_PERIOD];
};

/* Per transfer information is stored in this structure. Transfers are stored in
 * either the pending, active, completed, or discard queues.
 */
struct paintbox_dma_transfer {
	struct list_head entry;
	enum dma_dram_buffer_type buffer_type;
	dma_addr_t dma_addr;
	void *buf_vaddr;
	struct sg_table *sg_table;
	struct dma_buf *dma_buf;
	struct dma_buf_attachment *attach;
	size_t len_bytes;

	/* Register shadows for channel registers.  These fields are set during
	 * configuration and are written out to the hardware registers when the
	 * transfer is started.
	 */
	uint32_t chan_mode;
	uint32_t chan_img_format;
	uint32_t chan_img_size;
	uint32_t chan_img_pos_low;
	uint32_t chan_img_pos_high;
	uint32_t chan_bif_xfer;
	uint32_t chan_img_layout_low;
	uint32_t chan_img_layout_high;
	uint32_t chan_va_low;
	uint32_t chan_va_high;
	uint32_t chan_va_bdry_low;
	uint32_t chan_va_bdry_high;
	uint32_t chan_noc_xfer_low;
	uint32_t chan_noc_xfer_high;
	uint32_t chan_node;

	/* error is protected by pb->dma.dma_lock */
	int error;

	enum dma_data_direction dir;
	bool notify_on_completion;
	bool auto_load_transfer;
};

/* Data structure for information specific to a DMA channel.
 * One entry will be allocated for each channel on a DMA controller.
 *
 * Note that the channel id is stored with the paintbox_dma as a convenience
 * to avoid having to recover the channel id from the pb->dmas array when a
 * function only has the paintbox_dma object.
 */
struct paintbox_dma_channel {
	/* Session list entry, A DMA channel may be allocated to a session or
	 * released using the PB_ALLOCATE_DMA_CHANNEL and PB_RELEASE_DMA_CHANNEL
	 * ioctls.
	 */
	struct list_head session_entry;

	struct paintbox_debug debug;
	struct paintbox_session *session;

	/* protected by pb->lock and pb->dma.dma_lock */
	struct paintbox_mipi_stream *mipi_stream;

	struct completion stop_completion;
	unsigned int channel_id;
	struct paintbox_irq *irq;

	/* Access to the pending, active, and completed queues is controlled by
	 * pb->dma.dma_lock.
	 */
	struct list_head pending_list;
	struct list_head active_list;
	struct list_head completed_list;

	/* Access to the count and stop fields is controlled by pb->dma.dma_lock
	 */
	int pending_count;
	int active_count;
	int completed_count;
	bool stop_request;

	struct {
		unsigned int irq_activations;
		unsigned int eof_interrupts;
		unsigned int va_interrupts;
		unsigned int reported_completions;
		unsigned int reported_discards;
	} stats;
};

struct paintbox_dma {
	struct paintbox_debug debug;
	struct paintbox_dma_channel *channels;
	unsigned int num_channels;
	void __iomem *dma_base;

	/* Access to the discard queue and discard count is controlled by
	 * pb->dma.dma_lock.
	 */
	struct list_head discard_list;
	unsigned int discard_count;

	/* dma_lock protects access to DMA transfer queues and registers. */
	spinlock_t dma_lock;
};

/* Data structure for information specific to a Stencil Processor.
 * One entry will be allocated for each processor on the IPU.
 *
 * Note that the processor id is stored with the paintbox_stp as a convenience
 * to avoid having to recover the processor_id from the pb->stps array when a
 * function only has the paintbox_stp object.
 */
struct paintbox_stp {
	/* Session list entry, A stencil processor may be allocated to a session
	 * or released using the PB_ALLOCATE_PROCESSOR and PB_RELEASE_PROCESSOR
	 * ioctls.
	 */
	struct list_head session_entry;

	struct paintbox_debug debug;
	struct paintbox_session *session;
	struct paintbox_irq *irq;
	unsigned int stp_id;
	unsigned int interrupt_count;
	unsigned int inst_mem_size_in_instructions;
	unsigned int scalar_mem_size_in_words;
	unsigned int const_mem_size_in_words;
	unsigned int vector_mem_size_in_words;
	unsigned int halo_mem_size_in_words;
	bool pm_enabled;
	bool inited;
};

struct paintbox_lbp;

/* Data structure for information specific to a Line Buffer.
 * One entry will be allocated for each line buffer in a pool.
 */
struct paintbox_lb {
	struct paintbox_debug debug;
	struct paintbox_lbp *lbp;
	unsigned int lb_id;
	unsigned int fb_rows;
	unsigned int num_channels;
	unsigned int num_read_ptrs;
	unsigned int width_pixels;
	unsigned int height_pixels;
	unsigned int sb_rows;
	unsigned int sb_cols;
	bool configured;
};

/* Data structure for information specific to a Line Buffer Pool.
 * One entry will be allocated for each pool on the IPU.
 *
 * Note that the pool id is stored with the paintbox_lbp as a convenience to
 * avoid having to recover the pool id from the pb->lbps array when a
 * function only has the paintbox_lbp object.
 */
struct paintbox_lbp {
	/* Session list entry, A line buffer pool may be allocated to a session
	 * or released using the PB_ALLOCATE_LINE_BUFFER_POOL and
	 * PB_RELEASE_LINE_BUFFER_POOL ioctls.
	 */
	struct list_head session_entry;

	struct paintbox_debug debug;
	struct paintbox_session *session;
	struct paintbox_lb *lbs;
	unsigned int pool_id;
	bool pm_enabled;
};

struct paintbox_lbp_caps {
	uint32_t mem_size_bytes;
	uint32_t max_fb_rows;
	uint32_t max_lbs;
	uint32_t max_rptrs;
	uint32_t max_channels;
};

struct paintbox_waiter {
	struct list_head session_entry;
	struct interrupt_wait *wait;
	struct completion completion;
};

struct paintbox_data {
	struct mutex lock;

	/* irq_lock is used to protect data structures shared with the IRQ
	 * handlers.
	 */
	spinlock_t irq_lock;

	/* stp_lock is used to protect access to the STP registers betweeen
	 * threads and the STP interrupt handler.
	 */
	spinlock_t stp_lock;
	void __iomem *reg_base;
	void __iomem *lbp_base;
	void __iomem *stp_base;
#ifdef CONFIG_PAINTBOX_SIMULATOR_SUPPORT
	void __iomem *sim_base;
#endif
#ifdef CONFIG_PAINTBOX_FPGA_SUPPORT
	void __iomem *fpga_reg_base;
#endif
	struct miscdevice misc_device;
	struct platform_device *pdev;
	struct dentry *debug_root;
	struct dentry *regs_dentry;

	struct paintbox_power power;
	struct paintbox_io io;
	struct paintbox_io_ipu io_ipu;
	struct paintbox_dma dma;
	struct paintbox_irq *irqs;
	struct paintbox_stp *stps;
	struct paintbox_lbp_caps lbp_caps;
	struct paintbox_lbp *lbps;
	struct ipu_capabilities caps;
	size_t vdbg_log_len;
	char *vdbg_log;
};

static inline uint8_t lbp_id_to_noc_id(uint8_t lbp_id)
{
	return (lbp_id << 1) + 1;
}

static inline uint8_t stp_id_to_noc_id(uint8_t stp_id)
{
	return stp_id << 1;
}

void paintbox_alloc_debug_buffer(struct paintbox_data *pb, size_t len);


#endif /* __PAINTBOX_COMMON_H__ */
