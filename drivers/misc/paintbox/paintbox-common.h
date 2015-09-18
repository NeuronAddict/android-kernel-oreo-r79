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

#include <linux/miscdevice.h>
#include <linux/paintbox.h>
#include <linux/platform_device.h>

#define MAX_REUSE_ROWS       15
#define MAX_CHAN_OFFSET      15

#define IRQ_NO_DMA_CHANNEL   0xFF
#define DMA_NO_INTERRUPT     0xFF

#define RESOURCE_NAME_LEN    7

/* TODO(ahampson): This information should be retrieved through CAPS registers
 * on the FPGA but there are currently no CAPS registers.  In the interim this
 * information will be hardcoded here.
 */
#define FPGA_VERSION         2
#define FPGA_STP_COUNT       1
#define FPGA_INT_COUNT       2
#define FPGA_LBP_COUNT       3
#define FPGA_LB_COUNT        1
#define FPGA_DMA_COUNT       2

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
};

struct paintbox_io {
	struct dentry *debug_dir;
	struct dentry *regs_dentry;
	void __iomem *axi_base;
	void __iomem *apb_base;
	int irq;
	uint32_t dma_mask;
#ifdef CONFIG_PAINTBOX_FPGA_SUPPORT
	void __iomem *fpga_base;
	struct dentry *fpga_debug_dir;
	struct dentry *fpga_regs_dentry;
#endif
};

/* Data structure for information specific to an interrupt.
 * One entry will be allocated for each interrupt in the IPU's interrupt mask.
 *
 * Note that the interrupt id is stored with the paintbox_irq as a convenience
 * to avoid having to recovery the interrupt id from the pb->irqs array when a
 * function only has the paintbox_irq object.
 */
struct paintbox_irq {
	struct list_head entry;
	struct completion completion;
	struct paintbox_session *session;
	uint8_t channel_id;
	uint8_t interrupt_id;
	int error;
};

/* TODO(ahampson):  There will eventually be a queue of transfers for each
 * channel.  Per transfer information will be stored in this structure.  In the
 * interim there will be a single transfer structure assocated with each
 * channel.  After the interrupt refactor the proper queuing can be
 * implemented.
 */
struct paintbox_dma_transfer {
	struct list_head entry;
	dma_addr_t buf_paddr;
	void *buf_vaddr;
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
};

#ifdef CONFIG_PAINTBOX_FPGA_SUPPORT
struct paintbox_fpga_dma_channel {
	void __iomem *xilinx_base;
	uint32_t requested_transfer_len;
	uint32_t transfer_len;
	uint32_t bytes_transferred;
	bool to_ipu;
};

struct paintbox_fpga_dma {
	struct dentry *xilinx_debug_dir;
	struct dentry *xilinx_regs_dentry;
};
#endif

/* Data structure for information specific to a DMA channel.
 * One entry will be allocated for each channel on a DMA controller.
 *
 * Note that the channel id is stored with the paintbox_dma as a convenience
 * to avoid having to recovery the channel id from the pb->dmas array when a
 * function only has the paintbox_dma object.
 */
struct paintbox_dma_channel {
	struct list_head entry;
	struct paintbox_data *pb;
	struct paintbox_session *session;
#ifdef CONFIG_PAINTBOX_FPGA_SUPPORT
	struct paintbox_fpga_dma_channel fpga;
#endif
	/* TODO(ahampson):  Initially only a single transfer will be supported.
	 * a transfer queue will be implemented in the future.
	 */
	struct paintbox_dma_transfer transfer;
	struct dentry *debug_dir;
	struct dentry *regs_dentry;
	uint8_t channel_id;
	uint8_t interrupt_id;
};

struct paintbox_dma {
	struct paintbox_dma_channel *channels;
	unsigned int num_channels;
	void __iomem *dma_base;
	struct dentry *debug_dir;
	struct dentry *regs_dentry;
#ifdef CONFIG_PAINTBOX_FPGA_SUPPORT
	struct paintbox_fpga_dma fpga;
#endif
};

/* Data structure for information specific to a Stencil Processor.
 * One entry will be allocated for each processor on the IPU.
 *
 * Note that the processor id is stored with the paintbox_stp as a convenience
 * to avoid having to recovery the processor_id from the pb->stps array when a
 * function only has the paintbox_stp object.
 */
struct paintbox_stp {
	struct list_head entry;
	struct paintbox_data *pb;
	struct paintbox_session *session;
	struct dentry *debug_dir;
	struct dentry *regs_dentry;
	uint8_t stp_id;
};

struct paintbox_lbp;

/* Data structure for information specific to a Line Buffer.
 * One entry will be allocated for each line buffer in a pool.
 */
struct paintbox_lb {
	struct paintbox_lbp *lbp;
	struct dentry *debug_dir;
	struct dentry *regs_dentry;
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
 * avoid having to recovery the pool id from the pb->lbps array when a
 * function only has the paintbox_lbp object.
 */
struct paintbox_lbp {
	struct list_head entry;
	struct paintbox_data *pb;
	struct paintbox_session *session;
	struct paintbox_lb *lbs;
	struct dentry *debug_dir;
	struct dentry *regs_dentry;
	uint32_t mem_size;
	uint32_t max_fb_rows;
	uint32_t max_lbs;
	uint32_t max_rptrs;
	uint32_t max_channels;
	unsigned int pool_id;
};

struct paintbox_data {
	struct mutex lock;

	/* irq_lock is used to protect data structures shared with the IRQ
	 * handlers.
	 */
	spinlock_t irq_lock;
	void __iomem *reg_base;
	void __iomem *lbp_base;
	void __iomem *stp_base;
#ifdef CONFIG_PAINTBOX_SIMULATOR_SUPPORT
	void __iomem *sim_base;
#endif
	struct miscdevice misc_device;
	struct platform_device *pdev;
	struct dentry *debug_root;
	struct dentry *regs_dentry;

	struct paintbox_io io;
	struct paintbox_dma dma;
	struct paintbox_irq *irqs;
	struct paintbox_stp *stps;
	struct paintbox_lbp *lbps;
	struct ipu_capabilities caps;
	size_t vdbg_log_len;
	char *vdbg_log;
};

/* The caller to this function must hold pb->lock */
struct paintbox_irq *get_interrupt(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned int interrupt_id,
		int *err);

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
