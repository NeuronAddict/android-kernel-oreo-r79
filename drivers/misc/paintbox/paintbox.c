/*
 * Core driver for the Paintbox programmable IPU
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

#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#ifdef CONFIG_MNH_THERMAL
#include <linux/mnh_freq_cooling.h>
#endif
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <soc/mnh/mnh-trace.h>
#include <uapi/paintbox.h>

#include "paintbox-bif.h"
#include "paintbox-common.h"
#include "paintbox-debug.h"
#include "paintbox-dma.h"
#include "paintbox-fpga.h"
#include "paintbox-io.h"
#include "paintbox-irq.h"
#include "paintbox-lbp.h"
#include "paintbox-mipi.h"
#include "paintbox-mmu.h"
#include "paintbox-pmon.h"
#include "paintbox-power.h"
#include "paintbox-regs.h"
#include "paintbox-sim-regs.h"
#include "paintbox-stp.h"
#include "paintbox-stp-pc-histogram.h"
#include "paintbox-stp-sim.h"
#include "paintbox-stp-sram.h"

static int paintbox_open(struct inode *ip, struct file *fp)
{
	struct paintbox_session *session;
	struct paintbox_data *pb;
	struct miscdevice *m = fp->private_data;

#ifdef CONFIG_PAINTBOX_DEBUG
	ktime_t start_time;
#endif

	pb = container_of(m, struct paintbox_data, misc_device);
#ifdef CONFIG_PAINTBOX_DEBUG
	if (pb->stats.ioctl_time_enabled)
		start_time = ktime_get_boottime();
#endif

	session = kzalloc(sizeof(struct paintbox_session), GFP_KERNEL);
	if (!session)
		return -ENOMEM;

	session->dev = pb;
	INIT_LIST_HEAD(&session->irq_list);
	INIT_LIST_HEAD(&session->dma_list);
	INIT_LIST_HEAD(&session->stp_list);
	INIT_LIST_HEAD(&session->lbp_list);
	INIT_LIST_HEAD(&session->mipi_input_list);
	INIT_LIST_HEAD(&session->mipi_output_list);
	INIT_LIST_HEAD(&session->wait_list);

	init_completion(&session->release_completion);

	fp->private_data = session;

	mutex_lock(&pb->lock);

	pb->session_count++;

	mutex_unlock(&pb->lock);

#ifdef CONFIG_PAINTBOX_DEBUG
	if (pb->stats.ioctl_time_enabled)
		paintbox_debug_log_non_ioctl_stats(pb, PB_STATS_OPEN, start_time,
				ktime_get_boottime(), 0);
#endif
	return 0;
}

static int paintbox_release(struct inode *ip, struct file *fp)
{
	struct paintbox_session *session = fp->private_data;
	struct paintbox_data *pb = session->dev;
	struct paintbox_irq *irq, *irq_next;
#ifdef CONFIG_PAINTBOX_DEBUG
	ktime_t start_time;

	if (pb->stats.ioctl_time_enabled)
		start_time = ktime_get_boottime();
#endif

	mutex_lock(&pb->lock);

	/* TODO: Cleanup release sequence.  b/62372748 */

	paintbox_mipi_release(pb, session);
	paintbox_dma_release(pb, session);
	paintbox_stp_release(pb, session);

	/* Disable any interrupts associated with the session */
	list_for_each_entry_safe(irq, irq_next, &session->irq_list,
			session_entry)
		release_interrupt(pb, session, irq);

	paintbox_lbp_release(pb, session);

	paintbox_irq_wait_for_release_complete(pb, session);

	/* free any pmon allocations */
	if (pb->bif.pmon_session == session)
		pb->bif.pmon_session = NULL;

	if (pb->mmu.pmon_session == session)
		pb->mmu.pmon_session = NULL;

	if (pb->dma.pmon_session == session)
		pb->dma.pmon_session = NULL;

	if (WARN_ON(--pb->session_count < 0))
		pb->session_count = 0;

	mutex_unlock(&pb->lock);

	kfree(session);

#ifdef CONFIG_PAINTBOX_DEBUG
	if (pb->stats.ioctl_time_enabled)
		paintbox_debug_log_non_ioctl_stats(pb, PB_STATS_CLOSE, start_time,
				ktime_get_boottime(), 0);
#endif

	return 0;
}

static long paintbox_get_caps_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct ipu_capabilities caps;
	uint32_t version;

	memset(&caps, 0, sizeof(caps));

	caps.num_lbps = pb->lbp.num_lbps;
	caps.num_stps = pb->stp.num_stps;
	caps.num_dma_channels = pb->dma.num_channels;
	caps.num_interrupts = pb->io.num_interrupts;

	version = readl(pb->reg_base + IPU_VERSION);
	caps.version_major = (version & IPU_VERSION_MAJOR_MASK) >>
			IPU_VERSION_MAJOR_SHIFT;
	caps.version_minor = (version & IPU_VERSION_MINOR_MASK) >>
			IPU_VERSION_MAJOR_SHIFT;
	caps.version_build = version & IPU_VERSION_INCR_MASK;
	caps.is_fpga = !!(version & IPU_VERSION_FPGA_BUILD_MASK);

#ifdef CONFIG_PAINTBOX_SIMULATOR_SUPPORT
	caps.is_simulator = true;
#endif

#ifdef CONFIG_PAINTBOX_IOMMU
	caps.iommu_enabled = pb->mmu.iommu_enabled;
#endif

	caps.hardware_id = pb->hardware_id;

	if (copy_to_user((void __user *)arg, &caps, sizeof(caps)))
		return -EFAULT;

	return 0;
}

#ifdef CONFIG_MNH_THERMAL
static long paintbox_ipu_reset_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	int ret;

	mutex_lock(&pb->lock);

	if (pb->session_count > 1) {
		dev_warn(&pb->pdev->dev,
				"%s: ignoring reset request: multiple active sessions\n",
				__func__);
		ret = -EBUSY;
		goto unlock;
	}

	paintbox_io_disable_interrupt(pb, ~0ULL);
	mnh_ipu_reset();
	paintbox_io_apb_post_ipu_reset(pb);
	/* TODO(showarth): bif post ipu reset */
	paintbox_dma_post_ipu_reset(pb);
	paintbox_lbp_post_ipu_reset(pb);
	paintbox_mipi_post_ipu_reset(pb);
	/* TODO(showarth): mmu post ipu reset */
	paintbox_stp_post_ipu_reset(pb);
	ret = 0;
unlock:
	mutex_unlock(&pb->lock);

	return ret;
}
#endif

static long paintbox_ioctl(struct file *fp, unsigned int cmd,
		unsigned long arg)
{
	struct paintbox_session *session = fp->private_data;
	struct paintbox_data *pb = session->dev;
	int ret;
#ifdef CONFIG_PAINTBOX_DEBUG
	ktime_t start_time;

	if (pb->stats.ioctl_time_enabled)
		start_time = ktime_get_boottime();
#endif

	switch (cmd) {
	case PB_GET_IPU_CAPABILITIES:
		ret = paintbox_get_caps_ioctl(pb, session, arg);
		break;
	case PB_ALLOCATE_INTERRUPT:
		ret = allocate_interrupt_ioctl(pb, session, arg);
		break;
	case PB_WAIT_FOR_INTERRUPT:
		ret = wait_for_interrupt_ioctl(pb, session, arg);
		break;
	case PB_RELEASE_INTERRUPT:
		ret = release_interrupt_ioctl(pb, session, arg);
		break;
	case PB_FLUSH_INTERRUPTS:
		ret = paintbox_flush_interrupt_ioctl(pb, session, arg);
		break;
	case PB_FLUSH_ALL_INTERRUPTS:
		ret = paintbox_flush_all_interrupts_ioctl(pb, session, arg);
		break;
	case PB_ALLOCATE_DMA_CHANNEL:
		ret = allocate_dma_channel_ioctl(pb, session, arg);
		break;
	case PB_BIND_DMA_INTERRUPT:
		ret = bind_dma_interrupt_ioctl(pb, session, arg);
		break;
	case PB_UNBIND_DMA_INTERRUPT:
		ret = unbind_dma_interrupt_ioctl(pb, session, arg);
		break;
	case PB_START_DMA_TRANSFER:
		ret = start_dma_transfer_ioctl(pb, session, arg);
		break;
	case PB_STOP_DMA_TRANSFER:
		ret = stop_dma_transfer_ioctl(pb, session, arg);
		break;
	case PB_RELEASE_DMA_CHANNEL:
		ret = release_dma_channel_ioctl(pb, session, arg);
		break;
	case PB_GET_COMPLETED_UNREAD_COUNT:
		ret = get_completed_transfer_count_ioctl(pb, session, arg);
		break;
	case PB_FLUSH_DMA_TRANSFERS:
		ret = flush_dma_transfers_ioctl(pb, session, arg);
		break;
	case PB_ALLOCATE_LINE_BUFFER_POOL:
		ret = allocate_lbp_ioctl(pb, session, arg);
		break;
	case PB_SETUP_LINE_BUFFER:
		ret = setup_lb_ioctl(pb, session, arg);
		break;
	case PB_RELEASE_LINE_BUFFER_POOL:
		ret = release_lbp_ioctl(pb, session, arg);
		break;
	case PB_RESET_LINE_BUFFER_POOL:
		ret = reset_lbp_ioctl(pb, session, arg);
		break;
	case PB_RESET_LINE_BUFFER:
		ret = reset_lb_ioctl(pb, session, arg);
		break;
	case PB_WRITE_LBP_MEMORY:
		ret = write_lbp_memory_ioctl(pb, session, arg);
		break;
	case PB_READ_LBP_MEMORY:
		ret = read_lbp_memory_ioctl(pb, session, arg);
		break;
	case PB_ALLOCATE_PROCESSOR:
		ret = allocate_stp_ioctl(pb, session, arg);
		break;
	case PB_SETUP_PROCESSOR:
		ret = setup_stp_ioctl(pb, session, arg);
		break;
	case PB_WRITE_STP_MEMORY:
		ret = write_stp_scalar_sram_ioctl(pb, session, arg);
		break;
	case PB_READ_STP_MEMORY:
		ret = read_stp_scalar_sram_ioctl(pb, session, arg);
		break;
	case PB_WRITE_VECTOR_SRAM_COORDINATES:
		ret = write_stp_vector_sram_coordinates_ioctl(pb, session, arg);
		break;
	case PB_WRITE_VECTOR_SRAM_REPLICATE:
		ret = write_stp_vector_sram_replicate_ioctl(pb, session, arg);
		break;
	case PB_READ_VECTOR_SRAM_COORDINATES:
		ret = read_stp_vector_sram_coordinates_ioctl(pb, session, arg);
		break;
	case PB_START_PROCESSOR:
		ret = start_stp_ioctl(pb, session, arg);
		break;
	case PB_STOP_PROCESSOR:
		ret = stop_stp_ioctl(pb, session, arg);
		break;
	case PB_RESUME_PROCESSOR:
		ret = resume_stp_ioctl(pb, session, arg);
		break;
	case PB_RESET_PROCESSOR:
		ret = reset_stp_ioctl(pb, session, arg);
		break;
	case PB_RESET_ALL_PROCESSORS:
		ret = paintbox_reset_all_stp_ioctl(pb, session);
		break;
	case PB_GET_PROGRAM_STATE:
		ret = get_program_state_ioctl(pb, session, arg);
		break;
	case PB_GET_ALL_PROCESSOR_STATES:
		ret = paintbox_get_all_processor_states(pb, session, arg);
		break;
	case PB_RELEASE_PROCESSOR:
		ret = release_stp_ioctl(pb, session, arg);
		break;
	case PB_WAIT_FOR_ALL_PROCESSOR_IDLE:
#ifdef CONFIG_PAINTBOX_SIMULATOR_SUPPORT
		ret = sim_wait_for_idle_ioctl(pb, session, arg);
#else
		/* The simulator requires additional processing after the DMA
		 * interrupt before the processor goes idle.  This processing
		 * is fast enough on the actual hardware that we do not need
		 * to poll for idle.
		 */
		ret = 0;
#endif
		break;
	case PB_GET_PROCESSOR_IDLE:
#ifdef CONFIG_PAINTBOX_SIMULATOR_SUPPORT
		ret = sim_get_stp_idle_ioctl(pb, session, arg);
#else
		ret = -EINVAL;
#endif
		break;
	case PB_BIND_STP_INTERRUPT:
		ret = bind_stp_interrupt_ioctl(pb, session, arg);
		break;
	case PB_UNBIND_STP_INTERRUPT:
		ret = unbind_stp_interrupt_ioctl(pb, session, arg);
		break;
	case PB_STP_PC_HISTOGRAM_CLEAR:
		ret = stp_pc_histogram_clear_ioctl(pb, session, arg);
		break;
	case PB_STP_PC_HISTOGRAM_ENABLE:
		ret = stp_pc_histogram_enable_ioctl(pb, session, arg);
		break;
	case PB_STP_PC_HISTOGRAM_READ:
		ret = stp_pc_histogram_read_ioctl(pb, session, arg);
		break;
	case PB_SETUP_DMA_TRANSFER:
		ret = setup_dma_transfer_ioctl(pb, session, arg);
		break;
	case PB_READ_DMA_TRANSFER:
		ret = read_dma_transfer_ioctl(pb, session, arg);
		break;
	case PB_ALLOCATE_MIPI_IN_STREAM:
		ret = allocate_mipi_input_stream_ioctl(pb, session, arg);
		break;
	case PB_RELEASE_MIPI_IN_STREAM:
		ret = release_mipi_stream_ioctl(pb, session, arg, true);
		break;
	case PB_SETUP_MIPI_IN_STREAM:
		ret = setup_mipi_stream_ioctl(pb, session, arg, true);
		break;
	case PB_ENABLE_MIPI_IN_STREAM:
		ret = enable_mipi_stream_ioctl(pb, session, arg, true);
		break;
	case PB_DISABLE_MIPI_IN_STREAM:
		ret = disable_mipi_stream_ioctl(pb, session, arg, true);
		break;
	case PB_GET_MIPI_IN_FRAME_NUMBER:
		ret = get_mipi_frame_number_ioctl(pb, session, arg);
		break;
	case PB_CLEANUP_MIPI_IN_STREAM:
		ret = cleanup_mipi_stream_ioctl(pb, session, arg, true);
		break;
	case PB_ALLOCATE_MIPI_OUT_STREAM:
		ret = allocate_mipi_output_stream_ioctl(pb, session, arg);
		break;
	case PB_RELEASE_MIPI_OUT_STREAM:
		ret = release_mipi_stream_ioctl(pb, session, arg, false);
		break;
	case PB_SETUP_MIPI_OUT_STREAM:
		ret = setup_mipi_stream_ioctl(pb, session, arg, false);
		break;
	case PB_ENABLE_MIPI_OUT_STREAM:
		ret = enable_mipi_stream_ioctl(pb, session, arg, false);
		break;
	case PB_DISABLE_MIPI_OUT_STREAM:
		ret = disable_mipi_stream_ioctl(pb, session, arg, false);
		break;
	case PB_CLEANUP_MIPI_OUT_STREAM:
		ret = cleanup_mipi_stream_ioctl(pb, session, arg, false);
		break;
	case PB_BIND_MIPI_IN_INTERRUPT:
		ret = bind_mipi_interrupt_ioctl(pb, session, arg, true);
		break;
	case PB_UNBIND_MIPI_IN_INTERRUPT:
		ret = unbind_mipi_interrupt_ioctl(pb, session, arg, true);
		break;
	case PB_BIND_MIPI_OUT_INTERRUPT:
		ret = bind_mipi_interrupt_ioctl(pb, session, arg, false);
		break;
	case PB_UNBIND_MIPI_OUT_INTERRUPT:
		ret = unbind_mipi_interrupt_ioctl(pb, session, arg, false);
		break;
	case PB_ENABLE_MIPI_IN_STREAMS:
		ret = paintbox_mipi_enable_multiple_ioctl(pb, session, arg,
				true /* is_input */);
		break;
	case PB_DISABLE_MIPI_IN_STREAMS:
		ret = paintbox_mipi_disable_multiple_ioctl(pb, session, arg,
				true /* is_input */);
		break;
	case PB_ENABLE_MIPI_OUT_STREAMS:
		ret = paintbox_mipi_enable_multiple_ioctl(pb, session, arg,
				false /* is_input */);
		break;
	case PB_DISABLE_MIPI_OUT_STREAMS:
		ret = paintbox_mipi_disable_multiple_ioctl(pb, session, arg,
				false /* is_input */);
		break;
	case PB_WAIT_FOR_MIPI_INPUT_QUIESCENCE:
		ret = paintbox_mipi_input_wait_for_quiescence_ioctl(pb, session,
				arg);
		break;
#ifdef CONFIG_MNH_THERMAL
	case PB_RESET_IPU:
		ret = paintbox_ipu_reset_ioctl(pb, session, arg);
		break;
#endif
	case PB_PMON_ALLOCATE:
		ret = pmon_allocate_ioctl(pb, session, arg);
		break;
	case PB_PMON_RELEASE:
		ret = pmon_release_ioctl(pb, session, arg);
		break;
	case PB_PMON_CONFIG_WRITE:
		ret = pmon_config_write_ioctl(pb, session, arg);
		break;
	case PB_PMON_DATA_READ:
		ret = pmon_data_read_ioctl(pb, session, arg);
		break;
	case PB_PMON_DATA_WRITE:
		ret = pmon_data_write_ioctl(pb, session, arg);
		break;
	case PB_PMON_ENABLE:
		ret = pmon_enable_ioctl(pb, session, arg);
		break;
#ifdef CONFIG_PAINTBOX_TEST_SUPPORT
	case PB_TEST_DMA_RESET:
		ret = dma_test_reset_ioctl(pb, session, arg);
		break;
	case PB_TEST_DMA_CHANNEL_RESET:
		ret = dma_test_channel_reset_ioctl(pb, session, arg);
		break;
	case PB_TEST_MIPI_IN_RESET_STREAM:
		ret = mipi_test_stream_reset_ioctl(pb, session, arg, true);
		break;
	case PB_TEST_MIPI_OUT_RESET_STREAM:
		ret = mipi_test_stream_reset_ioctl(pb, session, arg, false);
		break;
	case PB_TEST_LBP_BROADCAST_WRITE_MEMORY:
		ret = lbp_test_broadcast_write_memory_ioctl(pb, session, arg);
		break;
#else
	case PB_TEST_DMA_RESET:
	case PB_TEST_DMA_CHANNEL_RESET:
	case PB_TEST_MIPI_IN_RESET_STREAM:
	case PB_TEST_MIPI_OUT_RESET_STREAM:
	case PB_TEST_LBP_BROADCAST_WRITE_MEMORY:
		ret = -EINVAL;
		break;
#endif
	default:
		dev_err(&pb->pdev->dev, "%s: unknown ioctl 0x%0x\n", __func__,
				cmd);
		return -EINVAL;
	}

#ifdef CONFIG_PAINTBOX_DEBUG
	if (pb->stats.ioctl_time_enabled)
		paintbox_debug_log_ioctl_stats(pb, cmd, start_time,
				ktime_get_boottime());
#endif

	return ret;
}

static const struct file_operations paintbox_fops = {
	.owner = THIS_MODULE,
	.open = paintbox_open,
	.release = paintbox_release,
	.unlocked_ioctl = paintbox_ioctl,
};

static int paintbox_get_capabilities(struct paintbox_data *pb)
{
	uint64_t hardware_id;
	uint32_t val;
	uint8_t major, minor, build;
	bool is_fpga;
	int ret;

	val = readl(pb->reg_base + IPU_VERSION);
	major = (val & IPU_VERSION_MAJOR_MASK) >> IPU_VERSION_MAJOR_SHIFT;
	minor = (val & IPU_VERSION_MINOR_MASK) >> IPU_VERSION_MAJOR_SHIFT;
	build = val & IPU_VERSION_INCR_MASK;
	is_fpga = !!(val & IPU_VERSION_FPGA_BUILD_MASK);

	val = readl(pb->reg_base + IPU_CAP);
	pb->stp.num_stps = val & IPU_CAP_NUM_STP_MASK;
	pb->lbp.num_lbps = (val & IPU_CAP_NUM_LBP_MASK) >>
		IPU_CAP_NUM_LBP_SHIFT;

	pb->dma.num_channels = readl(pb->dma.dma_base + DMA_CAP0) &
			DMA_CAP0_MAX_DMA_CHAN_MASK;

	val = readl(pb->io_ipu.ipu_base + MPI_CAP);
	pb->io_ipu.num_mipi_input_streams = (val & MPI_CAP_MAX_STRM_MASK) >>
			MPI_CAP_MAX_STRM_SHIFT;
	pb->io_ipu.num_mipi_input_interfaces = val & MPI_CAP_MAX_IFC_MASK;

	val = readl(pb->io_ipu.ipu_base + MPO_CAP);
	pb->io_ipu.num_mipi_output_streams = (val & MPO_CAP_MAX_STRM_MASK) >>
			MPO_CAP_MAX_STRM_SHIFT;
	pb->io_ipu.num_mipi_output_interfaces = val & MPO_CAP_MAX_IFC_MASK;

	ret = of_property_read_u64(pb->pdev->dev.of_node, "hardware-id",
			&hardware_id);
	if (ret < 0) {
		dev_err(&pb->pdev->dev,
				"%s: hardware-id not set in device tree, err %d\n",
				__func__, ret);
		return ret;
	}

	pb->hardware_id = (uint32_t)hardware_id;

#if defined(CONFIG_PAINTBOX_SIMULATOR_SUPPORT)
	dev_dbg(&pb->pdev->dev,
			"Paintbox IPU Version %u.%u.%u Simulator Hardware ID %u\n",
			major, minor, build, pb->hardware_id);
#else
	dev_dbg(&pb->pdev->dev,
			"Paintbox IPU Version %u.%u.%u %s Hardware ID %u\n",
			major, minor, build, is_fpga ? "FPGA" : "",
			pb->hardware_id);
#endif
	dev_dbg(&pb->pdev->dev,
			"STPs %u LBPs %u DMA Channels %u MIPI Input Streams %u MIPI Output Streams %u\n",
			pb->stp.num_stps, pb->lbp.num_lbps,
			pb->dma.num_channels, pb->io_ipu.num_mipi_input_streams,
			pb->io_ipu.num_mipi_output_streams);
	return 0;
}

static int paintbox_probe(struct platform_device *pdev)
{
	int ret;
	struct resource *r;
	struct paintbox_data *pb;
#ifdef CONFIG_PAINTBOX_DEBUG
	ktime_t start_time = ktime_get_boottime();
#endif
	mnh_trace(MNH_TRACE_PAINTBOX_PROBE);

	pb = devm_kzalloc(&pdev->dev, sizeof(*pb), GFP_KERNEL);
	if (pb == NULL)
		return -ENOMEM;

	pb->pdev = pdev;

	mutex_init(&pb->lock);

	spin_lock_init(&pb->irq_lock);

	paintbox_debug_init(pb);

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (r == NULL) {
		dev_err(&pdev->dev, "platform_get_resource failed\n");
		return -ENODEV;
	}

	pb->reg_base = devm_ioremap(&pdev->dev, r->start, resource_size(r));
	if (pb->reg_base == NULL) {
		dev_err(&pdev->dev, "unable to remap MMIO\n");
		return -ENOMEM;
	}

	ret = paintbox_fpga_init(pb);
	if (ret < 0)
		return ret;

	pb->io.irq = platform_get_irq(pdev, 0);
	if (pb->io.irq < 0) {
		dev_err(&pdev->dev, "platform_get_irq failed\n");
		return -ENODEV;
	}

	platform_set_drvdata(pdev, pb);

#if CONFIG_PAINTBOX_VERSION_MAJOR >= 1
	pb->io.aon_base = pb->reg_base + IPU_CSR_AON_OFFSET;
#else
	/* Easel doesn't have an AON group. But instead of ifdef'ing accesses
	 * to all of the registers which have moved from the APB group to the
	 * AON group in Canvas, we use the AON base across the board and point
	 * it to the APB base for Easel.
	 */
	pb->io.aon_base = pb->reg_base + IPU_CSR_APB_OFFSET;
#endif
	pb->io.apb_base = pb->reg_base + IPU_CSR_APB_OFFSET;
	pb->io.axi_base = pb->reg_base + IPU_CSR_AXI_OFFSET;
	pb->io_ipu.ipu_base = pb->reg_base + IPU_CSR_IO_OFFSET;
	pb->dma.dma_base = pb->reg_base + IPU_CSR_DMA_OFFSET;
	pb->stp.reg_base = pb->reg_base + IPU_CSR_STP_OFFSET;
	pb->lbp.reg_base = pb->reg_base + IPU_CSR_LBP_OFFSET;
#ifdef CONFIG_PAINTBOX_SIMULATOR_SUPPORT
	pb->sim_base = pb->reg_base + SIM_GROUP_OFFSET;
#endif

	paintbox_pm_enable_io(pb);

	ret = paintbox_get_capabilities(pb);
	if (ret < 0)
		return ret;

	ret = paintbox_mipi_init(pb);
	if (ret < 0)
		return ret;

	ret = paintbox_dma_init(pb);
	if (ret < 0)
		return ret;

	ret = paintbox_lbp_init(pb);
	if (ret < 0)
		return ret;

	ret = paintbox_stp_init(pb);
	if (ret < 0)
		return ret;

	/* Initialize the IO APB block after the blocks that can generate
	 * interrupts.  All interrupt sources need to be initialized first.
	 */
	ret = paintbox_io_apb_init(pb);
	if (ret < 0)
		return ret;

	ret = paintbox_pm_init(pb);
	if (ret < 0)
		return ret;

	paintbox_bif_init(pb);

	ret = paintbox_mmu_init(pb);
	if (ret < 0)
		return ret;

	/* Initialize the IRQ waiters after IO APB so the IRQ waiter code knows
	 * how many interrupts to allocate.
	 */
	ret = paintbox_irq_init(pb);
	if (ret < 0)
		return ret;

	/* register the misc device */
	pb->misc_device.minor = MISC_DYNAMIC_MINOR,
	pb->misc_device.name  = "paintbox",
	pb->misc_device.fops  = &paintbox_fops,

	ret = misc_register(&pb->misc_device);
	if (ret) {
		pr_err("Failed to register misc device node (ret = %d)", ret);
		return ret;
	}

#ifdef CONFIG_PAINTBOX_DEBUG
	pb->stats.probe_time = ktime_sub(ktime_get_boottime(), start_time);
#endif
	mnh_trace(MNH_TRACE_PAINTBOX_PROBE_DONE);

	return 0;
}

static int paintbox_remove(struct platform_device *pdev)
{
	struct paintbox_data *pb = platform_get_drvdata(pdev);

	misc_deregister(&pb->misc_device);
	paintbox_irq_remove(pb);
	paintbox_mmu_remove(pb);
	paintbox_bif_remove(pb);
	paintbox_mipi_remove(pb);
	paintbox_dma_remove(pb);
	paintbox_lbp_remove(pb);
	paintbox_stp_remove(pb);
	paintbox_pm_remove(pb);
	paintbox_fpga_remove(pb);
	paintbox_pm_disable_io(pb);
	paintbox_io_apb_remove(pb);

	devm_iounmap(&pdev->dev, pb->reg_base);
	paintbox_debug_remove(pb);
	mutex_destroy(&pb->lock);

	kfree(pb);

	return 0;
}

static const struct of_device_id paintbox_of_match[] = {
	{ .compatible = "google,paintbox", },
	{},
};
MODULE_DEVICE_TABLE(of, paintbox_of_match);

static struct platform_driver paintbox_driver = {
	.probe		= paintbox_probe,
	.remove		= paintbox_remove,
	.driver = {
		.name = "paintbox",
		.of_match_table = paintbox_of_match,
	}
};
module_platform_driver(paintbox_driver);

MODULE_AUTHOR("Google, Inc.");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Paintbox Driver");
