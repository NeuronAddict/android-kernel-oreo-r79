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
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <uapi/paintbox.h>

#include "paintbox-bif.h"
#include "paintbox-common.h"
#include "paintbox-dma.h"
#include "paintbox-fpga.h"
#include "paintbox-io.h"
#include "paintbox-irq.h"
#include "paintbox-lbp.h"
#include "paintbox-mipi.h"
#include "paintbox-mmu.h"
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

	pb = container_of(m, struct paintbox_data, misc_device);

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

	return 0;
}

static int paintbox_release(struct inode *ip, struct file *fp)
{
	struct paintbox_session *session = fp->private_data;
	struct paintbox_data *pb = session->dev;
	struct paintbox_irq *irq, *irq_next;
	struct paintbox_dma_channel *channel, *channel_next;
	struct paintbox_stp *stp, *stp_next;
	struct paintbox_lbp *lbp, *lbp_next;
	struct paintbox_mipi_stream *stream, *stream_next;

	mutex_lock(&pb->lock);

	/* TODO(ahampson): Cancel any pending DMA transfers */

	/* Unbind any interrupts bound to the session's dma channels. */
	list_for_each_entry_safe(channel, channel_next, &session->dma_list,
			session_entry) {
		unbind_dma_interrupt(pb, session, channel);
	}

	/* Disable any interrupts associated with the session */
	list_for_each_entry_safe(irq, irq_next, &session->irq_list,
			session_entry)
		release_interrupt(pb, session, irq);

	/* Disable any dma channels associated with the channel */
	list_for_each_entry_safe(channel, channel_next, &session->dma_list,
			session_entry)
		release_dma_channel(pb, session, channel);

	/* Release any STPs associated with the session */
	list_for_each_entry_safe(stp, stp_next, &session->stp_list,
			session_entry)
		release_stp(pb, session, stp);

	/* Release any Line Buffer Pools associated with the channel */
	list_for_each_entry_safe(lbp, lbp_next, &session->lbp_list,
			session_entry)
		release_lbp(pb, session, lbp);

	/* Release any MIPI Input Streams associated with the channel */
	list_for_each_entry_safe(stream, stream_next, &session->mipi_input_list,
			session_entry)
		release_mipi_stream(pb, session, stream);

	/* Release any MIPI Output Streams associated with the channel */
	list_for_each_entry_safe(stream, stream_next,
			&session->mipi_output_list, session_entry)
		release_mipi_stream(pb, session, stream);

	paintbox_irq_wait_for_release_complete(pb, session);

	mutex_unlock(&pb->lock);

	kfree(session);

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
	caps.iommu_enabled = pb->mmu.enabled;
#endif

	caps.hardware_id = pb->hardware_id;

	if (copy_to_user((void __user *)arg, &caps, sizeof(caps)))
		return -EFAULT;

	return 0;
}

static long paintbox_ioctl(struct file *fp, unsigned int cmd,
		unsigned long arg)
{
	struct paintbox_session *session = fp->private_data;
	struct paintbox_data *pb = session->dev;
	int ret;
#ifdef CONFIG_PAINTBOX_TEST_SUPPORT
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
	case PB_GET_PROGRAM_STATE:
		ret = get_program_state_ioctl(pb, session, arg);
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
	case PB_ENABLE_STP_INTERRUPT:
		ret = enable_stp_interrupt_ioctl(pb, session, arg);
		break;
	case PB_DISABLE_STP_INTERRUPT:
		ret = disable_stp_interrupt_ioctl(pb, session, arg);
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
#else
	case PB_TEST_DMA_RESET:
	case PB_TEST_DMA_CHANNEL_RESET:
	case PB_TEST_MIPI_IN_RESET_STREAM:
	case PB_TEST_MIPI_OUT_RESET_STREAM:
		ret = -EINVAL;
		break;
#endif
	default:
		dev_err(&pb->pdev->dev, "%s: unknown ioctl 0x%0x\n", __func__,
				cmd);
		return -EINVAL;
	}

#ifdef CONFIG_PAINTBOX_TEST_SUPPORT
	if (pb->stats.ioctl_time_enabled)
		paintbox_debug_log_ioctl_stats(pb, cmd, start_time,
				ktime_get_boottime());
#endif

	return ret;
}

void paintbox_alloc_debug_buffer(struct paintbox_data *pb, size_t len)
{
	if (pb->vdbg_log_len < len) {
		char *buf = pb->vdbg_log;

		buf = krealloc(pb->vdbg_log, len, GFP_KERNEL);
		if (buf) {
			pb->vdbg_log_len = len;
			pb->vdbg_log = buf;
		}
	}
}

static const struct file_operations paintbox_fops = {
	.owner = THIS_MODULE,
	.open = paintbox_open,
	.release = paintbox_release,
	.unlocked_ioctl = paintbox_ioctl,
};

static void paintbox_deinit(struct paintbox_data *pb)
{
	/* TODO(ahampson): Figure out proper IPU shutdown sequence. */
	paintbox_bif_shutdown(pb);

	paintbox_bif_remove(pb);
	paintbox_lbp_deinit(pb);

	paintbox_stp_deinit(pb);

	kfree(pb->dma.channels);
	kfree(pb->vdbg_log);
}

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
	dev_info(&pb->pdev->dev,
			"Paintbox IPU Version %u.%u.%u Simulator Hardware ID %u\n",
			major, minor, build, pb->hardware_id);
#else
	dev_info(&pb->pdev->dev,
			"Paintbox IPU Version %u.%u.%u %s Hardware ID %u\n",
			major, minor, build, is_fpga ? "FPGA" : "",
			pb->hardware_id);
#endif
	dev_info(&pb->pdev->dev,
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

	pb->io.apb_base = pb->reg_base + IPU_CSR_APB_OFFSET;
	pb->io.axi_base = pb->reg_base + IPU_CSR_AXI_OFFSET;
	pb->io_ipu.ipu_base = pb->reg_base + IPU_CSR_IO_OFFSET;
	pb->dma.dma_base = pb->reg_base + IPU_CSR_DMA_OFFSET;
	pb->stp.reg_base = pb->reg_base + IPU_CSR_STP_OFFSET;
	pb->lbp.reg_base = pb->reg_base + IPU_CSR_LBP_OFFSET;
#ifdef CONFIG_PAINTBOX_SIMULATOR_SUPPORT
	pb->sim_base = pb->reg_base + SIM_GROUP_OFFSET;
#endif

	ret = paintbox_get_capabilities(pb);
	if (ret < 0)
		return ret;

	ret = paintbox_bif_init(pb);
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

	paintbox_bif_start(pb);

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

	return 0;
}

static int paintbox_remove(struct platform_device *pdev)
{
	struct paintbox_data *pb = platform_get_drvdata(pdev);

	misc_deregister(&pb->misc_device);
	paintbox_irq_remove(pb);
	paintbox_deinit(pb);
	paintbox_pm_remove(pb);
	paintbox_fpga_remove(pb);
	devm_free_irq(&pdev->dev, pb->io.irq, pb);

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
