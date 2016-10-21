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
#include <linux/paintbox.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/version.h>

#include "paintbox-common.h"
#include "paintbox-dma.h"
#include "paintbox-dma-debug.h"
#include "paintbox-io.h"
#include "paintbox-irq.h"
#include "paintbox-lbp.h"
#include "paintbox-mipi.h"
#include "paintbox-mipi-debug.h"
#include "paintbox-regs.h"
#include "paintbox-stp.h"
#include "paintbox-stp-sim.h"
#include "paintbox-stp-sram.h"

#include "paintbox-regs.h"
#include "paintbox-sim-regs.h"

#ifdef CONFIG_PAINTBOX_FPGA_SUPPORT
#include "paintbox-fpga.h"
#endif

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

#ifdef CONFIG_PAINTBOX_FPGA_SUPPORT
	paintbox_fpga_soft_reset(pb);
#endif

	mutex_unlock(&pb->lock);

	kfree(session);

	return 0;
}

static long paintbox_ioctl(struct file *fp, unsigned int cmd,
		unsigned long arg)
{
	struct paintbox_session *session = fp->private_data;
	struct paintbox_data *pb = session->dev;

	switch (cmd) {
	case PB_GET_IPU_CAPABILITIES:
		if (copy_to_user((void __user *)arg, &pb->caps,
				sizeof(pb->caps)))
			return -EFAULT;
		return 0;
	case PB_ALLOCATE_INTERRUPT:
		return allocate_interrupt_ioctl(pb, session, arg);
	case PB_WAIT_FOR_INTERRUPT:
		return wait_for_interrupt_ioctl(pb, session, arg);
	case PB_RELEASE_INTERRUPT:
		return release_interrupt_ioctl(pb, session, arg);
	case PB_ALLOCATE_DMA_CHANNEL:
		return allocate_dma_channel_ioctl(pb, session, arg);
	case PB_BIND_DMA_INTERRUPT:
		return bind_dma_interrupt_ioctl(pb, session, arg);
	case PB_UNBIND_DMA_INTERRUPT:
		return unbind_dma_interrupt_ioctl(pb, session, arg);
	case PB_START_DMA_TRANSFER:
		return start_dma_transfer_ioctl(pb, session, arg);
	case PB_STOP_DMA_TRANSFER:
		return stop_dma_transfer_ioctl(pb, session, arg);
	case PB_RELEASE_DMA_CHANNEL:
		return release_dma_channel_ioctl(pb, session, arg);
	case PB_GET_COMPLETED_UNREAD_COUNT:
		return get_completed_transfer_count_ioctl(pb, session, arg);
	case PB_FLUSH_DMA_TRANSFERS:
		return flush_dma_transfers_ioctl(pb, session, arg);
	case PB_ALLOCATE_LINE_BUFFER_POOL:
		return allocate_lbp_ioctl(pb, session, arg);
	case PB_SETUP_LINE_BUFFER:
		return setup_lb_ioctl(pb, session, arg);
	case PB_RELEASE_LINE_BUFFER_POOL:
		return release_lbp_ioctl(pb, session, arg);
	case PB_RESET_LINE_BUFFER_POOL:
		return reset_lbp_ioctl(pb, session, arg);
	case PB_RESET_LINE_BUFFER:
		return reset_lb_ioctl(pb, session, arg);
	case PB_WRITE_LBP_MEMORY:
		return write_lbp_memory_ioctl(pb, session, arg);
	case PB_READ_LBP_MEMORY:
		return read_lbp_memory_ioctl(pb, session, arg);
	case PB_ALLOCATE_PROCESSOR:
		return allocate_stp_ioctl(pb, session, arg);
	case PB_INIT_PROCESSOR:
		return init_stp_ioctl(pb, session, arg);
	case PB_SETUP_PROCESSOR:
		return setup_stp_ioctl(pb, session, arg);
	case PB_WRITE_STP_MEMORY:
		return write_stp_scalar_sram_ioctl(pb, session, arg);
	case PB_READ_STP_MEMORY:
		return read_stp_scalar_sram_ioctl(pb, session, arg);
	case PB_WRITE_VECTOR_SRAM_COORDINATES:
		return write_stp_vector_sram_coordinates_ioctl(pb, session,
				arg);
	case PB_WRITE_VECTOR_SRAM_REPLICATE:
		return write_stp_vector_sram_replicate_ioctl(pb, session, arg);
	case PB_READ_VECTOR_SRAM_COORDINATES:
		return read_stp_vector_sram_coordinates_ioctl(pb, session, arg);
	case PB_START_PROCESSOR:
		return start_stp_ioctl(pb, session, arg);
	case PB_STOP_PROCESSOR:
		return stop_stp_ioctl(pb, session, arg);
	case PB_RESUME_PROCESSOR:
		return resume_stp_ioctl(pb, session, arg);
	case PB_RESET_PROCESSOR:
		return reset_stp_ioctl(pb, session, arg);
	case PB_GET_PROGRAM_STATE:
		return get_program_state_ioctl(pb, session, arg);
	case PB_RELEASE_PROCESSOR:
		return release_stp_ioctl(pb, session, arg);
	case PB_WAIT_FOR_ALL_PROCESSOR_IDLE:
#ifdef CONFIG_PAINTBOX_SIMULATOR_SUPPORT
		return sim_wait_for_idle_ioctl(pb, session, arg);
#else
		/* The simulator requires additional processing after the DMA
		 * interrupt before the processor goes idle.  This processing
		 * is fast enough on the actual hardware that we do not need
		 * to poll for idle.
		 */
		return 0;
#endif
	case PB_GET_PROCESSOR_IDLE:
#ifdef CONFIG_PAINTBOX_SIMULATOR_SUPPORT
		return sim_get_stp_idle_ioctl(pb, session, arg);
#else
		return -ENOSYS;
#endif
	case PB_ENABLE_STP_INTERRUPT:
		return enable_stp_interrupt_ioctl(pb, session, arg);
	case PB_DISABLE_STP_INTERRUPT:
		return disable_stp_interrupt_ioctl(pb, session, arg);
	case PB_BIND_STP_INTERRUPT:
		return bind_stp_interrupt_ioctl(pb, session, arg);
	case PB_UNBIND_STP_INTERRUPT:
		return unbind_stp_interrupt_ioctl(pb, session, arg);
	case PB_SETUP_DMA_TRANSFER:
		return setup_dma_transfer_ioctl(pb, session, arg);
	case PB_READ_DMA_TRANSFER:
		return read_dma_transfer_ioctl(pb, session, arg);
	case PB_ALLOCATE_MIPI_IN_STREAM:
		return allocate_mipi_input_stream_ioctl(pb, session, arg);
	case PB_RELEASE_MIPI_IN_STREAM:
		return release_mipi_stream_ioctl(pb, session, arg, true);
	case PB_SETUP_MIPI_IN_STREAM:
		return setup_mipi_stream_ioctl(pb, session, arg, true);
	case PB_ENABLE_MIPI_IN_STREAM:
		return enable_mipi_stream_ioctl(pb, session, arg, true);
	case PB_DISABLE_MIPI_IN_STREAM:
		return disable_mipi_stream_ioctl(pb, session, arg, true);
	case PB_RESET_MIPI_IN_STREAM:
		return reset_mipi_stream_ioctl(pb, session, arg, true);
	case PB_CLEANUP_MIPI_IN_STREAM:
		return cleanup_mipi_stream_ioctl(pb, session, arg, true);
	case PB_ENABLE_MIPI_IN_INTERRUPT:
		return enable_mipi_interrupt_ioctl(pb, session, arg, true);
	case PB_DISABLE_MIPI_IN_INTERRUPT:
		return disable_mipi_interrupt_ioctl(pb, session, arg, true);
	case PB_ALLOCATE_MIPI_OUT_STREAM:
		return allocate_mipi_output_stream_ioctl(pb, session, arg);
	case PB_RELEASE_MIPI_OUT_STREAM:
		return release_mipi_stream_ioctl(pb, session, arg, false);
	case PB_SETUP_MIPI_OUT_STREAM:
		return setup_mipi_stream_ioctl(pb, session, arg, false);
	case PB_ENABLE_MIPI_OUT_STREAM:
		return enable_mipi_stream_ioctl(pb, session, arg, false);
	case PB_DISABLE_MIPI_OUT_STREAM:
		return disable_mipi_stream_ioctl(pb, session, arg, false);
	case PB_RESET_MIPI_OUT_STREAM:
		return reset_mipi_stream_ioctl(pb, session, arg, false);
	case PB_CLEANUP_MIPI_OUT_STREAM:
		return cleanup_mipi_stream_ioctl(pb, session, arg, false);
	case PB_ENABLE_MIPI_OUT_INTERRUPT:
		return enable_mipi_interrupt_ioctl(pb, session, arg, false);
	case PB_DISABLE_MIPI_OUT_INTERRUPT:
		return disable_mipi_interrupt_ioctl(pb, session, arg, false);
	case PB_BIND_MIPI_IN_INTERRUPT:
		return bind_mipi_interrupt_ioctl(pb, session, arg, true);
	case PB_UNBIND_MIPI_IN_INTERRUPT:
		return unbind_mipi_interrupt_ioctl(pb, session, arg, true);
	case PB_BIND_MIPI_OUT_INTERRUPT:
		return bind_mipi_interrupt_ioctl(pb, session, arg, false);
	case PB_UNBIND_MIPI_OUT_INTERRUPT:
		return unbind_mipi_interrupt_ioctl(pb, session, arg, false);
#ifdef CONFIG_PAINTBOX_TEST_SUPPORT
	case PB_TEST_DMA_RESET:
		return dma_test_reset_ioctl(pb, session, arg);
	case PB_TEST_DMA_CHANNEL_RESET:
		return dma_test_channel_reset_ioctl(pb, session, arg);
#endif
	default:
		dev_err(&pb->pdev->dev, "%s: unknown ioctl 0x%0x\n", __func__,
				cmd);
		return -EINVAL;
	}
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

#ifdef CONFIG_PAINTBOX_SIMULATOR_SUPPORT

#define SIM_GROUP_OFFSET IPU_RESERVED_OFFSET

static void paintbox_sim_init(struct paintbox_data *pb)
{
	pb->sim_base = pb->reg_base + SIM_GROUP_OFFSET;

	dev_dbg(&pb->pdev->dev, "sim: base %p len %u\n", pb->sim_base,
			SIM_BLOCK_LEN);
}
#endif

static void paintbox_deinit(struct paintbox_data *pb)
{
	/* TODO(ahampson): Figure out proper IPU shutdown sequence. */

	paintbox_lbp_deinit(pb);

	kfree(pb->stps);
	kfree(pb->irqs);
	kfree(pb->dma.channels);
	kfree(pb->vdbg_log);
}

#ifdef CONFIG_DEBUG_FS
static int pb_debug_regs_show(struct seq_file *s, void *unused)
{
	struct paintbox_data *pb = s->private;
	unsigned int i, j;
	char *buf;
	size_t len;
	int ret, written = 0;

	len = seq_get_buf(s, &buf);
	if (!buf)
		return -ENOMEM;

	mutex_lock(&pb->lock);

	ret = dump_io_apb_registers(&pb->io.apb_debug, buf + written,
			len - written);
	if (ret < 0)
		goto err_exit;

	written += ret;

	ret = dump_io_axi_registers(&pb->io.axi_debug, buf + written,
			len - written);
	if (ret < 0)
		goto err_exit;

	written += ret;

	ret = dump_io_ipu_registers(&pb->io_ipu.debug, buf + written,
			len - written);
	if (ret < 0)
		goto err_exit;

	written += ret;

	for (i = 0; i < pb->io_ipu.num_mipi_input_streams; i++) {
		ret = dump_mipi_input_stream_registers(
				&pb->io_ipu.mipi_input_streams[i].debug,
				buf + written, len - written);
		if (ret < 0)
			goto err_exit;

		written += ret;
	}

	for (i = 0; i < pb->io_ipu.num_mipi_output_streams; i++) {
		ret = dump_mipi_output_stream_registers(
				&pb->io_ipu.mipi_output_streams[i].debug,
				buf + written, len - written);
		if (ret < 0)
			goto err_exit;

		written += ret;
	}

	ret = dump_dma_registers(&pb->dma.debug, buf + written, len - written);
	if (ret < 0)
		goto err_exit;

	written += ret;

	for (i = 0; i < pb->dma.num_channels; i++) {
		ret = dump_dma_channel_registers(&pb->dma.channels[i].debug,
				buf + written, len - written);
		if (ret < 0)
			goto err_exit;

		written += ret;
	}

	for (i = 0; i < pb->caps.num_stps; i++) {
		ret = dump_stp_registers(&pb->stps[i].debug, buf + written,
				len - written);
		if (ret < 0)
			goto err_exit;

		written += ret;
	}

	for (i = 0; i < pb->caps.num_lbps; i++) {
		ret = dump_lbp_registers(&pb->lbps[i].debug, buf + written,
				len - written);
		if (ret < 0)
			goto err_exit;

		written += ret;

		for (j = 0; j < pb->caps.max_line_buffers; j++) {
			ret = dump_lb_registers(&pb->lbps[i].lbs[j].debug,
					buf + written, len - written);
			if (ret < 0)
				goto err_exit;

			written += ret;
		}
	}

	mutex_unlock(&pb->lock);

	seq_commit(s, written);

	return 0;

err_exit:
	mutex_unlock(&pb->lock);
	dev_err(&pb->pdev->dev, "%s: register dump error, err = %d", __func__,
			ret);
	return ret;
}

static int pb_debug_regs_open(struct inode *inode, struct file *file)
{
	struct paintbox_data *pb = inode->i_private;
	size_t len;

	len = IO_APB_DEBUG_BUFFER_SIZE;
	len += IO_AXI_DEBUG_BUFFER_SIZE;
	len += pb->io_ipu.num_mipi_input_streams * MIPI_DEBUG_BUFFER_SIZE;
	len += pb->io_ipu.num_mipi_output_streams * MIPI_DEBUG_BUFFER_SIZE;
	len += pb->dma.num_channels * DMA_DEBUG_BUFFER_SIZE;
	len += pb->caps.num_stps * STP_DEBUG_BUFFER_SIZE;
	len += pb->caps.num_lbps * LBP_DEBUG_BUFFER_SIZE;
	len += pb->caps.num_lbps * pb->caps.max_line_buffers *
			LB_DEBUG_BUFFER_SIZE;

	return single_open_size(file, pb_debug_regs_show, inode->i_private,
			len);
}

static const struct file_operations pb_debug_regs_fops = {
	.open = pb_debug_regs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static void paintbox_debug_init(struct paintbox_data *pb)
{
	pb->debug_root = debugfs_create_dir("paintbox", NULL);

	pb->regs_dentry = debugfs_create_file("regs", 0644, pb->debug_root,
			pb, &pb_debug_regs_fops);
}
#endif

static void paintbox_get_version(struct paintbox_data *pb)
{
	uint32_t version = readl(pb->reg_base + IPU_VERSION);

	pb->caps.version_major = (version & IPU_VERSION_MAJOR_MASK) >>
			IPU_VERSION_MAJOR_SHIFT;
	pb->caps.version_minor = (version & IPU_VERSION_MINOR_MASK) >>
			IPU_VERSION_MAJOR_SHIFT;
	pb->caps.version_build = version & IPU_VERSION_INCR_MASK;
	pb->caps.is_fpga = !!(version & IPU_VERSION_FPGA_BUILD_MASK);

#if defined(CONFIG_PAINTBOX_SIMULATOR_SUPPORT)
	pb->caps.is_simulator = true;

	/* TODO(ahampson):  The RTL tests expect the version register to be zero
	 * so we are passing the hardware id out of band throughg simulator
	 * specific registers.  There needs to be some clean up of the hardware
	 * id system so that version 0.0.0 corresponds to the hardware id of
	 * Easel configuration.  b/30112936
	 */
	pb->caps.hardware_id = readl(pb->sim_base + SIM_ID);
#endif
	if (pb->caps.is_fpga)
		pb->caps.hardware_id = FPGA_HARDWARE_ID;

	dev_info(&pb->pdev->dev,
			"Paintbox IPU Version %u.%u Build %u %s Hardware ID %u"
			"\n", pb->caps.version_major, pb->caps.version_minor,
			pb->caps.version_build, pb->caps.is_simulator ?
			"Simulator" : pb->caps.is_fpga ? "FPGA" : "",
			pb->caps.hardware_id);
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

#ifdef CONFIG_PAINTBOX_FPGA_SUPPORT
	ret = paintbox_fpga_init(pb);
	if (ret < 0)
		return ret;
#endif

	pb->io.irq = platform_get_irq(pdev, 0);
	if (pb->io.irq < 0) {
		dev_err(&pdev->dev, "platform_get_irq failed\n");
		return -ENODEV;
	}

	platform_set_drvdata(pdev, pb);

#ifdef CONFIG_DEBUG_FS
	paintbox_debug_init(pb);
#endif

#if defined(CONFIG_PAINTBOX_SIMULATOR_SUPPORT)
	paintbox_sim_init(pb);
#endif

	paintbox_get_version(pb);

	ret = paintbox_io_axi_init(pb);
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

	/* Initialize the IRQ waiters after IO APB so the IRQ waiter code knows
	 * how many interrupts to allocate.
	 */
	ret = paintbox_irq_init(pb);
	if (ret < 0)
		return ret;

	/* TODO(ahampson):  This works on the assumption that all lbps have the
	 * same number of lbs.
	 */
	pb->caps.max_line_buffers = pb->lbps[0].max_lbs;
	pb->caps.max_read_ptrs = pb->lbps[0].max_rptrs;
	pb->caps.max_channels = pb->lbps[0].max_channels;
	pb->caps.max_fb_rows = pb->lbps[0].max_fb_rows;

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

	paintbox_deinit(pb);

#ifdef CONFIG_PAINTBOX_FPGA_SUPPORT
	paintbox_fpga_deinit(pb);
#endif

	devm_free_irq(&pdev->dev, pb->io.irq, pb);

	devm_iounmap(&pdev->dev, pb->reg_base);

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

MODULE_AUTHOR("Adam Hampson <ahampson@google.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Paintbox Driver");
