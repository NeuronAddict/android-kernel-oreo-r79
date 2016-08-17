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
#include "paintbox-dma-fpga.h"
#include "paintbox-io.h"
#include "paintbox-lbp.h"
#include "paintbox-stp.h"
#include "paintbox-stp-sim.h"

#include "paintbox-regs.h"
#include "paintbox-sim-regs.h"


/* The caller to this function must hold pb->lock */
static int validate_interrupt(struct paintbox_data *pb,
		struct paintbox_session *session, int interrupt_id)
{

	if (interrupt_id >= pb->caps.num_interrupts) {
		dev_err(&pb->pdev->dev, "%s: invalid interrupt_id %d\n",
				__func__, interrupt_id);
		return -EINVAL;
	}

	if (pb->irqs[interrupt_id].session != session) {
		dev_err(&pb->pdev->dev, "%s: access error: interrupt_id %d\n",
				__func__, interrupt_id);
		return -EACCES;
	}

	return 0;
}

/* The caller to this function must hold pb->lock */
struct paintbox_irq *get_interrupt(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned int interrupt_id,
		int *err)
{
	int ret = validate_interrupt(pb, session, interrupt_id);
	if (ret < 0) {
		*err = ret;
		return NULL;
	}

	*err = 0;
	return &pb->irqs[interrupt_id];
}

static int allocate_interrupt(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg) {
	unsigned int interrupt_id = (unsigned int)arg;
	struct paintbox_irq *irq;

	if (interrupt_id >= pb->caps.num_interrupts) {
		dev_err(&pb->pdev->dev,
				"%s: invalid interrupt_id %d, %d >= %d\n",
				__func__, interrupt_id, interrupt_id,
				pb->caps.num_interrupts);
		return -EINVAL;
	}

	mutex_lock(&pb->lock);
	irq = &pb->irqs[interrupt_id];
	if (irq->session) {
		dev_err(&pb->pdev->dev, "%s: access error: interrupt_id %d\n",
				__func__, interrupt_id);
		mutex_unlock(&pb->lock);
		return -EACCES;
	}

	irq->session = session;
	list_add_tail(&irq->entry, &session->irq_list);

	mutex_unlock(&pb->lock);

	return 0;
}

static int wait_for_interrupt(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct interrupt_wait __user *user_wait;
	struct interrupt_wait wait;
#ifdef CONFIG_PAINTBOX_SIMULATOR_SUPPORT
	struct paintbox_dma_channel *channel;
#endif
	struct paintbox_irq *irq;
	unsigned long irq_flags;
	long ret;
	int err;

	user_wait = (struct interrupt_wait __user *)arg;
	if (copy_from_user(&wait, user_wait, sizeof(wait)))
		return -EFAULT;

	mutex_lock(&pb->lock);
	irq = get_interrupt(pb, session, wait.interrupt_id, &err);
	if (err < 0) {
		mutex_unlock(&pb->lock);
		return err;
	}

	dev_dbg(&pb->pdev->dev, "%s: int%u: wait, timeout %llu" , __func__,
			wait.interrupt_id, wait.timeout_ns);

#ifdef CONFIG_PAINTBOX_SIMULATOR_SUPPORT
	/* The Simulator has extra execution control registers that allow it to
	 * be switched from configuration mode to execution mode.
	 *
	 * TODO(ahampson:  The HAL / Simulator notion that interrupts and
	 * DMA channels have independent number spaces needs to be cleaned up.
	 * The hardware has a one to one mapping between interrupt ids and
	 * channel ids.  Therefore we need to convert the interrupt id that was
	 * passed in to a channel id;
	 */
	channel = get_dma_channel(pb, session, irq->channel_id, &err);
	if (err < 0) {
		mutex_unlock(&pb->lock);
		return err;
	}

	sim_execute(pb, channel->channel_id, wait.timeout_ns);
#endif

	mutex_unlock(&pb->lock);

	if (wait.timeout_ns != INT_MAX) {
		ret = wait_for_completion_interruptible_timeout(
				&irq->completion,
				nsecs_to_jiffies64(wait.timeout_ns));
		if (ret == 0)
			return -ETIMEDOUT;
		if (ret < 0)
			return ret;
	} else {
		err = wait_for_completion_interruptible(&irq->completion);
		if (err < 0)
			return err;
	}

	spin_lock_irqsave(&pb->irq_lock, irq_flags);
	if (irq->error) {
		err = irq->error;
		irq->error = 0;
		spin_unlock_irqrestore(&pb->irq_lock, irq_flags);
		return err;
	}
	spin_unlock_irqrestore(&pb->irq_lock, irq_flags);

	return 0;
}

/* The caller to this function must hold pb->lock */
static int release_interrupt(struct paintbox_data *pb,
		struct paintbox_session *session, struct paintbox_irq *irq)
{
	/* Unblock any current waiters */
	/* TODO(ahampson): We may want to return an error if we complete due to
	 * the interrupt being disabled.
	 */
	complete_all(&pb->irqs[irq->interrupt_id].completion);

	list_del(&irq->entry);
	irq->session = NULL;
	irq->channel_id = IRQ_NO_DMA_CHANNEL;
	irq->error = -EPIPE;

	return 0;
}

static int release_interrupt_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	uint8_t interrupt_id = (uint8_t)arg;
	struct paintbox_irq *irq;
	int ret;

	mutex_lock(&pb->lock);
	irq = get_interrupt(pb, session, interrupt_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	/* Unbind any associated dma channel */
	if (irq->channel_id != IRQ_NO_DMA_CHANNEL) {
		ret = unbind_dma_interrupt(pb, session, interrupt_id,
				irq->channel_id);
		if (ret < 0) {
			mutex_unlock(&pb->lock);
			return ret;
		}
	}

	release_interrupt(pb, session, irq);

	mutex_unlock(&pb->lock);

	return 0;
}

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

	fp->private_data = session;

	return 0;
}

static int paintbox_release(struct inode *ip, struct file *fp)
{
	struct paintbox_session *session = fp->private_data;
	struct paintbox_data *pb = session->dev;
	struct paintbox_irq *irq, *irq_next;
	struct paintbox_dma_channel *dma, *dma_next;
	struct paintbox_stp *stp, *stp_next;
	struct paintbox_lbp *lbp, *lbp_next;

	mutex_lock(&pb->lock);

	/* TODO(ahampson): Cancel any pending DMA transfers */

	/* Unbind any interrupts bound to the session's dma channels. */
	list_for_each_entry_safe(dma, dma_next, &session->dma_list, entry) {
		if (dma->interrupt_id != DMA_NO_INTERRUPT)
			unbind_dma_interrupt(pb, session, dma->interrupt_id,
					dma->channel_id);
	}

	/* Disable any interrupts associated with the session */
	list_for_each_entry_safe(irq, irq_next, &session->irq_list, entry)
		release_interrupt(pb, session, irq);

	/* Disable any dma channels associated with the channel */
	list_for_each_entry_safe(dma, dma_next, &session->dma_list, entry)
		release_dma_channel(pb, session, dma);

	/* Release any STPs associated with the session */
	list_for_each_entry_safe(stp, stp_next, &session->stp_list, entry)
		release_stp(pb, session, stp);

	/* Release any Line Buffer Pools associated with the channel */
	list_for_each_entry_safe(lbp, lbp_next, &session->lbp_list, entry)
		release_lbp(pb, session, lbp);

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
		return allocate_interrupt(pb, session, arg);
	case PB_WAIT_FOR_INTERRUPT:
		return wait_for_interrupt(pb, session, arg);
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
	case PB_RELEASE_DMA_CHANNEL:
		return release_dma_channel_ioctl(pb, session, arg);
	case PB_ALLOCATE_LINE_BUFFER_POOL:
		return allocate_lbp_ioctl(pb, session, arg);
	case PB_SETUP_LINE_BUFFER:
		return setup_lb_ioctl(pb, session, arg);
	case PB_RELEASE_LINE_BUFFER_POOL:
		return release_lbp_ioctl(pb, session, arg);
	case PB_WRITE_LBP_MEMORY:
		return write_lbp_memory_ioctl(pb, session, arg);
	case PB_READ_LBP_MEMORY:
		return read_lbp_memory_ioctl(pb, session, arg);
	case PB_ALLOCATE_PROCESSOR:
		return allocate_stp_ioctl(pb, session, arg);
	case PB_SETUP_PROCESSOR:
		return setup_stp_ioctl(pb, session, arg);
	case PB_WRITE_STP_MEMORY:
		return write_stp_memory_ioctl(pb, session, arg);
	case PB_READ_STP_MEMORY:
		return read_stp_memory_ioctl(pb, session, arg);
	case PB_WRITE_STP_VECTOR_MEMORY:
		return write_stp_vector_memory_ioctl(pb, session, arg);
	case PB_READ_STP_VECTOR_MEMORY:
		return read_stp_vector_memory_ioctl(pb, session, arg);
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
	case PB_SETUP_DMA_TRANSFER:
		return setup_dma_transfer_ioctl(pb, session, arg);
	case PB_READ_DMA_TRANSFER:
		return read_dma_transfer_ioctl(pb, session, arg);
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

static int paintbox_irq_init(struct paintbox_data *pb)
{
	unsigned int i;

	pb->irqs = kzalloc(sizeof(struct paintbox_irq) *
			pb->caps.num_interrupts, GFP_KERNEL);
	if (!pb->irqs)
		return -ENOMEM;

	for (i = 0; i < pb->caps.num_interrupts; i++) {
		/* Store interrupt id with object as a convenience to avoid
		 * doing a lookup later on.
		 */
		pb->irqs[i].interrupt_id = i;
		pb->irqs[i].channel_id = IRQ_NO_DMA_CHANNEL;
		init_completion(&pb->irqs[i].completion);
	}

	return 0;
}

#ifdef CONFIG_PAINTBOX_SIMULATOR_SUPPORT

#define SIM_GROUP_OFFSET IPU_RESERVED_OFFSET

static int paintbox_sim_init(struct paintbox_data *pb)
{
	uint32_t caps;

	pb->sim_base = pb->reg_base + SIM_GROUP_OFFSET;

	pb->caps.version = readl(pb->sim_base + SIM_ID);

	caps = readl(pb->sim_base + SIM_CAP0);

	pb->caps.num_interrupts = (caps & SIM_CAP0_INT_MASK) >>
			SIM_CAP0_INT_SHIFT;
	pb->caps.num_lbps = (caps & SIM_CAP0_LBP_MASK) >>
			SIM_CAP0_LBP_SHIFT;
	pb->caps.num_stps = (caps & SIM_CAP0_STP_MASK) >>
			SIM_CAP0_STP_SHIFT;

	dev_dbg(&pb->pdev->dev, "sim: base %p len %u ints%u id 0x%08x\n",
			pb->sim_base, SIM_BLOCK_LEN, pb->caps.num_interrupts,
			pb->caps.version);

	return 0;
}
#endif

#ifdef CONFIG_PAINTBOX_FPGA_SUPPORT
static int paintbox_fpga_init(struct paintbox_data *pb)
{
	pb->caps.version = FPGA_VERSION;
	pb->caps.num_interrupts = FPGA_INT_COUNT;
	pb->caps.num_lbps = FPGA_LBP_COUNT;
	pb->caps.num_stps = FPGA_STP_COUNT;

	dev_dbg(&pb->pdev->dev, "ints%u id 0x%08x\n",
			pb->caps.num_interrupts, pb->caps.version);

	return 0;
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

#ifdef CONFIG_PAINTBOX_FPGA_SUPPORT
	ret = dump_io_fpga_registers(pb, buf + written, len - written);
	if (ret < 0)
		goto err_exit;

	written += ret;
#endif

	ret = dump_io_axi_registers(pb, buf + written, len - written);
	if (ret < 0)
		goto err_exit;

	written += ret;

	ret = dump_dma_registers(pb, buf + written, len - written);
	if (ret < 0)
		goto err_exit;

	written += ret;

	for (i = 0; i < pb->dma.num_channels; i++) {
		ret = dump_dma_channel_registers(pb, i, buf + written,
				len - written);
		if (ret < 0)
			goto err_exit;

		written += ret;
	}

	for (i = 1; i <= pb->caps.num_stps; i++) {
		writel(i, pb->stp_base + STP_SEL);
		ret = dump_stp_registers(pb, buf + written, len - written);
		if (ret < 0)
			goto err_exit;

		written += ret;
	}

	for (i = 0; i < pb->caps.num_lbps; i++) {
		writel(i, pb->lbp_base + LBP_SEL);
		ret = dump_lbp_registers(pb, buf + written, len - written);
		if (ret < 0)
			goto err_exit;

		written += ret;

		for (j = 0; j < pb->caps.max_line_buffers; j++) {
			writel(i | (j << LBP_LB_SEL_SHIFT),
					pb->lbp_base + LBP_SEL);
			ret = dump_lb_registers(pb, buf + written,
					len - written);
			if (ret < 0)
				goto err_exit;

			written += ret;
		}
	}

#ifdef CONFIG_PAINTBOX_FPGA_SUPPORT
	ret = dump_xilinx_registers(pb, buf + written, len - written);
	if (ret < 0)
		goto err_exit;

	written += ret;
#endif

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

	len = IO_AXI_DEBUG_BUFFER_SIZE;
	len += pb->caps.num_stps * STP_DEBUG_BUFFER_SIZE;
	len += pb->caps.num_lbps * LBP_DEBUG_BUFFER_SIZE;
	len += pb->caps.num_lbps * pb->caps.max_line_buffers *
			LB_DEBUG_BUFFER_SIZE;
#ifdef CONFIG_PAINTBOX_FPGA_SUPPORT
	len += IO_DEBUG_BUFFER_SIZE + XILINX_DEBUG_BUFFER_SIZE;
#endif

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
  pr_err("paintbox create debugfs\n");
	pb->debug_root = debugfs_create_dir("paintbox", NULL);

	pb->regs_dentry = debugfs_create_file("regs", 0644, pb->debug_root,
			pb, &pb_debug_regs_fops);
}
#endif

static int paintbox_probe(struct platform_device *pdev)
{
	int ret;
	struct resource *r;
	struct paintbox_data *pb;
	dev_err(&pdev->dev, "paintbox probe\n");

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
	ret = paintbox_sim_init(pb);
#elif defined(CONFIG_PAINTBOX_FPGA_SUPPORT)
	ret = paintbox_fpga_init(pb);
#else
	ret = -ENOSYS;
#endif
	if (ret < 0)
		return ret;

	ret = paintbox_io_init(pb);
	if (ret < 0)
		return ret;

	ret = paintbox_dma_init(pb);
	if (ret < 0)
		return ret;

	ret = paintbox_irq_init(pb);
	if (ret < 0)
		return ret;

	ret = paintbox_lbp_init(pb);
	if (ret < 0)
		return ret;

	ret = paintbox_stp_init(pb);
	if (ret < 0)
		return ret;

	/* TODO(ahampson):  This works on the assumption that all lbps have the
	 * same number of lbs.
	 */
	pb->caps.max_line_buffers = pb->lbps[0].max_lbs;
	pb->caps.max_read_ptrs = pb->lbps[0].max_rptrs;
	pb->caps.max_channels = pb->lbps[0].max_channels;
	pb->caps.max_fb_rows = pb->lbps[0].max_fb_rows;
	pb->caps.mem_size = pb->lbps[0].mem_size;

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

	/* TODO(ahampson): The FPGA does not currently present an interrupt. */
#ifndef CONFIG_PAINTBOX_FPGA_SUPPORT
	devm_free_irq(&pdev->dev, pb->io.irq, pb);
#endif
	devm_iounmap(&pdev->dev, pb->reg_base);

	mutex_destroy(&pb->lock);

	kfree(pb);

	return 0;
}

static const struct of_device_id paintbox_of_match[] = {
	{ .compatible = "generic,paintbox", },
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
