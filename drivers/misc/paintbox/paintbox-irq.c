/*
 * Interrupt support for the Paintbox programmable IPU
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

#include <linux/completion.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/paintbox.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include "paintbox-common.h"
#include "paintbox-irq.h"
#include "paintbox-dma.h"
#include "paintbox-stp-sim.h"


/* The caller to this function must hold pb->irq_lock */
static void release_all_waiters(struct paintbox_data *pb,
		struct paintbox_irq *irq)
{
	if (irq->wait_count > 0) {
		irq->error = -EPIPE;
		complete_all(&irq->completion);
	}
}

/* The caller to this function must hold pb->lock */
int validate_interrupt(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned int interrupt_id)
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

int allocate_interrupt_ioctl(struct paintbox_data *pb,
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
	list_add_tail(&irq->session_entry, &session->irq_list);

	mutex_unlock(&pb->lock);

	return 0;
}

/* The caller to this function must hold pb->lock */
int bind_dma_interrupt(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_dma_channel *channel,
		unsigned int interrupt_id)
{
	struct paintbox_irq *irq;
	unsigned long irq_flags;
	int ret;

	irq = get_interrupt(pb, session, interrupt_id, &ret);
	if (ret < 0)
		return ret;

	spin_lock_irqsave(&pb->irq_lock, irq_flags);

	if (irq->source != IRQ_SRC_NONE) {
		spin_unlock_irqrestore(&pb->irq_lock, irq_flags);
		dev_err(&pb->pdev->dev,
				"%s: dma channel%u: unable to bind int%u\n",
				__func__, channel->channel_id, interrupt_id);
		return -EEXIST;
	}

	irq->dma_channel = channel;
	irq->source = IRQ_SRC_DMA_CHANNEL;
	channel->irq = irq;

	spin_unlock_irqrestore(&pb->irq_lock, irq_flags);

	dev_dbg(&pb->pdev->dev, "%s: dma channel%u: bind int%u" , __func__,
			channel->channel_id, interrupt_id);

	return 0;
}

/* The caller to this function must hold pb->lock */
int unbind_dma_interrupt(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_dma_channel *channel)
{
	struct paintbox_irq *irq;
	unsigned long irq_flags;

	spin_lock_irqsave(&pb->irq_lock, irq_flags);

	irq = channel->irq;
	if (!irq) {
		spin_unlock_irqrestore(&pb->irq_lock, irq_flags);
		return -ENOENT;
	}

	release_all_waiters(pb, irq);

	channel->irq = NULL;
	irq->dma_channel = NULL;
	irq->source = IRQ_SRC_NONE;

	spin_unlock_irqrestore(&pb->irq_lock, irq_flags);

	return 0;
}

/* The caller to this function must hold pb->lock */
int bind_stp_interrupt(struct paintbox_data *pb,
		struct paintbox_session *session, struct paintbox_stp *stp,
		unsigned int interrupt_id)
{
	struct paintbox_irq *irq;
	unsigned long irq_flags;
	int ret;

	irq = get_interrupt(pb, session, interrupt_id, &ret);
	if (ret < 0)
		return ret;

	spin_lock_irqsave(&pb->irq_lock, irq_flags);

	if (irq->source != IRQ_SRC_NONE) {
		spin_unlock_irqrestore(&pb->irq_lock, irq_flags);
		dev_err(&pb->pdev->dev,
				"%s: stp%u: unable to bind int%u\n", __func__,
				stp->stp_id, interrupt_id);
		return -EEXIST;
	}

	irq->stp = stp;
	irq->source = IRQ_SRC_STP;
	stp->irq = irq;

	spin_unlock_irqrestore(&pb->irq_lock, irq_flags);

	dev_dbg(&pb->pdev->dev, "%s: stp%u: bind int%u" , __func__, stp->stp_id,
			interrupt_id);

	return 0;
}

/* The caller to this function must hold pb->lock */
int unbind_stp_interrupt(struct paintbox_data *pb,
		struct paintbox_session *session, struct paintbox_stp *stp)
{
	struct paintbox_irq *irq;
	unsigned long irq_flags;

	spin_lock_irqsave(&pb->irq_lock, irq_flags);

	irq = stp->irq;
	if (!irq) {
		spin_unlock_irqrestore(&pb->irq_lock, irq_flags);
		return -ENOENT;
	}

	release_all_waiters(pb, irq);

	stp->irq = NULL;
	irq->stp = NULL;
	irq->source = IRQ_SRC_NONE;

	spin_unlock_irqrestore(&pb->irq_lock, irq_flags);

	return 0;
}

/* The caller to this function must hold pb->lock */
int bind_mipi_interrupt(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_mipi_stream *stream,
		unsigned int interrupt_id, bool is_input)
{
	struct paintbox_irq *irq;
	unsigned long irq_flags;
	int ret;

	irq = get_interrupt(pb, session, interrupt_id, &ret);
	if (ret < 0)
		return ret;

	spin_lock_irqsave(&pb->irq_lock, irq_flags);

	if (irq->source != IRQ_SRC_NONE) {
		spin_unlock_irqrestore(&pb->irq_lock, irq_flags);
		dev_err(&pb->pdev->dev,
				"%s: mipi stream%u: unable to bind int%u\n",
				__func__, stream->stream_id, interrupt_id);
		return -EEXIST;
	}

	irq->mipi_stream = stream;
	irq->source = is_input ? IRQ_SRC_MIPI_IN_STREAM :
			IRQ_SRC_MIPI_OUT_STREAM;
	stream->irq = irq;

	spin_unlock_irqrestore(&pb->irq_lock, irq_flags);

	dev_dbg(&pb->pdev->dev, "%s: mipi stream%u: bind int%u" , __func__,
			stream->stream_id, interrupt_id);

	return 0;
}

/* The caller to this function must hold pb->lock */
int unbind_mipi_interrupt(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_mipi_stream *stream)
{
	struct paintbox_irq *irq;
	unsigned long irq_flags;

	spin_lock_irqsave(&pb->irq_lock, irq_flags);

	irq = stream->irq;
	if (!irq) {
		spin_unlock_irqrestore(&pb->irq_lock, irq_flags);
		return -ENOENT;
	}

	release_all_waiters(pb, irq);

	stream->irq = NULL;
	irq->mipi_stream = NULL;
	irq->source = IRQ_SRC_NONE;

	spin_unlock_irqrestore(&pb->irq_lock, irq_flags);

	return 0;
}

int wait_for_interrupt_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct interrupt_wait __user *user_wait;
	struct interrupt_wait wait;
	struct paintbox_irq *irq;
	unsigned long irq_flags;
	long wait_timeout;
	int ret, wait_ret;

	user_wait = (struct interrupt_wait __user *)arg;
	if (copy_from_user(&wait, user_wait, sizeof(wait)))
		return -EFAULT;

	mutex_lock(&pb->lock);
	irq = get_interrupt(pb, session, wait.interrupt_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	dev_dbg(&pb->pdev->dev, "%s: int%u: wait, timeout %llu" , __func__,
			wait.interrupt_id, wait.timeout_ns);

	spin_lock_irqsave(&pb->irq_lock, irq_flags);

#ifdef CONFIG_PAINTBOX_SIMULATOR_SUPPORT
	/* Check to see if the interrupt has already occured before running the
	 * Simulator.
	 */
	if (try_wait_for_completion(&irq->completion)) {
		spin_unlock_irqrestore(&pb->irq_lock, irq_flags);
		mutex_unlock(&pb->lock);
		return 0;
	} else {
		/* The Simulator has extra execution control registers that
		 * allow it to be switched from configuration mode to execution
		 * mode.
		 */
		ret = sim_execute(pb, irq, wait.timeout_ns);
		if (ret < 0) {
			spin_unlock_irqrestore(&pb->irq_lock, irq_flags);
			mutex_unlock(&pb->lock);
			return ret;
		}
	}
#endif

	irq->wait_count++;

	spin_unlock_irqrestore(&pb->irq_lock, irq_flags);

	mutex_unlock(&pb->lock);

	if (wait.timeout_ns != INT_MAX) {
		wait_timeout = wait_for_completion_interruptible_timeout(
				&irq->completion,
				nsecs_to_jiffies64(wait.timeout_ns));
		if (wait_timeout == 0)
			wait_ret = -ETIMEDOUT;
		else if (wait_timeout < 0)
			wait_ret = wait_timeout;
		else
			wait_ret = 0;
	} else {
		wait_ret = wait_for_completion_interruptible(&irq->completion);
	}

	mutex_lock(&pb->lock);

	/* Attempt to reacquire the interrupt resource to make sure it did not
	 * get released out from under us while waiting.
	 */
	irq = get_interrupt(pb, session, wait.interrupt_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	spin_lock_irqsave(&pb->irq_lock, irq_flags);

	irq->wait_count--;

	if (wait_ret < 0) {
		spin_unlock_irqrestore(&pb->irq_lock, irq_flags);
		mutex_unlock(&pb->lock);
		return wait_ret;
	}

	if (irq->error) {
		ret = irq->error;
		irq->error = 0;
	}

	spin_unlock_irqrestore(&pb->irq_lock, irq_flags);

	mutex_unlock(&pb->lock);

	return ret;
}

/* Must be called with interrupts disabled */
void signal_waiters(struct paintbox_data *pb, struct paintbox_irq *irq, int err)
{
	spin_lock(&pb->irq_lock);

	if (!irq) {
		spin_unlock(&pb->irq_lock);
		return;
	}

	irq->error = err;

	complete(&irq->completion);

	spin_unlock(&pb->irq_lock);
}

/* The caller to this function must hold pb->lock */
void init_waiters(struct paintbox_data *pb, struct paintbox_irq *irq)
{
	unsigned long irq_flags;

	if (!irq)
		return;

	spin_lock_irqsave(&pb->irq_lock, irq_flags);

	reinit_completion(&irq->completion);

	spin_unlock_irqrestore(&pb->irq_lock, irq_flags);
}

/* The caller to this function must hold pb->lock */
int release_interrupt(struct paintbox_data *pb,
		struct paintbox_session *session, struct paintbox_irq *irq)
{
	switch (irq->source) {
	case IRQ_SRC_NONE:
		break;
	case IRQ_SRC_DMA_CHANNEL:
		unbind_dma_interrupt(pb, session, irq->dma_channel);
		break;
	case IRQ_SRC_MIPI_IN_STREAM:
	case IRQ_SRC_MIPI_OUT_STREAM:
		unbind_mipi_interrupt(pb, session, irq->mipi_stream);
		break;
	case IRQ_SRC_STP:
		unbind_stp_interrupt(pb, session, irq->stp);
		break;
	default:
		dev_err(&pb->pdev->dev, "%s: invalid interrupt source\n",
				__func__);
		return -EINVAL;
	};

	list_del(&irq->session_entry);
	irq->session = NULL;
	irq->source = IRQ_SRC_NONE;

	return 0;
}

int release_interrupt_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	unsigned int interrupt_id = (unsigned int)arg;
	struct paintbox_irq *irq;
	int ret;

	mutex_lock(&pb->lock);
	irq = get_interrupt(pb, session, interrupt_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	ret = release_interrupt(pb, session, irq);

	mutex_unlock(&pb->lock);

	return ret;
}

int paintbox_irq_init(struct paintbox_data *pb)
{
	unsigned int interrupt_id;

	pb->irqs = kzalloc(sizeof(struct paintbox_irq) *
			pb->caps.num_interrupts, GFP_KERNEL);
	if (!pb->irqs)
		return -ENOMEM;

	for (interrupt_id = 0; interrupt_id < pb->caps.num_interrupts;
			interrupt_id++) {
		/* Store interrupt id with object as a convenience to avoid
		 * doing a lookup later on.
		 */
		pb->irqs[interrupt_id].interrupt_id = interrupt_id;
		pb->irqs[interrupt_id].source = IRQ_SRC_NONE;
		init_completion(&pb->irqs[interrupt_id].completion);
	}

	return 0;
}
