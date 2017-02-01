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
#include "paintbox-stp.h"
#include "paintbox-stp-sim.h"


/* The caller to this function must hold pb->irq_lock */
static void release_all_waiters(struct paintbox_data *pb,
		struct paintbox_irq *irq)
{
	/* walk list of waiting threads, wake any that are waiting on this
	 * interrupt
	 */
	struct paintbox_waiter *waiter, *waiter_next;
	uint64_t mask;

	if (!irq)
		return;

	mask = 1ULL << irq->interrupt_id;

	/* waiting threads have priority on getting interrupts (longest waiting
	 * first)
	 */
	list_for_each_entry_safe(waiter, waiter_next, &irq->session->wait_list,
			session_entry) {
		uint64_t waiting_for_mask = (waiter->wait->interrupt_mask_all |
				waiter->wait->interrupt_mask_any) &
				~waiter->wait->interrupt_mask_fired;
		if ((waiting_for_mask & mask) == 0)
			/* this waiting thread isn't looking for this interrupt
			 * id
			 */
			continue;

		if (waiter->wait->error != 0)
			/* we've already stored an error in this thread (and
			 * woken it up) but it hasn't taken itself out of the
			 * list yet.
			 */
			continue;

		waiter->wait->error = -EPIPE;
		waiter->wait->interrupt_id_error = irq->interrupt_id;

		complete(&waiter->completion);
	}

	/* zero out the number of pending interrupts and interrupt data */
	reinit_completion(&irq->completion);
	irq->data_count = 0;
	memset(&irq->data, 0, sizeof(irq->data[0]) * IRQ_MAX_PER_WAIT_PERIOD);
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
				"%s: dma channel%u unable to bind int%u\n",
				__func__, channel->channel_id, interrupt_id);
		return -EEXIST;
	}

	irq->dma_channel = channel;
	irq->source = IRQ_SRC_DMA_CHANNEL;
	channel->irq = irq;

	spin_unlock_irqrestore(&pb->irq_lock, irq_flags);

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
				"%s: stp%u unable to bind int%u\n", __func__,
				stp->stp_id, interrupt_id);
		return -EEXIST;
	}

	irq->stp = stp;
	irq->source = IRQ_SRC_STP;
	stp->irq = irq;

	spin_unlock_irqrestore(&pb->irq_lock, irq_flags);

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
				"%s: mipi stream%u unable to bind int%u\n",
				__func__, stream->stream_id, interrupt_id);
		return -EEXIST;
	}

	irq->mipi_stream = stream;
	irq->source = is_input ? IRQ_SRC_MIPI_IN_STREAM :
			IRQ_SRC_MIPI_OUT_STREAM;
	stream->irq = irq;

	spin_unlock_irqrestore(&pb->irq_lock, irq_flags);

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

/* pop irq data off the head of the queue
 * pb->lock and pb->irq_lock must be held before calling this function
 */
int irq_pop_interrupt_data(struct paintbox_irq *irq)
{
	int ret = irq->data[0];
	if (!WARN_ON(irq->data_count == 0)) {
		unsigned int i, count = --irq->data_count;
		/* If there are other interrupts queued on this waiter then
		 * shift the errors down in the queue.
		 */
		for (i = 0; i < count; i++) {
			irq->data[i] = irq->data[i + 1];
		}
	}
	return ret;
}

/* Return true if the set of interrupts we were waiting for have fired
 * successfully.
 */
int interrupt_wait_finished(const struct interrupt_wait *wait)
{
	uint64_t all = wait->interrupt_mask_fired & wait->interrupt_mask_all;
	return (wait->interrupt_mask_fired & wait->interrupt_mask_any) ||
			(wait->interrupt_mask_all && all ==
			wait->interrupt_mask_all);
}

size_t interrupt_wait_size(int num_stps) {
	return sizeof(struct interrupt_wait) + (num_stps - 1) *
			sizeof(uint16_t);
}

int wait_for_interrupt_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct interrupt_wait __user *user_wait;
	struct interrupt_wait *wait = NULL;
	struct paintbox_waiter paintbox_waiter;
	unsigned long irq_flags;
	uint64_t interrupt_mask;
	int interrupt_id;
	int ret;
	size_t wait_size = interrupt_wait_size(pb->caps.num_stps);
	unsigned long start = jiffies;

	wait = kzalloc(wait_size, GFP_KERNEL);
	if (!wait)
		return -ENOMEM;

	user_wait = (struct interrupt_wait __user *)arg;
	if (copy_from_user(wait, user_wait, wait_size)) {
		kfree(wait);
		return -EFAULT;
	}

	/* initialize interrupt_wait output members */
	wait->interrupt_mask_fired = 0;
	memset(wait->interrupt_code, 0, sizeof(uint16_t) * pb->caps.num_stps);
	wait->error = 0;
	wait->interrupt_id_error = -1;  /* invalid interrupt id */

	dev_dbg(&pb->pdev->dev, "%s: mask_all:%llx mask_any:%llx, timeout %lld",
			__func__, wait->interrupt_mask_all,
			wait->interrupt_mask_any, wait->timeout_ns);

	mutex_lock(&pb->lock);

	/* prevent the interrupt handler from changing the state of completions
	 */
	spin_lock_irqsave(&pb->irq_lock, irq_flags);

	interrupt_mask = wait->interrupt_mask_all | wait->interrupt_mask_any;

	/* grab interrupts that have already occurred */
	for (interrupt_id = 0; interrupt_mask; interrupt_id++,
			interrupt_mask >>= 1) {
		struct paintbox_irq *irq;
		int get_interrupt_result;

		if ((interrupt_mask & 1) == 0)
			/* not waiting for this interrupt id */
			continue;

		irq = get_interrupt(pb, session, interrupt_id,
				&get_interrupt_result);
		if (get_interrupt_result < 0) {
			/* Trying to access an interrupt that we shouldn't.
			 * Something is very wrong, return ASAP.
			 */
			wait->error = get_interrupt_result;
			wait->interrupt_id_error = interrupt_id;
			break;
		}

		if (try_wait_for_completion(&irq->completion)) {
			/* data from non-stp interrupts is 0 or a -errno code.
			 * If it is an error the wait will end.
			 *
			 * data from stp interrupts are interrupt codes.  They
			 * need to be recorded but might not cause the wait to
			 * end so they are handled differently.
			 */
			int data = irq_pop_interrupt_data(irq);
			if (irq->source == IRQ_SRC_STP) {
				/* normal stp interrupt */
				unsigned int stp_index = stp_id_to_index(
						irq->stp->stp_id);
				wait->interrupt_code[stp_index] =
						(uint16_t)data;
				wait->interrupt_mask_fired |= 1ULL <<
						interrupt_id;
			} else if (data == 0) {
				/* normal (non-stp) interrupt */
				wait->interrupt_mask_fired |= 1ULL <<
						interrupt_id;
			} else {
				/* error */
				wait->error = data;
				wait->interrupt_id_error = interrupt_id;
				break;
			}
		}
	}

	/* If we got all of the interrupts we need to continue or an error, we
	 * can return now
	 */
	if (interrupt_wait_finished(wait) || wait->error) {
		spin_unlock_irqrestore(&pb->irq_lock, irq_flags);
		mutex_unlock(&pb->lock);

		ret = wait->error;
		goto cleanup_and_exit;
	}

	/* add this thread to the end of the list of threads waiting for
	 * interrupts
	 */
	paintbox_waiter.wait = wait;
	init_completion(&paintbox_waiter.completion);
	list_add_tail(&paintbox_waiter.session_entry, &session->wait_list);

	spin_unlock_irqrestore(&pb->irq_lock, irq_flags);
	mutex_unlock(&pb->lock);

#ifdef CONFIG_PAINTBOX_SIMULATOR_SUPPORT
	/* tell the simulator to run until the wait conditions have been
	 * satisfied
	 */
	ret = sim_execute(pb, wait);
	if (ret < 0)
		goto cleanup_and_exit;
#endif

	if (wait->timeout_ns == LONG_MAX)
		ret = wait_for_completion_interruptible(
				&paintbox_waiter.completion);
	else {
		long time_remaining;

		/* wait_for_completion_interruptible_timeout returns
		 * -ERESTARTSYS if it receives a signal.  Otherwise it returns
		 * the number of jiffies until the timeout.  So 0 means it timed
		 * out
		 */
		time_remaining = wait_for_completion_interruptible_timeout(
				&paintbox_waiter.completion,
				nsecs_to_jiffies64(wait->timeout_ns));
		if (time_remaining == 0) {
			wait->timeout_ns = 0;
			ret = -ETIMEDOUT;
		}	else if (time_remaining < 0) {
			/* TODO(ahampson) This really needs to be re-examined
			 * after reading this:
			 * https://goo.gl/dcqBPP
			 */
			unsigned long elapsed = jiffies - start;
			int64_t elapsed_ns = jiffies_to_nsecs(elapsed);
			if (elapsed_ns >= wait->timeout_ns) {
				wait->timeout_ns = 0;
			} else {
				wait->timeout_ns -= elapsed_ns;
			}
			ret = time_remaining; /* -ERESTARTSYS */
		} else {
			/* time_remaining > 0
			 * completion was signaled before timeout
			 */
			wait->timeout_ns = jiffies_to_nsecs(time_remaining);
			ret = 0;
		}
	}

	mutex_lock(&pb->lock);
	spin_lock_irqsave(&pb->irq_lock, irq_flags);

	/* remove this thread from the waiting thread list */
	list_del(&paintbox_waiter.session_entry);

	spin_unlock_irqrestore(&pb->irq_lock, irq_flags);
	mutex_unlock(&pb->lock);

	/* decide on a return value.  At this point, |ret| one of 0, -ETIMEOUT
	 * or -ERESTARTSYS
	 */
	if (wait->error) {
		/* irq errors take precidence over 0, -ETIMEOUT or -ERESTARTSYS
		 */
		ret = wait->error;
		goto cleanup_and_exit;
	}

	if (interrupt_wait_finished(wait)) {
		/* This is very unlikely, but it is possible that between waking
		 * up from wait_for_completion_interruptible* and getting the
		 * irq_lock that another interrupt was received that satified
		 * the wait condition.
		 */
		ret = 0;
		goto cleanup_and_exit;
	}

cleanup_and_exit:
	if (copy_to_user(user_wait, wait, wait_size)) {
		/* let the caller know that the wait structure is garbage */
		ret = -EFAULT;
	}
	kfree(wait);
	return ret;
}

/* Must be called with interrupts disabled */
void signal_waiters(struct paintbox_data *pb, struct paintbox_irq *irq,
		int data)
{
	struct paintbox_waiter *waiter, *waiter_next;
	uint64_t mask;

	spin_lock(&pb->irq_lock);

	/* why not check IRQ pointer against null and return before locking?
	 * Probably not a problem on Easel but on a future SMP system it may be
	 * possible for one CPU to be processing an interrupt while another CPU
	 * is in the process of releasing the IRQ object.
	 */
	if (!irq) {
		spin_unlock(&pb->irq_lock);
		return;
	}

	mask = 1ULL << irq->interrupt_id;

	/* waiting threads have priority on getting interrupts (longest waiting
	 * first)
	 */
	list_for_each_entry_safe(waiter, waiter_next, &irq->session->wait_list,
			session_entry) {

		uint64_t waiting_for_mask = (waiter->wait->interrupt_mask_all |
				waiter->wait->interrupt_mask_any) &
				~waiter->wait->interrupt_mask_fired;

		if ((waiting_for_mask & mask) == 0)
			/* this waiting thread isn't looking for this interrupt
			 * id
			 */
			continue;

		if (waiter->wait->error != 0)
			/* we've already stored an error in this thread (and
			 * woken it up) but it hasn't taken itself out of the
			 * list yet.
			 */
			continue;

		/* data from non-stp interrupts is 0 or a -errno code.  If it is
		 * an error the wait will end.
		 *
		 * data from stp interrupts are interrupt codes.  They need to
		 * be recorded but might not cause the wait to end so they are
		 * handled differently.
		 */
		if (irq->source == IRQ_SRC_STP) {
			/* normal stp interrupt */
			int stp_index = stp_id_to_index(irq->stp->stp_id);
			waiter->wait->interrupt_code[stp_index] =
					(uint16_t)data;
			waiter->wait->interrupt_mask_fired |= 1ULL <<
					irq->interrupt_id;
		} else if (data == 0) {
			/* normal (non-stp) interrupt */
			waiter->wait->interrupt_mask_fired |= 1ULL <<
					irq->interrupt_id;
		} else {
			/* error */
			waiter->wait->error = data;
			waiter->wait->interrupt_id_error = irq->interrupt_id;
		}

		/* if the signal is an error or the thread's wait conditions
		 * have been satisfied, wake the thread
		 */
		if (interrupt_wait_finished(waiter->wait) ||
				waiter->wait->error)
			complete(&waiter->completion);

		/* we've recorded the interrupt, we're done */
		spin_unlock(&pb->irq_lock);
		return;
	}

	/* if there are no waiting threads that can take this interrupt event,
	 * store the event in a completion and the data in irq->data
	 */
	if (!WARN_ON(irq->data_count == IRQ_MAX_PER_WAIT_PERIOD))
		irq->data[irq->data_count++] = data;

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

	irq->data_count = 0;

	memset(&irq->data, 0, sizeof(irq->data[0]) * IRQ_MAX_PER_WAIT_PERIOD);

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
