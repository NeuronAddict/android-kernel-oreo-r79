/*
 * Android/Easel coprocessor communication.
 *
 * Copyright 2016 Google Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

// #define DEBUG

#include <uapi/linux/google-easel-comm.h>
#include "google-easel-comm-shared.h"
#include "google-easel-comm-private.h"

#include <asm/uaccess.h>
#include <linux/compat.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/types.h>

/* Per-Easel-service state */
static struct easelcomm_service easelcomm_service[EASELCOMM_SERVICE_COUNT];

/* Local command channel local state */
struct easelcomm_cmd_channel_local {
	char *buffer; /* start of command buffer */
	char *readp; /* ptr to next entry to read */
	uint64_t consumer_seqnbr_next; /* next cmd seq# to be consumed */
};
static struct easelcomm_cmd_channel_local cmd_channel_local;

/* Remote command channel local state */
struct easelcomm_cmd_channel_remote {
	/* Protects access to all mutable fields below */
	struct mutex mutex;
	/* offset of next entry to write */
	uint64_t write_offset;
	/* next command sequence number to write */
	uint64_t write_seqnbr;
	/* remote consumer has caught up, ready to wrap channel buffer */
	struct completion wrap_ready;
	/* remote channel is initialized */
	bool initialized;
	/* channel initialization or link shutdown received from remote */
	struct completion init_state_changed;
};
static struct easelcomm_cmd_channel_remote cmd_channel_remote;

/* Per-file-descriptor (user) state */
struct easelcomm_user_state {
	/* Which Easel service is registered for this fd, or NULL if none */
	struct easelcomm_service *service;
};

static int easelcomm_open(struct inode *inode, struct file *file);
static int easelcomm_release(struct inode *inode, struct file *file);
static long easelcomm_ioctl(
	struct file *file, unsigned int cmd, unsigned long arg);

#ifdef CONFIG_COMPAT
/* 32-bit userspace on 64-bit OS conversion defines */
struct easelcomm_compat_kbuf_desc {
	easelcomm_msgid_t message_id;  /* ID of message for this transfer */
	compat_uptr_t __user buf;      /* local userspace buffer ptr */
	uint32_t buf_size;	       /* size of the local buffer */
};

static long easelcomm_compat_ioctl(
	struct file *file, unsigned int cmd, unsigned long arg);
#else
#define easelcomm_compat_ioctl NULL
#endif

static const struct file_operations easelcomm_fops = {
	.owner		= THIS_MODULE,
	.open		= easelcomm_open,
	.release	= easelcomm_release,
	.unlocked_ioctl = easelcomm_ioctl,
	.compat_ioctl	= easelcomm_compat_ioctl,
	.llseek		= no_llseek,
};

struct miscdevice easelcomm_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
#ifdef CONFIG_GOOGLE_EASEL_AP
	.name = "easelcomm-client",
#else
	.name = "easelcomm-server",
#endif
	.fops = &easelcomm_fops,
};
EXPORT_SYMBOL(easelcomm_miscdev);

/*
 * Remote side of link indicates command channel ready.  Client calls this when
 * the EP bootstrap procedure is complete.  Server calls this when a LINK_INIT
 * command is received from the client.
 *
 * Init stuff and wakeup anybody waiting for the channel to be ready (to send
 * a command).
 */
static void easelcomm_cmd_channel_remote_set_ready(void)
{
	struct easelcomm_cmd_channel_remote *channel =
		&cmd_channel_remote;

	mutex_lock(&channel->mutex);
	/* Next write offset starts at top after header, cmd seq# zero */
	channel->write_offset =
		(sizeof(struct easelcomm_cmd_channel_header) + 7) & ~0x7;
	channel->write_seqnbr = 0;
	/* mark channel initialized, wakeup waiters. */
	channel->initialized = true;
	complete_all(&channel->init_state_changed);
	mutex_unlock(&channel->mutex);
}

/*
 * LINK_INIT command received from client, init local state for remote command
 * channel.
 */
static void easelcomm_handle_cmd_link_init(
	char *command_args, int command_arg_len)
{
	dev_dbg(easelcomm_miscdev.this_device, "recv cmd LINK_INIT\n");
	easelcomm_cmd_channel_remote_set_ready();
}

/*
 * Wait for remote command channel ready and initialized, prior to sending a
 * command.  Returns 0 if command channel ready and channel mutex is held, or
 * non-zero if signal received while waiting, channel mutex not held.
 */
static int easelcomm_wait_channel_initialized(
	struct easelcomm_cmd_channel_remote *channel)
{
	int ret = 0;

	mutex_lock(&channel->mutex);
	while (!channel->initialized) {
		mutex_unlock(&channel->mutex);
		ret = wait_for_completion_interruptible(
			&channel->init_state_changed);
		if (ret)
			return ret;
		mutex_lock(&channel->mutex);
	}

	return ret;
}

/* Command received from remote, dispatch. */
static void easelcomm_handle_command(struct easelcomm_cmd_header *cmdhdr)
{
	struct easelcomm_service *service;
	char *cmdargs = (char *)cmdhdr + sizeof(struct easelcomm_cmd_header);

	if (cmdhdr->service_id >= EASELCOMM_SERVICE_COUNT) {
		dev_err(easelcomm_miscdev.this_device,
			"invalid service ID %u received\n",
			cmdhdr->service_id);
		return;
	}
	service = &easelcomm_service[cmdhdr->service_id];

	switch(cmdhdr->command_code) {
	case EASELCOMM_CMD_LINK_INIT:
		easelcomm_handle_cmd_link_init(
			cmdargs, cmdhdr->command_arg_len);
		break;
	default:
		dev_err(easelcomm_miscdev.this_device,
			"svc %u invalid command code %u received\n",
			cmdhdr->service_id, cmdhdr->command_code);
	}
}

/*
 * Bump the consumer sequence number in the local command channel state,
 * indicating that we've consumed messages prior to the new number in
 * sequence.  The sequence number cannot be the reserved "command buffer
 * wrapped" marker; bump twice if needed to avoid.
 *
 * Not locked, but there's only one thread handling local command channel
 * data, in worker context.
 */
static void easelcomm_cmd_channel_bump_consumer_seqnbr(
	struct easelcomm_cmd_channel_local *channel)
{
	channel->consumer_seqnbr_next++;

	if (channel->consumer_seqnbr_next == CMD_BUFFER_WRAP_MARKER)
		channel->consumer_seqnbr_next++;
}

/*
 * Interrupt for new local command channel data ready was received.  This
 * function is called on a workqueue worker.  Process any new command channel
 * data found.
 *
 * Not locked, but there's only one thread handling local command channel
 * data, in worker context.
 */
void easelcomm_cmd_channel_data_handler(void)
{
	struct easelcomm_cmd_channel_local *channel =
		&cmd_channel_local;
	struct easelcomm_cmd_channel_header *channel_buf_hdr =
		(struct easelcomm_cmd_channel_header *)
		channel->buffer;

	/* While we haven't caught up to producer in sequence #s processed. */
	while (channel->consumer_seqnbr_next !=
		channel_buf_hdr->producer_seqnbr_next) {
		struct easelcomm_cmd_header *cmdhdr =
			(struct easelcomm_cmd_header *)channel->readp;

		dev_dbg(easelcomm_miscdev.this_device, "cmdchan consumer loop"
			" prodseq=%llu consseq=%llu off=%lx\n",
			channel_buf_hdr->producer_seqnbr_next,
			channel->consumer_seqnbr_next,
			channel->readp - channel->buffer);

		/*
		 * If producer dropped a wrap marker at the current position
		 * then wrap our read poitner and let the producer know we
		 * wrapped and both sides are ready to continue at the top of
		 * the buffer.
		 */
		if (cmdhdr->sequence_nbr == CMD_BUFFER_WRAP_MARKER) {
			dev_dbg(easelcomm_miscdev.this_device,
				"cmdchan consumer wrap at off=%lx\n",
				channel->readp - channel->buffer);
			channel->readp =
				channel->buffer +
				sizeof(struct easelcomm_cmd_channel_header);
			/* Send consumer wrapped IRQ to remote */
			easelcomm_hw_send_wrap_interrupt();
			/* Wrapping consumes a seqnbr */
			easelcomm_cmd_channel_bump_consumer_seqnbr(channel);
			continue;
		}

		dev_dbg(easelcomm_miscdev.this_device,
			"cmdchan recv cmd seq=%llu svc=%u"
			" cmd=%u len=%u off=%lx\n",
			cmdhdr->sequence_nbr, cmdhdr->service_id,
			cmdhdr->command_code, cmdhdr->command_arg_len,
			channel->readp - channel->buffer);
		if (sizeof(struct easelcomm_cmd_header) +
			cmdhdr->command_arg_len >
			EASELCOMM_CMD_CHANNEL_SIZE) {
			dev_err(easelcomm_miscdev.this_device,
				"command channel corruption detected:"
				" seq=%llu svc=%u cmd=%u len=%u off=%lx\n",
				cmdhdr->sequence_nbr, cmdhdr->service_id,
				cmdhdr->command_code, cmdhdr->command_arg_len,
				channel->readp - channel->buffer);
			break;
		}

		if (cmdhdr->sequence_nbr !=
			channel->consumer_seqnbr_next) {
			dev_err(easelcomm_miscdev.this_device,
				"command channel corruption detected:"
				" expected seq# %llu, got %llu\n",
				channel->consumer_seqnbr_next,
				cmdhdr->sequence_nbr);
			break;
		}

		/* Process the command. */
		easelcomm_handle_command(cmdhdr);
		channel->readp += sizeof(struct easelcomm_cmd_header) +
			cmdhdr->command_arg_len;
		/* 8-byte-align next entry pointer */
		if ((uintptr_t)channel->readp & 0x7)
			channel->readp += 8 - ((uintptr_t)channel->readp & 0x7);
		easelcomm_cmd_channel_bump_consumer_seqnbr(channel);
	}
	dev_dbg(easelcomm_miscdev.this_device, "cmdchan consumer exiting"
		" prodseq=%llu consseq=%llu off=%lx\n",
		channel_buf_hdr->producer_seqnbr_next,
		channel->consumer_seqnbr_next,
		channel->readp - channel->buffer);
}
EXPORT_SYMBOL(easelcomm_cmd_channel_data_handler);

/*
 * Interrupt from remote indicating remote has followed our channel buffer
 * wrap received.  This function called in workqueue worker context.  Wakeup
 * the command producer that is waiting on the remote to follow the wrap.
 */
void easelcomm_cmd_channel_wrap_handler(void)
{
	struct easelcomm_cmd_channel_remote *channel =
		&cmd_channel_remote;
	dev_dbg(easelcomm_miscdev.this_device,
		"cmdchan remote wrap IRQ received\n");
	complete(&channel->wrap_ready);
}
EXPORT_SYMBOL(easelcomm_cmd_channel_wrap_handler);

/* Device file open.  Allocate a user state structure. */
static int easelcomm_open(struct inode *inode, struct file *file)
{
	struct easelcomm_user_state *user_state =
		kzalloc(sizeof(struct easelcomm_user_state), GFP_KERNEL);
	if (!user_state)
		return -ENOMEM;
	file->private_data = user_state;
	return 0;
}

/* Device file descriptor release, initiate service shutdown and cleanup */
static int easelcomm_release(struct inode *inode, struct file *file)
{
	struct easelcomm_user_state *user_state = file->private_data;

	if (user_state) {
		struct easelcomm_service *service = user_state->service;

		if (service) {
			dev_dbg(easelcomm_miscdev.this_device,
				"svc %u release\n", service->service_id);

			spin_lock(&service->lock);
			service->user = NULL;
			spin_unlock(&service->lock);
		}

		kfree(file->private_data);
		file->private_data = NULL;
	}
	return 0;
}

/*
 * Bump the producer next sequence number in the remote command channel buffer
 * header.  After this, the remote side can then see that entries prior to
 * that sequence number are present.  The sequence number cannot be the
 * reserved "command buffer wrapped" marker; bump twice to avoid if needed.
 *
 * Call with remote channel producer mutex held.
 */
static int easelcomm_cmd_channel_bump_producer_seqnbr(
	struct easelcomm_cmd_channel_remote *channel)
{
	int ret;

	channel->write_seqnbr++;
	if (channel->write_seqnbr == CMD_BUFFER_WRAP_MARKER)
		channel->write_seqnbr++;
	ret = easelcomm_hw_remote_write(
		&channel->write_seqnbr, sizeof(uint64_t),
		(uint64_t)EASELCOMM_CMDCHAN_PRODUCER_SEQNBR);
	return ret;
}

/*
 * Start the process of writing a new command to the remote command channel.
 * Wait for channel init'ed and grab the mutex.	 Wraparound the buffer if
 * needed.  Write the command header.
 * If no error then return zero and channel mutex is locked on return.
 * If error then return non-zero and channel mutex is unlocked.
 */
int easelcomm_start_cmd(
	struct easelcomm_service *service, int command_code,
	int command_arg_len)
{
	struct easelcomm_cmd_channel_remote *channel =
		&cmd_channel_remote;
	struct easelcomm_cmd_header cmdhdr;
	unsigned int cmdbuf_size =
	    sizeof(struct easelcomm_cmd_header) + command_arg_len;
	int ret;

	ret = easelcomm_wait_channel_initialized(channel);
	/* If no error then channel->mutex is locked */
	if (ret)
		return ret;

	/* Choose a spot for the new entry, wrap around if needed. */
	if (channel->write_offset + cmdbuf_size >
		EASELCOMM_CMD_CHANNEL_SIZE -
		sizeof(struct easelcomm_cmd_channel_header)) {
		uint64_t wrap_marker = CMD_BUFFER_WRAP_MARKER;

		/* Write the "buffer wrapped" marker in place of sequence # */
		dev_dbg(easelcomm_miscdev.this_device, "cmdchan producer wrap"
			" at off=%llx seq=%llu\n",
			channel->write_offset, channel->write_seqnbr);
		ret = easelcomm_hw_remote_write(
			&wrap_marker, sizeof(wrap_marker),
			channel->write_offset);
		if (ret)
			goto error;

		/*
		 * Bump the producer seqnbr for the wrap marker (so consumer
		 * knows its there) and send IRQ to remote to consume the new
		 * data.
		 *
		 * Consumer will process entries up to and including the wrap
		 * marker and then send a "wrapped" interrupt that wakes up
		 * the wrap_ready completion.
		 */
		ret = easelcomm_cmd_channel_bump_producer_seqnbr(channel);
		if (ret)
			goto error;
		ret = easelcomm_hw_send_data_interrupt();
		if (ret)
			goto error;

		channel->write_offset =
			(sizeof(struct easelcomm_cmd_channel_header) + 7)
			& ~0x7;
		/* Wait for remote to catch up.	 IRQ from remote wakes this. */
		ret = wait_for_completion_interruptible(&channel->wrap_ready);
		if (ret)
			goto error;
	}

	dev_dbg(easelcomm_miscdev.this_device, "cmdchan producer sending cmd"
		" seq#%llu svc=%u cmd=%u arglen=%u off=%llx\n",
		channel->write_seqnbr, service->service_id, command_code,
		command_arg_len, channel->write_offset);
	cmdhdr.service_id = service->service_id;
	cmdhdr.sequence_nbr = channel->write_seqnbr;
	cmdhdr.command_code = command_code;
	cmdhdr.command_arg_len = command_arg_len;

	/*
	 * Send the command header. Subsequent calls to
	 * easelcomm_append_cmd_args() and easelcomm_send_cmd() will finish
	 * the in-progress command.
	 */
	ret = easelcomm_hw_remote_write(
		&cmdhdr, sizeof(cmdhdr), channel->write_offset);
	if (ret)
		goto error;

	/* Advance write offset */
	channel->write_offset += sizeof(cmdhdr);
	return 0;

error:
	mutex_unlock(&channel->mutex);
	return ret;
}
EXPORT_SYMBOL(easelcomm_start_cmd);

/*
 * Append arguments to the in-progress command being sent.
 * Call with remote channel producer mutex held.
 */
int easelcomm_append_cmd_args(
	struct easelcomm_service *service, void *cmd_args,
	size_t cmd_args_len)
{
	struct easelcomm_cmd_channel_remote *channel =
		&cmd_channel_remote;
	int ret = easelcomm_hw_remote_write(
		cmd_args, cmd_args_len, channel->write_offset);
	if (ret) {
		mutex_unlock(&channel->mutex);
		return ret;
	}
	channel->write_offset += cmd_args_len;
	return 0;
}
EXPORT_SYMBOL(easelcomm_append_cmd_args);

/* Start and send a command with no arguments. */
int easelcomm_send_cmd_noargs(
	struct easelcomm_service *service, int command_code)
{
	int ret;

	ret = easelcomm_start_cmd(service, command_code, 0);
	if (ret)
		return ret;
	return easelcomm_send_cmd(service);
}

/*
 * Finish sending an in-progress command to the remote.
 */
int easelcomm_send_cmd(struct easelcomm_service *service)
{
	struct easelcomm_cmd_channel_remote *channel =
		&cmd_channel_remote;
	int ret;

	/*
	 * Bump the producer sequence number.  Remote may now see and process
	 * the new entry as soon as this value is visible to it.
	 */
	ret = easelcomm_cmd_channel_bump_producer_seqnbr(channel);
	if (ret)
		goto out;
	/* Send data ready IRQ to remote */
	ret = easelcomm_hw_send_data_interrupt();
	if (ret)
		goto out;

	/* Bump write offset for next command, 8-byte-integer-aligned */
	if (channel->write_offset & 0x7)
		channel->write_offset += 8 - (channel->write_offset & 0x7);

out:
	mutex_unlock(&channel->mutex);
	return ret;
}
EXPORT_SYMBOL(easelcomm_send_cmd);

/* Handle REGISTER ioctl, register an open fd for an Easel service. */
static int easelcomm_register(struct easelcomm_user_state *user_state,
			unsigned int service_id) {
	struct easelcomm_service *service;
	if (service_id >= EASELCOMM_SERVICE_COUNT)
		return -EINVAL;
	service = &easelcomm_service[service_id];

	spin_lock(&service->lock);
	if (service->user) {
		spin_unlock(&service->lock);
		return -EBUSY;
	}
	user_state->service = service;
	service->user = user_state;
	service->shutdown_local = false;
	service->shutdown_remote = false;
	spin_unlock(&service->lock);
	dev_dbg(easelcomm_miscdev.this_device, "REGISTER svc %u\n",
		service_id);

	/*
	 * New client flushes any messages generated from activity by previous
	 * clients.
	 *
	 * New server handles any client messages that were waiting for the
	 * server to startup and process, does not flush at start/restart.
	 * Server can explicitly call the flush ioctl if wanted, but with the
	 * understanding any clients starting at the same time may have their
	 * traffic dropped.
	 */
	if (easelcomm_is_client())
		easelcomm_flush_service(service);
	return 0;
}

/*
 * Client calls this in kworker context upon receipt of BOOTSTRAP_READY IRQ
 * from server.	 Call MNH host driver to setup iATU mappings for the command
 * channels and send LINK_INIT.
 */
int easelcomm_client_remote_cmdchan_ready_handler(void)
{
	int ret;

	/* Sync up client and server sides using MNH host driver APIs */
	ret = easelcomm_hw_ap_setup_cmdchans();
	if (ret)
		return ret;
	easelcomm_cmd_channel_remote_set_ready();
	/* Client can now send link init to server */
	dev_dbg(easelcomm_miscdev.this_device, "send cmd LINK_INIT\n");
	return easelcomm_send_cmd_noargs(
		&easelcomm_service[EASELCOMM_SERVICE_SYSCTRL],
		EASELCOMM_CMD_LINK_INIT);
}
EXPORT_SYMBOL(easelcomm_client_remote_cmdchan_ready_handler);

/*
 * Handle ioctls that take a struct easelcomm_kbuf_desc * argument, convert
 * field layout and buf field pointer for 32-bit compat if needed.
 */
static int easelcomm_ioctl_kbuf_desc(
	struct easelcomm_service *service, unsigned int cmd,
	unsigned long arg, bool compat)
{
	struct easelcomm_kbuf_desc kbuf_desc;
	int ret;

	if (compat) {
#ifdef CONFIG_COMPAT
		struct easelcomm_compat_kbuf_desc compat_kbuf_desc;

		if (copy_from_user(
				&compat_kbuf_desc, (void __user *) arg,
				sizeof(struct easelcomm_compat_kbuf_desc)))
			return -EFAULT;

		kbuf_desc.message_id = compat_kbuf_desc.message_id;
		kbuf_desc.buf = compat_ptr(compat_kbuf_desc.buf);
		kbuf_desc.buf_size = compat_kbuf_desc.buf_size;
#else
		return -EINVAL;
#endif
	} else {
		if (copy_from_user(
				&kbuf_desc, (void __user *) arg,
				sizeof(struct easelcomm_kbuf_desc)))
			return -EFAULT;
	}

	switch (cmd) {
	case EASELCOMM_IOC_WRITEDATA:
		ret = easelcomm_write_msgdata(service, &kbuf_desc);
		break;
	case EASELCOMM_IOC_READDATA:
		ret = easelcomm_read_msgdata(service, &kbuf_desc);
		break;
	case EASELCOMM_IOC_SENDDMA:
		ret = easelcomm_send_dma(service, &kbuf_desc);
		break;
	case EASELCOMM_IOC_RECVDMA:
		ret = easelcomm_receive_dma(service, &kbuf_desc);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

/* 32-bit and 64-bit userspace ioctl handling, dispatch. */
static long easelcomm_ioctl_common(
	struct file *file, unsigned int cmd, unsigned long arg, bool compat)
{
	int ret = 0;
	struct easelcomm_user_state *user_state = file->private_data;
	struct easelcomm_service *service;

	if (_IOC_TYPE(cmd) != EASELCOMM_IOC_MAGIC)
		return -EINVAL;

	if (WARN_ON(!user_state))
		return -EINVAL;

	/* REGISTER is the only ioctl that doesn't need a service registered. */
	if (cmd == EASELCOMM_IOC_REGISTER)
		return easelcomm_register(user_state, (unsigned int) arg);

	service = user_state->service;
	if (!service) {
		dev_err(easelcomm_miscdev.this_device,
			"user has not registered a service for the file\n");
		return -EINVAL;
	}

	switch (cmd) {
	case EASELCOMM_IOC_SENDMSG:
		ret = easelcomm_send_message_ioctl(
			service, (struct easelcomm_kmsg_desc *)arg);
		break;
	case EASELCOMM_IOC_WRITEDATA:
	case EASELCOMM_IOC_READDATA:
	case EASELCOMM_IOC_SENDDMA:
	case EASELCOMM_IOC_RECVDMA:
		/* Convert 32-bit kbuf_desc argument if needed and dispatch. */
		ret = easelcomm_ioctl_kbuf_desc(service, cmd, arg, compat);
		break;
	case EASELCOMM_IOC_WAITMSG:
		ret = easelcomm_wait_message(
			service, (struct easelcomm_kmsg_desc *)arg);
		break;
	case EASELCOMM_IOC_WAITREPLY:
		ret = easelcomm_wait_reply(
			service, (struct easelcomm_kmsg_desc *)arg);
		break;
	case EASELCOMM_IOC_SHUTDOWN:
		easelcomm_initiate_service_shutdown(service);
		break;
	case EASELCOMM_IOC_FLUSH:
		easelcomm_flush_service(service);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

/* 64-bit ioctl handling. */
static long easelcomm_ioctl(
	struct file *file, unsigned int cmd, unsigned long arg)
{
	return easelcomm_ioctl_common(file, cmd, arg, false);
}

#ifdef CONFIG_COMPAT
/* 32-bit ioctl handling. */
static long easelcomm_compat_ioctl(
	struct file *file, unsigned int cmd, unsigned long arg)
{
	unsigned int cmd_nr = _IOC_NR(cmd);
	int ret;

	/* Fixup pointer argument size for ioctls 1..7 */
	if ((cmd_nr >= 1 && cmd_nr <= 7) &&
		_IOC_SIZE(cmd) == sizeof(compat_uptr_t)) {
		cmd &= ~(_IOC_SIZEMASK << _IOC_SIZESHIFT);
		cmd |= sizeof(void *) << _IOC_SIZESHIFT;
	}

	switch (cmd) {
	case EASELCOMM_IOC_SENDMSG:
	case EASELCOMM_IOC_WAITMSG:
	case EASELCOMM_IOC_WAITREPLY:
	case EASELCOMM_IOC_WRITEDATA:
	case EASELCOMM_IOC_READDATA:
	case EASELCOMM_IOC_SENDDMA:
	case EASELCOMM_IOC_RECVDMA:
		/* pointer argument, fixup */
		ret = easelcomm_ioctl_common(file, cmd,
					(unsigned long)compat_ptr(arg), true);
		break;

	case EASELCOMM_IOC_REGISTER:
	case EASELCOMM_IOC_SHUTDOWN:
	case EASELCOMM_IOC_FLUSH:
		/* no ioctl argument conversion needed */
		ret = easelcomm_ioctl_common(file, cmd, arg, true);
		break;

	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif /* CONFIG_COMPAT */

/* Init local state for remote command channel. */
static void easelcomm_init_cmd_channel_remote(
	struct easelcomm_cmd_channel_remote *channel)
{
	mutex_init(&channel->mutex);
	channel->initialized = false;
	init_completion(&channel->init_state_changed);
	init_completion(&channel->wrap_ready);
	/* Write pointer starts after the channel header */
	channel->write_offset =
		(sizeof(struct easelcomm_cmd_channel_header) + 7) & ~0x7;
	channel->write_seqnbr = 0;
}

/* Init local command channel. */
static int easelcomm_init_cmd_channel_local(
	struct easelcomm_cmd_channel_local *channel, void *cmdchan_buffer)
{
	channel->buffer = cmdchan_buffer;
	if (!channel->buffer)
		return -ENOMEM;
	channel->readp = channel->buffer +
		sizeof(struct easelcomm_cmd_channel_header);

	/* next sequence # to be produced/consumed starts at zero */
	channel->consumer_seqnbr_next = 0;
	((struct easelcomm_cmd_channel_header *)
		channel->buffer)->producer_seqnbr_next = 0;
	return 0;
}

/*
 * Callback from h/w layer when PCIe ready (which is immediate for server,
 * happens on first PCIe host driver probe on client).	The h/w layer has
 * allocated the local command channel buffer (which needs the host PCIe
 * driver on client), the pointer to which is passed as an argument.  Can now
 * init our local command channel state, and do remote too.
 */
int easelcomm_init_pcie_ready(void *local_cmdchan_buffer)
{
	int ret;

	ret = easelcomm_init_cmd_channel_local(
		&cmd_channel_local, local_cmdchan_buffer);
	easelcomm_init_cmd_channel_remote(&cmd_channel_remote);
	return ret;
}
EXPORT_SYMBOL(easelcomm_init_pcie_ready);

static int __init easelcomm_init(void)
{
	int ret;
	int i;

	ret = misc_register(&easelcomm_miscdev);
	if (ret) {
		dev_err(easelcomm_miscdev.this_device,
			"misc_register failed\n");
		return ret;
	}
	dev_info(easelcomm_miscdev.this_device,
		"registered at misc device minor %d\n",
		easelcomm_miscdev.minor);

	for (i = 0; i < EASELCOMM_SERVICE_COUNT; i++) {
		struct easelcomm_service *service = &easelcomm_service[i];

		service->service_id = i;
		service->user = NULL;
		service->shutdown_local = false;
		service->shutdown_remote = false;
		spin_lock_init(&service->lock);
		INIT_LIST_HEAD(&service->local_list);
		INIT_LIST_HEAD(&service->receivemsg_queue);
		init_completion(&service->receivemsg_queue_new);
		INIT_LIST_HEAD(&service->remote_list);
		init_completion(&service->flush_done);
	}

	easelcomm_hw_init();

	/*
	 * TODO-LATER: register reboot notifier that flushes/aborts and
	 * prevents creating new requests.
	 */

	return 0;
}

static void __exit easelcomm_exit(void)
{
	int i;

	// TODO-LATER: Grab mutex for remote channel, disallow further messages
	// TODO-LATER: Set local shutdown flag, disallow further activity

	for (i = 0; i < EASELCOMM_SERVICE_COUNT; i++) {
		struct easelcomm_service *service = &easelcomm_service[i];
		easelcomm_flush_local_service(service);
	}

	// TODO-LATER: Send link shutdown command, wait for remote ACK

	kfree(cmd_channel_local.buffer);
	misc_deregister(&easelcomm_miscdev);
}

module_init(easelcomm_init);
module_exit(easelcomm_exit);

MODULE_AUTHOR("Google Inc.");
MODULE_DESCRIPTION("Easel coprocessor communication driver");
MODULE_LICENSE("GPL");
