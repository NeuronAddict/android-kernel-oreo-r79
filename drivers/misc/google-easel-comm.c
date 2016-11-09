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

	misc_deregister(&easelcomm_miscdev);
}

module_init(easelcomm_init);
module_exit(easelcomm_exit);

MODULE_AUTHOR("Google Inc.");
MODULE_DESCRIPTION("Easel coprocessor communication driver");
MODULE_LICENSE("GPL");
