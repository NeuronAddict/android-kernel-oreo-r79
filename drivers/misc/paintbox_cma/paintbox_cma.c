/*
 * Driver for the IPU CMA. This driver is for register trace testing only.
 * Implementation reference: drivers/misc/paintbox/paintbox-dma-dram.c
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

#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/paintbox_cma.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/version.h>

struct paintbox_cma_data {
    struct mutex lock;
    struct miscdevice misc_device;
    struct platform_device *pdev;
};

struct paintbox_cma_session {
    struct paintbox_cma_data *dev;
    struct list_head buffer_list;
};

struct paintbox_cma_buffer_list_entry {
    size_t len_bytes; /* size of the buffer allocated. */
    uint64_t buf_paddr; /* cma buffer phys address. */
    void *buf_vaddr; /* cma buffer virt address. */
    struct list_head list_entry;
};

static int paintbox_cma_open(struct inode *ip, struct file *fp)
{
    struct paintbox_cma_session *session;
    struct paintbox_cma_data *pb;
    struct miscdevice *m = fp->private_data;

    pb = container_of(m, struct paintbox_cma_data, misc_device);

    session = kzalloc(sizeof(struct paintbox_cma_session), GFP_KERNEL);
    if (!session) {
        return -ENOMEM;
    }

    session->dev = pb;

    INIT_LIST_HEAD(&session->buffer_list);

    fp->private_data = session;

    return 0;
}

static int paintbox_cma_release(struct inode *ip, struct file *fp)
{
    struct paintbox_cma_session *session = fp->private_data;
    struct paintbox_cma_data *pb = session->dev;
    struct paintbox_cma_buffer_list_entry *entry;
    struct paintbox_cma_buffer_list_entry *entry_next;

    mutex_lock(&pb->lock);
    list_for_each_entry_safe(entry, entry_next, &session->buffer_list, list_entry) {
        dma_free_coherent(&pb->pdev->dev, entry->len_bytes, entry->buf_vaddr, entry->buf_paddr);
        list_del(&entry->list_entry);
        kfree(entry);
    }
    mutex_unlock(&pb->lock);

    kfree(session);

    return 0;
}

int write_buffer_ioctl(struct paintbox_cma_session *session, unsigned long arg)
{
    struct paintbox_cma_data *pb = session->dev;
    struct write_info __user *user_req;
    struct write_info req;

    bool entry_found = false;
    struct paintbox_cma_buffer_list_entry *entry;
    struct paintbox_cma_buffer_list_entry *entry_next;

    size_t min_len_bytes;

    user_req = (struct write_info __user *)arg;
    if (copy_from_user(&req, user_req, sizeof(req))) {
        return -EFAULT;
    }

    list_for_each_entry_safe(entry, entry_next, &session->buffer_list, list_entry) {
        if (entry->buf_paddr == req.buf_paddr) {
            min_len_bytes = min(entry->len_bytes, req.len_bytes);
            if (copy_from_user(entry->buf_vaddr, req.usr_buf, min_len_bytes)) {
                dev_err(&pb->pdev->dev, "%s %d error!\n", __FUNCTION__, __LINE__);
                return -EFAULT;
            }

            entry_found = true;
            break;
        }
    }

    if (!entry_found) {
        dev_err(&pb->pdev->dev, "%s %d allocation not found!\n", __FUNCTION__, __LINE__);
        return -EINVAL;
    }

    return 0;
}

int read_buffer_ioctl(struct paintbox_cma_session *session, unsigned long arg)
{
    struct paintbox_cma_data *pb = session->dev;
    struct read_info __user *user_req;
    struct read_info req;

    bool entry_found = false;
    struct paintbox_cma_buffer_list_entry *entry;
    struct paintbox_cma_buffer_list_entry *entry_next;

    size_t min_len_bytes;

    user_req = (struct read_info __user *)arg;
    if (copy_from_user(&req, user_req, sizeof(req))) {
        dev_err(&pb->pdev->dev, "%s %d error!\n", __FUNCTION__, __LINE__);
        return -EFAULT;
    }

    list_for_each_entry_safe(entry, entry_next, &session->buffer_list, list_entry) {
        if (entry->buf_paddr == req.buf_paddr) {
            min_len_bytes = min(entry->len_bytes, req.len_bytes);
            if (copy_to_user(req.usr_buf, entry->buf_vaddr, min_len_bytes)) {
                dev_err(&pb->pdev->dev, "%s %d error!\n", __FUNCTION__, __LINE__);
                return -EFAULT;
            }
            entry_found = true;
            break;
        }
    }

    if (!entry_found) {
        dev_err(&pb->pdev->dev, "%s %d allocation not found!\n", __FUNCTION__, __LINE__);
        return -EINVAL;
    }

    return 0;
}

int alloc_buffer_ioctl(struct paintbox_cma_session *session, unsigned long arg)
{
    struct paintbox_cma_data *pb = session->dev;
    struct alloc_info __user *user_req;
    struct alloc_info req;
    void *vaddr = NULL;

    struct paintbox_cma_buffer_list_entry *entry;

    user_req = (struct alloc_info __user *)arg;
    if (copy_from_user(&req, user_req, sizeof(req))) {
        dev_err(&pb->pdev->dev, "%s %d error!\n", __FUNCTION__, __LINE__);
        return -EFAULT;
    }

    vaddr = dma_alloc_coherent(&pb->pdev->dev, req.len_bytes, &req.buf_paddr, GFP_KERNEL);
    if (!vaddr) {
        dev_err(&pb->pdev->dev, "%s %d error!\n", __FUNCTION__, __LINE__);
        return -ENOMEM;
    }

    entry = kmalloc(sizeof(struct paintbox_cma_buffer_list_entry), GFP_KERNEL);
    if (!entry) {
        dma_free_coherent(&pb->pdev->dev, req.len_bytes, vaddr, req.buf_paddr);
        dev_err(&pb->pdev->dev, "%s %d error!\n", __FUNCTION__, __LINE__);
        return -ENOMEM;
    }
    entry->len_bytes = req.len_bytes;
    entry->buf_vaddr = vaddr;
    entry->buf_paddr = req.buf_paddr;

    if (copy_to_user(user_req, &req, sizeof(req))) {
        dma_free_coherent(&pb->pdev->dev, req.len_bytes, vaddr, req.buf_paddr);
        kfree(entry);
        dev_err(&pb->pdev->dev, "%s %d error!\n", __FUNCTION__, __LINE__);
        return -EFAULT;
    }

    list_add_tail(&entry->list_entry, &session->buffer_list);

    return 0;
}

int free_buffer_ioctl(struct paintbox_cma_session *session, unsigned long arg)
{
    struct paintbox_cma_data *pb = session->dev;
    struct free_info __user *user_req;
    struct free_info req;

    bool entry_found = false;
    struct paintbox_cma_buffer_list_entry *entry;
    struct paintbox_cma_buffer_list_entry *entry_next;

    user_req = (struct free_info __user *)arg;
    if (copy_from_user(&req, user_req, sizeof(req))) {
        dev_err(&pb->pdev->dev, "%s %d error!\n", __FUNCTION__, __LINE__);
        return -EFAULT;
    }

    list_for_each_entry_safe(entry, entry_next, &session->buffer_list, list_entry) {
        if (entry->buf_paddr == req.buf_paddr) {
            dma_free_coherent(&pb->pdev->dev, entry->len_bytes, entry->buf_vaddr, entry->buf_paddr);
            list_del(&entry->list_entry);
            kfree(entry);
            entry_found = true;
            break;
        }
    }

    if (!entry_found) {
        dev_err(&pb->pdev->dev, "%s %d allocation not found!\n", __FUNCTION__, __LINE__);
        return -EINVAL;
    }

    return 0;
}

static long paintbox_cma_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
    struct paintbox_cma_session *session = fp->private_data;
    struct paintbox_cma_data *pb = session->dev;
    int ret = 0;

    mutex_lock(&pb->lock);

    switch (cmd) {
        case PAINTBOX_CMA_ALLOC_BUFFER:
            ret = alloc_buffer_ioctl(session, arg);
            break;
        case PAINTBOX_CMA_FREE_BUFFER:
            ret = free_buffer_ioctl(session, arg);
            break;
        case PAINTBOX_CMA_WRITE_BUFFER:
            ret = write_buffer_ioctl(session, arg);
            break;
        case PAINTBOX_CMA_READ_BUFFER:
            ret = read_buffer_ioctl(session, arg);
            break;
        default:
            dev_err(&pb->pdev->dev, "%s: unknown ioctl 0x%0x\n", __func__, cmd);
            ret = -EINVAL;
            break;
    }

    mutex_unlock(&pb->lock);

    return ret;
}

static const struct file_operations paintbox_cma_fops = {
     .owner = THIS_MODULE,
     .open = paintbox_cma_open,
     .release = paintbox_cma_release,
     .unlocked_ioctl = paintbox_cma_ioctl,
};

static int paintbox_cma_probe(struct platform_device *pdev)
{
       int ret;
       struct paintbox_cma_data *pb;

       pb = devm_kzalloc(&pdev->dev, sizeof(*pb), GFP_KERNEL);
       if (pb == NULL)
           return -ENOMEM;

       pb->pdev = pdev;

       mutex_init(&pb->lock);

       platform_set_drvdata(pdev, pb);

       /* register the misc device */
       pb->misc_device.minor = MISC_DYNAMIC_MINOR,
       pb->misc_device.name  = "paintbox_cma",
       pb->misc_device.fops  = &paintbox_cma_fops,

       ret = misc_register(&pb->misc_device);
       if (ret) {
            pr_err("Failed to register misc device node (ret = %d)", ret);
            return ret;
       }

       return 0;
}

static int paintbox_cma_remove(struct platform_device *pdev)
{
      struct paintbox_cma_data *pb = platform_get_drvdata(pdev);

      misc_deregister(&pb->misc_device);

      mutex_destroy(&pb->lock);

      kfree(pb);

      return 0;
}

static const struct of_device_id paintbox_cma_of_match[] = {
{
      .compatible = "google,paintbox_cma", },
      {},
};
MODULE_DEVICE_TABLE(of, paintbox_cma_of_match);

static struct platform_driver paintbox_cma_driver = {
      .probe  = paintbox_cma_probe,
      .remove = paintbox_cma_remove,
      .driver = {
          .name = "paintbox_cma",
          .of_match_table = paintbox_cma_of_match,
      }
};
module_platform_driver(paintbox_cma_driver);

MODULE_AUTHOR("Fabrizio Basso <basso@google.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("PAINTBOX CMA Driver");
