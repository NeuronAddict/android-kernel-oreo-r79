/*
 * Driver interface for the PAINTBOX CMA. This driver is for register trace testing only.
 *
 * Copyright (C) 2016 Google, Inc.
 * Author: Fabrizio Basso <basso@google.com>
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

#ifndef __PAINTBOX_CMA_H__
#define __PAINTBOX_CMA_H__

#include <linux/compiler.h>
#include <linux/ioctl.h>

// Struct to request allocating a buffer with length specified by len_bytes.
// Returns buf_paddr and buf_vaddr to represent the buffer.
struct alloc_info {
        size_t   len_bytes; /* size of the buffer to allocate. */
        uint64_t buf_paddr; /* cma buffer phys address. */
};

// Struct to request freeing a buffer specified by free_info.
struct free_info {
        uint64_t buf_paddr; /* cma buffer phys address. */
};

// Struct to request write to allocated buffer with content from usr_buf.
struct write_info {
        void __user *usr_buf; /* user space buffer the data will be fetched from. */
        size_t   len_bytes;   /* size of the buffer to allocate. */
        uint64_t buf_paddr;   /* cma buffer phys address. */
};

// Struct to request read the content of the allocated buffer to usr_buf.
struct read_info {
        void __user *usr_buf; /* user space buffer the data will be written to. */
        size_t   len_bytes;   /* size of the buffer to allocate. */
        uint64_t buf_paddr;   /* cma buffer phys address. */
};

/* Ioctl interface to PAINTBOX CMA driver
 *
 * The following ioctls will return these error codes on error conditions:
 *
 * -EACCES: Resource access error
 * -EEXIST: Resource exists or is already in use
 * -EFAULT: Buffer transfer error (rare)
 * -EINVAL: Invalid Parameter
 * -ENOENT: No entry
 * -ENOMEN: Out of memory
 * -ENOSYS: Unimplemented Functionality
 *
 */

#define PAINTBOX_CMA_ALLOC_BUFFER      _IOW('p', 1, struct alloc_info)
#define PAINTBOX_CMA_FREE_BUFFER       _IOW('p', 2, struct free_info)
#define PAINTBOX_CMA_WRITE_BUFFER      _IOWR('p', 3, struct write_info)
#define PAINTBOX_CMA_READ_BUFFER       _IOWR('p', 4, struct read_info)

#endif /* __PAINTBOX_CMA_H__ */
