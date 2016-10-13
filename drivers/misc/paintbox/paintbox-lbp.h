/*
 * Linebuffer Pool Support for Paintbox IPU
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

#ifndef __PAINTBOX_LBP_H__
#define __PAINTBOX_LBP_H__

#include <linux/io.h>

#include "paintbox-common.h"


/* Size of the debug buffer used for debugfs or verbose logging.  These values
 * should be reevaluated whenever the dump_*_registers functions are changed.
 */
#define LBP_DEBUG_BUFFER_SIZE (LBP_NUM_REGS * REG_DEBUG_BUFFER_SIZE)
#define LB_DEBUG_BUFFER_SIZE (LB_NUM_REGS * REG_DEBUG_BUFFER_SIZE)

int allocate_lbp_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);
int release_lbp_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);
int setup_lb_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);
int reset_lbp_ioctl(struct paintbox_data *pb, struct paintbox_session *session,
		unsigned long arg);
int reset_lb_ioctl(struct paintbox_data *pb, struct paintbox_session *session,
		unsigned long arg);
int write_lbp_memory_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);
int read_lbp_memory_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);

int paintbox_lbp_init(struct paintbox_data *pb);
void paintbox_lbp_deinit(struct paintbox_data *pb);

/* The caller to these functions must hold pb->lock */
int validate_lbp(struct paintbox_data *pb, struct paintbox_session *session,
		int pool_id);
struct paintbox_lbp *get_lbp(struct paintbox_data *pb,
		struct paintbox_session *session, int pool_id, int *err);
struct paintbox_lb *get_lb(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned int lbp_id,
		unsigned int lb_id, int *err);
void release_lbp(struct paintbox_data *pb, struct paintbox_session *session,
		struct paintbox_lbp *lbp);
void reset_lb(struct paintbox_data *pb, unsigned int lbp_id,
		unsigned int lb_id);

#if defined(CONFIG_DEBUG_FS) || defined(VERBOSE_DEBUG)
int dump_lbp_registers(struct paintbox_debug *debug, char *buf, size_t len);
int dump_lb_registers(struct paintbox_debug *debug, char *buf, size_t len);
#endif

#endif /* __PAINTBOX_LBP_H__ */
