/*
 * STP support for the Paintbox programmable IPU
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

#ifndef __PAINTBOX_STP_H__
#define __PAINTBOX_STP_H__

#include <linux/io.h>

#include "paintbox-common.h"


/* Size of the debug buffer used for debugfs or verbose logging.  This value
 * should be reevaluated whenever the dump_stp_registers function is changed.
 */
#define STP_DEBUG_BUFFER_SIZE (STP_NUM_REGS * REG_DEBUG_BUFFER_SIZE)

int allocate_stp_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);
int release_stp_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);
int start_stp_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);
int stop_stp_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);
int resume_stp_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);
int reset_stp_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);
int setup_stp_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);
int write_stp_memory_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);
int read_stp_memory_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);
int write_stp_vector_memory_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);
int write_stp_vector_memory_replicate_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);
int read_stp_vector_memory_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);
int get_program_state_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);

int paintbox_stp_init(struct paintbox_data *pb);

/* The caller to this function must hold pb->lock */
int validate_stp(struct paintbox_data *pb, struct paintbox_session *session,
		unsigned int stp_id);
struct paintbox_stp *get_stp(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned int stp_id,
		int *err);
void release_stp(struct paintbox_data *pb,
		struct paintbox_session *session, struct paintbox_stp *stp);
void enable_stp_access_to_lbp(struct paintbox_data *pb,
		struct paintbox_session *session, struct paintbox_lbp *lbp);

#if defined(CONFIG_DEBUG_FS) || defined(VERBOSE_DEBUG)
int dump_stp_registers(struct paintbox_data *pb, char *buf, size_t len);
#endif

#endif /* __PAINTBOX_STP_H__ */
