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

#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/types.h>

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
int init_stp_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);
int setup_stp_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);
int get_program_state_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);
int enable_stp_interrupt_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);
int disable_stp_interrupt_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);
int bind_stp_interrupt_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);
int unbind_stp_interrupt_ioctl(struct paintbox_data *pb,
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
void disable_stp_access_to_lbp(struct paintbox_data *pb,
		struct paintbox_session *session, struct paintbox_lbp *lbp);

#if defined(CONFIG_DEBUG_FS) || defined(VERBOSE_DEBUG)
int dump_stp_registers(struct paintbox_debug *debug, char *buf, size_t len);
#endif

irqreturn_t paintbox_stp_interrupt(struct paintbox_data *pb, uint32_t stp_mask);

static inline unsigned int stp_id_to_index(unsigned int stp_id)
{
	return stp_id - 1;
}

static inline unsigned int stp_index_to_id(unsigned int stp_index)
{
	return stp_index + 1;
}

#endif /* __PAINTBOX_STP_H__ */
