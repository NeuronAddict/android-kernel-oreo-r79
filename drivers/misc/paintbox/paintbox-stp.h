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
#include <linux/ktime.h>
#include <linux/types.h>

#include "paintbox-common.h"

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
int paintbox_reset_all_stp_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session);
int setup_stp_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);
int get_program_state_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);
int paintbox_get_all_processor_states(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);
int bind_stp_interrupt_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);
int unbind_stp_interrupt_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);

int paintbox_stp_init(struct paintbox_data *pb);
int paintbox_stp_deinit(struct paintbox_data *pb);

/* The caller to this function must hold pb->lock */
int validate_stp(struct paintbox_data *pb, struct paintbox_session *session,
		unsigned int stp_id);
struct paintbox_stp *get_stp(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned int stp_id,
		int *err);
void release_stp(struct paintbox_data *pb,
		struct paintbox_session *session, struct paintbox_stp *stp);

/* The LBP access control masks are not implemented on the V0 IPU. */
#if CONFIG_PAINTBOX_VERSION_MAJOR >= 1
void enable_stp_access_to_lbp(struct paintbox_data *pb,
		struct paintbox_session *session, struct paintbox_lbp *lbp);
void disable_stp_access_to_lbp(struct paintbox_data *pb,
		struct paintbox_session *session, struct paintbox_lbp *lbp);
#endif

irqreturn_t paintbox_stp_interrupt(struct paintbox_data *pb, uint64_t stp_mask,
		ktime_t timestamp);

static inline unsigned int stp_id_to_index(unsigned int stp_id)
{
	return stp_id - 1;
}

static inline unsigned int stp_index_to_id(unsigned int stp_index)
{
	return stp_index + 1;
}

/* The caller to this function must hold pb->dma.dma_lock. */
static inline void paintbox_stp_select(struct paintbox_data *pb,
		unsigned int stp_id)
{
	if (pb->stp.selected_stp_id == stp_id)
		return;

	pb->stp.selected_stp_id = stp_id;
	writel(stp_id, pb->stp.reg_base + STP_SEL);
}

#endif /* __PAINTBOX_STP_H__ */
