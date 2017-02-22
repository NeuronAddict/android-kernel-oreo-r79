/*
 * STP support (Simulator Specific) for the Paintbox programmable IPU
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

#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include "paintbox-common.h"
#include "paintbox-regs.h"
#include "paintbox-sim-regs.h"
#include "paintbox-stp.h"
#include "paintbox-stp-sim.h"

int sim_get_stp_idle_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	unsigned int stp_id = (unsigned int)arg;
	int ret;

	mutex_lock(&pb->lock);
	ret = validate_stp(pb, session, stp_id);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	ret = !!(readl(pb->sim_base + SIM_STAT) & (1 << stp_id));

	mutex_unlock(&pb->lock);

	return ret;
}

#define STP_IDLE_MAX_RETRY    3
#define STP_IDLE_MIN_SLEEP_US 10
#define STP_IDLE_MAX_SLEEP_US 100

/* The caller to this function must hold pb->lock */
int sim_wait_for_idle(struct paintbox_data *pb, struct paintbox_stp *stp)
{
	unsigned int retry_count = 0;

	do {
		if (readl(pb->sim_base + SIM_STAT) & (1 << stp->stp_id))
			break;

		writel(SIM_WAIT_IDLE, pb->sim_base + SIM_CTRL);

		usleep_range(STP_IDLE_MIN_SLEEP_US, STP_IDLE_MAX_SLEEP_US);
	} while (retry_count++ <= STP_IDLE_MAX_RETRY);

	if (!(readl(pb->sim_base + SIM_STAT) & (1 << stp->stp_id))) {
		dev_err(&pb->pdev->dev, "%s: stp%u: timeout waiting for idle\n",
				__func__, stp->stp_id);
		return -ETIMEDOUT;
	}

	return 0;
}

int sim_wait_for_idle_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct paintbox_stp *stp, *stp_next;
	int ret;

	mutex_lock(&pb->lock);

	list_for_each_entry_safe(stp, stp_next, &session->stp_list,
			session_entry) {
		ret = sim_wait_for_idle(pb, stp);
		if (ret < 0) {
			mutex_unlock(&pb->lock);
			return ret;
		}
	}

	dev_dbg(&pb->pdev->dev, "%s: all processors idle\n",  __func__);

	mutex_unlock(&pb->lock);

	return 0;
}
