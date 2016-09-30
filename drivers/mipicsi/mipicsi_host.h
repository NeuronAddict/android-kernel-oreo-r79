#ifndef MIPICSI_HOST_H_
#define MIPICSI_HOST_H_

#include <linux/types.h>
#include "mipicsi_top.h"

struct mipicsi_host_dev {
	struct device	    *dev;     /* Device node */
	struct list_head    devlist;  /* Device list */
	spinlock_t          slock;    /* Spinlock */
	struct mutex        mutex;    /* Mutex */

	/** Device Tree Information */
	void __iomem      *base_address;
	uint32_t           mem_size;
	uint32_t           host_irq_number;
	uint32_t           host_irq;
};


void mipicsi_host_dphy_reset(enum mipicsi_top_dev dev);
void mipicsi_host_reset(enum mipicsi_top_dev dev);
int  mipicsi_host_start(struct mipicsi_top_cfg *config);
int mipicsi_host_stop(enum mipicsi_top_dev dev);
int mipicsi_host_hw_init(enum mipicsi_top_dev dev);

#endif
