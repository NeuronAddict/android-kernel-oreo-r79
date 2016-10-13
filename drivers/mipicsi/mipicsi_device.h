
#ifndef MIPICSI_DEVICE_H_
#define MIPICSI_DEVICE_H_

#include "mipicsi_top.h"

struct mipicsi_device_dev {
	struct device	    *dev;     /* Device node */
	struct list_head    devlist;  /* Device list */
	spinlock_t          slock;    /* Spinlock */
	struct mutex        mutex;    /* Mutex */

	/** Device Tree Information */
	void __iomem      *base_address;
	uint32_t           mem_size;
	uint32_t           device_irq_number;
	uint32_t           device_irq;
};


void mipicsi_device_reset(enum mipicsi_top_dev dev);
void mipicsi_device_dphy_reset(enum mipicsi_top_dev dev);
int mipicsi_device_start(struct mipicsi_top_cfg *config);
int mipicsi_device_stop(enum mipicsi_top_dev dev);
int mipicsi_device_hw_init(enum mipicsi_top_dev dev);
int mipicsi_device_vpg(struct mipicsi_top_vpg *vpg);

#endif /* MIPICSI_DEVICE_H_ */
