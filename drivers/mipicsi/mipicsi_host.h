#ifndef MIPICSI_HOST_H_
#define MIPICSI_HOST_H_

#include <linux/types.h>
#include "mipicsi_top.h"


void mipicsi_host_dphy_reset(enum mipicsi_top_dev dev);
void mipicsi_host_reset(enum mipicsi_top_dev dev);
int  mipicsi_host_start(struct mipicsi_top_cfg *config);
int mipicsi_host_stop(enum mipicsi_top_dev dev);
int mipicsi_host_hw_init(enum mipicsi_top_dev dev);

int mipicsi_host_get_interrupt_status(enum mipicsi_top_dev devid,
				      struct mipi_host_irq_st *status);
int mipicsi_host_set_interrupt_mask(enum mipicsi_top_dev devid,
				      struct mipi_host_irq_mask *mask);
int mipicsi_host_force_interrupt(enum mipicsi_top_dev devid,
				 struct mipi_host_irq_mask *mask);
#endif
