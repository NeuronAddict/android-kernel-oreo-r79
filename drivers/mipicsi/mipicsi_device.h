
#ifndef MIPICSI_DEVICE_H_
#define MIPICSI_DEVICE_H_

#include "mipicsi_top.h"

void mipicsi_dev_dphy_write(enum mipicsi_top_dev dev,
			    uint8_t command, uint8_t data);
void mipicsi_device_reset(enum mipicsi_top_dev dev);
void mipicsi_device_dphy_reset(enum mipicsi_top_dev dev);
int mipicsi_device_start(struct mipicsi_top_cfg *config);
int mipicsi_device_stop(enum mipicsi_top_dev dev);
int mipicsi_device_hw_init(enum mipicsi_top_dev dev);
int mipicsi_device_vpg(struct mipicsi_top_vpg *vpg);
int mipicsi_device_get_interrupt_status(enum mipicsi_top_dev devid,
					struct mipi_device_irq_st *status);
int mipicsi_device_set_interrupt_mask(enum mipicsi_top_dev devid,
				      struct mipi_device_irq_mask *mask);
int mipicsi_device_force_interrupt(enum mipicsi_top_dev devid,
				   struct mipi_device_irq_mask *mask);

#endif /* MIPICSI_DEVICE_H_ */
