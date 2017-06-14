
#ifndef MIPICSI_DEVICE_H_
#define MIPICSI_DEVICE_H_

#include "mipicsi_top.h"

#define DIVIDEUP(dividend, divisor) (((dividend) + ((divisor-1)))/(divisor))
#define PAD_PCT 20
#define PAD(time) (((time)*(100+PAD_PCT))/100)
#define TRIM(time) (((time)*(100-PAD_PCT))/100)
#define MAX(a,b) (((a)>(b))?(a):(b))
#define MIN(a,b) (((a)<(b))?(a):(b))

/* Daughtercard constants */
#define TLP_CONST_TIME 23
#define PREP_CONST_TIME 20
#define ZERO_CONST_TIME 80
#define TRAIL_CONST_TIME -30
#define EXIT_CONST_TIME -80 /* TBD */
#define POST_CONST_TIME 160
#define SETL_CONST_TIME 35  /* TBD */
#define MIPI_DDR_CLOCK 1000 /* TBD */

/* Gen3 constants */
/* Analog Timing Factors  - TBD */
#define TCLK_PREP_ATF_NS    (-11)
#define THS_PREP_ATF_NS     (-12)

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
void mipicsi_dev_dphy_write(enum mipicsi_top_dev dev, uint16_t command,
			    uint8_t data);
uint8_t mipicsi_dev_dphy_read(enum mipicsi_top_dev dev, uint16_t command);

#endif /* MIPICSI_DEVICE_H_ */
