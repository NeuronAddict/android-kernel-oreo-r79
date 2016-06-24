
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

struct mipicsi_pll {
	uint32_t  output_freq;          /* Active output frequency */
	uint32_t  input_freq;           /* Active input frequency */
	uint32_t  ref_freq;             /* Reference Frequency */
	uint32_t  hsfreq;               /* hsfreq */
	uint8_t   vco_range;            /* vcorange */
	uint8_t   cp_current;           /* icpctrl */
	uint8_t   lpf_resistor;         /* lpfctrl */
	uint32_t  loop_div;             /* (M) */
	uint32_t  input_div;            /* (N) */
	uint8_t	  output_div;           /* (P) */
};

void mipicsi_device_reset(enum mipicsi_top_dev dev);
void mipicsi_device_dphy_reset(enum mipicsi_top_dev dev);
int mipicsi_device_start(struct mipicsi_top_cfg *config);
int mipicsi_device_hw_init(enum mipicsi_top_dev dev);
int mipicsi_device_vpg(struct mipicsi_top_vpg *vpg);

#endif /* MIPICSI_DEVICE_H_ */
