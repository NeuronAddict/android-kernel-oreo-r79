#ifndef MIPICSI_DEBUG_H_
#define MIPICSI_DEBUG_H_

#include "mipicsi_top.h"

enum mipicsi_debug_vpg_res {
	VPG_VGA,
	VPG_1080P,
	VPG_12MP
};

int mipicsi_debug_vpg_preset(struct mipicsi_top_vpg *vpg,
			     enum mipicsi_debug_vpg_res res);

int mipicsi_debug_dump(enum mipicsi_top_dev device);
#endif
