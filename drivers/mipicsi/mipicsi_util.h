#ifndef MIPICSI_UTIL_H
#define MIPICSI_UTIL_H_

#include "mipicsi_top.h"

bool mipicsi_util_is_emulation(void);
void mipicsi_util_read_emulation(void);
uint16_t mipicsi_util_get_min_bitrate(void);
uint16_t mipicsi_util_get_max_bitrate(void);
void mipicsi_util_save_virt_addr(enum mipicsi_top_dev dev, void *base_addr);

#endif
