#ifndef MIPICSI_UTIL_H
#define MIPICSI_UTIL_H_

#include "mipicsi_top.h"


uint32_t mipicsi_read(enum mipicsi_top_dev dev, uint64_t offset);
void mipicsi_write(enum mipicsi_top_dev dev, uint64_t offset, uint32_t data);
void mipicsi_write_part(enum mipicsi_top_dev dev, uint32_t offset,
			uint32_t data, uint8_t shift, uint8_t width);
void mipicsi_util_save_virt_addr(enum mipicsi_top_dev dev, void *base_addr);

#endif
