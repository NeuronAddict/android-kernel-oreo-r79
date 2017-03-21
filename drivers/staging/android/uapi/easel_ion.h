#ifndef _UAPI_EASEL_ION_H
#define _UAPI_EASEL_ION_H

#include "ion.h"

#define ION_IS_CACHED(__flags)	((__flags) & ION_FLAG_CACHED)

#define ION_IOC_EASEL_MAGIC 'E'

#define ION_IOC_DMA_BUF_FLUSH_CACHE _IOW(ION_IOC_EASEL_MAGIC, 0, int)
#define ION_IOC_DMA_BUF_INVALIDATE_CACHE _IOW(ION_IOC_EASEL_MAGIC, 1, int)

#endif
