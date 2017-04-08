#ifndef _UAPI_EASEL_ION_H
#define _UAPI_EASEL_ION_H

#include "ion.h"

#define ION_IS_CACHED(__flags)	((__flags) & ION_FLAG_CACHED)

#define ION_IOC_EASEL_MAGIC 'E'

#define ION_IOC_DMA_BUF_FLUSH_CACHE _IOW(ION_IOC_EASEL_MAGIC, 0, \
		struct ion_fd_data)
#define ION_IOC_DMA_BUF_INVALIDATE_CACHE _IOW(ION_IOC_EASEL_MAGIC, 1, \
		struct ion_fd_data)
#define ION_IOC_DMA_BUF_FLUSH_INVALIDATE_CACHE _IOW(ION_IOC_EASEL_MAGIC, 2, \
		struct ion_fd_data)
#define ION_IOC_HANDLE_FLUSH_CACHE _IOW(ION_IOC_EASEL_MAGIC, 3, \
		struct ion_fd_data)
#define ION_IOC_HANDLE_INVALIDATE_CACHE _IOW(ION_IOC_EASEL_MAGIC, 4, \
		struct ion_fd_data)
#define ION_IOC_HANDLE_FLUSH_INVALIDATE_CACHE _IOW(ION_IOC_EASEL_MAGIC, 5, \
		struct ion_fd_data)

#endif
