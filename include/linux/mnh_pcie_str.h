
/** @mainpage  MNH PCIE EP(EndPoint) driver
*
* @section intro Introduction
* The PCIE endpoint driver is implemented as a character driver
* allowing the applications to access the entire address range
* exposed thru PCIE.
* Specific functions are exposed through the API's.
*/


/**
 * @file    mnh_pcie_str.h
 * @brief   PCIe EP driver header includes data structures & definitions
 * @author  Intel
 * @date    27 Apr 2016
 * @version 1.0
 */

#include <linux/types.h>
#include <linux/mnh_pcie_reg.h>
#ifndef __LINUX_MNH_PCIE_STR_H
#define __LINUX_MNH_PCIE_STR_H

#define VM_BYTE_8_9		0x0000

/* BAR 2&3 and BAR 4&5 are combined for 64bit addr */

#define BAR_2_3			2
#define BAR_4_5			4
#define BAR_2_BASE		0x000000000000000
#define BAR_4_BASE		0x000000040000000
#define REGION_0		0
#define REGION_1		1
#define REGION_2		2

#define DMA_LL_LENGTH			10 /*TODO Needs to be optimized */
#define LL_DATA_ELEMENT			0x1
#define LL_IRQ_DATA_ELEMENT		0x9
#define LL_LINK_ELEMENT			0x4
#define LL_LAST_LINK_ELEMENT	0x6

#define PCIE_SS_IRQ_MASK	0xF701FF

#define DMA_READ_DONE_MASK	0xFF		/* 7:0 */
#define DMA_READ_ABORT_MASK	0xFF00		/* 15:8 */
#define DMA_WRITE_DONE_MASK	0xFF0000	/* 16:23 */
#define DMA_WRITE_ABORT_MASK	0xFF000000	/* 24:31 */


#define UPPER(address) ((uint32_t)((address & 0xFFFFFFFF00000000) >> 32))
#define LOWER(address) ((uint32_t)(address & 0x00000000FFFFFFFF))

#define B8M_BAR			0x7FFFFF
#define B4M_BAR			0x3FFFFF


/* definitions to test Scatter gather API */

#define SGL_BUFFER 256
#define SGL_SIZE 64

struct mnh_pcie_ep_device {
	struct device *dev;
	char name[64];
	uint32_t sw_irq;
	uint32_t cluster_irq;
	uint32_t pendingmsi;
	uint32_t msimode;
	void *conf_mem;
	void *clust_mem;
	void *outb_mem;
	struct resource *config_mem;
	struct resource *cluster_mem;
	struct resource *outbound_mem;
};


/*****************************************************************************
 *
 *  Data structures for iATU programming
 *
 ****************************************************************************/

/** enum value used for mode setting of mnh_set_inbound() API */
typedef enum {
	ADDR_MATCH = 0,
	BAR_MATCH
} mnh_iatu_mode_t;

/** structure used for mnh_set_inbound() API */
struct mnh_inb_window {
	uint8_t mode;				 /**< type of region */
	uint32_t bar;				 /**< BAR ignored in addr mode */
	uint8_t region;              /**< iATU region to be programmed */
	uint8_t memmode;
	uint64_t base_pcie_address;  /**< start of src buffer
								 *ignored in BAR mode
								 */
	uint32_t limit_pcie_address; /**< end of src buffer
								 *ignored in BAR mode
								 */
	uint64_t target_mnh_address; /**< dest address */
};

/** structure used for mnh_set_outbound() API */
struct mnh_outb_region {
	uint8_t region;                /**< iATU region to be programmed */
	uint64_t base_mnh_address;     /**< start of src buffer */
	uint32_t limit_mnh_address;    /**< end of src buffer */
	uint64_t target_pcie_address;  /**< dest address */
};



#endif
