
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

#define UPPER(address) ((unsigned int)((address & 0xFFFFFFFF00000000) >> 32))
#define LOWER(address) ((unsigned int)(address & 0x00000000FFFFFFFF))

struct mnh_pcieep_device {
	struct device *dev;
	int irq;
	char name[64];
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

struct mnh_dma_ll_element {
	uint32_t header;
	uint32_t size;
	uint32_t sar_low;
	uint32_t sar_high;
	uint32_t dar_low;
	uint32_t dar_high;
};

#endif
