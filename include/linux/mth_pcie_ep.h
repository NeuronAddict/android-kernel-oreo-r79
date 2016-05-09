
/** @mainpage  MTH PCIE EP(EndPoint) driver
*
* @section intro Introduction
* The PCIE endpoint driver is implemented as a character driver
* allowing the applications to access the entire address range
* exposed thru PCIE.
* Specific functions are exposed through the API's.
*/


/**
 * @file    mth_pcie_ep.h
 * @brief   PCIe EP driver header includes function API and data structure
 * @author  Intel
 * @date    27 Apr 2016
 * @version 1.0
 */

#include <linux/types.h>
#ifndef __LINUX_MTH_PCIE_EP_H
#define __LINUX_MTH_PCIE_EP_H
#define MTH_PCIE_CONFIG_VENDOR_API_SUPPORT 1
/* TODO implement to mask sysfs and other code */

/*****************************************************************************
 *
 *  Data structure defined for APIs
 *
 ****************************************************************************/

/** enum value used for msi msg type for mth_send_msi() API */
typedef enum {
	INVAL_MSI = 0,
	MSG_SEND_M,
	PET_WATCHDOG,
	CRASH_DUMP,
	BOOTSTRAP_SET
} mth_msi_msg_t;

/** enum value used for irq structure passed IRQ callback function */
typedef enum {
	INVAL_IRQ = 0,
	MSG_SEND_I,
	DMA_DONE,
} mth_irq_msg_t;

/** enum value used for pcie interupts passed to IRQ callback function */
typedef enum {
	INVAL_PCIE = 0,
	MSI_SEND,
	VM_SEND,
	LTR_SEND
} mth_irq_pcie_t;

/** structure used for mth_send_vm() API */
struct mth_pcie_vm {
	uint32_t vm;       /**< vendor defined message */
};

/** structure used for IRQ callback function */
struct mth_pcie_irq {
	mth_irq_msg_t	msi_irq; /**< Will be 0 if no IRQ received
				 *from the AP otherwise it holds IRQ type
				 */
	mth_irq_pcie_t	pcie_irq; /**< Will be 0 if no IRQ in the pcie controller
				  *otherwise it will hold the IRQ type
				  */
	uint32_t		vm; /**< Will be 0 if no vm was received
				    *otherwise it holds the interrupt
				    */
};

struct mth_dma_irq {
	/*TODO populate deal with ripple effect */
};

/**
* Structure of scatter gather list entry
*/
struct mth_sg_entry {
	phys_addr_t paddr; /**< Physical address */
	size_t size;       /**< size of entry */
};

/*******************************************************************************
 *
 *  APIs exposed
 *
 ******************************************************************************/

/** API to generate MSI to AP */
int mth_send_msi(mth_msi_msg_t msi);

/** API to generate LTR to AP */
/* add more info */
int mth_send_ltr(uint32_t ltr);

/** API to send Vendor message to AP */
int mth_send_vm(struct mth_pcie_vm *vm);

/** API to set PCIE endpoint into L1 */
int mth_set_l_one(uint32_t enable, uint32_t clkpm);

/** API to register IRQ callback */
int mth_reg_irq_callback(int (*callback)(struct mth_pcie_irq *irq),
		int (*dmacallback)(struct mth_dma_irq *irq));

/** API to program ringbuffer base address to PCIE BOOTSTRAP REGISTER */
int mth_set_rb_base(uint64_t rb_base);

/** API to read data from AP */
int mth_pcie_read(uint8_t *buff, uint32_t size, uint64_t adr);

/** API to write data to AP */
int mth_pcie_write(uint8_t *buff, uint32_t size, uint64_t adr);

/**
* API to build Scatter Gather list to do Multi-block DMA transfer
* @param[in] dmadest  Starting virtual addr of the DMA destination
* @param[in] size     Totalsize of the transfer in bytes
* @param[out] sg   Array of maxsg pointers to struct mth_sg_entry,
*				allocated by caller and filled out by routine.
* @param[in] maxsg  Allocated max array number of the sg
* @return number of sg[] entries filled out by the routine,
* negative if overflow.
*/
int mth_sg_build(void *dmadest, size_t size, struct mth_sg_entry *sg[],
					uint32_t maxsg);


int mth_ll_build(struct mth_sg_entry *src_sg[], struct mth_sg_entry *dst_sg[],
					uint64_t *start_addr);

int mth_ll_destroy(uint64_t *start_addr);

#endif
