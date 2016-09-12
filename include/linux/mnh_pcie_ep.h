
/** @mainpage  MNH PCIE EP(EndPoint) driver
*
* @section intro Introduction
* The PCIE endpoint driver is implemented as a character driver
* allowing the applications to access the entire address range
* exposed thru PCIE.
* Specific functions are exposed through the API's.
*/


/**
 * @file    mnh_pcie_ep.h
 * @brief   PCIe EP driver header includes function API and data structure
 * @author  Intel
 * @date    27 Apr 2016
 * @version 1.0
 */

#include <linux/types.h>
#ifndef __LINUX_MNH_PCIE_EP_H
#define __LINUX_MNH_PCIE_EP_H
#define MNH_PCIE_DEBUG_ENABLE 1
#define MNH_MAX_LL 256
#define MNH_MAX_LL_ELEMENT 64
/* TODO implement to mask sysfs and other code */

/*****************************************************************************
 *
 *  Data structure defined for APIs
 *
 ****************************************************************************/

/** enum value used for msi msg type for mnh_send_msi() API */
enum  mnh_msi_msg_t {
	INVAL_MSI = 0,
	MSG_SEND_M,
	PET_WATCHDOG,
	CRASH_DUMP,
	BOOTSTRAP_SET,
	APPDEFINED_1_M,
};

/** enum value used for irq structure passed IRQ callback function */
enum mnh_irq_msg_t {
	INVAL_IRQ = 0,
	MSG_SEND_I,
	DMA_STATUS,
	APPDEFINED_1_I,
};

/** enum value used for pcie interupts passed to IRQ callback function */
enum mnh_irq_pcie_t {
	INVAL_PCIE = 0,
	MSI_SEND,
	VM_SEND,
	LTR_SEND
};

/** enum value used for pcie interupts passed to IRQ callback function */
enum  mnh_dma_type_t {
	MNH_DMA_READ = 0,
	MNH_DMA_WRITE
};

/** enum value used for pcie interupts passed to IRQ callback function */
enum mnh_dma_status_t {
	MNH_DMA_DONE = 0,
	MNH_DMA_ABORT
};


/** structure used for mnh_send_vm() API */
struct mnh_pcie_vm {
	uint32_t vm;       /**< vendor defined message */
};

/** structure used for IRQ callback function */
struct mnh_pcie_irq {
	enum mnh_irq_msg_t	msi_irq;
				/**< Will be 0 if no IRQ received
				 *from the AP otherwise it holds IRQ type
				 */
	enum mnh_irq_pcie_t	pcie_irq;
				/**< Will be 0 if no IRQ in the pcie controller
				  *otherwise it will hold the IRQ type
				  */
	uint32_t		vm; /**< Will be 0 if no vm was received
				    *otherwise it holds the interrupt
				    */
};

struct mnh_dma_irq {
	uint32_t		channel;
	enum mnh_dma_type_t	type;
	enum mnh_dma_status_t	status;
};

/**
* Structure of scatter gather list entry
*/
struct mnh_sg_entry {
	phys_addr_t paddr; /**< Physical address */
	size_t size;       /**< size of entry */
};

struct mnh_sg_list {
	struct page **mypage;
	struct scatterlist *sc_list;
	int n_num;
};


struct mnh_dma_ll_element {
	uint32_t header;
	uint32_t size;
	uint32_t sar_low;
	uint32_t sar_high;
	uint32_t dar_low;
	uint32_t dar_high;
};

struct mnh_dma_ll {
	uint32_t size;
	struct mnh_dma_ll_element *ll_element[MNH_MAX_LL_ELEMENT];
	dma_addr_t dma[MNH_MAX_LL_ELEMENT];
};


/*******************************************************************************
 *
 *  APIs exposed
 *
 ******************************************************************************/

/** API to generate MSI to AP */
int mnh_send_msi(enum mnh_msi_msg_t msi);

/** API to generate LTR to AP */
/* add more info */
int mnh_send_ltr(uint32_t ltr);

/** API to send Vendor message to AP */
int mnh_send_vm(struct mnh_pcie_vm *vm);

/** API to set PCIE endpoint into L1 */
int mnh_set_l_one(uint32_t enable, uint32_t clkpm);

/** API to register IRQ callback */
int mnh_reg_irq_callback(int (*callback)(struct mnh_pcie_irq *irq),
		int (*dmacallback)(struct mnh_dma_irq *irq));

/** API to program ringbuffer base address to PCIE BOOTSTRAP REGISTER */
int mnh_set_rb_base(uint64_t rb_base);

/** API to read data from AP */
int mnh_pcie_read(uint8_t *buff, uint32_t size, uint64_t adr);

/** API to write data to AP */
int mnh_pcie_write(uint8_t *buff, uint32_t size, uint64_t adr);

/**
* API to build Scatter Gather list to do Multi-block DMA transfer
* @param[in] dmadest  Starting virtual addr of the DMA destination
* @param[in] size     Totalsize of the transfer in bytes
* @param[out] sg   Array of maxsg pointers to struct mnh_sg_entry,
*				allocated by caller and filled out by routine.
* @param[in] maxsg  Allocated max array number of the sg
* @return number of sg[] entries filled out by the routine,
* negative if overflow.
*/
int mnh_sg_build(void *dmadest, size_t size, struct mnh_sg_entry *sg,
				struct mnh_sg_list *sgl, uint32_t maxsg);

int mnh_sg_sync(struct mnh_sg_list *sgl);

int mnh_sg_destroy(struct mnh_sg_list *sgl);


int mnh_ll_build(struct mnh_sg_entry *src_sg, struct mnh_sg_entry *dst_sg,
					struct mnh_dma_ll *ll);

int mnh_ll_destroy(struct mnh_dma_ll *ll);

void *mnh_alloc_coherent(size_t size, dma_addr_t *dma_adr);

void mnh_free_coherent(size_t size, void *cpu_addr, dma_addr_t dma_addr);

#endif
