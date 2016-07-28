/*
 * pcie_ep.c - Monhette Hill PCIe EndPoint driver
 *
 * Copyright (C) 2016 Intel Corporation
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program;
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Author: Marko Bartscherer <marko.bartscherer@intel.com>
 */


#include <linux/init.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/irqreturn.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/mnh_pcie_ep.h>
#include <linux/mnh_pcie_reg.h>
#include <linux/mnh_pcie_str.h>
#include <linux/mnh_dma_adr.h>
/* #include <asm-generic/page.h> */
#include <linux/mm.h>
#include <linux/rwsem.h>
#include <linux/scatterlist.h>
#include <linux/dma-mapping.h>
#include <linux/pci.h>
#include <linux/intel-hwio.h>
#include <soc/mnh/mnh-hwio-pcie-ep.h>
#include <soc/mnh/mnh-hwio-pcie-ss.h>
#include <linux/pagemap.h>



#define VENDOR_ID				0x8086
#define DEVICE_ID				0x3140

#define COMBINE_SG

int (*irq_callback)(struct mnh_pcie_irq *irq);
int (*dma_callback)(struct mnh_dma_irq *irq);
struct mnh_pcie_ep_device *pcie_ep_dev;
struct delayed_work msi_work;
struct work_struct msi_rx_work, pcie_irq_work;
#define DEVICE_NAME "mnh_pcie_ep"
#define CLASS_NAME "pcie_ep"
#define MSI_DELAY (HZ/20) /* TODO: Need to understand what this should be */
#define MAX_STR_COPY	32
uint32_t rw_address_sysfs, rw_size_sysfs;
struct mnh_pcie_irq sysfs_irq;

static int pcie_set_inbound_iatu(struct mnh_inb_window *inb);
static int pcie_clear_msi_mask(void);
static int pcie_ll_destroy(phys_addr_t *start_addr);

/* read from pcie cluster register */
static uint32_t pcie_cluster_read(uint64_t address)
{
	uint32_t data;
	uint64_t addr = (uint64_t)pcie_ep_dev->clust_mem + address;

	data = ioread32((uint32_t *) addr);
	return data;
}

/* write to pcie cluster register */

static int pcie_cluster_write(uint64_t address, uint32_t data)
{
	uint64_t addr = (uint64_t)pcie_ep_dev->clust_mem + address;

	iowrite32(data, (uint32_t *) addr);
	return 0;
}

/* read from pcie config register */

static uint32_t pcie_config_read(uint64_t address)
{
	uint32_t data;
	uint64_t addr = (uint64_t)pcie_ep_dev->conf_mem + address;

	data = ioread32((uint32_t *) addr);
	return data;
}

/* write to pcie config register */

static int pcie_config_write(uint64_t address, uint32_t data)
{
	uint64_t addr = (uint64_t)pcie_ep_dev->conf_mem + address;

	iowrite32(data, (uint32_t *) addr);
	return 0;
}

static int check_sram_init_done(void)
{

	return CSR_INf(PCIE_APP_STS, PCIE_PHY_SRAM_INIT_DONE);
}

static int pcie_link_up(void)
{
	uint32_t data, mask;

	mask = TYPE0_MASK(TYPE0_HDR_STATUS_COMMAND, PCI_TYPE0_IO_EN) |
		TYPE0_MASK(TYPE0_HDR_STATUS_COMMAND, PCI_TYPE0_MEM_SPACE_EN) |
		TYPE0_MASK(TYPE0_HDR_STATUS_COMMAND, PCI_TYPE0_BUS_MASTER_EN);
	data = TYPE0_IN(TYPE0_HDR_STATUS_COMMAND) & mask;
	if (data == 0)
		return 0;
	return 1;
}

static int pcie_link_init(void)
{
	struct mnh_inb_window iatu;

	if (!pcie_link_up()) {
		/* Magic value Only used during boot */
		CSR_OUTx(PCIE_GP, 0, 0x0);
		/* program PCIe controller registers
		* not needed at this point in time
		*/

		/* TODO: read eFuses to update device address */
		while (!check_sram_init_done())
			udelay(1);

		/* program PCIe PHY PCS PATCH registers
		*not needed at this point in time
		*/
		CSR_OUTf(PCIE_APP_CTRL, PCIE_PHY_SRAM_LD_DONE, 1);

		/* program PCIe PHY registers not needed
		* at this point in time
		*/
		/* Set BAR sizes */

		TYPE0S_OUT(TYPE0_HDR_BAR2, B8M_BAR);
		TYPE0S_OUT(TYPE0_HDR_BAR4, B4M_BAR);

		CSR_OUTf(PCIE_APP_CTRL, PCIE_APP_LTSSM_EN, 1);
		CSR_OUTf(PCIE_APP_CTRL, PCIE_APP_XFER_PEND, 1);

		/* waiting for magic value */
		//while (CSR_INx(PCIE_GP, 0) == 0)
		//	udelay(1);

		CSR_OUTf(PCIE_APP_CTRL, PCIE_APP_XFER_PEND, 0);

	};

	/* Enable interupts */
	CSR_OUT(PCIE_SS_INTR_EN, PCIE_SS_IRQ_MASK);
	pcie_clear_msi_mask();

	return  CSR_INx(PCIE_GP, 0);
}

static void force_link_up(void)
{
	if (CSR_INf(PCIE_APP_CTRL, PCIE_APP_REQ_EXIT_L1))
		CSR_OUTf(PCIE_APP_CTRL, PCIE_APP_REQ_EXIT_L1, 0);
	CSR_OUTf(PCIE_APP_CTRL, PCIE_APP_REQ_EXIT_L1, 1);
}

static void release_link(void)
{
	CSR_OUTf(PCIE_APP_CTRL, PCIE_APP_REQ_EXIT_L1, 0);
}

static int pcie_send_msi_p(uint32_t msi)
{
	uint32_t tc, data, msg;

	data = CSR_IN(PCIE_MSI_TRIG) &
		CSR_MASK(PCIE_MSI_TRIG, PCIE_VEN_MSI_REQ);
	tc = MNH_TC0 << 13;
	msg = msi << 8;
	data |= tc | msg;
	CSR_OUT(PCIE_MSI_TRIG, data);
	CSR_OUTf(PCIE_MSI_TRIG, PCIE_VEN_MSI_REQ, 1);
	return 0;
}


static int pcie_get_msi_mask(void)
{
	uint32_t data, mask;

	data = MSICAP_INf(MSI_CAP_PCI_MSI_CAP_ID_NEXT_CTRL,
			PCI_MSI_64_BIT_ADDR_CAP);
	if (data) {
	/* 64 bit ADDR used */
		mask = MSICAP_IN(MSI_CAP_MSI_CAP_OFF_10H);
	} else {
	/* 32 bit ADDR used */
		mask = MSICAP_IN(MSI_CAP_MSI_CAP_OFF_0CH);
	}

	return mask;
}

static int pcie_clear_msi_mask(void)
{
	uint32_t data;

	data = MSICAP_INf(MSI_CAP_PCI_MSI_CAP_ID_NEXT_CTRL,
			PCI_MSI_64_BIT_ADDR_CAP);
	if (data) {
	/* 64 bit ADDR used */
		MSICAP_OUT(MSI_CAP_MSI_CAP_OFF_10H, 0);
	} else {
	/* 32 bit ADDR used */
		MSICAP_OUT(MSI_CAP_MSI_CAP_OFF_0CH, 0);
	}

	return 0;
}

static int pcie_check_msi_mask(uint32_t msi)
{
	uint32_t vmask, mask;

	vmask = 0x1 << (msi-1);
	mask = pcie_get_msi_mask();
	if (mask & vmask) {
		/* need to set pending bit, than start polling */
		CSR_OUT(PCIE_MSI_PEND, CSR_IN(PCIE_MSI_PEND) | mask);
		cancel_delayed_work_sync(&msi_work);
		schedule_delayed_work(&msi_work, MSI_DELAY);
		return 1;
	}
	return 0;
}

static uint32_t b2h(uint32_t arg)
{
	/* Converts a hex number to a bit 3 -> 0100 */
	if (arg > 0 && (arg < 32))
		return (0x1 << (arg - 1));
	return 0;
}

static void process_pend_msi(uint32_t pend, uint32_t vmask, uint32_t arg)
{
	if ((pend & b2h(arg)) &&
		!(vmask & b2h(arg))) {
		pcie_send_msi_p(arg);
		CSR_OUT(PCIE_MSI_PEND, (pend & ~b2h(arg)));
	}
}

static void send_pending_msi(void)
{
	uint32_t vmask, pend;

	vmask = pcie_get_msi_mask();
	pend = pcie_cluster_read(MNH_PCIE_MSI_PEND_REG);
	process_pend_msi(pend, vmask, MSG_SEND_M);
	process_pend_msi(pend, vmask, PET_WATCHDOG);
	process_pend_msi(pend, vmask, CRASH_DUMP);
	process_pend_msi(pend, vmask, BOOTSTRAP_SET);
}

static void pcie_msi_worker(struct work_struct *work)
{
	uint32_t pend;

	force_link_up();
	send_pending_msi();
	pend = CSR_IN(PCIE_MSI_PEND);
	if (pend) {
		cancel_delayed_work_sync(&msi_work);
		schedule_delayed_work(&msi_work, MSI_DELAY);
	}

}

static int pcie_send_msi(uint32_t msi)
{
	uint32_t pend;

	if ((msi < 1) || (msi > 32))
		return -EINVAL;
	force_link_up();
	if (pcie_check_msi_mask(msi))
		return -EINVAL; /* MSI is masked */
	pend = CSR_IN(PCIE_MSI_PEND);
	if (pend)
		send_pending_msi();
	pcie_send_msi_p(msi);
	return 0;
}

static int pcie_send_vm(uint32_t vm)
{
	uint32_t data;

	force_link_up();
	data = (VENDOR_ID << 16) | VM_BYTE_8_9;
	CSR_OUT(PCIE_TX_MSG_PYLD_REG1, data);
	HW_OUT(pcie_ep_dev->clust_mem,  PCIE_SS, PCIE_TX_MSG_PYLD_REG1, data);
	CSR_OUT(PCIE_TX_MSG_PYLD_REG0, vm);
	HW_OUT(pcie_ep_dev->clust_mem,  PCIE_SS, PCIE_TX_MSG_PYLD_REG0, vm);
	CSR_OUTf(PCIE_TX_MSG_REQ, PCIE_TX_MSG_TAG, MNH_PCIE_VM_TAG);
	CSR_OUTf(PCIE_TX_MSG_REQ, PCIE_TX_MSG_CODE, MNH_PCIE_VM_MSG_CODE_VD);
	CSR_OUTf(PCIE_TX_MSG_REQ, PCIE_TX_MSG_REQ, 1);
	/*
	*data = (MNH_PCIE_VM_TAG << 24) | (MNH_PCIE_VM_MSG_CODE_VD << 16)
	*	| MNH_PCIE_TX_MSG_REQ;
	*pcie_cluster_write(MNH_PCIE_TX_MSG_REQ_REG, data);
	*/
	return 0;
}

static int pcie_send_ltr(uint32_t ltr)
{

	force_link_up();
	CSR_OUT(PCIE_LTR_MSG_LATENCY, ltr);
	CSR_OUTf(PCIE_LTR_MSG_CTRL, PCIE_LTR_MSG_REQ, 1);
	return 0;
}

/* Interrupt Service routines */

static int handle_vm(int vm)
{
	struct mnh_pcie_irq inc;

	inc.msi_irq = 0;
	inc.pcie_irq = 0;
	inc.vm = 0;

	if (vm == 0) {
		if (CSR_INf(PCIE_RX_VMSG0_ID, PCIE_RADM_MSG0_REQ_ID)
				& MNH_VALID_VM) {
			vm = CSR_IN(PCIE_RX_MSG0_PYLD_REG0);
			inc.vm = vm;
			dev_err(pcie_ep_dev->dev, "Vendor msg %x rcvd\n", vm);
			irq_callback(&inc);
			CSR_OUTf(PCIE_RX_VMSG0_ID, PCIE_RADM_MSG0_REQ_ID,
				MNH_CLEAR_VM);
		}
	} else {
		if (CSR_INf(PCIE_RX_VMSG1_ID, PCIE_RADM_MSG1_REQ_ID)
				& MNH_VALID_VM) {
			vm = CSR_IN(PCIE_RX_MSG0_PYLD_REG1);
			inc.vm = vm;
			dev_err(pcie_ep_dev->dev, "Vendor msg %x rcvd\n", vm);
			irq_callback(&inc);
			CSR_OUTf(PCIE_RX_VMSG1_ID, PCIE_RADM_MSG1_REQ_ID,
				MNH_CLEAR_VM);
		}
	}
	return 0;
}

static void dma_rx_handler(void)
{
	uint32_t data, i;
	struct mnh_dma_irq dma_event;

	if (dma_callback != NULL) {
		data = CSR_INx(PCIE_GP, 2);
		if (data & DMA_READ_DONE_MASK) {
			dma_event.type = MNH_DMA_READ;
			dma_event.status = MNH_DMA_DONE;
			data = data & DMA_READ_DONE_MASK;
			i = 0;
			while ((data != 1) && (i < 8)) {
				data = data >> 1;
				i++;
				};
			dma_event.channel = i;
		} else if (data & DMA_READ_ABORT_MASK) {
			dma_event.type = MNH_DMA_READ;
			dma_event.status = MNH_DMA_ABORT;
			data = (data & DMA_READ_ABORT_MASK) >> 8;
			i = 0;
			while ((data != 1) && (i < 8)) {
				data = data >> 1;
				i++;
				};
			dma_event.channel = i;
		} else if (data & DMA_WRITE_DONE_MASK) {
			dma_event.type = MNH_DMA_WRITE;
			dma_event.status = MNH_DMA_DONE;
			data = (data & DMA_WRITE_DONE_MASK) >> 16;
			i = 0;
			while ((data != 1) && (i < 8)) {
				data = data >> 1;
				i++;
				};
			dma_event.channel = i;
		} else if (data & DMA_WRITE_ABORT_MASK) {
			dma_event.type = MNH_DMA_WRITE;
			dma_event.status = MNH_DMA_ABORT;
			data = (data & DMA_READ_ABORT_MASK) >> 24;
			i = 0;
			while ((data != 1) && (i < 8)) {
				data = data >> 1;
				i++;
				};
			dma_event.channel = i;
		}
		CSR_OUTx(PCIE_GP, 2, 0x0);
		dma_callback(&dma_event);
	}
}
static void msi_rx_worker(struct work_struct *work)
{
	uint32_t apirq;
	struct mnh_pcie_irq inc;

	inc.msi_irq = 0;
	inc.pcie_irq = 0;
	inc.vm = 0;
	
	dev_err(pcie_ep_dev->dev, "AP IRQ routine called\n");
	apirq = CSR_IN(PCIE_SW_INTR_TRIGG);
	if (apirq != 0) {
		dev_err(pcie_ep_dev->dev, "AP IRQ %x received\n", apirq);

		if (apirq & (0x1 << MSG_SEND_I)) {
			if (irq_callback != NULL) {
				inc.msi_irq =  MSG_SEND_I;
				irq_callback(&inc);
			}
		}
		if (apirq & (0x1 << DMA_STATUS))
			dma_rx_handler();

		/* Clear all interrupts */
		CSR_OUT(PCIE_SW_INTR_TRIGG, MNH_PCIE_SW_IRQ_CLEAR);
	}
}

static void pcie_irq_worker(struct work_struct *work)
{
	uint32_t pcieirq;

	dev_err(pcie_ep_dev->dev, "PCIE IRQ routine called\n");
	pcieirq = CSR_IN(PCIE_SS_INTR_STS);
	if (pcieirq != 0) {
		if (pcieirq & MNH_PCIE_MSI_SENT) {
			dev_err(pcie_ep_dev->dev, "MSI Sent\n");
			release_link();
		}
		if (pcieirq & MNH_PCIE_VMSG_SENT) {
			dev_err(pcie_ep_dev->dev, "VM Sent\n");
			release_link();
		}
		if ((pcieirq & MNH_PCIE_VMSG1_RXD)
			& (irq_callback != NULL)) {
			dev_err(pcie_ep_dev->dev, "VM1 received\n");
			handle_vm(1);
		}
		if ((pcieirq & MNH_PCIE_VMSG0_RXD)
			& (irq_callback != NULL)) {
			dev_err(pcie_ep_dev->dev, "VM2 received\n");
			handle_vm(0);
		}
		if (pcieirq & MNH_PCIE_LINK_EQ_REQ_INT)
			dev_err(pcie_ep_dev->dev,
			"MNH_PCIE_LINK_EQ_REQ_INT received\n");
		if (pcieirq & MNH_PCIE_LINK_REQ_RST_NOT)
			dev_err(pcie_ep_dev->dev,
			"MNH_PCIE_LINK_REQ_RST_NOT received\n");
		if (pcieirq & MNH_PCIE_LTR_SENT)
			release_link();
		if (pcieirq & MNH_PCIE_COR_ERR)
			dev_err(pcie_ep_dev->dev,
			"MNH_PCIE_COR_ERR received\n");
		if (pcieirq & MNH_PCIE_NONFATAL_ERR)
			dev_err(pcie_ep_dev->dev,
			"MNH_PCIE_NONFATAL_ERR received\n");
		if (pcieirq & MNH_PCIE_FATAL_ERR)
			dev_err(pcie_ep_dev->dev,
			"MNH_PCIE_FATAL_ERR received\n");
		if (pcieirq & MNH_PCIE_RADM_MSG_UNLOCK)
			dev_err(pcie_ep_dev->dev,
			"MNH_PCIE_RADM_MSG_UNLOCK received\n");
		if (pcieirq & MNH_PCIE_PM_TURNOFF)
			dev_err(pcie_ep_dev->dev,
			"MNH_PCIE_PM_TURNOFF received\n");
		if (pcieirq & MNH_PCIE_RADM_CPL_TIMEOUT)
			dev_err(pcie_ep_dev->dev,
			"MNH_PCIE_RADM_CPL_TIMEOUT received\n");
		if (pcieirq & MNH_PCIE_TRGT_CPL_TIMEOUT)
			dev_err(pcie_ep_dev->dev,
			"MNH_PCIE_TRGT_CPL_TIMEOUT received\n");
	}

	/* Clear all interrupts */
	CSR_OUT(PCIE_SS_INTR_STS, PCIE_SS_IRQ_MASK);

}

static irqreturn_t pcie_handle_cluster_irq(int irq, void *dev_id)
{
	schedule_work(&pcie_irq_work);

	/* return interrupt handled */
	return IRQ_HANDLED;
}

static irqreturn_t pcie_handle_sw_irq(int irq, void *dev_id)
{
	schedule_work(&msi_rx_work);

	/* return interrupt handled */
	return IRQ_HANDLED;
}

static int pcie_set_inbound_iatu(struct mnh_inb_window *inb)
{
	uint32_t data, upper, lower;

	if (inb->mode == BAR_MATCH) {
		if ((inb->region > 0xF) || (inb->bar > 5))
			return -EINVAL;
		data = PORT_MASK(PORT_LOGIC_IATU_VIEWPORT_OFF, REGION_DIR)
			| inb->region;
		PORT_OUT(PORT_LOGIC_IATU_VIEWPORT_OFF, data);
		upper = UPPER(inb->target_mnh_address);
		lower = LOWER(inb->target_mnh_address);
		PORT_OUT(PORT_LOGIC_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_0,
			lower);
		PORT_OUT(PORT_LOGIC_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_0,
			upper);
		PORT_OUTf(PORT_LOGIC_IATU_REGION_CTRL_1_OFF_OUTBOUND_0, TYPE,
			MNH_IATU_MEM);
		data = PORT_MASK(PORT_LOGIC_IATU_REGION_CTRL_2_OFF_OUTBOUND_0,
			REGION_EN) |
			PORT_MASK(PORT_LOGIC_IATU_REGION_CTRL_2_OFF_OUTBOUND_0,
			RSVDP_30) | (inb->bar << 8);
		PORT_OUT(PORT_LOGIC_IATU_REGION_CTRL_2_OFF_OUTBOUND_0, data);
		while (PORT_IN(PORT_LOGIC_IATU_REGION_CTRL_2_OFF_OUTBOUND_0)
			!= data)
			udelay(1);
	} else {
		if ((inb->region > 0xF) ||
				(inb->target_mnh_address >
				MNH_PCIE_OUTBOUND_BASE) ||
				(inb->limit_pcie_address >
				MNH_PCIE_OUTBOUND_BASE) ||
				(inb->memmode > 0xf))
			return -EINVAL; /* address out of range */
		data = PORT_MASK(PORT_LOGIC_IATU_VIEWPORT_OFF, REGION_DIR)
			| inb->region;
		PORT_OUT(PORT_LOGIC_IATU_VIEWPORT_OFF, data);
		upper = UPPER(inb->base_pcie_address);
		lower = LOWER(inb->base_pcie_address);
		PORT_OUT(PORT_LOGIC_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_0,
			lower);
		PORT_OUT(PORT_LOGIC_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_0,
			upper);
		PORT_OUT(PORT_LOGIC_IATU_LIMIT_ADDR_OFF_OUTBOUND_0,
			inb->limit_pcie_address);
		upper = UPPER(inb->target_mnh_address);
		lower = LOWER(inb->target_mnh_address);
		PORT_OUT(PORT_LOGIC_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_0,
			lower);
		PORT_OUT(PORT_LOGIC_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_0,
			upper);
		PORT_OUTf(PORT_LOGIC_IATU_REGION_CTRL_1_OFF_OUTBOUND_0, TYPE,
			inb->memmode);
		PORT_OUTf(PORT_LOGIC_IATU_REGION_CTRL_2_OFF_OUTBOUND_0,
				REGION_EN, 1);
		while (PORT_INf(PORT_LOGIC_IATU_REGION_CTRL_2_OFF_OUTBOUND_0,
			REGION_EN)
				!= 1)
			udelay(1);
	}
	return 0;
}

static int pcie_set_outbound_iatu(struct mnh_outb_region *outb)
{
	uint32_t data, upper, lower;

	if ((outb->region > 0xF) || (outb->base_mnh_address
			> MNH_PCIE_OUTBOUND_BASE) ||
			(outb->limit_mnh_address
			> MNH_PCIE_OUTBOUND_BASE))
		return -EINVAL; /* address out of range */
	data = MNH_IATU_OUTBOUND | outb->region;
	PORT_OUT(PORT_LOGIC_IATU_VIEWPORT_OFF, data);
	upper = UPPER(outb->base_mnh_address);
	lower = LOWER(outb->base_mnh_address);
	PORT_OUT(PORT_LOGIC_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_0,
			lower);
		PORT_OUT(PORT_LOGIC_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_0,
			upper);
		PORT_OUT(PORT_LOGIC_IATU_LIMIT_ADDR_OFF_OUTBOUND_0,
			outb->limit_mnh_address);
	data = outb->limit_mnh_address;
	upper = UPPER(outb->target_pcie_address);
	lower = LOWER(outb->target_pcie_address);
	PORT_OUT(PORT_LOGIC_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_0,
			lower);
		PORT_OUT(PORT_LOGIC_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_0,
			upper);
	PORT_OUTf(PORT_LOGIC_IATU_REGION_CTRL_1_OFF_OUTBOUND_0, TYPE,
			MNH_IATU_MEM);
	PORT_OUTf(PORT_LOGIC_IATU_REGION_CTRL_2_OFF_OUTBOUND_0, REGION_EN, 1);
		while (PORT_INf(PORT_LOGIC_IATU_REGION_CTRL_2_OFF_OUTBOUND_0,
			REGION_EN)
				!= 1)
			udelay(1);
	return 0;
}

static int pcie_set_rb_base(uint32_t base)
{
	uint32_t msi = BOOTSTRAP_SET;

	CSR_OUTx(PCIE_GP, 1, base);
	pcie_send_msi(msi);
	return 0;
}

static int pcie_read_data(uint8_t *buff, uint32_t size, uint64_t adr)
{

	uint64_t addr = (uint64_t)pcie_ep_dev->outb_mem + adr;

	memcpy(buff, (char *) addr, size);
	dev_err(pcie_ep_dev->dev, "finished copy\n");
	return 0;
}

static int pcie_write_data(uint8_t *buff, uint32_t size, uint64_t adr)
{

	uint64_t addr = (uint64_t)pcie_ep_dev->outb_mem + adr;

	memcpy((char *) addr, buff, size);
	dev_err(pcie_ep_dev->dev, "finished copy\n");
	return 0;
}

static int pcie_set_l_one(uint32_t enable, uint32_t clkpm)
{

	if (clkpm == 1) {
		CSR_OUTf(PCIE_APP_CTRL, PCIE_APP_CLK_PM_EN, 1);
	} else {
		CSR_OUTf(PCIE_APP_CTRL, PCIE_APP_CLK_PM_EN, 0);
	}
	if (enable == 1) {
		CSR_OUTf(PCIE_APP_CTRL, PCIE_APP_REQ_EXIT_L1, 1);
	} else {
		CSR_OUTf(PCIE_APP_CTRL, PCIE_APP_REQ_EXIT_L1, 0);
	}

	return 0;
}

int pcie_sg_build(void *dmadest, size_t size, struct mnh_sg_entry *sg,
						uint32_t maxsg)
{
	struct page **mypage;
	struct scatterlist *sc_list;
	struct scatterlist *in_sg;
	int i, u, fp_offset, count;
	int n_num, p_num = size/PAGE_SIZE;

	dma_addr_t test_addr;
	int test_len;

	mypage = kcalloc(p_num, sizeof(struct page *), GFP_KERNEL);
	if (!mypage) {
		dev_err(pcie_ep_dev->dev, "failed to assign pages\n");
		return -EINVAL;
	}
	sc_list = kcalloc(p_num, sizeof(struct scatterlist), GFP_KERNEL);
	if (!sc_list) {
		dev_err(pcie_ep_dev->dev, "failed to assign sc_list\n");
		return -EINVAL;
	}
	fp_offset = (uint64_t) dmadest & ~PAGE_MASK;
	down_read(&current->mm->mmap_sem);
	n_num = get_user_pages(current, current->mm,
			(unsigned long) dmadest, p_num, 1, 1, mypage, NULL);
	up_read(&current->mm->mmap_sem);
	if (n_num < 0)
		return -EINVAL;
	if (n_num < maxsg) {
		sg_init_table(sc_list, n_num);
		sg_set_page(sc_list, *mypage,
					PAGE_SIZE - fp_offset, fp_offset);
		for (i = 1; i <= n_num-1; i++)
			sg_set_page(sc_list + i, *(mypage + i), PAGE_SIZE, 0);
		sg_set_page(sc_list + n_num, *(mypage + n_num),
			size - (PAGE_SIZE - fp_offset)
			- ((n_num-1)*PAGE_SIZE), 0);
		count = dma_map_sg(pcie_ep_dev->dev, sc_list,
				n_num, DMA_BIDIRECTIONAL);
		i = 0;
		u = 0;
		for_each_sg(sc_list, in_sg, count, i) {
			if (u < maxsg) {
				sg[u].paddr = FPGA_ADR(sg_dma_address(in_sg));
				sg[u].size = sg_dma_len(in_sg);
				test_addr = FPGA_ADR(sg_dma_address(in_sg));
				test_len = sg_dma_len(in_sg);
#ifdef COMBINE_SG
				if ((u > 0) && (sg[u-1].paddr + sg[u-1].size ==
					sg[u].paddr)) {
					sg[u-1].size = sg[u-1].size
						+ sg[u].size;
				} else {
					u++;
				}
#else
				u++;
#endif
			} else {
				dev_err(pcie_ep_dev->dev, "maxsg exceeded\n");
				dma_unmap_sg(pcie_ep_dev->dev,
					sc_list, n_num, DMA_BIDIRECTIONAL);
				kfree(mypage);
				kfree(sc_list);
				return -EINVAL;
			}

		}
		sg[u].paddr = NULL;
	} else {
		dev_err(pcie_ep_dev->dev, "maxsg exceeded\n");
		kfree(mypage);
		kfree(sc_list);
		return -EINVAL;
	}
	dma_unmap_sg(pcie_ep_dev->dev, sc_list, n_num, DMA_BIDIRECTIONAL);
	page_cache_release(*mypage);
	kfree(mypage);
	kfree(sc_list);
	return 0;
}

static int pcie_ll_build(struct mnh_sg_entry *src_sg,
			struct mnh_sg_entry *dst_sg, phys_addr_t **start_addr)
{
	struct mnh_dma_ll_element *ll_element, *tmp_element;
	struct mnh_sg_entry sg_dst, sg_src;
	int i, s, u;

	ll_element = kcalloc(DMA_LL_LENGTH,
			sizeof(struct mnh_dma_ll_element), GFP_KERNEL);
	*start_addr = FPGA_ADR(virt_to_phys(ll_element));
	if (!ll_element)
		return -EINVAL;
	i = 0;
	s = 0;
	u = 0;
	sg_src = src_sg[i];
	sg_dst = dst_sg[s];
	while ((sg_src.paddr != NULL) && (sg_dst.paddr != NULL)) {
		if (sg_src.size == sg_dst.size) {
			ll_element[u].header = LL_DATA_ELEMENT;
			ll_element[u].size = sg_src.size;
			ll_element[u].sar_low = LOWER(sg_src.paddr);
			ll_element[u].sar_high = UPPER(sg_src.paddr);
			ll_element[u].dar_low = LOWER(sg_dst.paddr);
			ll_element[u].dar_high = UPPER(sg_dst.paddr);
			i++;
			s++;
			sg_src = src_sg[i];
			sg_dst = dst_sg[s];
		} else if (sg_src.size > sg_dst.size) {
			ll_element[u].header = LL_DATA_ELEMENT;
			ll_element[u].size = sg_dst.size;
			ll_element[u].sar_low = LOWER(sg_src.paddr);
			ll_element[u].sar_high = UPPER(sg_src.paddr);
			ll_element[u].dar_low = LOWER(sg_dst.paddr);
			ll_element[u].dar_high = UPPER(sg_dst.paddr);
			sg_src.paddr = sg_src.paddr + sg_dst.size;
			sg_src.size = sg_src.size - sg_dst.size;
			s++;
			sg_dst = dst_sg[s];
		} else {

			ll_element[u].size = sg_src.size;
			ll_element[u].sar_low = LOWER(sg_src.paddr);
			ll_element[u].sar_high = UPPER(sg_src.paddr);
			ll_element[u].dar_low = LOWER(sg_dst.paddr);
			ll_element[u].dar_high = UPPER(sg_dst.paddr);
			sg_dst.paddr = sg_dst.paddr + sg_src.size;
			sg_dst.size = sg_dst.size - sg_src.size;
			i++;
			sg_src = src_sg[i];
		}
		u++;
		if (u == DMA_LL_LENGTH) {
			ll_element[u].header = LL_LINK_ELEMENT;
			if ((sg_src.paddr == NULL) || (sg_dst.paddr == NULL)) {
				ll_element[u-1].header = LL_IRQ_DATA_ELEMENT;
				ll_element[u].header = LL_LAST_LINK_ELEMENT;
				ll_element[u].sar_low =
					LOWER((uint64_t) *start_addr);
				ll_element[u].sar_high =
					UPPER((uint64_t) *start_addr);
				return 0;
			}
			tmp_element = kcalloc(DMA_LL_LENGTH,
					sizeof(struct mnh_dma_ll_element),
					GFP_KERNEL);
			if (!tmp_element) {
				ll_element[u-1].header = LL_IRQ_DATA_ELEMENT;
				ll_element[u].header = LL_LAST_LINK_ELEMENT;
				ll_element[u].sar_low =
					LOWER((uint64_t) *start_addr);
				ll_element[u].sar_high =
					UPPER((uint64_t) *start_addr);
				pcie_ll_destroy(start_addr);
				return -EINVAL;
			}
			ll_element[u].sar_low =
				LOWER((uint64_t)
					FPGA_ADR(virt_to_phys(tmp_element)));
			ll_element[u].sar_high =
				UPPER((uint64_t)
					FPGA_ADR(virt_to_phys(tmp_element)));
			ll_element = tmp_element;
			u = 0;
		}
	}
	ll_element[u-1].header = LL_IRQ_DATA_ELEMENT;
	ll_element[u].header = LL_LAST_LINK_ELEMENT;
	ll_element[u].sar_low = LOWER((uint64_t) start_addr);
	ll_element[u].sar_high = UPPER((uint64_t) start_addr);
	return 0;
}

static int pcie_ll_destroy(phys_addr_t *start_addr)
{
	int i;
	struct mnh_dma_ll_element *ll_element, *tmp_element;

	ll_element = phys_to_virt(CPU_ADR(start_addr));
	i = 0;
	while (1) {
		if (ll_element[i].header == LL_LINK_ELEMENT) {
			tmp_element = phys_to_virt(CPU_ADR(ll_element[i].sar_low
				+ (((uint64_t) ll_element[i].sar_high) << 32)));
			kfree(ll_element);
			ll_element = tmp_element;
			i = 0;
		} else if (ll_element[i].header == LL_LAST_LINK_ELEMENT) {
			kfree(ll_element);
			break;
		} else {
			i++;
		}
	}
	return 0;
}

/* APIs exposed to message passing protocol */

/* API to generate MSI to AP */
int mnh_send_msi(enum mnh_msi_msg_t msi)
{

	return pcie_send_msi(msi);
}

EXPORT_SYMBOL(mnh_send_msi);

/* API to generate LTR to AP */
/* Latency Tolerance Reporting */
int mnh_send_ltr(uint32_t ltr)
{

	return pcie_send_ltr(ltr);
}

EXPORT_SYMBOL(mnh_send_ltr);

/* API to set PCIE endpoint into L1 */
int mnh_set_l_one(uint32_t enable, uint32_t clkpm)
{

	return pcie_set_l_one(enable, clkpm);
}

EXPORT_SYMBOL(mnh_set_l_one);

/* API to send Vendor message to AP */
int mnh_send_vm(struct mnh_pcie_vm *vm)
{
	return pcie_send_vm(vm->vm);
}

EXPORT_SYMBOL(mnh_send_vm);

/* API to register IRQ callbacks */
int mnh_reg_irq_callback(int (*callback)(struct mnh_pcie_irq *irq),
					int (*dmacallback)(struct mnh_dma_irq *irq))
{
	irq_callback = *callback;
	dma_callback = *dmacallback;

	return 0;
}

EXPORT_SYMBOL(mnh_reg_irq_callback);

/* API to program ringbuffer base address to PCIE BOOTSTRAP REGISTER */
int mnh_set_rb_base(uint64_t rb_base)
{
	return pcie_set_rb_base(LOWER(rb_base));
}

EXPORT_SYMBOL(mnh_set_rb_base);

/* API to read data from AP */
int mnh_pcie_read(uint8_t *buff, uint32_t size, uint64_t adr)
{
	return pcie_read_data(buff, size, adr);
}

EXPORT_SYMBOL(mnh_pcie_read);

/* API to write data from AP */
int mnh_pcie_write(uint8_t *buff, uint32_t size, uint64_t adr)
{
	return pcie_write_data(buff, size, adr);
}

EXPORT_SYMBOL(mnh_pcie_write);

int mnh_sg_build(void *dmadest, size_t size, struct mnh_sg_entry *sg,
				uint32_t maxsg)
{
	return pcie_sg_build(dmadest, size, sg, maxsg);
}

EXPORT_SYMBOL(mnh_sg_build);

int mnh_ll_build(struct mnh_sg_entry *src_sg, struct mnh_sg_entry *dst_sg,
				phys_addr_t **start_addr)
{
	return pcie_ll_build(src_sg, dst_sg, start_addr);
}

EXPORT_SYMBOL(mnh_ll_build);

int mnh_ll_destroy(phys_addr_t *start_addr)
{
	return pcie_ll_destroy(start_addr);
}

EXPORT_SYMBOL(mnh_ll_destroy);

static int config_mem(struct platform_device *pdev)
{
	pcie_ep_dev->config_mem =
		platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!pcie_ep_dev->config_mem)
		return -ENOMEM;
	if (!request_mem_region(pcie_ep_dev->config_mem->start,
		resource_size(pcie_ep_dev->config_mem),
		pcie_ep_dev->name)) {
		dev_err(&pdev->dev, "unable to request mem region\n");
		return -ENOMEM;
	}
	pcie_ep_dev->conf_mem =
		ioremap_nocache(pcie_ep_dev->config_mem->start,
			resource_size(pcie_ep_dev->config_mem));
	if (!pcie_ep_dev->conf_mem) {
		release_mem_region(pcie_ep_dev->config_mem->start,
				resource_size(pcie_ep_dev->config_mem));
		return -ENOMEM;
	}
	pcie_ep_dev->cluster_mem =
		platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!pcie_ep_dev->cluster_mem) {
		iounmap(&pcie_ep_dev->conf_mem);
		release_mem_region(pcie_ep_dev->config_mem->start,
				resource_size(pcie_ep_dev->config_mem));
		return -ENOMEM;
	}
	if (!request_mem_region(pcie_ep_dev->cluster_mem->start,
		resource_size(pcie_ep_dev->cluster_mem), pcie_ep_dev->name)) {
		dev_err(&pdev->dev, "unable to request mem region\n");
		iounmap(&pcie_ep_dev->conf_mem);
		release_mem_region(pcie_ep_dev->config_mem->start,
				resource_size(pcie_ep_dev->config_mem));
		return -ENOMEM;
	}
	pcie_ep_dev->clust_mem =
		ioremap_nocache(pcie_ep_dev->cluster_mem->start,
			resource_size(pcie_ep_dev->cluster_mem));
	if (!pcie_ep_dev->clust_mem) {
		dev_err(&pdev->dev, "unable to request mem region\n");
		iounmap(&pcie_ep_dev->conf_mem);
		release_mem_region(pcie_ep_dev->config_mem->start,
				resource_size(pcie_ep_dev->config_mem));
		release_mem_region(pcie_ep_dev->cluster_mem->start,
				resource_size(pcie_ep_dev->cluster_mem));
		return -ENOMEM;
	}
	pcie_ep_dev->outbound_mem = platform_get_resource(pdev,
					IORESOURCE_MEM, 2);
	if (!pcie_ep_dev->outbound_mem) {
		iounmap(&pcie_ep_dev->clust_mem);
		iounmap(&pcie_ep_dev->conf_mem);
		release_mem_region(pcie_ep_dev->config_mem->start,
			resource_size(pcie_ep_dev->config_mem));
		release_mem_region(pcie_ep_dev->cluster_mem->start,
				resource_size(pcie_ep_dev->cluster_mem));
		return -ENOMEM;
	}
	if (!request_mem_region(pcie_ep_dev->outbound_mem->start,
		resource_size(pcie_ep_dev->outbound_mem), pcie_ep_dev->name)) {
		dev_err(&pdev->dev, "unable to request mem region\n");
		iounmap(&pcie_ep_dev->clust_mem);
		iounmap(&pcie_ep_dev->conf_mem);
		release_mem_region(pcie_ep_dev->config_mem->start,
				resource_size(pcie_ep_dev->config_mem));
		release_mem_region(pcie_ep_dev->cluster_mem->start,
				resource_size(pcie_ep_dev->cluster_mem));
		return -ENOMEM;
		}
	pcie_ep_dev->outb_mem = ioremap(pcie_ep_dev->outbound_mem->start,
				resource_size(pcie_ep_dev->outbound_mem));
	if (!pcie_ep_dev->outb_mem) {
		dev_err(&pdev->dev, "unable to request mem region\n");
		iounmap(&pcie_ep_dev->clust_mem);
		iounmap(&pcie_ep_dev->conf_mem);
		release_mem_region(pcie_ep_dev->config_mem->start,
				resource_size(pcie_ep_dev->config_mem));
		release_mem_region(pcie_ep_dev->cluster_mem->start,
				resource_size(pcie_ep_dev->cluster_mem));
		release_mem_region(pcie_ep_dev->outbound_mem->start,
				resource_size(pcie_ep_dev->outbound_mem));
		return -ENOMEM;
	}
	return 0;
}

static int clear_mem(void)
{
	iounmap(&pcie_ep_dev->conf_mem);
	iounmap(&pcie_ep_dev->clust_mem);
	iounmap(&pcie_ep_dev->outb_mem);
	release_mem_region(pcie_ep_dev->config_mem->start,
				resource_size(pcie_ep_dev->config_mem));
	release_mem_region(pcie_ep_dev->cluster_mem->start,
				resource_size(pcie_ep_dev->cluster_mem));
	release_mem_region(pcie_ep_dev->outbound_mem->start,
				resource_size(pcie_ep_dev->outbound_mem));
	return 0;
}

/* IRQ Callback function for testing purposes */

int test_callback(struct mnh_pcie_irq *irq)
{
	sysfs_irq = *irq;
	dev_err(pcie_ep_dev->dev, "PCIE MSI IRQ %x PCIE IRQ %x VM IRQ %x",
		sysfs_irq.msi_irq, sysfs_irq.pcie_irq, sysfs_irq.vm);
	return 0;
}

int test_dma_callback(struct mnh_dma_irq *irq)
{
	/*TODO do something */
	dev_err(pcie_ep_dev->dev, "DMA Event channel %x type %x Event %x",
		irq->channel, irq->type, irq->status);
	return 0;
}

/* SYS_FS for debugging and testing */

static ssize_t show_sysfs_send_msi(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	return snprintf(buf, MAX_STR_COPY, "No support\n");
}

static ssize_t sysfs_send_msi(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;
	if ((val < 0) | (val > 31))
		return -EINVAL;
	mnh_send_msi((enum mnh_msi_msg_t) val);
	return count;
}

static DEVICE_ATTR(send_msi, S_IRUGO | S_IWUSR | S_IWGRP,
			show_sysfs_send_msi, sysfs_send_msi);

static ssize_t show_sysfs_send_vm(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	return snprintf(buf, MAX_STR_COPY, "No support\n");
}

static ssize_t sysfs_send_vm(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	unsigned long val;
	struct mnh_pcie_vm vm;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;
	vm.vm = (uint32_t)val;
	mnh_send_vm(&vm);
	return count;
}

static DEVICE_ATTR(send_vm, S_IRUGO | S_IWUSR | S_IWGRP,
			show_sysfs_send_vm, sysfs_send_vm);


static ssize_t show_sysfs_rb_base(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	uint64_t rb_base;

	rb_base = pcie_cluster_read(MNH_PCIE_GP_1);
	return snprintf(buf, MAX_STR_COPY, "%llx\n", rb_base);
}

static ssize_t sysfs_rb_base(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;
	mnh_set_rb_base(val);
	return count;
}

static DEVICE_ATTR(rb_base, S_IRUGO | S_IWUSR | S_IWGRP,
			show_sysfs_rb_base, sysfs_rb_base);

static ssize_t show_sysfs_rw_address(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	return snprintf(buf, MAX_STR_COPY, "%lx\n",
				(unsigned long) rw_address_sysfs);
}

static ssize_t sysfs_rw_address(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;
	rw_address_sysfs = (uint32_t)val;
	return count;
}

static DEVICE_ATTR(rw_address, S_IRUGO | S_IWUSR | S_IWGRP,
			show_sysfs_rw_address, sysfs_rw_address);

static ssize_t show_sysfs_rw_size(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	return snprintf(buf, MAX_STR_COPY, "%lx\n",
				(unsigned long) rw_size_sysfs);
}

static ssize_t sysfs_rw_size(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;
	rw_size_sysfs = (uint32_t)val;
	return count;
}

static DEVICE_ATTR(rw_size, S_IRUGO | S_IWUSR | S_IWGRP,
			show_sysfs_rw_size, sysfs_rw_size);

static ssize_t show_sysfs_read_cluster(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{	unsigned long val;

	val = (unsigned long) pcie_cluster_read(rw_address_sysfs);
	return snprintf(buf, MAX_STR_COPY, "%lx\n", val);
}

static ssize_t sysfs_write_cluster(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;
	pcie_cluster_write(rw_address_sysfs, (uint32_t)val);
	return count;
}

static DEVICE_ATTR(rw_cluster, S_IRUGO | S_IWUSR | S_IWGRP,
			show_sysfs_read_cluster, sysfs_write_cluster);

static ssize_t show_sysfs_read_config(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{	unsigned long val;

	val = (unsigned long)pcie_config_read(rw_address_sysfs);
	return snprintf(buf, MAX_STR_COPY, "%lx\n", val);
}

static ssize_t sysfs_write_config(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;
	pcie_config_write(rw_address_sysfs, (uint32_t)val);
	return count;
}

static DEVICE_ATTR(rw_config, S_IRUGO | S_IWUSR | S_IWGRP,
			show_sysfs_read_config, sysfs_write_config);

static ssize_t show_sysfs_read_data(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	unsigned long val;
	uint8_t *buffer;

	buffer = kmalloc(MAX_STR_COPY, GFP_KERNEL);
	if (!buffer)
		return -EINVAL;
	if (mnh_pcie_read(buffer, rw_size_sysfs, rw_address_sysfs)) {
		kfree(buffer);
		return -EINVAL;
	}
	val = copy_to_user(buf, buffer, rw_size_sysfs);
	if (val) {
		kfree(buffer);
		return -EINVAL;
	}
	kfree(buffer);
	return rw_size_sysfs;
}

static ssize_t sysfs_write_data(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	if (mnh_pcie_write((uint8_t *)buf, count, rw_address_sysfs)) {
		dev_err(pcie_ep_dev->dev, "Write buffer failed\n");
		return -EINVAL;
	}
	return count;
}

static DEVICE_ATTR(rw_data, S_IRUGO | S_IWUSR | S_IWGRP,
			show_sysfs_read_data, sysfs_write_data);

static ssize_t show_sysfs_set_iob(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	return snprintf(buf, MAX_STR_COPY, "No support\n");
}

static ssize_t sysfs_set_iob(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	unsigned long val;
	struct mnh_inb_window inb;
	uint8_t *token;
	const char *delim = ";";

	token = strsep((char **) &buf, delim);
	if (token) {
		if (kstrtoul(token, 0, &val))
		return -EINVAL;
	if ((val < 0) || (val > 1))
		return -EINVAL;
	inb.mode = val;
	dev_err(pcie_ep_dev->dev, "Inbound mode is %lx\n", val);
	} else {
		return -EINVAL;
	}

	token = strsep((char **) &buf, delim);

	if (token) {
		if (kstrtoul(token, 0, &val))
		return -EINVAL;
	if ((val < 0) || (val > 2))
		return -EINVAL;
	inb.bar = val;
	dev_err(pcie_ep_dev->dev, "Inbound bar is %lx\n", val);
	} else {
		return -EINVAL;
	}

	token = strsep((char **) &buf, delim);

	if (token) {
		if (kstrtoul(token, 0, &val))
			return -EINVAL;
		if ((val < 0) || (val > 15))
			return -EINVAL;
		inb.region = val;
		dev_err(pcie_ep_dev->dev, "Inbound region is %lx\n", val);
	} else {
		return -EINVAL;
	}

	token = strsep((char **) &buf, delim);

	if (token) {
		if (kstrtoul(token, 0, &val))
			return -EINVAL;
		if ((val < 0) || (val > 15))
			return -EINVAL;
		inb.memmode = val;
		dev_err(pcie_ep_dev->dev, "Inbound memmode is %lx\n", val);
	} else {
		return -EINVAL;
	}

	token = strsep((char **) &buf, delim);

	if (token) {
		if (kstrtoul(token, 0, &val))
			return -EINVAL;
		inb.base_pcie_address = val;
		dev_err(pcie_ep_dev->dev, "Inbound base is %lx\n", val);
	} else {
		return -EINVAL;
	}

	token = strsep((char **) &buf, delim);

	if (token) {
		if (kstrtoul(token, 0, &val))
			return -EINVAL;
		inb.limit_pcie_address = val;
		dev_err(pcie_ep_dev->dev, "Inbound limit is %lx\n", val);
	} else {
		return -EINVAL;
	}

	token = strsep((char **) &buf, delim);

	if (token) {
		if (kstrtoul(token, 0, &val))
			return -EINVAL;
		inb.target_mnh_address = val;
		dev_err(pcie_ep_dev->dev, "Inbound target is %lx\n", val);
	} else {
		return -EINVAL;
	}
	pcie_set_inbound_iatu(&inb);
	return count;
}

static DEVICE_ATTR(set_inbound, S_IRUGO | S_IWUSR | S_IWGRP,
			show_sysfs_set_iob, sysfs_set_iob);

static ssize_t show_sysfs_set_outb(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	return snprintf(buf, MAX_STR_COPY, "No support\n");
}

static ssize_t sysfs_set_outb(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	unsigned long val;
	struct mnh_outb_region outb;
	uint8_t *token;
	const char *delim = ";";

	token = strsep((char **) &buf, delim);

	if (token) {
		if (kstrtoul(token, 0, &val))
			return -EINVAL;
		if ((val < 0) || (val > 32))
			return -EINVAL;
		outb.region = val;
		dev_err(pcie_ep_dev->dev, "Outbound region is %lx\n", val);
	} else {
		return -EINVAL;
	}

	token = strsep((char **) &buf, delim);

	if (token) {
		if (kstrtoul(token, 0, &val))
			return -EINVAL;
		outb.base_mnh_address = val;
		dev_err(pcie_ep_dev->dev, "Outbound base is %lx\n", val);
	} else {
		return -EINVAL;
	}

	token = strsep((char **) &buf, delim);

	if (token) {
		if (kstrtoul(token, 0, &val))
			return -EINVAL;
		outb.limit_mnh_address = val;
		dev_err(pcie_ep_dev->dev, "Outbound limit is %lx\n", val);
	} else {
		return -EINVAL;
	}

	token = strsep((char **) &buf, delim);

	if (token) {
		if (kstrtoul(token, 0, &val))
			return -EINVAL;
		outb.target_pcie_address = val;
		dev_err(pcie_ep_dev->dev, "Outbound target is %lx\n", val);
	} else {
		return -EINVAL;
	}
	pcie_set_outbound_iatu(&outb);
	return count;
}

static DEVICE_ATTR(set_outbound, S_IRUGO | S_IWUSR | S_IWGRP,
			show_sysfs_set_outb, sysfs_set_outb);

static ssize_t show_sysfs_test_callback(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	return snprintf(buf, MAX_STR_COPY,
		"PCIE MSI IRQ %d PCIE IRQ %d VM IRQ %d",
		sysfs_irq.msi_irq, sysfs_irq.pcie_irq, sysfs_irq.vm);
}

static ssize_t sysfs_test_callback(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;
	if (val == 1)
		mnh_reg_irq_callback(&test_callback, &test_dma_callback);
	else
		mnh_reg_irq_callback(NULL, NULL);

	return count;
}

static DEVICE_ATTR(test_callback, S_IRUGO | S_IWUSR | S_IWGRP,
			show_sysfs_test_callback, sysfs_test_callback);

static ssize_t show_sysfs_send_ltr(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	return snprintf(buf, MAX_STR_COPY, "No support\n");
}

static ssize_t sysfs_send_ltr(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;
	mnh_send_ltr((uint32_t) val);
	return count;
}

static DEVICE_ATTR(send_ltr, S_IRUGO | S_IWUSR | S_IWGRP,
			show_sysfs_send_ltr, sysfs_send_ltr);

static ssize_t show_sysfs_set_lone(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	return snprintf(buf, MAX_STR_COPY, "No support\n");
}

static ssize_t sysfs_set_lone(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	unsigned long val;
	uint32_t enable, pmclock;
	uint8_t *token;
	const char *delim = ";";

	token = strsep((char **) &buf, delim);

	if (token) {
		if (kstrtoul(token, 0, &val))
			return -EINVAL;
		if ((val < 0) || (val > 1))
			return -EINVAL;
		enable = val;
		dev_err(pcie_ep_dev->dev, "L1 state is %lx\n", val);
	} else {
		return -EINVAL;
	}

	token = strsep((char **) &buf, delim);

	if (token) {
		if (kstrtoul(token, 0, &val))
			return -EINVAL;
		if ((val < 0) || (val > 1))
			return -EINVAL;
		pmclock = val;
		dev_err(pcie_ep_dev->dev, "PM Clock is %lx\n", val);
	} else {
		return -EINVAL;
	}
	mnh_set_l_one(enable, pmclock);
	return count;
}

static DEVICE_ATTR(set_lone, S_IRUGO | S_IWUSR | S_IWGRP,
			show_sysfs_set_lone, sysfs_set_lone);

static int init_sysfs(void)
{
	int ret;

	ret = device_create_file(pcie_ep_dev->dev,
			&dev_attr_send_msi);
	if (ret) {
		dev_err(pcie_ep_dev->dev, "Failed to create sysfs: send_msi\n");
		return -EINVAL;
	}
	ret = device_create_file(pcie_ep_dev->dev,
			&dev_attr_send_vm);
	if (ret) {
		dev_err(pcie_ep_dev->dev, "Failed to create sysfs: send_vm\n");
		device_remove_file(pcie_ep_dev->dev,
			&dev_attr_send_msi);
		return -EINVAL;
	}
	ret = device_create_file(pcie_ep_dev->dev,
			&dev_attr_rb_base);
	if (ret) {
		dev_err(pcie_ep_dev->dev, "Failed to create sysfs: rb_base\n");
		device_remove_file(pcie_ep_dev->dev,
			&dev_attr_send_msi);
		device_remove_file(pcie_ep_dev->dev,
			&dev_attr_send_vm);
		return -EINVAL;
	}
	rw_address_sysfs = 0x0;
	ret = device_create_file(pcie_ep_dev->dev,
			&dev_attr_rw_address);
	if (ret) {
		dev_err(pcie_ep_dev->dev, "Failed to create sysfs: rw_address\n");
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_send_msi);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_send_vm);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_rb_base);
		return -EINVAL;
	}
	rw_size_sysfs = 0x1;
	ret = device_create_file(pcie_ep_dev->dev,
			&dev_attr_rw_size);
	if (ret) {
		dev_err(pcie_ep_dev->dev, "Failed to create sysfs: rw_size\n");
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_send_msi);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_send_vm);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_rb_base);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_rw_address);
		return -EINVAL;
	}
	ret = device_create_file(pcie_ep_dev->dev,
			&dev_attr_rw_cluster);
	if (ret) {
		dev_err(pcie_ep_dev->dev, "Failed to create sysfs: rw_cluster\n");
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_send_msi);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_send_vm);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_rb_base);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_rw_address);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_rw_size);
		return -EINVAL;
	}
	ret = device_create_file(pcie_ep_dev->dev,
			&dev_attr_rw_config);
	if (ret) {
		dev_err(pcie_ep_dev->dev, "Failed to create sysfs: rw_config\n");
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_send_msi);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_send_vm);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_rb_base);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_rw_address);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_rw_size);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_rw_cluster);
		return -EINVAL;
	}
	ret = device_create_file(pcie_ep_dev->dev,
			&dev_attr_rw_data);
	if (ret) {
		dev_err(pcie_ep_dev->dev, "Failed to create sysfs: rw_data\n");
				device_remove_file(pcie_ep_dev->dev,
				&dev_attr_send_msi);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_send_vm);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_rb_base);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_rw_address);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_rw_size);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_rw_cluster);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_rw_config);
		return -EINVAL;
	}
	ret = device_create_file(pcie_ep_dev->dev,
			&dev_attr_set_inbound);
	if (ret) {
		dev_err(pcie_ep_dev->dev, "Failed to create sysfs: set_inbound\n");
				device_remove_file(pcie_ep_dev->dev,
				&dev_attr_send_msi);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_send_vm);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_rb_base);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_rw_address);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_rw_size);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_rw_cluster);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_rw_config);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_rw_data);
		return -EINVAL;
	}
	ret = device_create_file(pcie_ep_dev->dev,
			&dev_attr_set_outbound);
	if (ret) {
		dev_err(pcie_ep_dev->dev, "Failed to create sysfs: set_outbound\n");
				device_remove_file(pcie_ep_dev->dev,
				&dev_attr_send_msi);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_send_vm);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_rb_base);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_rw_address);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_rw_size);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_rw_cluster);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_rw_config);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_rw_data);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_set_inbound);
		return -EINVAL;
	}
	ret = device_create_file(pcie_ep_dev->dev,
			&dev_attr_test_callback);
	if (ret) {
		dev_err(pcie_ep_dev->dev, "Failed to create sysfs: test_callback\n");
				device_remove_file(pcie_ep_dev->dev,
				&dev_attr_send_msi);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_send_vm);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_rb_base);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_rw_address);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_rw_size);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_rw_cluster);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_rw_config);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_rw_data);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_set_inbound);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_set_outbound);
		return -EINVAL;
	}
	ret = device_create_file(pcie_ep_dev->dev,
			&dev_attr_send_ltr);
	if (ret) {
		dev_err(pcie_ep_dev->dev, "Failed to create sysfs: send_ltr\n");
				device_remove_file(pcie_ep_dev->dev,
				&dev_attr_send_msi);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_send_vm);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_rb_base);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_rw_address);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_rw_size);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_rw_cluster);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_rw_config);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_rw_data);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_set_inbound);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_set_outbound);
		device_remove_file(pcie_ep_dev->dev,
			&dev_attr_test_callback);
		return -EINVAL;
	}
	ret = device_create_file(pcie_ep_dev->dev,
			&dev_attr_set_lone);
	if (ret) {
		dev_err(pcie_ep_dev->dev, "Failed to create sysfs: set_lone\n");
				device_remove_file(pcie_ep_dev->dev,
				&dev_attr_send_msi);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_send_vm);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_rb_base);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_rw_address);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_rw_size);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_rw_cluster);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_rw_config);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_rw_data);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_set_inbound);
		device_remove_file(pcie_ep_dev->dev,
				&dev_attr_set_outbound);
		device_remove_file(pcie_ep_dev->dev,
			&dev_attr_test_callback);
		device_remove_file(pcie_ep_dev->dev,
			&dev_attr_send_ltr);
		return -EINVAL;
	}

	return 0;
}

static void clean_sysfs(void)
{
	device_remove_file(pcie_ep_dev->dev,
			&dev_attr_send_msi);
	device_remove_file(pcie_ep_dev->dev,
			&dev_attr_send_vm);
	device_remove_file(pcie_ep_dev->dev,
			&dev_attr_rb_base);
	device_remove_file(pcie_ep_dev->dev,
			&dev_attr_rw_address);
	device_remove_file(pcie_ep_dev->dev,
			&dev_attr_rw_size);
	device_remove_file(pcie_ep_dev->dev,
			&dev_attr_rw_cluster);
	device_remove_file(pcie_ep_dev->dev,
			&dev_attr_rw_config);
	device_remove_file(pcie_ep_dev->dev,
			&dev_attr_rw_data);
	device_remove_file(pcie_ep_dev->dev,
			&dev_attr_set_inbound);
	device_remove_file(pcie_ep_dev->dev,
			&dev_attr_set_outbound);
	device_remove_file(pcie_ep_dev->dev,
			&dev_attr_test_callback);
	device_remove_file(pcie_ep_dev->dev,
			&dev_attr_send_ltr);
	device_remove_file(pcie_ep_dev->dev,
			&dev_attr_set_lone);
}

static int mnh_pcie_ep_probe(struct platform_device *pdev)
{
	int err;

	dev_err(&pdev->dev, "PCIE endpoint probe start\n");
	pcie_ep_dev = kzalloc(sizeof(*pcie_ep_dev), GFP_KERNEL);
	if (!pcie_ep_dev) {
		dev_err(&pdev->dev, "Could not allocated pcie_ep_dev\n");
		return -ENOMEM;
	}
	pcie_ep_dev->dev = &pdev->dev;
	strcpy(pcie_ep_dev->name, DEVICE_NAME);
	irq_callback = NULL;
	dma_callback = NULL;
	err = config_mem(pdev);
	if (err)
		return err;

	/* Register IRQs */

	pcie_ep_dev->sw_irq = platform_get_irq(pdev, 0);

	err = request_irq(pcie_ep_dev->sw_irq, pcie_handle_sw_irq,
			IRQF_SHARED, DEVICE_NAME, pcie_ep_dev);
	if (err) {
		clear_mem();
		return -EINVAL;
	}
	pcie_ep_dev->cluster_irq = platform_get_irq(pdev, 1);
	err = request_irq(pcie_ep_dev->cluster_irq, pcie_handle_cluster_irq,
			IRQF_SHARED, DEVICE_NAME, pcie_ep_dev->dev);
	if (err) {
		free_irq(pcie_ep_dev->sw_irq, pcie_ep_dev->dev);
		clear_mem();
		return -EINVAL;
	}


/* TODO: handle PCIe wake IRQ */

/* declare MSI worker */
	INIT_DELAYED_WORK(&msi_work, pcie_msi_worker);
	INIT_WORK(&msi_rx_work, msi_rx_worker);
	INIT_WORK(&pcie_irq_work, pcie_irq_worker);
	init_sysfs();
	pcie_link_init();
	return 0;
}

static int mnh_pcie_ep_remove(struct platform_device *pdev)
{
	cancel_delayed_work_sync(&msi_work);
	clean_sysfs();
	clear_mem();
	free_irq(pcie_ep_dev->sw_irq, pcie_ep_dev->dev);
	free_irq(pcie_ep_dev->cluster_irq, pcie_ep_dev->dev);
	return 0;
}

/*
 * of_device_id structure
 */
static const struct of_device_id mnh_pcie_ep[] = {
	{ .compatible = "snps, dw_pcie_ep" },
	{ }
};

MODULE_DEVICE_TABLE(of, mnh_pcie_ep);
/*
 * Platform driver structure
 */
static struct platform_driver __refdata mnh_pcie_ep_pdrv = {
	.remove = mnh_pcie_ep_remove,
	.probe  = mnh_pcie_ep_probe,
	.driver   = {
		.name   = "snps, dw_pcie_ep",
		.owner = THIS_MODULE,
		.of_match_table = mnh_pcie_ep,
	},
};


module_platform_driver(mnh_pcie_ep_pdrv);

MODULE_AUTHOR("Marko Bartscherer <marko.bartscherer@intel.com>");
MODULE_DESCRIPTION("Monhette Hill PCIE EndPoint Driver");
MODULE_LICENSE("GPL");
