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
/* #include <asm-generic/page.h> */
#include <linux/mm.h>
#include <linux/rwsem.h>
#include <linux/scatterlist.h>
#include <linux/dma-mapping.h>
#include <linux/pci.h>

#define VENDOR_ID				0x8086
#define DEVICE_ID				0x3140

int (*irq_callback)(struct mnh_pcie_irq *irq);
int (*dma_callback)(struct mnh_dma_irq *irq);
struct mnh_pcieep_device *pci_ep_dev;
dev_t mht_pcie_ep_dev;
static struct class *pcie_ep_class;
static struct device *pcie_ep_device;
static struct resource *conf_mem;
static struct resource *cluster_mem;
static struct resource *outbound_mem;
static uint64_t *pcie_conf_mem, *pcie_clust_mem, *pcie_outb_mem;
struct delayed_work msi_work;
struct work_struct msi_rx_work, pcie_irq_work, dma_irq_work;
#define DEVICE_NAME "mnh_pcie_ep"
#define CLASS_NAME "pcie_ep"
#define MSI_DELAY (HZ/20) /* TODO: Need to understand what this should be */
#define MAX_STR_COPY	32
uint32_t rw_address_sysfs, rw_size_sysfs;
struct mnh_pcie_irq sysfs_irq;
uint32_t pcie_sw_irq, pcie_cluster_irq;

static int pcie_set_inbound_iatu(struct mnh_inb_window *inb);
static int pcie_clear_msi_mask(void);
static int pcie_ll_destroy(uint64_t *start_addr);

/* read from pcie cluster register */
static uint32_t pcie_cluster_read(uint64_t address)
{
	uint32_t data;
	uint64_t addr = (uint64_t)pcie_clust_mem + address;

	data = ioread32(addr);
	return data;
}

/* write to pcie cluster register */

static int pcie_cluster_write(uint64_t address, uint32_t data)
{
	uint64_t addr = (uint64_t)pcie_clust_mem + address;

	iowrite32(data, addr);
	return 0;
}

/* read from pcie config register */

static uint32_t pcie_config_read(uint64_t address)
{
	uint32_t data;
	uint64_t addr = (uint64_t)pcie_conf_mem + address;

	data = ioread32(addr);
	return data;
}

/* write to pcie config register */

static int pcie_config_write(uint64_t address, uint32_t data)
{
	uint64_t addr = (uint64_t)pcie_conf_mem + address;

	iowrite32(data, addr);
	return 0;
}

static int check_sram_init_done(void)
{
	uint32_t data;

	data = pcie_cluster_read(MNH_PCIE_APP_STS_REG);
	data &= MNH_PCIE_PHY_SRAM_INIT_DONE;
	if (data != MNH_PCIE_PHY_SRAM_INIT_DONE)
		return  0;

	return 1;
}

static int pcie_link_up(void)
{
	uint32_t data;

	data = pcie_config_read(MNH_STATUS_COMMAND_REG);
	data &= MNH_PCIE_LINK_MASK;
	if (data == 0)
		return 0;
	return 1;
}

static int pcie_link_init(void)
{
	struct mnh_inb_window iatu;

	if (!pcie_link_up()) {
		/* Magic value Only used during boot */
		pcie_cluster_write(MNH_PCIE_GP_0, 0x0);
		/* program PCIe controller registers
		* not needed at this point in time
		*/

		/* TODO: read eFuses to update device address */
		while (!check_sram_init_done())
			udelay(1);

		/* program PCIe PHY PCS PATCH registers
		*not needed at this point in time
		*/
		pcie_cluster_write(MNH_PCIE_APP_CTRL,
			pcie_cluster_read(MNH_PCIE_APP_CTRL) |
				MNH_PCIE_PHY_SRAM_LD_DONE);

		/* program PCIe PHY registers not needed
		* at this point in time
		*/

		/* Programing the inbound IATU */
		iatu.mode = BAR_MATCH;
		iatu.bar = BAR_2_3;
		iatu.region = REGION_1;
		iatu.target_mnh_address = BAR_2_BASE;
		pcie_set_inbound_iatu(&iatu);
		iatu.mode = BAR_MATCH;
		iatu.bar = BAR_4_5;
		iatu.region = REGION_2;
		iatu.target_mnh_address = BAR_4_BASE;
		pcie_set_inbound_iatu(&iatu);
		pcie_clear_msi_mask();

		pcie_cluster_write(MNH_PCIE_APP_CTRL,
			pcie_cluster_read(MNH_PCIE_APP_CTRL) |
				MNH_PCIE_APP_LTSSM_ENABLE);
		pcie_cluster_write(MNH_PCIE_APP_CTRL,
			pcie_cluster_read(MNH_PCIE_APP_CTRL) |
				MNH_PCIE_APP_XFER_PEND);

		/* waiting for magic value */
		while (pcie_cluster_read(MNH_PCIE_GP_0) == 0)
			udelay(1);

		pcie_cluster_write(MNH_PCIE_APP_CTRL,
			pcie_cluster_read(MNH_PCIE_APP_CTRL) &
				~(MNH_PCIE_APP_XFER_PEND));
			};
	return  pcie_cluster_read(MNH_PCIE_GP_0);
}

static void force_link_up(void)
{
	if (pcie_cluster_read(MNH_PCIE_APP_CTRL) &
				(MNH_PCIE_APP_REQ_EXIT_L1)) {
		pcie_cluster_write(MNH_PCIE_APP_CTRL,
			pcie_cluster_read(MNH_PCIE_APP_CTRL) &
				~(MNH_PCIE_APP_REQ_EXIT_L1));
	}
	pcie_cluster_write(MNH_PCIE_APP_CTRL,
			pcie_cluster_read(MNH_PCIE_APP_CTRL) |
				(MNH_PCIE_APP_REQ_EXIT_L1));
}

static void release_link(void)
{
	pcie_cluster_write(MNH_PCIE_APP_CTRL,
		pcie_cluster_read(MNH_PCIE_APP_CTRL) &
		~(MNH_PCIE_APP_REQ_EXIT_L1));
}

static int pcie_send_msi_p(uint32_t msi)
{
	uint32_t tc, data, msg;

	data = pcie_cluster_read(MNH_PCIE_MSI_TRIG_REG);
	data &= MNH_TRIGGER_MSI;
	tc = MNH_TC0 << 13;
	msg = msi << 8;
	data |= tc | msg;
	pcie_cluster_write(MNH_PCIE_MSI_TRIG_REG, data);
	data = pcie_cluster_read(MNH_PCIE_MSI_TRIG_REG);
	data |= MNH_TRIGGER_MSI;
	pcie_cluster_write(MNH_PCIE_MSI_TRIG_REG, data);
	return 0;
}


static int pcie_get_msi_mask(void)
{
	uint32_t data, mask;

	data = pcie_config_read(MNH_PCI_MSI_CAP_ID_NEXT_CTRL);
	data &=  MNH_PCI_MSI_64_BIT_ADDR_CAP;
	if (data) {
	/* 64 bit ADDR used */
		mask = pcie_config_read(MNH_MSI_CAP_OFF_10H_REG);
	} else {
	/* 32 bit ADDR used */
		mask = pcie_config_read(MNH_MSI_CAP_OFF_0CH_REG);
	}
	return mask;
}

static int pcie_clear_msi_mask(void)
{
	uint32_t data;

	data = pcie_config_read(MNH_PCI_MSI_CAP_ID_NEXT_CTRL);
	data &=  MNH_PCI_MSI_64_BIT_ADDR_CAP;
	if (data) {
	/* 64 bit ADDR used */
		pcie_config_write(MNH_MSI_CAP_OFF_10H_REG, 0);
	} else {
	/* 32 bit ADDR used */
		pcie_config_write(MNH_MSI_CAP_OFF_0CH_REG, 0);
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
		pcie_cluster_write(MNH_PCIE_MSI_PEND_REG,
			(pcie_cluster_read(MNH_PCIE_MSI_PEND_REG) | mask));
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
		pcie_cluster_write(MNH_PCIE_MSI_PEND_REG,
			(pend & ~b2h(arg)));
	}
}

static void send_pending_msi(void)
{
	uint32_t vmask, pend;

	vmask = pcie_get_msi_mask();
	pend = pcie_cluster_read(MNH_PCIE_MSI_PEND_REG);
	process_pend_msi(pend, vmask, PET_WATCHDOG);
	process_pend_msi(pend, vmask, CRASH_DUMP);
	process_pend_msi(pend, vmask, BOOTSTRAP_SET);
}

static void pcie_msi_worker(struct work_struct *work)
{
	uint32_t pend;

	force_link_up();
	send_pending_msi();
	pend = pcie_cluster_read(MNH_PCIE_MSI_PEND_REG);
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
	pend = pcie_cluster_read(MNH_PCIE_MSI_PEND_REG);
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
	pcie_cluster_write(MNH_PCIE_TX_MSG_PYLD_REG1, data);
	pcie_cluster_write(MNH_PCIE_TX_MSG_PYLD_REG0, vm);
	data = (MNH_PCIE_VM_TAG << 24) | (MNH_PCIE_VM_MSG_CODE_VD << 16)
		| MNH_PCIE_TX_MSG_REQ;
	pcie_cluster_write(MNH_PCIE_TX_MSG_REQ_REG, data);
	return 0;
}

static int pcie_send_ltr(uint32_t ltr)
{
	uint32_t data;

	force_link_up();
	pcie_cluster_write(MNH_PCIE_LTR_MSG_LATENCY_REG, ltr);
	data = pcie_cluster_read(MNH_PCIE_LTR_MSG_CTRL_REG) | MNH_LTR_TRIGGER;
	pcie_cluster_write(MNH_PCIE_LTR_MSG_CTRL_REG, data);
	return 0;
}

/* Interrupt Service routines */

static int handle_vm(int vm)
{
	struct mnh_pcie_irq inc;

	if (vm == 0) {
		if (pcie_cluster_read(MNH_PCIE_RX_VMSG0_ID) & MNH_VALID_VM) {
			vm = pcie_cluster_read(MNH_PCIE_RX_MSG0_PYLD_REG0);
			inc.vm = vm;
			dev_err(pcie_ep_device, "Vendor msg %lx rcvd\n", vm);
			irq_callback(&inc);
			pcie_cluster_write(MNH_PCIE_RX_VMSG0_ID, MNH_CLEAR_VM);
		}
	} else {
		if (pcie_cluster_read(MNH_PCIE_RX_VMSG1_ID) & MNH_VALID_VM) {
			vm = pcie_cluster_read(MNH_PCIE_RX_MSG1_PYLD_REG0);
			dev_err(pcie_ep_device, "Vendor msg %lx rcvd\n", vm);
			inc.vm = vm;
			irq_callback(&inc);
			pcie_cluster_write(MNH_PCIE_RX_VMSG1_ID, MNH_CLEAR_VM);
		}
	}
	return 0;
}

static void msi_rx_worker(struct work_struct *work)
{
	uint32_t apirq;
	struct mnh_pcie_irq inc;
	dev_err(pcie_ep_device, "AP IRQ routine called \n");
	
	apirq = pcie_cluster_read(MNH_PCIE_SW_INTR_TRIGG);
	if (apirq != 0) {
		dev_err(pcie_ep_device, "AP IRQ %lx received \n", apirq);
		if (irq_callback != NULL) {
			if (apirq & (0x1 << MSG_SEND_I)) {
				inc.msi_irq =  MSG_SEND_I;
				irq_callback(&inc);
			}
			if (apirq & (0x1 << DMA_DONE)) {
				inc.msi_irq =  DMA_DONE;
				irq_callback(&inc);
			}
		}
		/* Clear all interrupts */
		pcie_cluster_write(MNH_PCIE_SW_INTR_TRIGG,
			MNH_PCIE_SW_IRQ_CLEAR);
	}
}

static void pcie_irq_worker(struct work_struct *work)
{
	uint32_t pcieirq;

	dev_err(pcie_ep_device, "PCIE IRQ routine called \n");
	pcieirq = pcie_cluster_read(MNH_PCIE_SS_INTR_STS);
	if (pcieirq != 0) {
		if (pcieirq & MNH_PCIE_MSI_SENT) {
			dev_err(pcie_ep_device, "MSI Sent\n");
			release_link();
		}
		if (pcieirq & MNH_PCIE_VMSG_SENT) {
			dev_err(pcie_ep_device, "VM Sent\n");
			release_link();
		}
		if ((pcieirq & MNH_PCIE_VMSG1_RXD)
			& (irq_callback != NULL)) {
			dev_err(pcie_ep_device, "VM1 received\n");
			handle_vm(1);
		}
		if ((pcieirq & MNH_PCIE_VMSG0_RXD)
			& (irq_callback != NULL)) {
			dev_err(pcie_ep_device, "VM2 received\n");
			handle_vm(0);
		}
		if (pcieirq & MNH_PCIE_LINK_EQ_REQ_INT)
			dev_err(pcie_ep_device, "MNH_PCIE_LINK_EQ_REQ_INT received\n");
		if (pcieirq & MNH_PCIE_LINK_REQ_RST_NOT)
			dev_err(pcie_ep_device, "MNH_PCIE_LINK_REQ_RST_NOT received\n");
		if (pcieirq & MNH_PCIE_LTR_SENT)
			release_link();
		if (pcieirq & MNH_PCIE_COR_ERR)
			dev_err(pcie_ep_device, "MNH_PCIE_COR_ERR received\n");
		if (pcieirq & MNH_PCIE_NONFATAL_ERR)
			dev_err(pcie_ep_device, "MNH_PCIE_NONFATAL_ERR received\n");
		if (pcieirq & MNH_PCIE_FATAL_ERR)
			dev_err(pcie_ep_device, "MNH_PCIE_FATAL_ERR received\n");
		if (pcieirq & MNH_PCIE_RADM_MSG_UNLOCK)
			dev_err(pcie_ep_device, "MNH_PCIE_RADM_MSG_UNLOCK received\n");
		if (pcieirq & MNH_PCIE_PM_TURNOFF)
			dev_err(pcie_ep_device, "MNH_PCIE_PM_TURNOFF received\n");
		if (pcieirq & MNH_PCIE_RADM_CPL_TIMEOUT)
			dev_err(pcie_ep_device, "MNH_PCIE_RADM_CPL_TIMEOUT received\n");
		if (pcieirq & MNH_PCIE_TRGT_CPL_TIMEOUT)
			dev_err(pcie_ep_device, "MNH_PCIE_TRGT_CPL_TIMEOUT received\n");
	}

	/* Clear all interrupts */
	pcie_cluster_write(MNH_PCIE_SS_INTR_STS, 0xFFFF);

}

static void dma_irq_worker(struct work_struct *work)
{
	/* TODO need to investigate */
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
static irqreturn_t pcie_handle_dma_irq(int irq, void *dev_id)
{
	schedule_work(&dma_irq_work);
	/* return interrupt handled */
	return IRQ_HANDLED;
}


static int pcie_set_inbound_iatu(struct mnh_inb_window *inb)
{
	uint32_t data, upper, lower;

	if (inb->mode == BAR_MATCH) {
		if ((inb->region > 0xF) || (inb->bar > 5))
			return -EINVAL;
		data = MNH_IATU_INBOUND | inb->region;
		pcie_config_write(MNH_IATU_VIEWPORT, data);
		upper = UPPER(inb->target_mnh_address);
		lower = LOWER(inb->target_mnh_address);
		pcie_config_write(MNH_IATU_LWR_TARGET_ADDR, lower);
		pcie_config_write(MNH_IATU_UPPER_TARGET_ADDR, upper);
		data = MNH_IATU_MEM;
		pcie_config_write(MNH_IATU_REGION_CTRL_1, data);
		data = MNH_IATU_BAR_MODE | (inb->bar << 8);
		pcie_config_write(MNH_IATU_REGION_CTRL_2, data);
		while (pcie_config_read(MNH_IATU_REGION_CTRL_2) != data)
			udelay(1);
	} else {
		if ((inb->region > 0xF) ||
				(inb->target_mnh_address >
				MNH_PCIE_OUTBOUND_BASE) ||
				((inb->target_mnh_address
				+ inb->limit_pcie_address)
				> MNH_PCIE_OUTBOUND_BASE) ||
				(inb->memmode > 0xf))
			return -EINVAL; /* address out of range */
		data = MNH_IATU_INBOUND | inb->region;
		pcie_config_write(MNH_IATU_VIEWPORT, data);
		upper = UPPER(inb->base_pcie_address);
		lower = LOWER(inb->base_pcie_address);
		pcie_config_write(MNH_IATU_LWR_BASE_ADDR, lower);
		pcie_config_write(MNH_IATU_UPPER_BASE_ADDR, upper);
		data = inb->limit_pcie_address;
		pcie_config_write(MNH_IATU_LIMIT_ADDR, data);
		upper = UPPER(inb->target_mnh_address);
		lower = LOWER(inb->target_mnh_address);
		pcie_config_write(MNH_IATU_LWR_TARGET_ADDR, lower);
		pcie_config_write(MNH_IATU_UPPER_TARGET_ADDR, upper);
		pcie_config_write(MNH_IATU_REGION_CTRL_1, inb->memmode);
		pcie_config_write(MNH_IATU_REGION_CTRL_2, MNH_IATU_ENABLE);
		while (pcie_config_read(MNH_IATU_REGION_CTRL_2)
				!= MNH_IATU_ENABLE)
			udelay(1);
	}
	return 0;
}

static int pcie_set_outbound_iatu(struct mnh_outb_region *outb)
{
	uint32_t data, upper, lower;

	if ((outb->region > 0xF) || (outb->base_mnh_address
			> MNH_PCIE_OUTBOUND_BASE) ||
			((outb->base_mnh_address
			+ outb->limit_mnh_address)
			> MNH_PCIE_OUTBOUND_BASE))
		return -EINVAL; /* address out of range */
	data = MNH_IATU_OUTBOUND | outb->region;
	pcie_config_write(MNH_IATU_VIEWPORT, data);
	upper = UPPER(outb->base_mnh_address);
	lower = LOWER(outb->base_mnh_address);
	pcie_config_write(MNH_IATU_LWR_BASE_ADDR, lower);
	pcie_config_write(MNH_IATU_UPPER_BASE_ADDR, upper);
	data = outb->limit_mnh_address;
	pcie_config_write(MNH_IATU_LIMIT_ADDR, data);
	upper = UPPER(outb->target_pcie_address);
	lower = LOWER(outb->target_pcie_address);
	pcie_config_write(MNH_IATU_LWR_TARGET_ADDR, lower);
	pcie_config_write(MNH_IATU_UPPER_TARGET_ADDR, upper);
	pcie_config_write(MNH_IATU_REGION_CTRL_1, MNH_IATU_MEM);
	pcie_config_write(MNH_IATU_REGION_CTRL_2, MNH_IATU_ENABLE);
	while (pcie_config_read(MNH_IATU_REGION_CTRL_2) != MNH_IATU_ENABLE)
		udelay(1);
	return 0;
}

static int pcie_set_rb_base(uint32_t base)
{
	uint32_t msi = BOOTSTRAP_SET;

	pcie_cluster_write(MNH_PCIE_GP_1, base);
	pcie_send_msi(msi);
	return 0;
}

static int pcie_read_data(uint8_t *buff, uint32_t size, uint64_t adr)
{

	uint64_t addr = (uint64_t)pcie_outb_mem + adr;

	memcpy(buff, addr, size);
	dev_err(pcie_ep_device, "finished copy\n");
	return 0;
}

static int pcie_write_data(uint8_t *buff, uint32_t size, uint64_t adr)
{

	uint64_t addr = (uint64_t)pcie_outb_mem + adr;

	memcpy(addr, buff, size);
	dev_err(pcie_ep_device, "finished copy\n");
	return 0;
}

static int pcie_set_l_one(uint32_t enable, uint32_t clkpm)
{
	uint32_t data;

	if (clkpm == 1) {
		data = pcie_cluster_read(MNH_PCIE_APP_CTRL)
				| MNH_PCIE_APP_CLK_PM_EN;
		pcie_cluster_write(MNH_PCIE_APP_CTRL, data);
	} else {
		data = pcie_cluster_read(MNH_PCIE_APP_CTRL)
				& ~(MNH_PCIE_APP_CLK_PM_EN);
		pcie_cluster_write(MNH_PCIE_APP_CTRL, data);
	}
	if (enable == 1) {
		data = pcie_cluster_read(MNH_PCIE_APP_CTRL)
				| MNH_PCIE_APP_REQ_ENTRY_L1;
		pcie_cluster_write(MNH_PCIE_APP_CTRL, data);
	} else {
		data = pcie_cluster_read(MNH_PCIE_APP_CTRL)
				| MNH_PCIE_APP_REQ_EXIT_L1;
		pcie_cluster_write(MNH_PCIE_APP_CTRL, data);
	}

	return 0;
}

int pcie_sg_build(void *dmadest, size_t size, struct mnh_sg_entry *sg[],
						uint32_t maxsg)
{
	struct page *mypage;
	struct scatterlist *sc_list;
	struct scatterlist *in_sg;
	int i, fp_offset, count;
	int n_num, p_num = size/PAGE_SIZE;

	mypage = kcalloc(p_num, sizeof(struct page), GFP_KERNEL);
	sc_list = kcalloc(p_num, sizeof(struct scatterlist), GFP_KERNEL);
	fp_offset = (uint64_t) dmadest & PAGE_MASK;
	down_read(&current->mm->mmap_sem);
	n_num = get_user_pages(current, current->mm,
			(uint64_t) dmadest, p_num, 1, 1, &mypage, NULL);
	up_read(&current->mm->mmap_sem);
	if (n_num) {
		sg_init_table(sc_list, n_num);
		sg_set_page(&sc_list[0], &mypage[0],
					PAGE_SIZE - fp_offset, fp_offset);
		for (i = 1; i <= n_num-1; i++)
			sg_set_page(&sc_list[i], &mypage[i], PAGE_SIZE, 0);

		sg_set_page(&sc_list[n_num], &mypage[n_num],
			size - (PAGE_SIZE - fp_offset)
			- ((n_num-1)*PAGE_SIZE), 0);
		count = dma_map_sg(pcie_ep_device, sc_list,
				n_num, DMA_BIDIRECTIONAL);
		for_each_sg(sc_list, in_sg, count, i) {
			if (i < maxsg) {
			sg[i]->paddr = sg_dma_address(in_sg);
			sg[i]->size = sg_dma_len(in_sg);
			} else {
				dma_unmap_sg(pcie_ep_device,
					sc_list, n_num, DMA_BIDIRECTIONAL);
				kfree(mypage);
				kfree(sc_list);
				return -EINVAL;
			}

		}
		i++;
		sg[i]->paddr = NULL;
	} else {
		kfree(mypage);
		kfree(sc_list);
		return -EINVAL;
	}
	dma_map_sg(pcie_ep_device, sc_list, n_num, DMA_BIDIRECTIONAL);
	kfree(mypage);
	kfree(sc_list);
	return 0;
}

static int pcie_ll_build(struct mnh_sg_entry *src_sg[],
			struct mnh_sg_entry *dst_sg[],	uint64_t *start_addr)
{
	struct mnh_dma_ll_element **ll_element, **tmp_element;
	struct mnh_sg_entry sg_dst, sg_src;
	int i, s, u;

	ll_element = kcalloc(DMA_LL_LENGTH,
			sizeof(struct mnh_dma_ll_element), GFP_KERNEL);
	start_addr = ll_element;
	if (!ll_element)
		return -EINVAL;
	i = 0;
	s = 0;
	u = 0;
	sg_src = *src_sg[i];
	sg_dst = *dst_sg[s];
	while ((sg_src.paddr != NULL) && (sg_dst.paddr != NULL)) {
		if (sg_src.size == sg_dst.size) {
			ll_element[u]->header = LL_DATA_ELEMENT;
			ll_element[u]->size = sg_src.size;
			ll_element[u]->sar_low = LOWER(sg_src.paddr);
			ll_element[u]->sar_high = UPPER(sg_src.paddr);
			ll_element[u]->sar_low = LOWER(sg_dst.paddr);
			ll_element[u]->sar_high = UPPER(sg_dst.paddr);
			i++;
			s++;
			sg_src = *src_sg[i];
			sg_dst = *dst_sg[s];
		} else if (sg_src.size > sg_dst.size) {
			ll_element[u]->header = LL_DATA_ELEMENT;
			ll_element[u]->size = sg_dst.size;
			ll_element[u]->sar_low = LOWER(sg_src.paddr);
			ll_element[u]->sar_high = UPPER(sg_src.paddr);
			ll_element[u]->sar_low = LOWER(sg_dst.paddr);
			ll_element[u]->sar_high = UPPER(sg_dst.paddr);
			sg_src.paddr = sg_src.paddr + sg_dst.size;
			sg_src.size = sg_src.size - sg_dst.size;
			s++;
			sg_dst = *dst_sg[s];
		} else {

			ll_element[u]->size = sg_src.size;
			ll_element[u]->sar_low = LOWER(sg_src.paddr);
			ll_element[u]->sar_high = UPPER(sg_src.paddr);
			ll_element[u]->sar_low = LOWER(sg_dst.paddr);
			ll_element[u]->sar_high = UPPER(sg_dst.paddr);
			sg_dst.paddr = sg_dst.paddr + sg_src.size;
			sg_dst.size = sg_dst.size - sg_src.size;
			i++;
			sg_src = *src_sg[i];
		}
		u++;
		if (u == DMA_LL_LENGTH) {
			ll_element[u]->header = LL_LINK_ELEMENT;
			if ((sg_src.paddr == NULL) || (sg_dst.paddr == NULL)) {
				ll_element[u]->sar_low =
					LOWER((unsigned int) start_addr);
				ll_element[u]->sar_high =
					UPPER((unsigned int) start_addr);
				return 0;
			}
			tmp_element = kcalloc(DMA_LL_LENGTH,
					sizeof(struct mnh_dma_ll_element),
					GFP_KERNEL);
			if (!tmp_element) {
				ll_element[u]->sar_low =
					LOWER((unsigned int) start_addr);
				ll_element[u]->sar_high =
					UPPER((unsigned int) start_addr);
				pcie_ll_destroy(start_addr);
				return -EINVAL;
			}
			ll_element[u]->sar_low =
				LOWER((unsigned int) tmp_element);
			ll_element[u]->sar_high =
				UPPER((unsigned int) tmp_element);
			ll_element = tmp_element;
			u = 0;
		}
	}
	ll_element[u-1]->header = LL_IRQ_DATA_ELEMENT;
	ll_element[u]->header = LL_LAST_LINK_ELEMENT;
	ll_element[u]->sar_low = LOWER((unsigned int) start_addr);
	ll_element[u]->sar_high = UPPER((unsigned int) start_addr);
	return 0;
}

static int pcie_ll_destroy(uint64_t *start_addr)
{
	int i;
	struct mnh_dma_ll_element **ll_element, **tmp_element;

	ll_element = start_addr;

	i = 0;

	while (1) {
		if (ll_element[i]->header == LL_LINK_ELEMENT) {
			if (start_addr == (ll_element[i]->sar_low
				+ (ll_element[i]->sar_high << 32))) {
				kfree(ll_element);
				break;
			} else {
				tmp_element = ll_element[i]->sar_low
					+ (ll_element[i]->sar_high << 32);
				kfree(ll_element);
				ll_element = tmp_element;
				i = 0;
			}
		} else {
			i++;
		}
	}

	return 0;
}

/* APIs exposed to message passing protocol */

/* API to generate MSI to AP */
int mnh_send_msi(mnh_msi_msg_t msi)
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

int mnh_sg_build(void *dmadest, size_t size, struct mnh_sg_entry *sg[],
				uint32_t maxsg)
{
	return pcie_sg_build(dmadest, size, sg, maxsg);
}

EXPORT_SYMBOL(mnh_sg_build);

int mnh_ll_build(struct mnh_sg_entry *src_sg[], struct mnh_sg_entry *dst_sg[],
				uint64_t *start_addr)
{
	return pcie_ll_build(src_sg, dst_sg, start_addr);
}

EXPORT_SYMBOL(mnh_ll_build);

int mnh_ll_destroy(uint64_t *start_addr)
{
	return pcie_ll_destroy(start_addr);
}

EXPORT_SYMBOL(mnh_ll_destroy);

static int config_mem(struct platform_device *pdev)
{
	conf_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!conf_mem) {
		return -ENOMEM;
	}
	if (!request_mem_region(conf_mem->start, resource_size(conf_mem),
			pci_ep_dev->name)) {
		dev_err(&pdev->dev, "unable to request mem region\n");
		return -ENOMEM;
	}
	pcie_conf_mem = ioremap_nocache(conf_mem->start,
			resource_size(conf_mem));
	if (!pcie_conf_mem) {
		release_mem_region(conf_mem->start, resource_size(conf_mem));
		return -ENOMEM;
	}
	cluster_mem = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!cluster_mem) {
		iounmap(&pcie_conf_mem);
		release_mem_region(conf_mem->start, resource_size(conf_mem));
		return -ENOMEM;
	}
	if (!request_mem_region(cluster_mem->start, resource_size(cluster_mem),
			pci_ep_dev->name)) {
		dev_err(&pdev->dev, "unable to request mem region\n");
		iounmap(&pcie_conf_mem);
		release_mem_region(conf_mem->start, resource_size(conf_mem));
		return -ENOMEM;
	}
	pcie_clust_mem = ioremap_nocache(cluster_mem->start,
					resource_size(cluster_mem));
	if (!pcie_clust_mem) {
		dev_err(&pdev->dev, "unable to request mem region\n");
		iounmap(&pcie_conf_mem);
		release_mem_region(conf_mem->start, resource_size(conf_mem));
		release_mem_region(cluster_mem->start,
					resource_size(cluster_mem));
		return -ENOMEM;
	}
	outbound_mem = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	if (!outbound_mem) {
		iounmap(&pcie_clust_mem);
		iounmap(&pcie_conf_mem);
		release_mem_region(conf_mem->start, resource_size(conf_mem));
		release_mem_region(cluster_mem->start,
					resource_size(cluster_mem));
		return -ENOMEM;
	}
	if (!request_mem_region(outbound_mem->start, resource_size(outbound_mem),
				pci_ep_dev->name)) {
		dev_err(&pdev->dev, "unable to request mem region\n");
		iounmap(&pcie_clust_mem);
		iounmap(&pcie_conf_mem);
		release_mem_region(conf_mem->start, resource_size(conf_mem));
		release_mem_region(cluster_mem->start,
					resource_size(cluster_mem));
		return -ENOMEM;
		}
	pcie_outb_mem = ioremap(outbound_mem->start, 
				resource_size(outbound_mem));
	if (!pcie_outb_mem) {
		dev_err(&pdev->dev, "unable to request mem region\n");
		iounmap(&pcie_clust_mem);
		iounmap(&pcie_conf_mem);
		release_mem_region(conf_mem->start, resource_size(conf_mem));
		release_mem_region(cluster_mem->start,
					resource_size(cluster_mem));
		release_mem_region(outbound_mem->start,
					resource_size(outbound_mem));
		return -ENOMEM;
	}
	return 0;
}

static int clear_mem(void)
{
	iounmap(&pcie_conf_mem);
	iounmap(&pcie_clust_mem);
	iounmap(&pcie_outb_mem);
	release_mem_region(conf_mem->start, resource_size(conf_mem));
	release_mem_region(cluster_mem->start,
					resource_size(cluster_mem));
	release_mem_region(outbound_mem->start,
					resource_size(outbound_mem));
	return 0;
}

/* IRQ Callback function for testing purposes */

int test_callback(struct mnh_pcie_irq *irq)
{
	sysfs_irq = *irq;
	dev_err(pcie_ep_device, "PCIE MSI IRQ %lx PCIE IRQ %lx VM IRQ %lx",
		sysfs_irq.msi_irq, sysfs_irq.pcie_irq, sysfs_irq.vm);
	return 0;
}

int test_dma_callback(struct mnh_dma_irq *irq)
{
	/*TODO do something */
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
	mnh_send_msi((mnh_msi_msg_t) val);
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
	if (mnh_pcie_write(buf, count, rw_address_sysfs)) {
		dev_err(pcie_ep_device, "Write buffer failed\n");
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

	token = strsep(&buf, delim);
	if (token) {
		if (kstrtoul(token, 0, &val))
		return -EINVAL;
	if ((val < 0) || (val > 1))
		return -EINVAL;
	inb.mode = val;
	dev_err(pcie_ep_device, "Inbound mode is %lx\n", val);
	} else {
		return -EINVAL;
	}

	token = strsep(&buf, delim);

	if (token) {
		if (kstrtoul(token, 0, &val))
		return -EINVAL;
	if ((val < 0) || (val > 2))
		return -EINVAL;
	inb.bar = val;
	dev_err(pcie_ep_device, "Inbound bar is %lx\n", val);
	} else {
		return -EINVAL;
	}

	token = strsep(&buf, delim);

	if (token) {
		if (kstrtoul(token, 0, &val))
			return -EINVAL;
		if ((val < 0) || (val > 15))
			return -EINVAL;
		inb.region = val;
		dev_err(pcie_ep_device, "Inbound region is %lx\n", val);
	} else {
		return -EINVAL;
	}

	token = strsep(&buf, delim);

	if (token) {
		if (kstrtoul(token, 0, &val))
			return -EINVAL;
		if ((val < 0) || (val > 15))
			return -EINVAL;
		inb.memmode = val;
		dev_err(pcie_ep_device, "Inbound memmode is %lx\n", val);
	} else {
		return -EINVAL;
	}

	token = strsep(&buf, delim);

	if (token) {
		if (kstrtoul(token, 0, &val))
			return -EINVAL;
		inb.base_pcie_address = val;
		dev_err(pcie_ep_device, "Inbound base is %lx\n", val);
	} else {
		return -EINVAL;
	}

	token = strsep(&buf, delim);

	if (token) {
		if (kstrtoul(token, 0, &val))
			return -EINVAL;
		inb.limit_pcie_address = val;
		dev_err(pcie_ep_device, "Inbound limit is %lx\n", val);
	} else {
		return -EINVAL;
	}

	token = strsep(&buf, delim);

	if (token) {
		if (kstrtoul(token, 0, &val))
			return -EINVAL;
		inb.target_mnh_address = val;
		dev_err(pcie_ep_device, "Inbound target is %lx\n", val);
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

	token = strsep(&buf, delim);

	if (token) {
		if (kstrtoul(token, 0, &val))
			return -EINVAL;
		if ((val < 0) || (val > 32))
			return -EINVAL;
		outb.region = val;
		dev_err(pcie_ep_device, "Outbound region is %lx\n", val);
	} else {
		return -EINVAL;
	}

	token = strsep(&buf, delim);

	if (token) {
		if (kstrtoul(token, 0, &val))
			return -EINVAL;
		outb.base_mnh_address = val;
		dev_err(pcie_ep_device, "Outbound base is %lx\n", val);
	} else {
		return -EINVAL;
	}

	token = strsep(&buf, delim);

	if (token) {
		if (kstrtoul(token, 0, &val))
			return -EINVAL;
		outb.limit_mnh_address = val;
		dev_err(pcie_ep_device, "Outbound limit is %lx\n", val);
	} else {
		return -EINVAL;
	}

	token = strsep(&buf, delim);

	if (token) {
		if (kstrtoul(token, 0, &val))
			return -EINVAL;
		outb.target_pcie_address = val;
		dev_err(pcie_ep_device, "Outbound target is %lx\n", val);
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

	token = strsep(&buf, delim);

	if (token) {
		if (kstrtoul(token, 0, &val))
			return -EINVAL;
		if ((val < 0) || (val > 1))
			return -EINVAL;
		enable = val;
		dev_err(pcie_ep_device, "L1 state is %lx\n", val);
	} else {
		return -EINVAL;
	}

	token = strsep(&buf, delim);

	if (token) {
		if (kstrtoul(token, 0, &val))
			return -EINVAL;
		if ((val < 0) || (val > 1))
			return -EINVAL;
		pmclock = val;
		dev_err(pcie_ep_device, "PM Clock is %lx\n", val);
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

	ret = device_create_file(pcie_ep_device,
			&dev_attr_send_msi);
	if (ret) {
		dev_err(pcie_ep_device, "Failed to create sysfs: send_msi\n");
		return -EINVAL;
	}
	ret = device_create_file(pcie_ep_device,
			&dev_attr_send_vm);
	if (ret) {
		dev_err(pcie_ep_device, "Failed to create sysfs: send_vm\n");
		device_remove_file(pcie_ep_device,
			&dev_attr_send_msi);
		return -EINVAL;
	}
	ret = device_create_file(pcie_ep_device,
			&dev_attr_rb_base);
	if (ret) {
		dev_err(pcie_ep_device, "Failed to create sysfs: rb_base\n");
		device_remove_file(pcie_ep_device,
			&dev_attr_send_msi);
		device_remove_file(pcie_ep_device,
			&dev_attr_send_vm);
		return -EINVAL;
	}
	rw_address_sysfs = 0x0;
	ret = device_create_file(pcie_ep_device,
			&dev_attr_rw_address);
	if (ret) {
		dev_err(pcie_ep_device, "Failed to create sysfs: rw_address\n");
		device_remove_file(pcie_ep_device,
				&dev_attr_send_msi);
		device_remove_file(pcie_ep_device,
				&dev_attr_send_vm);
		device_remove_file(pcie_ep_device,
				&dev_attr_rb_base);
		return -EINVAL;
	}
	rw_size_sysfs = 0x1;
	ret = device_create_file(pcie_ep_device,
			&dev_attr_rw_size);
	if (ret) {
		dev_err(pcie_ep_device, "Failed to create sysfs: rw_size\n");
		device_remove_file(pcie_ep_device,
				&dev_attr_send_msi);
		device_remove_file(pcie_ep_device,
				&dev_attr_send_vm);
		device_remove_file(pcie_ep_device,
				&dev_attr_rb_base);
		device_remove_file(pcie_ep_device,
				&dev_attr_rw_address);
		return -EINVAL;
	}
	ret = device_create_file(pcie_ep_device,
			&dev_attr_rw_cluster);
	if (ret) {
		dev_err(pcie_ep_device, "Failed to create sysfs: rw_cluster\n");
		device_remove_file(pcie_ep_device,
				&dev_attr_send_msi);
		device_remove_file(pcie_ep_device,
				&dev_attr_send_vm);
		device_remove_file(pcie_ep_device,
				&dev_attr_rb_base);
		device_remove_file(pcie_ep_device,
				&dev_attr_rw_address);
		device_remove_file(pcie_ep_device,
				&dev_attr_rw_size);
		return -EINVAL;
	}
	ret = device_create_file(pcie_ep_device,
			&dev_attr_rw_config);
	if (ret) {
		dev_err(pcie_ep_device, "Failed to create sysfs: rw_config\n");
		device_remove_file(pcie_ep_device,
				&dev_attr_send_msi);
		device_remove_file(pcie_ep_device,
				&dev_attr_send_vm);
		device_remove_file(pcie_ep_device,
				&dev_attr_rb_base);
		device_remove_file(pcie_ep_device,
				&dev_attr_rw_address);
		device_remove_file(pcie_ep_device,
				&dev_attr_rw_size);
		device_remove_file(pcie_ep_device,
				&dev_attr_rw_cluster);
		return -EINVAL;
	}
	ret = device_create_file(pcie_ep_device,
			&dev_attr_rw_data);
	if (ret) {
		dev_err(pcie_ep_device, "Failed to create sysfs: rw_data\n");
				device_remove_file(pcie_ep_device,
				&dev_attr_send_msi);
		device_remove_file(pcie_ep_device,
				&dev_attr_send_vm);
		device_remove_file(pcie_ep_device,
				&dev_attr_rb_base);
		device_remove_file(pcie_ep_device,
				&dev_attr_rw_address);
		device_remove_file(pcie_ep_device,
				&dev_attr_rw_size);
		device_remove_file(pcie_ep_device,
				&dev_attr_rw_cluster);
		device_remove_file(pcie_ep_device,
				&dev_attr_rw_config);
		return -EINVAL;
	}
	ret = device_create_file(pcie_ep_device,
			&dev_attr_set_inbound);
	if (ret) {
		dev_err(pcie_ep_device, "Failed to create sysfs: set_inbound\n");
				device_remove_file(pcie_ep_device,
				&dev_attr_send_msi);
		device_remove_file(pcie_ep_device,
				&dev_attr_send_vm);
		device_remove_file(pcie_ep_device,
				&dev_attr_rb_base);
		device_remove_file(pcie_ep_device,
				&dev_attr_rw_address);
		device_remove_file(pcie_ep_device,
				&dev_attr_rw_size);
		device_remove_file(pcie_ep_device,
				&dev_attr_rw_cluster);
		device_remove_file(pcie_ep_device,
				&dev_attr_rw_config);
		device_remove_file(pcie_ep_device,
				&dev_attr_rw_data);
		return -EINVAL;
	}
	ret = device_create_file(pcie_ep_device,
			&dev_attr_set_outbound);
	if (ret) {
		dev_err(pcie_ep_device, "Failed to create sysfs: set_outbound\n");
				device_remove_file(pcie_ep_device,
				&dev_attr_send_msi);
		device_remove_file(pcie_ep_device,
				&dev_attr_send_vm);
		device_remove_file(pcie_ep_device,
				&dev_attr_rb_base);
		device_remove_file(pcie_ep_device,
				&dev_attr_rw_address);
		device_remove_file(pcie_ep_device,
				&dev_attr_rw_size);
		device_remove_file(pcie_ep_device,
				&dev_attr_rw_cluster);
		device_remove_file(pcie_ep_device,
				&dev_attr_rw_config);
		device_remove_file(pcie_ep_device,
				&dev_attr_rw_data);
		device_remove_file(pcie_ep_device,
				&dev_attr_set_inbound);
		return -EINVAL;
	}
	ret = device_create_file(pcie_ep_device,
			&dev_attr_test_callback);
	if (ret) {
		dev_err(pcie_ep_device, "Failed to create sysfs: test_callback\n");
				device_remove_file(pcie_ep_device,
				&dev_attr_send_msi);
		device_remove_file(pcie_ep_device,
				&dev_attr_send_vm);
		device_remove_file(pcie_ep_device,
				&dev_attr_rb_base);
		device_remove_file(pcie_ep_device,
				&dev_attr_rw_address);
		device_remove_file(pcie_ep_device,
				&dev_attr_rw_size);
		device_remove_file(pcie_ep_device,
				&dev_attr_rw_cluster);
		device_remove_file(pcie_ep_device,
				&dev_attr_rw_config);
		device_remove_file(pcie_ep_device,
				&dev_attr_rw_data);
		device_remove_file(pcie_ep_device,
				&dev_attr_set_inbound);
		device_remove_file(pcie_ep_device,
				&dev_attr_set_outbound);
		return -EINVAL;
	}
	ret = device_create_file(pcie_ep_device,
			&dev_attr_send_ltr);
	if (ret) {
		dev_err(pcie_ep_device, "Failed to create sysfs: send_ltr\n");
				device_remove_file(pcie_ep_device,
				&dev_attr_send_msi);
		device_remove_file(pcie_ep_device,
				&dev_attr_send_vm);
		device_remove_file(pcie_ep_device,
				&dev_attr_rb_base);
		device_remove_file(pcie_ep_device,
				&dev_attr_rw_address);
		device_remove_file(pcie_ep_device,
				&dev_attr_rw_size);
		device_remove_file(pcie_ep_device,
				&dev_attr_rw_cluster);
		device_remove_file(pcie_ep_device,
				&dev_attr_rw_config);
		device_remove_file(pcie_ep_device,
				&dev_attr_rw_data);
		device_remove_file(pcie_ep_device,
				&dev_attr_set_inbound);
		device_remove_file(pcie_ep_device,
				&dev_attr_set_outbound);
		device_remove_file(pcie_ep_device,
			&dev_attr_test_callback);
		return -EINVAL;
	}
	ret = device_create_file(pcie_ep_device,
			&dev_attr_set_lone);
	if (ret) {
		dev_err(pcie_ep_device, "Failed to create sysfs: set_lone\n");
				device_remove_file(pcie_ep_device,
				&dev_attr_send_msi);
		device_remove_file(pcie_ep_device,
				&dev_attr_send_vm);
		device_remove_file(pcie_ep_device,
				&dev_attr_rb_base);
		device_remove_file(pcie_ep_device,
				&dev_attr_rw_address);
		device_remove_file(pcie_ep_device,
				&dev_attr_rw_size);
		device_remove_file(pcie_ep_device,
				&dev_attr_rw_cluster);
		device_remove_file(pcie_ep_device,
				&dev_attr_rw_config);
		device_remove_file(pcie_ep_device,
				&dev_attr_rw_data);
		device_remove_file(pcie_ep_device,
				&dev_attr_set_inbound);
		device_remove_file(pcie_ep_device,
				&dev_attr_set_outbound);
		device_remove_file(pcie_ep_device,
			&dev_attr_test_callback);
		device_remove_file(pcie_ep_device,
			&dev_attr_send_ltr);
		return -EINVAL;
	}
	return 0;
}

static void clean_sysfs(void)
{
	device_remove_file(pcie_ep_device,
			&dev_attr_send_msi);
	device_remove_file(pcie_ep_device,
			&dev_attr_send_vm);
	device_remove_file(pcie_ep_device,
			&dev_attr_rb_base);
	device_remove_file(pcie_ep_device,
			&dev_attr_rw_address);
	device_remove_file(pcie_ep_device,
			&dev_attr_rw_size);
	device_remove_file(pcie_ep_device,
			&dev_attr_rw_cluster);
	device_remove_file(pcie_ep_device,
			&dev_attr_rw_config);
	device_remove_file(pcie_ep_device,
			&dev_attr_rw_data);
	device_remove_file(pcie_ep_device,
			&dev_attr_set_inbound);
	device_remove_file(pcie_ep_device,
			&dev_attr_set_outbound);
	device_remove_file(pcie_ep_device,
			&dev_attr_test_callback);
	device_remove_file(pcie_ep_device,
			&dev_attr_send_ltr);
	device_remove_file(pcie_ep_device,
			&dev_attr_set_lone);
}

static int mnh_pcie_ep_probe(struct platform_device *pdev)
{
	int err;

	pci_ep_dev = kzalloc(sizeof(*pci_ep_dev), GFP_KERNEL);
	if (!pci_ep_dev) {
		dev_err(&pdev->dev, "Could not allocated pcie_ep_dev\n");
		return -ENOMEM;
	}
	pci_ep_dev->dev = &pdev->dev;
	pcie_ep_device = kzalloc(sizeof(*pcie_ep_device), GFP_KERNEL);
	if (!pcie_ep_device) {
		dev_err(&pdev->dev, "Could not allocated pcie_ep_device\n");
		return -ENOMEM;
	}
	pcie_ep_device = &pdev->dev;
	strcpy(pci_ep_dev->name, DEVICE_NAME);
	irq_callback = NULL;
	dma_callback = NULL;
	err = config_mem(pdev);
	if (err)
		return err;

	/* Register IRQs */

	pcie_sw_irq = platform_get_irq(pdev, 0);

	err = request_irq(pcie_sw_irq, pcie_handle_sw_irq,
			IRQF_SHARED, DEVICE_NAME, pci_ep_dev);
	if (err) {
		clear_mem();
		return -EINVAL;
	}
	pcie_cluster_irq = platform_get_irq(pdev, 1);
	err = request_irq(pcie_cluster_irq, pcie_handle_cluster_irq,
			IRQF_SHARED, DEVICE_NAME, pcie_ep_device);
	if (err) {
		free_irq(pcie_sw_irq, pcie_ep_device);
		clear_mem();
		return -EINVAL;
	}


/* TODO: handle PCIe wake IRQ */

/* declare MSI worker */
	INIT_DELAYED_WORK(&msi_work, pcie_msi_worker);
	INIT_WORK(&msi_rx_work, msi_rx_worker);
	INIT_WORK(&pcie_irq_work, pcie_irq_worker);
	INIT_WORK(&dma_irq_work, dma_irq_worker);
	init_sysfs();
	/* pcie_link_init(); */
	pcie_cluster_write(MNH_PCIE_SS_INTR_EN, PCIE_SS_IRQ_MASK);
	return 0;
}

static int mnh_pcie_ep_remove(struct platform_device *pdev)
{
	cancel_delayed_work_sync(&msi_work);
	clean_sysfs();
	clear_mem();
	free_irq(pcie_sw_irq, pcie_ep_device);
	free_irq(pcie_cluster_irq, pcie_ep_device);
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
