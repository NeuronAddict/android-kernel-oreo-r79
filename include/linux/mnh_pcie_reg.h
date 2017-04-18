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

/* PCIE address regions */

#ifndef __LINUX_MNH_PCIE_REG_H
#define __LINUX_MNH_PCIE_REG_H

#define MNH_PCIE_FPGA

#ifdef MNH_PCIE_FPGA

#define MNH_PCIE_CONFIG_BASE_ADDRESS		0x60040000
#define MNH_PCIE_CONFIG_END			0x60041FFF
#define MNH_PCIE_OUTBOUND_BASE			0x69000000
#define MNH_PCIE_OUTBOUND_END			0x6FFFFFFF
#define MNH_PCIE_CLUSTER_BASE			0x60102000
#define MNH_PCIE_CLUSTER_END			0x60102FFF
#define MNH_PCIE_PHY_BASE			0x600C2000

#else

#define MNH_PCIE_CONFIG_BASE_ADDRESS		0x00200000
#define MNH_PCIE_CONFIG_END			0x00200FFF
#define MNH_PCIE_OUTBOUND_BASE			0x80000000
#define MNH_PCIE_OUTBOUND_END			0xFFFFFFFF
#define MNH_PCIE_CLUSTER_BASE			0x040C0000
#define MNH_PCIE_CLUSTER_END			0x040C0FFF
#define MNH_PCIE_PHY_BASE			0x04080000

#endif

/* PCIE Cluster Registers */

#define MNH_PCIE_SW_INTR_TRIGG			0x0
#define MNH_PCIE_SW_INTR_EN			0x4
#define MNH_PCIE_SS_INTR_STS			0x8
#define MNH_PCIE_SS_INTR_EN			0xC
#define MNH_PCIE_APP_CTRL			0x10
#define MNH_PCIE_APP_STS_REG			0x14
#define MNH_PCIE_MSI_TRIG_REG			0x18
#define MNH_PCIE_MSI_PEND_REG			0x1C
#define MNH_PCIE_CURR_LTR_STS_REG		0x20
#define MNH_PCIE_LTR_MSG_LATENCY_REG		0x24
#define MNH_PCIE_LTR_MSG_CTRL_REG		0x28
#define MNH_PCIE_RX_VMSG0_ID			0x30
#define MNH_PCIE_RX_MSG0_PYLD_REG1		0x34
#define MNH_PCIE_RX_MSG0_PYLD_REG0		0x38
#define MNH_PCIE_RX_VMSG1_ID			0x40
#define MNH_PCIE_RX_MSG1_PYLD_REG1		0x44
#define MNH_PCIE_RX_MSG1_PYLD_REG0		0x48
#define MNH_PCIE_TX_MSG_PYLD_REG1		0x50
#define MNH_PCIE_TX_MSG_PYLD_REG0		0x54
#define MNH_PCIE_TX_MSG_FIELD_REG		0x58
#define MNH_PCIE_TX_MSG_REQ_REG			0x5C
#define MNH_PCIE_CORE_DEBUG_REG1		0x60
#define MNH_PCIE_CORE_DEBUG_REG0		0x64
#define MNH_PCIE_CORE_EIDEBUG_REG0		0x6C
#define MNH_PCIE_GP_0				0x80
#define MNH_PCIE_GP_1				0x84
#define MNH_PCIE_GP_2				0x88
#define MNH_PCIE_GP_3				0x8C
#define MNH_PCIE_GP_4				0x90
#define MNH_PCIE_GP_5				0x94
#define MNH_PCIE_GP_6				0x98
#define MNH_PCIE_GP_7				0x9C

/* PCIe config registers */

/* IATU Registers */
#define MNH_IATU_VIEWPORT			0x900
#define MNH_IATU_LWR_BASE_ADDR			0x90C
#define MNH_IATU_UPPER_BASE_ADDR		0x910
#define MNH_IATU_LIMIT_ADDR			0x914
#define MNH_IATU_LWR_TARGET_ADDR		0x918
#define MNH_IATU_UPPER_TARGET_ADDR		0x91C
#define	MNH_IATU_REGION_CTRL_1			0x904
#define MNH_IATU_REGION_CTRL_2			0x908


#define MNH_STATUS_COMMAND_REG			0x4

#define MNH_PCI_MSI_CAP_ID_NEXT_CTRL		0x50
#define MNH_MSI_CAP_OFF_0CH_REG			0x5c
#define MNH_MSI_CAP_OFF_8H_REG			0x58
#define MNH_MSI_CAP_OFF_10H_REG			0x60

/* PCIE Definitions */

#define MNH_PCIE_APP_LTSSM_ENABLE		0x1
#define MNH_PCIE_APP_REQ_ENTRY_L1		0x80
#define MNH_PCIE_APP_REQ_EXIT_L1		0x200
#define	MNH_PCIE_APP_XFER_PEND			0x400
#define MNH_PCIE_APP_CLK_PM_EN			0x800

#define MNH_PCIE_SW_IRQ_CLEAR			0x7FFFFFFF

#define MNH_PCIE_VM_TAG				0x0000
#define MNH_PCIE_VM_MSG_CODE_VD			0x7F

#define MNH_PCIE_TX_MSG_REQ			0x1

#define MNH_PCIE_PHY_SRAM_LD_DONE		0x100000
#define MNH_PCIE_PHY_SRAM_INIT_DONE		0x80000000

#define MNH_IATU_IO				0x02
#define MNH_IATU_ENABLE				0x80000000
#define MNH_IATU_OUTBOUND			0x0
#define MNH_IATU_INBOUND			0x80000000
#define MNH_IATU_MEM				0x0
#define MNH_IATU_BAR_MODE			0xC0000000

#define MNH_PCIE_LINK_MASK			0x7
#define MNH_PCI_MSI_64_BIT_ADDR_CAP		0x800000

/* PCIE Cluster IRQs */

#define MNH_PCIE_LINK_EQ_REQ_INT		0x1
#define MNH_PCIE_LINK_REQ_RST_NOT		0x2
#define MNH_PCIE_MSI_SENT			0x4
#define MNH_PCIE_VMSG_SENT			0x8
#define MNH_PCIE_VMSG1_RXD			0x10
#define MNH_PCIE_VMSG0_RXD			0x20
#define MNH_PCIE_LTR_SENT			0x40
#define MNH_PCIE_COR_ERR			0x1000
#define MNH_PCIE_NONFATAL_ERR			0x2000
#define MNH_PCIE_FATAL_ERR			0x4000
#define MNH_PCIE_RADM_MSG_UNLOCK		0x10000
#define MNH_PCIE_PM_TURNOFF			0x20000
#define MNH_PCIE_RADM_CPL_TIMEOUT		0x40000
#define MNH_PCIE_TRGT_CPL_TIMEOUT		0x80000

#define HWIO_PCIE_EP_TYPE0_HDR_BASE_ADDR	0x0
#define HWIO_PCIE_EP_SPCIE_CAP_BASE_ADDR	0x148
#define HWIO_PCIE_EP_L1SUB_CAP_BASE_ADDR	0x170
#define HWIO_PCIE_EP_PM_CAP_BASE_ADDR		0x40
#define HWIO_PCIE_EP_PCIE_CAP_BASE_ADDR		0x70
#define HWIO_PCIE_EP_AER_CAP_BASE_ADDR		0x100
#define HWIO_PCIE_EP_MSI_CAP_BASE_ADDR		0x50
#define HWIO_PCIE_EP_LTR_CAP_BASE_ADDR		0x168
#define HWIO_PCIE_EP_PORT_LOGIC_BASE_ADDR	0x700



/* PCIE System interrupt */

#ifdef MNH_PCIE_FPGA

#define MNH_IRQ_PCIE_DMA_WRITE_0		25
#define MNH_IRQ_PCIE_DMA_WRITE_1		25
#define MNH_IRQ_PCIE_DMA_WRITE_2		25
#define MNH_IRQ_PCIE_DMA_WRITE_3		25
#define MNH_IRQ_PCIE_DMA_READ_0			25
#define MNH_IRQ_PCIE_DMA_READ_1			25
#define MNH_IRQ_PCIE_DMA_READ_2			25
#define MNH_IRQ_PCIE_DMA_READ_3			25
#define MNH_IRQ_PCIE_SW				25
#define MNH_IRQ_PCIE_CLUSTER			25
#define MNH_IRQ_PCIE_WAKE			25

#else

#define MNH_IRQ_PCIE_DMA_WRITE_0		17
#define MNH_IRQ_PCIE_DMA_WRITE_1		18
#define MNH_IRQ_PCIE_DMA_WRITE_2		19
#define MNH_IRQ_PCIE_DMA_WRITE_3		20
#define MNH_IRQ_PCIE_DMA_READ_0			21
#define MNH_IRQ_PCIE_DMA_READ_1			22
#define MNH_IRQ_PCIE_DMA_READ_2			23
#define MNH_IRQ_PCIE_DMA_READ_3			24
#define MNH_IRQ_PCIE_SW				25
#define MNH_IRQ_PCIE_CLUSTER			26
#define MNH_IRQ_PCIE_WAKE			27

#endif

/* VM handling */
#define MNH_VALID_VM				0x100
#define MNH_CLEAR_VM				0x100

/* LTR Management */

#define MNH_LTR_TRIGGER				0x01

/* MSI Management */
#define MNH_TC0					0x0
#define MNH_TRIGGER_MSI				0x01

/*HW IO macros */

#define TYPE0_AD (pcie_ep_dev->conf_mem + HWIO_PCIE_EP_TYPE0_HDR_BASE_ADDR)

#define TYPE0_IN(reg)			HW_IN(TYPE0_AD, PCIE_EP, reg)
#define TYPE0_INf(reg, fld)		HW_INf(TYPE0_AD, PCIE_EP, reg, fld)
#define TYPE0_OUT(reg, val)		HW_OUT(TYPE0_AD, PCIE_EP, reg, val)
#define TYPE0_OUTf(reg, fld, val)	HW_OUTf(TYPE0_AD, PCIE_EP,\
							reg, fld, val)
#define TYPE0_SD (pcie_ep_dev->conf_mem + 0x1000 +\
					HWIO_PCIE_EP_TYPE0_HDR_BASE_ADDR)

#define TYPE0S_IN(reg)			HW_IN(TYPE0_SD, PCIE_EP, reg)
#define TYPE0S_INf(reg, fld)		HW_INf(TYPE0_SD, PCIE_EP, reg, fld)
#define TYPE0S_OUT(reg, val)		HW_OUT(TYPE0_SD, PCIE_EP, reg, val)
#define TYPE0S_OUTf(reg, fld, val)	HW_OUTf(TYPE0_SD, PCIE_EP,\
								reg, fld, val)


#define TYPE0_MASK(reg, fld)		HWIO_PCIE_EP_##reg##_##fld##_FLDMASK

#define MSIAD (pcie_ep_dev->conf_mem + HWIO_PCIE_EP_MSI_CAP_BASE_ADDR)

#define MSICAP_IN(reg)			HW_IN(MSIAD, PCIE_EP, reg)
#define MSICAP_INf(reg, fld)		HW_INf(MSIAD, PCIE_EP, reg, fld)
#define MSICAP_OUT(reg, val)		HW_OUT(MSIAD, PCIE_EP, reg, val)
#define MSICAP_OUTf(reg, fld, val)	HW_OUTf(MSIAD, PCIE_EP, reg, fld, val)

#define MSICAP_MASK(reg, fld)		HWIO_PCIE_EP_##reg##_##fld##_FLDMASK

#define PCIEAD (pcie_ep_dev->conf_mem + HWIO_PCIE_EP_PCIE_CAP_BASE_ADDR)

#define PCIECAP_IN(reg)		HW_IN(PCIEAD, PCIE_EP, reg)
#define PCIECAP_INf(reg, fld)		HW_INf(PCIEAD, PCIE_EP, reg, fld)
#define PCIECAP_OUT(reg, val)		HW_OUT(PCIEAD, PCIE_EP, reg, val)
#define PCIECAP_OUTf(reg, fld, val)	HW_OUTf(PCIEAD, PCIE_EP, reg, fld, val)

#define PCIECAP_MASK(reg, fld)		HWIO_PCIE_EP_##reg##_##fld##_FLDMASK

#define PCIE_L1SUB (pcie_ep_dev->conf_mem + HWIO_PCIE_EP_L1SUB_CAP_BASE_ADDR)

#define PCIECAP_L1SUB_IN(reg)			HW_IN(PCIE_L1SUB, PCIE_EP, reg)
#define PCIECAP_L1SUB_INf(reg, fld)		HW_INf(PCIE_L1SUB, PCIE_EP, reg, fld)
#define PCIECAP_L1SUB_OUT(reg, val)		HW_OUT(PCIE_L1SUB, PCIE_EP, reg, val)
#define PCIECAP_L1SUB_OUTf(reg, fld, val)	HW_OUTf(PCIE_L1SUB, PCIE_EP, reg, fld, val)

#define PORT_AD (pcie_ep_dev->conf_mem + HWIO_PCIE_EP_PORT_LOGIC_BASE_ADDR)

#define PORT_IN(reg)			HW_IN(PORT_AD, PCIE_EP, reg)
#define PORT_INf(reg, fld)		HW_INf(PORT_AD, PCIE_EP, reg, fld)
#define PORT_OUT(reg, val)		HW_OUT(PORT_AD, PCIE_EP, reg, val)
#define PORT_OUTf(reg, fld, val)	HW_OUTf(PORT_AD, PCIE_EP, reg, fld, val)

#define PORT_MASK(reg, fld)		HWIO_PCIE_EP_##reg##_##fld##_FLDMASK

#define CSR_AD pcie_ep_dev->clust_mem

#define CSR_IN(reg)			HW_IN(CSR_AD,   PCIE_SS, reg)
#define CSR_INx(reg, inst)		HW_INx(CSR_AD,  PCIE_SS, reg, inst)
#define CSR_INf(reg, fld)		HW_INf(CSR_AD,  PCIE_SS, reg, fld)
#define CSR_OUT(reg, val)		HW_OUT(CSR_AD,  PCIE_SS, reg, val)
#define CSR_OUTx(reg, inst, val)	HW_OUTx(CSR_AD,  PCIE_SS,\
						reg, inst, val)
#define CSR_OUTf(reg, fld, val)		HW_OUTf(CSR_AD, PCIE_SS, reg, fld, val)

#define CSR_MASK(reg, fld)		HWIO_PCIE_SS_##reg##_##fld##_FLDMASK

#define SCUS_IN(reg, fld)		HW_IN(pcie_ep_dev->scu, SCU, reg)
#define SCUS_INx(reg, inst)		HW_INx(pcie_ep_dev->scu, SCU, reg, inst)
#define SCUS_OUT(reg, fld, val)		HW_OUT(pcie_ep_dev->scu, SCU,\
							reg, val)
#define SCUS_INf(reg, fld)		HW_INf(pcie_ep_dev->scu, SCU, reg,\
						fld)
#define SCUS_OUTf(reg, fld, val)		HW_OUTf(pcie_ep_dev->scu, SCU,\
							reg, fld, val)
#define SCUS_OUTx(reg, inst, val)		HW_OUTx(pcie_ep_dev->scu, SCU,\
							reg, inst, val)


#define MNH_BAD_ADDR  ((void *)0xFFFFFFFF)


#endif
