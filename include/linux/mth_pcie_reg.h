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

#ifndef __LINUX_MTH_PCIE_REG_H
#define __LINUX_MTH_PCIE_REG_H

#define MTH_PCIE_FPGA

#ifdef MTH_PCIE_FPGA

#define MTH_PCIE_CONFIG_BASE_ADDRESS		0x60040000
#define MTH_PCIE_CONFIG_END			0x60041FFF
#define MTH_PCIE_OUTBOUND_BASE			0x69000000
#define MTH_PCIE_OUTBOUND_END			0x6FFFFFFF
#define MTH_PCIE_CLUSTER_BASE			0x60102000
#define MTH_PCIE_CLUSTER_END			0x60102FFF
#define MTH_PCIE_PHY_BASE			0x600C2000

#else

#define MTH_PCIE_CONFIG_BASE_ADDRESS		0x00200000
#define MTH_PCIE_CONFIG_END			0x00200FFF
#define MTH_PCIE_OUTBOUND_BASE			0x80000000
#define MTH_PCIE_OUTBOUND_END			0xFFFFFFFF
#define MTH_PCIE_CLUSTER_BASE			0x040C0000
#define MTH_PCIE_CLUSTER_END			0x040C0FFF
#define MTH_PCIE_PHY_BASE			0x04080000

#endif

/* PCIE Cluster Registers */

#define MTH_PCIE_SW_INTR_TRIGG			0x0
#define MTH_PCIE_SW_INTR_EN			0x4
#define MTH_PCIE_SS_INTR_STS			0x8
#define MTH_PCIE_SS_INTR_EN			0xC
#define MTH_PCIE_APP_CTRL			0x10
#define MTH_PCIE_APP_STS_REG			0x14
#define MTH_PCIE_MSI_TRIG_REG			0x18
#define MTH_PCIE_MSI_PEND_REG			0x1C
#define MTH_PCIE_CURR_LTR_STS_REG		0x20
#define MTH_PCIE_LTR_MSG_LATENCY_REG		0x24
#define MTH_PCIE_LTR_MSG_CTRL_REG		0x28
#define MTH_PCIE_RX_VMSG0_ID			0x30
#define MTH_PCIE_RX_MSG0_PYLD_REG1		0x34
#define MTH_PCIE_RX_MSG0_PYLD_REG0		0x38
#define MTH_PCIE_RX_VMSG1_ID			0x40
#define MTH_PCIE_RX_MSG1_PYLD_REG1		0x44
#define MTH_PCIE_RX_MSG1_PYLD_REG0		0x48
#define MTH_PCIE_TX_MSG_PYLD_REG1		0x50
#define MTH_PCIE_TX_MSG_PYLD_REG0		0x54
#define MTH_PCIE_TX_MSG_FIELD_REG		0x58
#define MTH_PCIE_TX_MSG_REQ_REG			0x5C
#define MTH_PCIE_CORE_DEBUG_REG1		0x60
#define MTH_PCIE_CORE_DEBUG_REG0		0x64
#define MTH_PCIE_CORE_EIDEBUG_REG0		0x6C
#define MTH_PCIE_GP_0				0x80
#define MTH_PCIE_GP_1				0x84
#define MTH_PCIE_GP_2				0x88
#define MTH_PCIE_GP_3				0x8C
#define MTH_PCIE_GP_4				0x90
#define MTH_PCIE_GP_5				0x94
#define MTH_PCIE_GP_6				0x98
#define MTH_PCIE_GP_7				0x9C

/* PCIe config registers */

/* IATU Registers */
#define MTH_IATU_VIEWPORT			0x900
#define MTH_IATU_LWR_BASE_ADDR			0x90C
#define MTH_IATU_UPPER_BASE_ADDR		0x910
#define MTH_IATU_LIMIT_ADDR			0x914
#define MTH_IATU_LWR_TARGET_ADDR		0x918
#define MTH_IATU_UPPER_TARGET_ADDR		0x91C
#define	MTH_IATU_REGION_CTRL_1			0x904
#define MTH_IATU_REGION_CTRL_2			0x908


#define MTH_STATUS_COMMAND_REG			0x4

#define MTH_PCI_MSI_CAP_ID_NEXT_CTRL		0x50
#define MTH_MSI_CAP_OFF_0CH_REG			0x5c
#define MTH_MSI_CAP_OFF_8H_REG			0x58
#define MTH_MSI_CAP_OFF_10H_REG			0x60

/* PCIE Definitions */

#define MTH_PCIE_APP_LTSSM_ENABLE		0x1
#define MTH_PCIE_APP_REQ_ENTRY_L1		0x80
#define MTH_PCIE_APP_REQ_EXIT_L1		0x200
#define	MTH_PCIE_APP_XFER_PEND			0x400
#define MTH_PCIE_APP_CLK_PM_EN			0x800

#define MTH_PCIE_SW_IRQ_CLEAR			0x7FFF

#define MTH_PCIE_VM_TAG				0x0000
#define MTH_PCIE_VM_MSG_CODE_VD			0x7F

#define MTH_PCIE_TX_MSG_REQ			0x1

#define MTH_PCIE_PHY_SRAM_LD_DONE		0x100000
#define MTH_PCIE_PHY_SRAM_INIT_DONE		0x80000000

#define MTH_IATU_IO				0x02
#define MTH_IATU_ENABLE				0x80000000
#define MTH_IATU_OUTBOUND			0x0
#define MTH_IATU_INBOUND			0x80000000
#define MTH_IATU_MEM				0x0
#define MTH_IATU_BAR_MODE			0xC0000000

#define MTH_PCIE_LINK_MASK			0x7
#define MTH_PCI_MSI_64_BIT_ADDR_CAP		0x800000

/* PCIE Cluster IRQs */

#define MTH_PCIE_LINK_EQ_REQ_INT		0x1
#define MTH_PCIE_LINK_REQ_RST_NOT		0x2
#define MTH_PCIE_MSI_SENT			0x4
#define MTH_PCIE_VMSG_SENT			0x8
#define MTH_PCIE_VMSG1_RXD			0x10
#define MTH_PCIE_VMSG0_RXD			0x20
#define MTH_PCIE_LTR_SENT			0x40
#define MTH_PCIE_COR_ERR			0x1000
#define MTH_PCIE_NONFATAL_ERR			0x2000
#define MTH_PCIE_FATAL_ERR			0x4000
#define MTH_PCIE_RADM_MSG_UNLOCK		0x10000
#define MTH_PCIE_PM_TURNOFF			0x20000
#define MTH_PCIE_RADM_CPL_TIMEOUT		0x40000
#define MTH_PCIE_TRGT_CPL_TIMEOUT		0x80000

/* PCIE System interrupt */

#ifdef MTH_PCIE_FPGA

#define MTH_IRQ_PCIE_DMA_WRITE_0		25
#define MTH_IRQ_PCIE_DMA_WRITE_1		25
#define MTH_IRQ_PCIE_DMA_WRITE_2		25
#define MTH_IRQ_PCIE_DMA_WRITE_3		25
#define MTH_IRQ_PCIE_DMA_READ_0			25
#define MTH_IRQ_PCIE_DMA_READ_1			25
#define MTH_IRQ_PCIE_DMA_READ_2			25
#define MTH_IRQ_PCIE_DMA_READ_3			25
#define MTH_IRQ_PCIE_SW				25
#define MTH_IRQ_PCIE_CLUSTER			25
#define MTH_IRQ_PCIE_WAKE			25

#else

#define MTH_IRQ_PCIE_DMA_WRITE_0		17
#define MTH_IRQ_PCIE_DMA_WRITE_1		18
#define MTH_IRQ_PCIE_DMA_WRITE_2		19
#define MTH_IRQ_PCIE_DMA_WRITE_3		20
#define MTH_IRQ_PCIE_DMA_READ_0			21
#define MTH_IRQ_PCIE_DMA_READ_1			22
#define MTH_IRQ_PCIE_DMA_READ_2			23
#define MTH_IRQ_PCIE_DMA_READ_3			24
#define MTH_IRQ_PCIE_SW				25
#define MTH_IRQ_PCIE_CLUSTER			26
#define MTH_IRQ_PCIE_WAKE			27

#endif

/* VM handling */
#define MTH_VALID_VM				0x100
#define MTH_CLEAR_VM				0x100

/* LTR Management */

#define MTH_LTR_TRIGGER				0x01

/* MSI Management */
#define MTH_TC0					0x0
#define MTH_TRIGGER_MSI				0x01

#endif
