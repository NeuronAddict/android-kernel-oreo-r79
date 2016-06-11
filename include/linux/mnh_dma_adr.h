
/** @mainpage  MNH FPGA Address correction
*
* @section intro Introduction
* In MNH FPGA emulation the dma engines see a differnt
* Address map from the CPU
* These utilities correct that problem
* Specific functions are exposed through the API's.
*/


/**
 * @file    mnh_dma_adr.h
 * @brief   PCIe EP driver header includes data structures & definitions
 * @author  Intel
 * @date    27 Apr 2016
 * @version 1.0
 */

#include <linux/types.h>
#ifndef __LINUX_MNH_DMA_ADR_H
#define __LINUX_MNH_DMA_ADR_H

#define EMULATION

#define FPGA_DDR_BASE	0x40000000
#define CPU_DDR_BASE	0x60850000

#define FPGA_DEV_BASE	0x04000000
#define CPU_DEV_BASE	0x60042000


#ifdef EMULATION
#define FPGA_DEV_ADR(address) (((((void *) address)  - CPU_DEV_BASE) + FPGA_DEV_BASE))
#define FPGA_ADR(address) (((((void *) address)  - CPU_DDR_BASE) + FPGA_DDR_BASE))

#define CPU_ADR(address) (((((void *) address) - FPGA_DDR_BASE) + CPU_DDR_BASE))

#else

#define FPGA_ADR(address) ((phys_addr_t) address)
#define CPU_ADR(address) ((phys_addr_t) address)

#endif

#endif
