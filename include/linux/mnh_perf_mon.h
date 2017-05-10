
/** @mainpage  MNH Performance Monitor driver
*
* @section intro Introduction
* TODO:
*
*
*
*/


/**
 * @file    mnh_perf_mon.h
 * @brief   Performance Monitor header includes function API and data structure
 * @author  Intel
 * @date    27 Apr 2016
 * @version 1.0
 */

#include <linux/types.h>
#ifndef __LINUX_MNH_PERF_MON_H
#define __LINUX_MNH_PERF_MON_H

/*****************************************************************************
 *
 *  Data structure used internal to driver
 *
 ****************************************************************************/

#define PRMU_COUNT	6

#define TIMESTAMP_OVFL	1
#define PRMU_ERR	2

#define READ_MODE	0
#define WRITE_MODE	1

#define GAMMA_AXI_SPEED	50000
#define DELTA_AXI_SPEED	6000

#define AXI_SPD_DIV	1000

# define PRMU_1_MASK	0x1
# define PRMU_2_MASK	0x2
# define PRMU_3_MASK	0x4
# define PRMU_4_MASK	0x8
# define PRMU_5_MASK	0x10
# define PRMU_6_MASK	0x20








struct mnh_perf_mon_device {
	struct device *dev;
	char name[64];
	char mode[PRMU_COUNT];
	uint32_t irq;
	void *mem;
	void *scu;
	uint32_t status;
	uint32_t error;
	uint64_t axi_speed;
	struct resource *memr;
	struct resource *scur;
};

struct prmu_results {
	uint32_t error;
	uint32_t mode;
	uint32_t rd_req_rdy_latency;
	uint32_t rd_req_rdy_latency_min;
	uint32_t rd_req_rdy_latency_max;
	uint32_t rd_req_rdy_latency_avg;
	uint32_t rd_req_rdy_min_lat_addr;
	uint32_t rd_req_rdy_max_lat_addr;
	uint32_t resp_rdy_latency;
	uint32_t resp_rdy_latency_min;
	uint32_t resp_rdy_latency_max;
	uint32_t resp_rdy_latency_avg;
	uint32_t req_resp_rdy_min_lat_addr;
	uint32_t req_resp_rdy_max_lat_addr;
	uint32_t rd_req_cnt;
	uint32_t rd_resp_cnt;
	uint32_t out_rd_resp_cnt;
	uint64_t rd_req_len_cnt;
	uint64_t rd_bandwidth;
	uint32_t wr_req_rdy_latency;
	uint32_t wr_req_rdy_latency_min;
	uint32_t wr_req_rdy_latency_max;
	uint32_t wr_req_rdy_latency_avg;
	uint32_t wr_req_rdy_min_lat_addr;
	uint32_t wr_req_rdy_max_lat_addr;
	uint32_t wr_req_cnt;
	uint32_t wr_resp_cnt;
	uint32_t wr_out_cnt;
	uint64_t wr_req_len_cnt;
	uint64_t wr_act_req_len_cnt;
	uint64_t wr_bandwidth;
};

struct perf_result {
	uint32_t error;
	uint64_t time_stamp;
	struct prmu_results prmu[6];
	/*struct prmu_results prmu2;
	struct prmu_results prmu3;
	struct prmu_results prmu4;
	struct prmu_results prmu5;
	struct prmu_results prmu6;
	*/
};

#define PMON_IN(reg)			HW_IN(perf_mon_dev->mem, PMON, reg)
#define PMON_INf(reg, fld)		HW_INf(perf_mon_dev->mem, PMON, reg,\
						fld)
#define PMON_OUT(reg, val)		HW_OUT(perf_mon_dev->mem, PMON, reg,\
						val)
#define PMON_OUTf(reg, fld, val)	HW_OUTf(perf_mon_dev->mem, PMON,\
							reg, fld, val)
#define PMON_MASK(reg, fld)		HWIO_PMON_##reg##_##fld##_FLDMASK

#define PRMU_IN(inst, reg)		HW_IN((perf_mon_dev->mem +\
					(0x100 * (inst))), PRMU, reg)
#define PRMU_INf(inst, reg, fld)	HW_INf((perf_mon_dev->mem +\
					(0x100 * (inst))), PRMU, reg, fld)
#define PRMU_INx(inst, ins, reg)	HW_INx((perf_mon_dev->mem +\
					(0x100 * (inst))), PRMU, reg, ins)
#define PRMU_PRTx(inst, ins, reg)	HW_PRTx((perf_mon_dev->mem +\
					(0x100 * (inst))), PRMU, reg, ins)
#define PRMU_INxf(inst, ins, reg, fld)	HW_INxf((perf_mon_dev->mem +\
					(0x100 * (inst))), PRMU, reg, ins, fld)
#define PRMU_OUT(inst, reg, val)	HW_OUT((perf_mon_dev->mem +\
					(0x100 * (inst))), PRMU, reg, val)
#define PRMU_PRT(inst, reg)		HW_PRT((perf_mon_dev->mem +\
					(0x100 * (inst))), PRMU, reg)
#define PRMU_OUTf(inst, reg, fld, val)	HW_OUTf((perf_mon_dev->mem +\
					(0x100 * (inst))), PRMU, reg, fld, val)
#define PRMU_MASK(reg, fld)		HWIO_PRMU_##reg##_##fld##_FLDMASK

#define SCU_INf(reg, fld)		HW_INf(perf_mon_dev->scu, SCU, reg,\
						fld)
#define SCU_OUTf(reg, fld, val)		HW_OUTf(perf_mon_dev->scu, SCU, reg,\
							fld, val)
#define SCU_INxf(reg, inst, fld)	HW_INxf(perf_mon_dev->scu, SCU, reg,\
						inst, fld)
#define SCU_OUTxf(reg, inst, fld, val)	HW_OUTxf(perf_mon_dev->scu, SCU, reg,\
							inst, fld, val)





/*****************************************************************************
 *
 *  Data structure defined for APIs
 *
 ****************************************************************************/



/*******************************************************************************
 *
 *  APIs exposed
 *
 ******************************************************************************/



#endif
