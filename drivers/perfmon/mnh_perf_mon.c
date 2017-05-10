/*
 * perf_mon.c - Monhette Hill Performance Monitor driver
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
#include <linux/of.h>
#include <linux/uaccess.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/mnh_perf_mon.h>
#include <linux/mnh_pcie_str.h>
#include <linux/intel-hwio.h>
#include <soc/mnh/mnh-hwio-pmon.h>
#include <soc/mnh/mnh-hwio-prmu.h>
#include <soc/mnh/mnh-hwio-scu.h>

#define DEVICE_NAME "mnh_perf_mon"
#define MAX_STR_COPY	4096
#define EMULATION	1

struct mnh_perf_mon_device *perf_mon_dev;
static uint32_t perfmon_stop_free_test(void);
static void init_perfmon(void);

static uint32_t read_emulation_setting(void)
{
	uint32_t val;
	struct device_node *node =
		of_find_node_by_name(NULL, "chosen");

	if (node && !of_property_read_u32(node, "emulation", &val))
		return val;
	else
		return 0;
}



static void set_axi_clk(void)
{
	int fsp, fbdiv, sys200, axi_clk_div, ref_clk, axi_clk;


	if (read_emulation_setting() == 1)
		perf_mon_dev->axi_speed = GAMMA_AXI_SPEED;
	else if (read_emulation_setting() == 2)
		perf_mon_dev->axi_speed = DELTA_AXI_SPEED;
	else {
		fsp = SCU_INf(LPDDR4_LOW_POWER_STS, LPDDR4_CUR_FSP);
		switch (fsp) {

		case 0x0:
		case 0x1:
		case 0x2:
		case 0x3:
			/* read from LPDDR4_FSP0_SETTINGS */
			fbdiv = SCU_INxf(LPDDR4_FSP_SETTING, fsp, FSP_FBDIV);
			sys200 = SCU_INxf(LPDDR4_FSP_SETTING, fsp, FSP_SYS200_MODE);
			axi_clk_div = SCU_INxf(LPDDR4_FSP_SETTING, fsp,
						FSP_AXI_FABRIC_CLK_DIV);
			break;
		case 0x7:
			fbdiv = SCU_INf(LPDDR4_REFCLK_PLL_INTGR_DIV, FBDIV);
			sys200 = SCU_INf(CCU_CLK_CTL, LP4_AXI_SYS200_MODE);
			axi_clk_div = SCU_INf(CCU_CLK_DIV, AXI_FABRIC_CLK_DIV);
			break;
		default:
			sys200 = 1;
			axi_clk_div = 1;
			fbdiv = 1;
			break;
		}
		ref_clk = SCU_INf(HW_STRAP, REF_CLK_SEL) ? 24000000 : 19200000;
		if (sys200 == 1)
			axi_clk = (200000000) / (axi_clk_div + 1);
		else
			axi_clk = (ref_clk * (fbdiv / 2)) / (axi_clk_div + 1);

		perf_mon_dev->axi_speed = axi_clk / AXI_SPD_DIV;
	}
}


static uint32_t config_prmu(void)
{
	int i = 0;
	while (i < PRMU_COUNT) {

		if (perf_mon_dev->mode[i] == 'r') {
			PRMU_OUTf(i+1, READ_CTRL, RD_REQ2RDY_EN, 1);
			PRMU_OUTf(i+1, READ_CTRL, RD_REQ2RESP_EN, 1);
			PRMU_OUTf(i+1, READ_CTRL, RD_REQ_CNT_EN, 1);
			PRMU_OUTf(i+1, READ_CTRL, RD_RESP_CNT_EN, 1);
			PRMU_OUTf(i+1, READ_CTRL,
					RD_REQ_TOTAL_BYTE_CNT_EN, 1);

			PRMU_OUTf(i+1, WRITE_CTRL, WR_REQ2RDY_EN, 0);
			PRMU_OUTf(i+1, WRITE_CTRL, WR_REQ2RESP_EN, 0);
			PRMU_OUTf(i+1, WRITE_CTRL, WR_REQ_CNT_EN, 0);
			PRMU_OUTf(i+1, WRITE_CTRL, WR_RESP_CNT_EN, 0);
			PRMU_OUTf(i+1, WRITE_CTRL,
					WR_REQ_TOTAL_BYTE_CNT_EN, 0);

			PRMU_OUTf(i+1, READ_CTRL, RD_REQ2RDY_CLR, 1);
			PRMU_OUTf(i+1, READ_CTRL, RD_REQ2RESP_CLR, 1);
			PRMU_OUTf(i+1, READ_CTRL, RD_REQ_CNT_CLR, 1);
			PRMU_OUTf(i+1, READ_CTRL, RD_RESP_CNT_CLR, 1);
			PRMU_OUTf(i+1, READ_CTRL,
					RD_REQ_TOTAL_BYTE_CNT_CLR, 1);

			PRMU_OUTf(i+1, READ_CTRL, RD_REQ2RDY_CLR, 0);
			PRMU_OUTf(i+1, READ_CTRL, RD_REQ2RESP_CLR, 0);
			PRMU_OUTf(i+1, READ_CTRL, RD_REQ_CNT_CLR, 0);
			PRMU_OUTf(i+1, READ_CTRL, RD_RESP_CNT_CLR, 0);
			PRMU_OUTf(i+1, READ_CTRL,
					RD_REQ_TOTAL_BYTE_CNT_CLR, 0);

			PRMU_OUTf(i+1, WRITE_CTRL, WR_REQ2RDY_CLR, 1);
			PRMU_OUTf(i+1, WRITE_CTRL, WR_REQ2RESP_CLR, 1);
			PRMU_OUTf(i+1, WRITE_CTRL, WR_REQ_CNT_CLR, 1);
			PRMU_OUTf(i+1, WRITE_CTRL, WR_RESP_CNT_CLR, 1);
			PRMU_OUTf(i+1, WRITE_CTRL,
					WR_REQ_TOTAL_BYTE_CNT_CLR, 1);

			PRMU_OUTf(i+1, WRITE_CTRL, WR_REQ2RDY_CLR, 0);
			PRMU_OUTf(i+1, WRITE_CTRL, WR_REQ2RESP_CLR, 0);
			PRMU_OUTf(i+1, WRITE_CTRL, WR_REQ_CNT_CLR, 0);
			PRMU_OUTf(i+1, WRITE_CTRL, WR_RESP_CNT_CLR, 0);
			PRMU_OUTf(i+1, WRITE_CTRL,
					WR_REQ_TOTAL_BYTE_CNT_CLR, 0);

			PRMU_OUTf(i+1, IRQ_EN, RD_REQ_TOTAL_BYTE_OVFL_IE, 1);
			PRMU_OUTf(i+1, IRQ_EN, RD_REQ_CNT_OVFL_IE, 1);
			PRMU_OUTf(i+1, IRQ_EN, RD_RESP_CNT_OVFL_IE, 1);

			PRMU_OUTf(i+1, IRQ_EN, WR_REQ_TOTAL_BYTE_OVFL_IE, 0);
			PRMU_OUTf(i+1, IRQ_EN, WR_REQ_CNT_OVFL_IE, 0);
			PRMU_OUTf(i+1, IRQ_EN, WR_RESP_CNT_OVFL_IE, 0);

		} else if (perf_mon_dev->mode[i] == 'w') {
			PRMU_OUTf(i+1, READ_CTRL, RD_REQ2RDY_EN, 0);
			PRMU_OUTf(i+1, READ_CTRL, RD_REQ2RESP_EN, 0);
			PRMU_OUTf(i+1, READ_CTRL, RD_REQ_CNT_EN, 0);
			PRMU_OUTf(i+1, READ_CTRL, RD_RESP_CNT_EN, 0);
			PRMU_OUTf(i+1, READ_CTRL,
					RD_REQ_TOTAL_BYTE_CNT_EN, 0);

			PRMU_OUTf(i+1, WRITE_CTRL, WR_REQ2RDY_EN, 1);
			PRMU_OUTf(i+1, WRITE_CTRL, WR_REQ2RESP_EN, 1);
			PRMU_OUTf(i+1, WRITE_CTRL, WR_REQ_CNT_EN, 1);
			PRMU_OUTf(i+1, WRITE_CTRL, WR_RESP_CNT_EN, 1);
			PRMU_OUTf(i+1, WRITE_CTRL,
					WR_REQ_TOTAL_BYTE_CNT_EN, 1);

			PRMU_OUTf(i+1, READ_CTRL, RD_REQ2RDY_CLR, 1);
			PRMU_OUTf(i+1, READ_CTRL, RD_REQ2RESP_CLR, 1);
			PRMU_OUTf(i+1, READ_CTRL, RD_REQ_CNT_CLR, 1);
			PRMU_OUTf(i+1, READ_CTRL, RD_RESP_CNT_CLR, 1);
			PRMU_OUTf(i+1, READ_CTRL,
					RD_REQ_TOTAL_BYTE_CNT_CLR, 1);

			PRMU_OUTf(i+1, READ_CTRL, RD_REQ2RDY_CLR, 0);
			PRMU_OUTf(i+1, READ_CTRL, RD_REQ2RESP_CLR, 0);
			PRMU_OUTf(i+1, READ_CTRL, RD_REQ_CNT_CLR, 0);
			PRMU_OUTf(i+1, READ_CTRL, RD_RESP_CNT_CLR, 0);
			PRMU_OUTf(i+1, READ_CTRL,
					RD_REQ_TOTAL_BYTE_CNT_CLR, 0);

			PRMU_OUTf(i+1, WRITE_CTRL, WR_REQ2RDY_CLR, 1);
			PRMU_OUTf(i+1, WRITE_CTRL, WR_REQ2RESP_CLR, 1);
			PRMU_OUTf(i+1, WRITE_CTRL, WR_REQ_CNT_CLR, 1);
			PRMU_OUTf(i+1, WRITE_CTRL, WR_RESP_CNT_CLR, 1);
			PRMU_OUTf(i+1, WRITE_CTRL,
					WR_REQ_TOTAL_BYTE_CNT_CLR, 1);

			PRMU_OUTf(i+1, WRITE_CTRL, WR_REQ2RDY_CLR, 0);
			PRMU_OUTf(i+1, WRITE_CTRL, WR_REQ2RESP_CLR, 0);
			PRMU_OUTf(i+1, WRITE_CTRL, WR_REQ_CNT_CLR, 0);
			PRMU_OUTf(i+1, WRITE_CTRL, WR_RESP_CNT_CLR, 0);
			PRMU_OUTf(i+1, WRITE_CTRL,
					WR_REQ_TOTAL_BYTE_CNT_CLR, 0);

			PRMU_OUTf(i+1, IRQ_EN, RD_REQ_TOTAL_BYTE_OVFL_IE, 0);
			PRMU_OUTf(i+1, IRQ_EN, RD_REQ_CNT_OVFL_IE, 0);
			PRMU_OUTf(i+1, IRQ_EN, RD_RESP_CNT_OVFL_IE, 0);

			PRMU_OUTf(i+1, IRQ_EN, WR_REQ_TOTAL_BYTE_OVFL_IE, 1);
			PRMU_OUTf(i+1, IRQ_EN, WR_REQ_CNT_OVFL_IE, 1);
			PRMU_OUTf(i+1, IRQ_EN, WR_RESP_CNT_OVFL_IE, 1);
		} else
			return 1;
		i++;
	}
	return 0;

}
static uint32_t perfmon_start_test(uint64_t time)
{
	uint64_t ticks;

	if ((perf_mon_dev->status == 1) || (perf_mon_dev->status == 3))
		return 1;
	set_axi_clk();
	if (((0xFFFFFFFFFFFFFFFF/time) < perf_mon_dev->axi_speed)
			|| (time == 0))
		return 3;
	ticks = perf_mon_dev->axi_speed * time; /* need to correct */
	/* Enable Clock */
	SCU_OUTf(RSTC, PMON_RST, 1);
	SCU_OUTf(CCU_CLK_CTL, PMON_CLKEN, 1);
	SCU_OUTf(RSTC, PMON_RST, 0);
	/* program registers */
	init_perfmon();
	PMON_OUT(SNAPSHOT_TIME_LO, LOWER(ticks));
	PMON_OUT(SNAPSHOT_TIME_HI, UPPER(ticks));
	if (config_prmu() == 1)
		return 2;
	PMON_OUTf(GLOBAL_CTRL, TIMESTAMP_CNT_CLR, 1);
	PMON_OUTf(GLOBAL_CTRL, TIMESTAMP_CNT_CLR, 0);
	PMON_OUTf(GLOBAL_CTRL, TIMESTAMP_CNT_EN, 1);
	perf_mon_dev->error = 0;
	perf_mon_dev->status = 1;
	return 0;

}

static uint32_t perfmon_read_results(struct perf_result *data)
{

	int i = 0;
	uint32_t prmu_error = 0;

	if (perf_mon_dev->status == 3)
		perfmon_stop_free_test();
	data->error = 0;
	if ((perf_mon_dev->status == 2) || (perf_mon_dev->status == 4)) {
		/*read results */
		if (perf_mon_dev->error == TIMESTAMP_OVFL)
			data->error = TIMESTAMP_OVFL;
		else if (perf_mon_dev->error != 0)
			prmu_error = (perf_mon_dev->error & 0xFF00) >> 16;
		data->time_stamp = PMON_IN(TIMESTAMP_LO) +
				((uint64_t)PMON_IN(TIMESTAMP_HI) << 32);
		while (i < PRMU_COUNT) {
			if (perf_mon_dev->mode[i] == 'r')
				data->prmu[i].mode = READ_MODE;
			else
				data->prmu[i].mode = WRITE_MODE;
			if (prmu_error == (i+1))
				data->prmu[i].error = prmu_error &
						~PRMU_MASK(IRQ_STS, RSVD0);
			else
				data->prmu[i].error = 0;
			data->prmu[i].rd_req_rdy_latency =
				PRMU_INf(i + 1,
					RD_REQ2RDY_LATENCY_CUR_MIN, LAT_CUR);
			data->prmu[i].rd_req_rdy_latency_min =
				PRMU_INf(i + 1,
					RD_REQ2RDY_LATENCY_CUR_MIN, LAT_MIN);
			data->prmu[i].rd_req_rdy_latency_max =
				PRMU_INf(i + 1,
					RD_REQ2RDY_LATENCY_MAX_AVG, LAT_MAX);
			data->prmu[i].rd_req_rdy_latency_avg =
				PRMU_INf(i + 1,
					RD_REQ2RDY_LATENCY_MAX_AVG, LAT_AVG);
			data->prmu[i].rd_req_rdy_min_lat_addr =
				PRMU_INf(i + 1,
					RD_REQ2RDY_MIN_ADDR, ADDR);
			data->prmu[i].rd_req_rdy_max_lat_addr =
				PRMU_INf(i + 1, RD_REQ2RDY_MAX_ADDR, ADDR);
			data->prmu[i].resp_rdy_latency =
				PRMU_INf(i + 1,
					REQ2RESP_LATENCY_CUR_MIN, LAT_CUR);
			data->prmu[i].resp_rdy_latency_min =
				PRMU_INf(i + 1,
					REQ2RESP_LATENCY_CUR_MIN, LAT_MIN);
			data->prmu[i].resp_rdy_latency_max =
				PRMU_INf(i + 1,
					REQ2RESP_LATENCY_MAX_AVG, LAT_MAX);
			data->prmu[i].resp_rdy_latency_avg =
				PRMU_INf(i + 1,
					REQ2RESP_LATENCY_MAX_AVG, LAT_AVG);
			data->prmu[i].req_resp_rdy_min_lat_addr =
				PRMU_INf(i + 1, REQ2RESP_MIN_ADDR, ADDR);
			data->prmu[i].req_resp_rdy_max_lat_addr =
				PRMU_INf(i + 1, REQ2RESP_MAX_ADDR, ADDR);
			data->prmu[i].rd_req_cnt =
				PRMU_INf(i + 1, RD_REQ_CNT, CNT);
			data->prmu[i].rd_resp_cnt =
				PRMU_INf(i + 1, RD_RESP_CNT, CNT);
			data->prmu[i].out_rd_resp_cnt =
				PRMU_INf(i + 1, RD_REQ_OUTSTANDING, CNT);
			data->prmu[i].rd_req_len_cnt =
				PRMU_INf(i + 1, RD_REQ_TOTAL_BYTE_CNT_LO, CNT)
				+ ((uint64_t)PRMU_INf(i + 1,
				RD_REQ_TOTAL_BYTE_CNT_HI, CNT) << 32);
			data->prmu[i].rd_bandwidth =
				(uint64_t)perf_mon_dev->axi_speed *
				AXI_SPD_DIV * data->prmu[i].rd_req_len_cnt /
				data->time_stamp;
			data->prmu[i].wr_req_rdy_latency =
				PRMU_INf(i + 1,
					WR_REQ2RDY_LATENCY_CUR_MIN, LAT_CUR);
			data->prmu[i].wr_req_rdy_latency_min =
				PRMU_INf(i + 1,
					WR_REQ2RDY_LATENCY_CUR_MIN, LAT_MIN);
			data->prmu[i].wr_req_rdy_latency_max =
				PRMU_INf(i + 1,
					WR_REQ2RDY_LATENCY_MAX_AVG, LAT_MAX);
			data->prmu[i].wr_req_rdy_latency_avg =
				PRMU_INf(i + 1,
					WR_REQ2RDY_LATENCY_MAX_AVG, LAT_AVG);
			data->prmu[i].wr_req_rdy_min_lat_addr =
				PRMU_INf(i + 1, WR_REQ2RDY_MIN_ADDR, ADDR);
			data->prmu[i].wr_req_rdy_max_lat_addr =
				PRMU_INf(i + 1, WR_REQ2RDY_MAX_ADDR, ADDR);
			data->prmu[i].wr_req_cnt =
				PRMU_INf(i + 1, WR_REQ_CNT, CNT);
			data->prmu[i].wr_resp_cnt =
				PRMU_INf(i + 1, WR_RESP_CNT, CNT);
			data->prmu[i].wr_out_cnt =
				PRMU_INf(i + 1, WR_REQ_OUTSTANDING, CNT);
			data->prmu[i].wr_req_len_cnt =
				PRMU_INf(i + 1,
					WR_REQ_TOTAL_BYTE_CNT_LO, CNT) +
				((uint64_t)PRMU_INf(i + 1,
				WR_REQ_TOTAL_BYTE_CNT_HI, CNT) << 32);
			data->prmu[i].wr_act_req_len_cnt =
				PRMU_INf(i + 1,
					WR_REQ_ACTUAL_BYTE_CNT_LO, CNT) +
				((uint64_t)PRMU_INf(i + 1,
				WR_REQ_ACTUAL_BYTE_CNT_HI, CNT) << 32);
			data->prmu[i].wr_bandwidth =
				(uint64_t)perf_mon_dev->axi_speed *
				AXI_SPD_DIV * data->prmu[i].wr_act_req_len_cnt /
				data->time_stamp;
			i++;
		}
		return 0;
	} else
		return 1;
}

static void handle_prmu_irq(uint32_t prmu)
{
	uint32_t irq;

	irq = PRMU_IN(prmu, IRQ_STS);
	perf_mon_dev->error = (prmu<<16) + (irq & ~PRMU_MASK(IRQ_STS, RSVD0));
}

static irqreturn_t perfmon_handle_irq(int irq, void *dev_id)
{
	uint32_t prmu_irq;
	/* check actual interrupt content */
	if (PMON_INf(IRQ_STATUS, SNAPSHOT_TIME_INTR_STS) == 1) {
		if (perf_mon_dev->status == 1) {
			perf_mon_dev->status = 2;
			dev_err(perf_mon_dev->dev, "Perfmon timed test complete\n");
		}
		PMON_OUTf(IRQ_STATUS, SNAPSHOT_TIME_INTR_STS, 1);
	}
	if (PMON_INf(IRQ_STATUS, TIMESTAMP_OVFL_INTR_STS) == 1) {
		perf_mon_dev->status = 2;
		perf_mon_dev->error = TIMESTAMP_OVFL;
		perfmon_stop_free_test();
		dev_err(perf_mon_dev->dev, "Timestamp overflow\n");
		PMON_OUTf(IRQ_STATUS, TIMESTAMP_OVFL_INTR_STS, 1);
	}

	/* Handle prmu IRQs */
	prmu_irq = PMON_IN(PRMU_INTR);
	if ((prmu_irq & PRMU_1_MASK) == PRMU_1_MASK)
		handle_prmu_irq(1);
	if ((prmu_irq & PRMU_2_MASK) == PRMU_2_MASK)
		handle_prmu_irq(2);
	if ((prmu_irq & PRMU_3_MASK) == PRMU_3_MASK)
		handle_prmu_irq(3);
	if ((prmu_irq & PRMU_4_MASK) == PRMU_4_MASK)
		handle_prmu_irq(4);
	if ((prmu_irq & PRMU_5_MASK) == PRMU_5_MASK)
		handle_prmu_irq(5);
	if ((prmu_irq & PRMU_6_MASK) == PRMU_6_MASK)
		handle_prmu_irq(6);
	if (prmu_irq != 0x0) {
		perf_mon_dev->status = 2;
		perf_mon_dev->error = PRMU_ERR;
		perfmon_stop_free_test();
	}

	/* return interrupt handled */
	return IRQ_HANDLED;
}

static uint32_t perfmon_start_free_test(void)
{

	if ((perf_mon_dev->status == 1) || (perf_mon_dev->status == 3))
		return 1;
	set_axi_clk();
	/* Enable Clock */
	SCU_OUTf(RSTC, PMON_RST, 1);
	SCU_OUTf(CCU_CLK_CTL, PMON_CLKEN, 1);
	SCU_OUTf(RSTC, PMON_RST, 0);
	/* program registers */
	init_perfmon();
	dev_err(perf_mon_dev->dev, "GC programed\n");
	PMON_OUT(SNAPSHOT_TIME_LO, 0x0);
	PMON_OUT(SNAPSHOT_TIME_HI, 0x0);
	dev_err(perf_mon_dev->dev, "ST programmed\n");
	if (config_prmu() == 1)
		return 2;
	PMON_OUTf(GLOBAL_CTRL, TIMESTAMP_CNT_CLR, 1);
	PMON_OUTf(GLOBAL_CTRL, TIMESTAMP_CNT_CLR, 0);
	PMON_OUTf(GLOBAL_CTRL, TIMESTAMP_CNT_EN, 1);
	dev_err(perf_mon_dev->dev, "Test started\n");
	perf_mon_dev->error = 0;
	perf_mon_dev->status = 3;
	return 0;
}

static uint32_t perfmon_stop_free_test(void)
{
	if ((perf_mon_dev->status == 1) || (perf_mon_dev->status == 3)) {
		/* program registers */
		PMON_OUTf(GLOBAL_CTRL, GLBL_EN, 0);
		perf_mon_dev->status = 2;
		return 0;
	} else
		return 1;
}

static void init_perfmon(void)
{
	PMON_OUTf(GLOBAL_CTRL, GLBL_EN, 1);
	PMON_OUTf(IRQ_ENABLE, SNAPSHOT_TIME_INTR_EN, 1);
	PMON_OUTf(IRQ_ENABLE, TIMESTAMP_OVFL_INTR_EN, 1);
	PMON_OUT(PRMU_INTR, 0x3F);
	PMON_OUTf(GLOBAL_CTRL, GLBL_CLR, 1);
	PMON_OUTf(GLOBAL_CTRL, GLBL_CLR, 0);
}

static int config_mem(struct platform_device *pdev)
{
	perf_mon_dev->memr = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!perf_mon_dev->memr)
		return -ENOMEM;
	if (!request_mem_region(perf_mon_dev->memr->start,
			resource_size(perf_mon_dev->memr),
			perf_mon_dev->name)) {
		dev_err(&pdev->dev, "unable to request mem region\n");
		return -ENOMEM;
	}
	perf_mon_dev->mem = ioremap_nocache(perf_mon_dev->memr->start,
			resource_size(perf_mon_dev->memr));
	if (!perf_mon_dev->mem) {
		release_mem_region(perf_mon_dev->memr->start,
			resource_size(perf_mon_dev->memr));
		return -ENOMEM;
	}
	perf_mon_dev->scur = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!perf_mon_dev->scur) {
		iounmap(&perf_mon_dev->mem);
		release_mem_region(perf_mon_dev->memr->start,
			resource_size(perf_mon_dev->memr));
		return -ENOMEM;
	}
	perf_mon_dev->scu = ioremap_nocache(perf_mon_dev->scur->start,
			resource_size(perf_mon_dev->scur));
	if (!perf_mon_dev->scu) {
		iounmap(&perf_mon_dev->mem);
		release_mem_region(perf_mon_dev->memr->start,
			resource_size(perf_mon_dev->memr));
		return -ENOMEM;
	}

	return 0;
}

static int clear_mem(void)
{
	iounmap(&perf_mon_dev->mem);
	iounmap(&perf_mon_dev->scu);
	release_mem_region(perf_mon_dev->memr->start,
			resource_size(perf_mon_dev->memr));

	return 0;
}

/* SYS_FS for debugging and testing */

static int print_device(struct perf_result *data, int i, char *buf)
{
	char error_str[64];
	char mode_str[64];

	if (data->prmu[i].mode == READ_MODE)
		strcpy(mode_str, "Read Mode\n");
	else
		strcpy(mode_str, "Write Mode\n");
	if (data->error == TIMESTAMP_OVFL)
		strcpy(error_str, "ERROR: TIMESTAMP Overflow\n");
	else
		switch (data->prmu[i].error) {
		case 0:
			strcpy(error_str, " ");
			break;
		case 0x1:
			strcpy(error_str,
			"ERROR: Read Request Length Counter overflow\n");
			break;
		case 0x2:
			strcpy(error_str,
			"ERROR: Read Request Counter overflow\n");
			break;
		case 0x4:
			strcpy(error_str,
			"ERROR: Read Response Counter overflow\n");
			break;
		case 0x8:
			strcpy(error_str,
			"ERROR: Write Request Length Counter overflow\n");
			break;
		case 0x10:
			strcpy(error_str,
			"ERROR: Write Request Counter overflow\n");
			break;
		case 0x20:
			strcpy(error_str,
			"ERROR: Write Response Length Counter overflow\n");
			break;
		default:
			strcpy(error_str,
			"ERROR: Multiple Counter overflow\n");
			break;
	}
	return snprintf(buf, MAX_STR_COPY, " Timestamp %llx\n"
			" %s"
			" %s"
			"Read Request To Ready Latency\n"
			"        %lx\n"
			"Minimum %lx\n"
			"Maximum %lx\n"
			"Average %lx\n"
			"Minimum Read Request Latency Address %lx\n"
			"Maximum Read Request Latency Address %lx\n"
			"Read/Write Request To Response Latency\n"
			"        %lx\n"
			"Minimum %lx\n"
			"Maximum %lx\n"
			"Average %lx\n"
			"Minimum Read/Write Response Latency Address %lx\n"
			"Maximum Read/Write Response Latency Address %lx\n"
			"Read Request Generated %lx\n"
			"Read response Received %lx\n"
			"Outstanding Read Requests %lx\n"
			"Total Read Requests %llx\n"
			"Total Read Bandwidth %lld bytes/second\n"
			"Write Request To Ready Latency\n"
			"        %lx\n"
			"Minimum %lx\n"
			"Maximum %lx\n"
			"Average %lx\n"
			"Minimum Write Request Latency Address %lx\n"
			"Maximum Write Request Latency Address %lx\n"
			"Write Request Generated %lx\n"
			"Write response Received %lx\n"
			"Outstanding Write Requests %lx\n"
			"Total Write Requests %llx\n"
			"Actual Write Requests %llx\n"
			"Total Write Bandwidth %lld bytes/second\n",
			(unsigned long long) data->time_stamp,
			error_str,
			mode_str,
			(unsigned long) data->prmu[i].rd_req_rdy_latency,
			(unsigned long) data->prmu[i].rd_req_rdy_latency_min,
			(unsigned long) data->prmu[i].rd_req_rdy_latency_max,
			(unsigned long) data->prmu[i].rd_req_rdy_latency_avg,
			(unsigned long) data->prmu[i].rd_req_rdy_min_lat_addr,
			(unsigned long) data->prmu[i].rd_req_rdy_max_lat_addr,
			(unsigned long) data->prmu[i].resp_rdy_latency,
			(unsigned long) data->prmu[i].resp_rdy_latency_min,
			(unsigned long) data->prmu[i].resp_rdy_latency_max,
			(unsigned long) data->prmu[i].resp_rdy_latency_avg,
			(unsigned long) data->prmu[i].req_resp_rdy_min_lat_addr,
			(unsigned long) data->prmu[i].req_resp_rdy_max_lat_addr,
			(unsigned long) data->prmu[i].rd_req_cnt,
			(unsigned long) data->prmu[i].rd_resp_cnt,
			(unsigned long) data->prmu[i].out_rd_resp_cnt,
			(unsigned long long) data->prmu[i].rd_req_len_cnt,
			(unsigned long long) data->prmu[i].rd_bandwidth,
			(unsigned long) data->prmu[i].wr_req_rdy_latency,
			(unsigned long) data->prmu[i].wr_req_rdy_latency_min,
			(unsigned long) data->prmu[i].wr_req_rdy_latency_max,
			(unsigned long) data->prmu[i].wr_req_rdy_latency_avg,
			(unsigned long) data->prmu[i].wr_req_rdy_min_lat_addr,
			(unsigned long) data->prmu[i].wr_req_rdy_max_lat_addr,
			(unsigned long) data->prmu[i].wr_req_cnt,
			(unsigned long) data->prmu[i].wr_resp_cnt,
			(unsigned long) data->prmu[i].wr_out_cnt,
			(unsigned long long) data->prmu[i].wr_req_len_cnt,
			(unsigned long long) data->prmu[i].wr_act_req_len_cnt,
			(unsigned long long) data->prmu[i].wr_bandwidth);
}


static ssize_t sysfs_free_measure(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	unsigned long val;
	int ret;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;
	if (val == 1) {
		ret = perfmon_start_free_test();
		if (ret == 1)
			dev_err(perf_mon_dev->dev, "Measurement ongoing\n");
		if (ret == 2)
		dev_err(perf_mon_dev->dev, "Bad mode\n");
	} else if (val == 0) {
		ret = perfmon_stop_free_test();
		if (ret == 1)
			dev_err(perf_mon_dev->dev, "No measurement started\n");
	}
	return count;
}

static DEVICE_ATTR(free_test, S_IRUGO | S_IWUSR | S_IWGRP,
			NULL, sysfs_free_measure);

static ssize_t sysfs_start_measure(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	unsigned long val;
	int ret;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;
	if (val <= 0)
		return -EINVAL;
	ret = perfmon_start_test(val);
	if (ret == 1)
		dev_err(perf_mon_dev->dev, "Measurement ongoing\n");
	if (ret == 2)
		dev_err(perf_mon_dev->dev, "Bad mode\n");
	if (ret == 3)
		dev_err(perf_mon_dev->dev, "Time out of range\n");
	return count;
}

static DEVICE_ATTR(timed_test, S_IRUGO | S_IWUSR | S_IWGRP,
			NULL, sysfs_start_measure);

static ssize_t show_sysfs_comp_results(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct perf_result data;

	if (perfmon_read_results(&data) == 0) {
		return snprintf(buf, MAX_STR_COPY, " %lx; %llx;"
			" %lx; %lx; %lx; %lx; %lx; %lx; %lx; %lx; %lx; %lx;"
			" %lx, %lx; %lx; %lx; %lx; %lx; %lx; %llx; %lx; %lx;"
			" %lx; %lx; %lx; %lx; %lx; %lx; %lx; %llx; %llx"
			" %lx; %lx; %lx; %lx; %lx; %lx; %lx; %lx %lx; %lx;"
			" %lx, %lx; %lx; %lx; %lx; %lx; %lx; %llx; %lx; %lx;"
			" %lx; %lx; %lx; %lx; %lx; %lx; %lx; %llx; %llx"
			" %lx; %lx; %lx; %lx; %lx; %lx; %lx; %lx; %lx; %lx;"
			" %lx, %lx; %lx; %lx; %lx; %lx; %lx; %llx; %lx; %lx;"
			" %lx; %lx; %lx; %lx; %lx; %lx; %lx; %llx; %llx"
			" %lx; %lx; %lx; %lx; %lx; %lx; %lx; %lx; %lx; %lx;"
			" %lx, %lx; %lx; %lx; %lx; %lx; %lx; %llx; %lx; %lx;"
			" %lx; %lx; %lx; %lx; %lx; %lx; %lx; %llx; %llx"
			" %lx; %lx; %lx; %lx; %lx; %lx; %lx; %lx; %lx; %lx;"
			" %lx, %lx; %lx; %lx; %lx; %lx; %lx; %llx; %lx; %lx;"
			" %lx; %lx; %lx; %lx; %lx; %lx; %lx; %llx; %llx"
			" %lx; %lx; %lx; %lx; %lx; %lx; %lx; %lx; %lx; %lx;"
			" %lx, %lx; %lx; %lx; %lx; %lx; %lx; %llx; %lx; %lx;"
			" %lx; %lx; %lx; %lx; %lx; %lx; %lx; %llx; %llx"
			"\n",
			(unsigned long) data.error,
			(unsigned long long) data.time_stamp,

			(unsigned long) data.prmu[0].error,
			(unsigned long) data.prmu[0].mode,
			(unsigned long) data.prmu[0].rd_req_rdy_latency,
			(unsigned long) data.prmu[0].rd_req_rdy_latency_min,
			(unsigned long) data.prmu[0].rd_req_rdy_latency_max,
			(unsigned long) data.prmu[0].rd_req_rdy_latency_avg,
			(unsigned long) data.prmu[0].rd_req_rdy_min_lat_addr,
			(unsigned long) data.prmu[0].rd_req_rdy_max_lat_addr,
			(unsigned long) data.prmu[0].resp_rdy_latency,
			(unsigned long) data.prmu[0].resp_rdy_latency_min,
			(unsigned long) data.prmu[0].resp_rdy_latency_max,
			(unsigned long) data.prmu[0].resp_rdy_latency_avg,
			(unsigned long) data.prmu[0].req_resp_rdy_min_lat_addr,
			(unsigned long) data.prmu[0].req_resp_rdy_max_lat_addr,
			(unsigned long) data.prmu[0].rd_req_cnt,
			(unsigned long) data.prmu[0].rd_resp_cnt,
			(unsigned long) data.prmu[0].out_rd_resp_cnt,
			(unsigned long long) data.prmu[0].rd_req_len_cnt,
			(unsigned long) data.prmu[0].wr_req_rdy_latency,
			(unsigned long) data.prmu[0].wr_req_rdy_latency_min,
			(unsigned long) data.prmu[0].wr_req_rdy_latency_max,
			(unsigned long) data.prmu[0].wr_req_rdy_latency_avg,
			(unsigned long) data.prmu[0].wr_req_rdy_min_lat_addr,
			(unsigned long) data.prmu[0].wr_req_rdy_max_lat_addr,
			(unsigned long) data.prmu[0].wr_req_cnt,
			(unsigned long) data.prmu[0].wr_resp_cnt,
			(unsigned long) data.prmu[0].wr_out_cnt,
			(unsigned long long) data.prmu[0].wr_req_len_cnt,
			(unsigned long long) data.prmu[0].wr_act_req_len_cnt,

			(unsigned long) data.prmu[1].error,
			(unsigned long) data.prmu[1].mode,
			(unsigned long) data.prmu[1].rd_req_rdy_latency,
			(unsigned long) data.prmu[1].rd_req_rdy_latency_min,
			(unsigned long) data.prmu[1].rd_req_rdy_latency_max,
			(unsigned long) data.prmu[1].rd_req_rdy_latency_avg,
			(unsigned long) data.prmu[1].rd_req_rdy_min_lat_addr,
			(unsigned long) data.prmu[1].rd_req_rdy_max_lat_addr,
			(unsigned long) data.prmu[1].resp_rdy_latency,
			(unsigned long) data.prmu[1].resp_rdy_latency_min,
			(unsigned long) data.prmu[1].resp_rdy_latency_max,
			(unsigned long) data.prmu[1].resp_rdy_latency_avg,
			(unsigned long) data.prmu[1].req_resp_rdy_min_lat_addr,
			(unsigned long) data.prmu[1].req_resp_rdy_max_lat_addr,
			(unsigned long) data.prmu[1].rd_req_cnt,
			(unsigned long) data.prmu[1].rd_resp_cnt,
			(unsigned long) data.prmu[1].out_rd_resp_cnt,
			(unsigned long long) data.prmu[1].rd_req_len_cnt,
			(unsigned long) data.prmu[1].wr_req_rdy_latency,
			(unsigned long) data.prmu[1].wr_req_rdy_latency_min,
			(unsigned long) data.prmu[1].wr_req_rdy_latency_max,
			(unsigned long) data.prmu[1].wr_req_rdy_latency_avg,
			(unsigned long) data.prmu[1].wr_req_rdy_min_lat_addr,
			(unsigned long) data.prmu[1].wr_req_rdy_max_lat_addr,
			(unsigned long) data.prmu[1].wr_req_cnt,
			(unsigned long) data.prmu[1].wr_resp_cnt,
			(unsigned long) data.prmu[1].wr_out_cnt,
			(unsigned long long) data.prmu[1].wr_req_len_cnt,
			(unsigned long long) data.prmu[1].wr_act_req_len_cnt,

			(unsigned long) data.prmu[2].error,
			(unsigned long) data.prmu[2].mode,
			(unsigned long) data.prmu[2].rd_req_rdy_latency,
			(unsigned long) data.prmu[2].rd_req_rdy_latency_min,
			(unsigned long) data.prmu[2].rd_req_rdy_latency_max,
			(unsigned long) data.prmu[2].rd_req_rdy_latency_avg,
			(unsigned long) data.prmu[2].rd_req_rdy_min_lat_addr,
			(unsigned long) data.prmu[2].rd_req_rdy_max_lat_addr,
			(unsigned long) data.prmu[2].resp_rdy_latency,
			(unsigned long) data.prmu[2].resp_rdy_latency_min,
			(unsigned long) data.prmu[2].resp_rdy_latency_max,
			(unsigned long) data.prmu[2].resp_rdy_latency_avg,
			(unsigned long) data.prmu[2].req_resp_rdy_min_lat_addr,
			(unsigned long) data.prmu[2].req_resp_rdy_max_lat_addr,
			(unsigned long) data.prmu[2].rd_req_cnt,
			(unsigned long) data.prmu[2].rd_resp_cnt,
			(unsigned long) data.prmu[2].out_rd_resp_cnt,
			(unsigned long long) data.prmu[2].rd_req_len_cnt,
			(unsigned long) data.prmu[2].wr_req_rdy_latency,
			(unsigned long) data.prmu[2].wr_req_rdy_latency_min,
			(unsigned long) data.prmu[2].wr_req_rdy_latency_max,
			(unsigned long) data.prmu[2].wr_req_rdy_latency_avg,
			(unsigned long) data.prmu[2].wr_req_rdy_min_lat_addr,
			(unsigned long) data.prmu[2].wr_req_rdy_max_lat_addr,
			(unsigned long) data.prmu[2].wr_req_cnt,
			(unsigned long) data.prmu[2].wr_resp_cnt,
			(unsigned long) data.prmu[2].wr_out_cnt,
			(unsigned long long) data.prmu[2].wr_req_len_cnt,
			(unsigned long long) data.prmu[2].wr_act_req_len_cnt,

			(unsigned long) data.prmu[3].error,
			(unsigned long) data.prmu[3].mode,
			(unsigned long) data.prmu[3].rd_req_rdy_latency,
			(unsigned long) data.prmu[3].rd_req_rdy_latency_min,
			(unsigned long) data.prmu[3].rd_req_rdy_latency_max,
			(unsigned long) data.prmu[3].rd_req_rdy_latency_avg,
			(unsigned long) data.prmu[3].rd_req_rdy_min_lat_addr,
			(unsigned long) data.prmu[3].rd_req_rdy_max_lat_addr,
			(unsigned long) data.prmu[3].resp_rdy_latency,
			(unsigned long) data.prmu[3].resp_rdy_latency_min,
			(unsigned long) data.prmu[3].resp_rdy_latency_max,
			(unsigned long) data.prmu[3].resp_rdy_latency_avg,
			(unsigned long) data.prmu[3].req_resp_rdy_min_lat_addr,
			(unsigned long) data.prmu[3].req_resp_rdy_max_lat_addr,
			(unsigned long) data.prmu[3].rd_req_cnt,
			(unsigned long) data.prmu[3].rd_resp_cnt,
			(unsigned long) data.prmu[3].out_rd_resp_cnt,
			(unsigned long long) data.prmu[3].rd_req_len_cnt,
			(unsigned long) data.prmu[3].wr_req_rdy_latency,
			(unsigned long) data.prmu[3].wr_req_rdy_latency_min,
			(unsigned long) data.prmu[3].wr_req_rdy_latency_max,
			(unsigned long) data.prmu[3].wr_req_rdy_latency_avg,
			(unsigned long) data.prmu[3].wr_req_rdy_min_lat_addr,
			(unsigned long) data.prmu[3].wr_req_rdy_max_lat_addr,
			(unsigned long) data.prmu[3].wr_req_cnt,
			(unsigned long) data.prmu[3].wr_resp_cnt,
			(unsigned long) data.prmu[3].wr_out_cnt,
			(unsigned long long) data.prmu[3].wr_req_len_cnt,
			(unsigned long long) data.prmu[3].wr_act_req_len_cnt,

			(unsigned long) data.prmu[4].error,
			(unsigned long) data.prmu[4].mode,
			(unsigned long) data.prmu[4].rd_req_rdy_latency,
			(unsigned long) data.prmu[4].rd_req_rdy_latency_min,
			(unsigned long) data.prmu[4].rd_req_rdy_latency_max,
			(unsigned long) data.prmu[4].rd_req_rdy_latency_avg,
			(unsigned long) data.prmu[4].rd_req_rdy_min_lat_addr,
			(unsigned long) data.prmu[4].rd_req_rdy_max_lat_addr,
			(unsigned long) data.prmu[4].resp_rdy_latency,
			(unsigned long) data.prmu[4].resp_rdy_latency_min,
			(unsigned long) data.prmu[4].resp_rdy_latency_max,
			(unsigned long) data.prmu[4].resp_rdy_latency_avg,
			(unsigned long) data.prmu[4].req_resp_rdy_min_lat_addr,
			(unsigned long) data.prmu[4].req_resp_rdy_max_lat_addr,
			(unsigned long) data.prmu[4].rd_req_cnt,
			(unsigned long) data.prmu[4].rd_resp_cnt,
			(unsigned long) data.prmu[4].out_rd_resp_cnt,
			(unsigned long long) data.prmu[4].rd_req_len_cnt,
			(unsigned long) data.prmu[4].wr_req_rdy_latency,
			(unsigned long) data.prmu[4].wr_req_rdy_latency_min,
			(unsigned long) data.prmu[4].wr_req_rdy_latency_max,
			(unsigned long) data.prmu[4].wr_req_rdy_latency_avg,
			(unsigned long) data.prmu[4].wr_req_rdy_min_lat_addr,
			(unsigned long) data.prmu[4].wr_req_rdy_max_lat_addr,
			(unsigned long) data.prmu[4].wr_req_cnt,
			(unsigned long) data.prmu[4].wr_resp_cnt,
			(unsigned long) data.prmu[4].wr_out_cnt,
			(unsigned long long) data.prmu[4].wr_req_len_cnt,
			(unsigned long long) data.prmu[4].wr_act_req_len_cnt,

			(unsigned long) data.prmu[5].error,
			(unsigned long) data.prmu[5].mode,
			(unsigned long) data.prmu[5].rd_req_rdy_latency,
			(unsigned long) data.prmu[5].rd_req_rdy_latency_min,
			(unsigned long) data.prmu[5].rd_req_rdy_latency_max,
			(unsigned long) data.prmu[5].rd_req_rdy_latency_avg,
			(unsigned long) data.prmu[5].rd_req_rdy_min_lat_addr,
			(unsigned long) data.prmu[5].rd_req_rdy_max_lat_addr,
			(unsigned long) data.prmu[5].resp_rdy_latency,
			(unsigned long) data.prmu[5].resp_rdy_latency_min,
			(unsigned long) data.prmu[5].resp_rdy_latency_max,
			(unsigned long) data.prmu[5].resp_rdy_latency_avg,
			(unsigned long) data.prmu[5].req_resp_rdy_min_lat_addr,
			(unsigned long) data.prmu[5].req_resp_rdy_max_lat_addr,
			(unsigned long) data.prmu[5].rd_req_cnt,
			(unsigned long) data.prmu[5].rd_resp_cnt,
			(unsigned long) data.prmu[5].out_rd_resp_cnt,
			(unsigned long long) data.prmu[5].rd_req_len_cnt,
			(unsigned long) data.prmu[5].wr_req_rdy_latency,
			(unsigned long) data.prmu[5].wr_req_rdy_latency_min,
			(unsigned long) data.prmu[5].wr_req_rdy_latency_max,
			(unsigned long) data.prmu[5].wr_req_rdy_latency_avg,
			(unsigned long) data.prmu[5].wr_req_rdy_min_lat_addr,
			(unsigned long) data.prmu[5].wr_req_rdy_max_lat_addr,
			(unsigned long) data.prmu[5].wr_req_cnt,
			(unsigned long) data.prmu[5].wr_resp_cnt,
			(unsigned long) data.prmu[5].wr_out_cnt,
			(unsigned long long) data.prmu[5].wr_req_len_cnt,
			(unsigned long long) data.prmu[5].wr_act_req_len_cnt
		);
	} else
		return snprintf(buf, MAX_STR_COPY, "No results available\n");
}

static DEVICE_ATTR(comp_results, S_IRUGO | S_IWUSR | S_IWGRP,
			show_sysfs_comp_results, NULL);

static ssize_t show_sysfs_ipu_results(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct perf_result data;

	if (perfmon_read_results(&data) == 0)
		return print_device(&data, 0, buf);
	else
		return snprintf(buf, MAX_STR_COPY, "No results available\n");
}

static DEVICE_ATTR(ipu_axi_results, S_IRUGO | S_IWUSR | S_IWGRP,
			show_sysfs_ipu_results, NULL);

static ssize_t show_sysfs_pcie_results(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct perf_result data;

	if (perfmon_read_results(&data) == 0)
		return print_device(&data, 1, buf);
	else
		return snprintf(buf, MAX_STR_COPY, "No results available\n");
}

static DEVICE_ATTR(pcie_axi_master_results, S_IRUGO | S_IWUSR | S_IWGRP,
			show_sysfs_pcie_results, NULL);

static ssize_t show_sysfs_a53_results(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct perf_result data;

	if (perfmon_read_results(&data) == 0)
		return print_device(&data, 2, buf);
	else
		return snprintf(buf, MAX_STR_COPY, "No results available\n");
}

static DEVICE_ATTR(a53_axi_results, S_IRUGO | S_IWUSR | S_IWGRP,
			show_sysfs_a53_results, NULL);

static ssize_t show_sysfs_axi_results(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct perf_result data;

	if (perfmon_read_results(&data) == 0)
		return print_device(&data, 3, buf);
	else
		return snprintf(buf, MAX_STR_COPY, "No results available\n");
}

static DEVICE_ATTR(axi_master_results, S_IRUGO | S_IWUSR | S_IWGRP,
			show_sysfs_axi_results, NULL);

static ssize_t show_sysfs_ddr_results(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct perf_result data;

	if (perfmon_read_results(&data) == 0)
		return print_device(&data, 4, buf);
	else
		return snprintf(buf, MAX_STR_COPY, "No results available\n");
}

static DEVICE_ATTR(lpddr_axi_results, S_IRUGO | S_IWUSR | S_IWGRP,
			show_sysfs_ddr_results, NULL);

static ssize_t show_sysfs_pcie_slave_results(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct perf_result data;

	if (perfmon_read_results(&data) == 0)
		return print_device(&data, 5, buf);
	else
		return snprintf(buf, MAX_STR_COPY, "No results available\n");
}

static DEVICE_ATTR(pcie_axi_slave_results, S_IRUGO | S_IWUSR | S_IWGRP,
			show_sysfs_pcie_slave_results, NULL);

static ssize_t sysfs_test_mode(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	if ((perf_mon_dev->status == 1) || (perf_mon_dev->status == 3))  {
		dev_err(perf_mon_dev->dev, "Measurement ongoing\n");
		return -EINVAL;
		}
	if ((count == (PRMU_COUNT + 1)) && (strspn(buf, "rw") == PRMU_COUNT))
		strcpy(perf_mon_dev->mode, buf);
	else {
		dev_err(perf_mon_dev->dev, "Wrong entry\n");
		return -EINVAL;
	}
	return count;
}

static DEVICE_ATTR(test_mode, S_IRUGO | S_IWUSR | S_IWGRP,
			NULL, sysfs_test_mode);

static struct attribute *perfmon_dev_attributes[] = {
	&dev_attr_timed_test.attr,
	&dev_attr_free_test.attr,
	&dev_attr_test_mode.attr,
	&dev_attr_comp_results.attr,
	&dev_attr_ipu_axi_results.attr,
	&dev_attr_pcie_axi_master_results.attr,
	&dev_attr_a53_axi_results.attr,
	&dev_attr_axi_master_results.attr,
	&dev_attr_lpddr_axi_results.attr,
	&dev_attr_pcie_axi_slave_results.attr,
	NULL
};

static struct attribute_group perfmon_group = {
	.name = "perfmon",
	.attrs = perfmon_dev_attributes
};



static int init_sysfs(void)
{
	int ret;

	ret = sysfs_create_group(kernel_kobj,
			&perfmon_group);
	if (ret) {
		dev_err(perf_mon_dev->dev, "Failed to create sysfs\n");
		return -EINVAL;
	}

	return 0;
}

static void clean_sysfs(void)
{
	sysfs_remove_group(kernel_kobj,
			&perfmon_group);
}

static int mnh_perf_mon_probe(struct platform_device *pdev)
{
	int err;

	perf_mon_dev = kzalloc(sizeof(*perf_mon_dev), GFP_KERNEL);
	if (!perf_mon_dev) {
		dev_err(&pdev->dev, "Could not allocated pcie_ep_dev\n");
		return -ENOMEM;
	}
	perf_mon_dev->dev = &pdev->dev;
	strcpy(perf_mon_dev->name, DEVICE_NAME);
	strcpy(perf_mon_dev->mode, "rrrrrr");
	err = config_mem(pdev);
	if (err)
		return err;

	/* Register IRQs */

	perf_mon_dev->irq = platform_get_irq(pdev, 0);
	perf_mon_dev->status = 0;

	err = request_irq(perf_mon_dev->irq, perfmon_handle_irq,
			IRQF_SHARED, DEVICE_NAME, perf_mon_dev);
	if (err) {
		dev_err(&pdev->dev, "Could not allocated irq\n");
		clear_mem();
		return -EINVAL;
	}
	init_sysfs();
	return 0;
}

static int mnh_perf_mon_remove(struct platform_device *pdev)
{
	clean_sysfs();
	clear_mem();
	free_irq(perf_mon_dev->irq, perf_mon_dev);
	return 0;
}

/*
 * of_device_id structure
 */
static const struct of_device_id mnh_perf_mon[] = {
	{ .compatible = "intel, perf_mon" },
	{ }
};

MODULE_DEVICE_TABLE(of, mnh_perf_mon);
/*
 * Platform driver structure
 */
static struct platform_driver __refdata mnh_perf_mon_pdrv = {
	.remove = mnh_perf_mon_remove,
	.probe  = mnh_perf_mon_probe,
	.driver   = {
		.name   = "intel, perf_mon",
		.owner = THIS_MODULE,
		.of_match_table = mnh_perf_mon,
	},
};


module_platform_driver(mnh_perf_mon_pdrv);

MODULE_AUTHOR("Marko Bartscherer <marko.bartscherer@intel.com>");
MODULE_DESCRIPTION("Monette Hill Performance Monitor Driver");
MODULE_LICENSE("GPL");
