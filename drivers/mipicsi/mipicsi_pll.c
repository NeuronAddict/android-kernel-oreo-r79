/*
 * Copyright (c) 2016, Intel Corporation. All rights reserved.
 *
 * Author: Archana Vohra <archana.vohra@intel.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of Intel nor the names of its contributors may be used
 * to endorse or promote products derived from this software without specific
 * prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/log2.h>
#include "mipicsi_top.h"
#include "mipicsi_pll.h"
#include "mipicsi_util.h"

#define DC_PLL_M_MIN 250
#define DC_PLL_M_MAX 1000
#define DC_PLL_N_MIN 1
#define DC_PLL_N_MAX 12
#define DC_PLL_P_MIN 1
#define DC_PLL_P_MAX 8
#define DC_PLL_FVCO_MIN 500
#define DC_PLL_FVCO_MAX 2000

#define G3_PLL_M_MIN 40
#define G3_PLL_M_MAX 625
#define G3_PLL_N_MIN 1
#define G3_PLL_N_MAX 16
#define G3_PLL_P_MIN 1
#define G3_PLL_P_MAX 8
#define G3_PLL_FVCO_MIN 40
#define G3_PLL_FVCO_MAX 1250


struct pll_hsfreq {
	uint32_t min_range;
	uint32_t max_range;
	uint32_t default_bps;
	uint8_t hsfreq;
	uint8_t phy_stop_wait;
};

/* Table 5-2: Frequency Ranges and Defaults */
struct pll_hsfreq hsfreq_tbl_dc[] = {
	{  80000000,  110250000,   80000000, 0x00},
	{  80000000,  120750000,   90000000, 0x10},
	{  80000000,  131250000,  100000000, 0x20},
	{  80750000,  141750000,  110000000, 0x30},
	{  90250000,  152250000,  120000000, 0x01},
	{  99750000,  162750000,  130000000, 0x11},
	{ 109250000,  173250000,  140000000, 0x21},
	{ 118750000,  183750000,  150000000, 0x31},
	{ 128250000,  194250000,  160000000, 0x02},
	{ 137750000,  204750000,  170000000, 0x12},
	{ 147250000,  215250000,  180000000, 0x22},
	{ 156750000,  225750000,  190000000, 0x32},
	{ 171000000,  241500000,  205000000, 0x03},
	{ 185250000,  257250000,  220000000, 0x13},
	{ 199500000,  273000000,  235000000, 0x23},
	{ 237500000,  275625000,  250000000, 0x33},
	{ 249375000,  301875000,  275000000, 0x04},
	{ 273125000,  328125000,  300000000, 0x14},
	{ 296875000,  354375000,  325000000, 0x05},
	{ 320625000,  393750000,  350000000, 0x15},
	{ 356250000,  446250000,  400000000, 0x25},
	{ 403750000,  498750000,  450000000, 0x06},
	{ 451250000,  551250000,  500000000, 0x16},
	{ 498750000,  603750000,  550000000, 0x07},
	{ 546250000,  656250000,  600000000, 0x17},
	{ 593750000,  708750000,  650000000, 0x08},
	{ 641250000,  761250000,  700000000, 0x18},
	{ 688750000,  813750000,  750000000, 0x09},
	{ 736250000,  866250000,  800000000, 0x19},
	{ 783750000,  918750000,  850000000, 0x29},
	{ 831250000,  971250000,  900000000, 0x39},
	{ 878750000, 1023750000,  950000000, 0x0A},
	{ 926250000, 1076250000, 1000000000, 0x1A},
	{ 973750000, 1128750000, 1050000000, 0x2A},
	{1021250000, 1181250000, 1100000000, 0x3A},
	{1068750000, 1233750000, 1150000000, 0x0B},
	{1116250000, 1286250000, 1200000000, 0x1B},
	{1163750000, 1338750000, 1250000000, 0x2B},
	{1211250000, 1391250000, 1300000000, 0x3B},
	{1258750000, 1443750000, 1350000000, 0x0C},
	{1306250000, 1496250000, 1400000000, 0x1C},
	{1353750000, 1548750000, 1450000000, 0x2C},
	{1401250000, 1601250000, 1500000000, 0x3C},
	{1448750000, 1653750000, 1550000000, 0x0D},
	{1496250000, 1706250000, 1600000000, 0x1D},
	{1543750000, 1758750000, 1650000000, 0x2D},
	{1591250000, 1811250000, 1700000000, 0x0E},
	{1638750000, 1863750000, 1750000000, 0x1E},
	{1686250000, 1916250000, 1800000000, 0x2E},
	{1733750000, 1968750000, 1850000000, 0x3E}
};

struct pll_cp_lpf_ctrl {
	uint16_t vco_fmin;
	uint16_t vco_fmax;
	uint8_t  vco_range;
	uint8_t  icpctrl;
	uint8_t  lpfctrl;
};

/* Table 6-9: PLL CP and LPF Control Bits */
struct pll_cp_lpf_ctrl pll_cp_lpf_ctrl_dc[]= {
	{ 500,  740, 0x00, 0x07, 0x10},
	{ 660, 1060, 0x01, 0x05, 0x04},
	{ 940, 1480, 0x02, 0x05, 0x04},
	{1320, 2000, 0x03, 0x06, 0x08}
};

struct pll_hsfreq hsfreq_tbl_g3[] = {
	{  80000000,   97125000,   80000000, 0x00, 15},
	{  80000000,  107625000,   90000000, 0x10, 16},
	{  83125000,  118125000,  100000000, 0x20, 16},
	{  92625000,  128625000,  110000000, 0x30, 17},
	{ 102125000,  139125000,  120000000, 0x01, 18},
	{ 111625000,  149625000,  130000000, 0x11, 19},
	{ 121125000,  160125000,  140000000, 0x21, 19},
	{ 130625000,  170625000,  150000000, 0x31, 20},
	{ 140125000,  181125000,  160000000, 0x02, 22},
	{ 149625000,  191625000,  170000000, 0x12, 23},
	{ 159125000,  202125000,  180000000, 0x22, 23},
	{ 168625000,  212625000,  190000000, 0x32, 24},
	{ 182875000,  228375000,  205000000, 0x03, 25},
	{ 197125000,  244125000,  220000000, 0x13, 27},
	{ 211375000,  259875000,  235000000, 0x23, 27},
	{ 225625000,  275625000,  250000000, 0x33, 30},
	{ 249375000,  301875000,  275000000, 0x04, 32},
	{ 273125000,  328125000,  300000000, 0x14, 35},
	{ 296875000,  354375000,  325000000, 0x25, 36},
	{ 320625000,  380625000,  350000000, 0x35, 40},
	{ 368125000,  433125000,  400000000, 0x05, 44},
	{ 415625000,  485625000,  450000000, 0x16, 49},
	{ 463125000,  538125000,  500000000, 0x26, 54},
	{ 510625000,  590625000,  550000000, 0x37, 57},
	{ 558125000,  643125000,  600000000, 0x07, 64},
	{ 605625000,  695625000,  650000000, 0x18, 67},
	{ 653125000,  748125000,  700000000, 0x28, 71},
	{ 700625000,  800625000,  750000000, 0x39, 75},
	{ 748125000,  853125000,  800000000, 0x09, 82},
	{ 795625000,  905625000,  850000000, 0x19, 85},
	{ 843125000,  958125000,  900000000, 0x29, 88},
	{ 890625000, 1010625000,  950000000, 0x3A, 93},
	{ 938125000, 1063125000, 1000000000, 0x0A, 99},
	{ 985625000, 1115625000, 1050000000, 0x1A, 102},
	{1033125000, 1168125000, 1100000000, 0x2A, 106},
	{1080625000, 1220625000, 1150000000, 0x3B, 112},
	{1128125000, 1273125000, 1200000000, 0x0B, 117},
	{1175625000, 1325625000, 1250000000, 0x1B, 120},
	{1223125000, 1378125000, 1300000000, 0x2B, 124},
	{1270625000, 1430625000, 1350000000, 0x3C, 130},
	{1318125000, 1483125000, 1400000000, 0x0C, 134},
	{1365625000, 1535625000, 1450000000, 0x1C, 138},
	{1413125000, 1588125000, 1500000000, 0x2C, 143},
	{1460625000, 1640625000, 1550000000, 0x3D, 147},
	{1508125000, 1693125000, 1600000000, 0x0D, 152},
	{1555625000, 1745625000, 1650000000, 0x1D, 155},
	{1603125000, 1798125000, 1700000000, 0x2E, 161},
	{1650625000, 1850625000, 1750000000, 0x3E, 165},
	{1698125000, 1903125000, 1800000000, 0x0E, 171},
	{1745625000, 1955625000, 1850000000, 0x1E, 175},
	{1793125000, 2008125000, 1900000000, 0x2F, 180},
	{1840625000, 2060625000, 1950000000, 0x3F, 185},
	{1888125000, 2113125000, 2000000000, 0x0F, 190},
	{1935625000, 2165625000, 2050000000, 0x40, 194},
	{1983125000, 2218125000, 2100000000, 0x41, 199},
	{2030625000, 2270625000, 2150000000, 0x42, 204},
	{2078125000, 2323125000, 2200000000, 0x43, 210},
	{2125625000, 2375625000, 2250000000, 0x44, 213},
	{2173125000, 2428125000, 2300000000, 0x45, 217},
	{2220625000, 2480625000, 2350000000, 0x46, 225},
	{2268125000, 2500000000, 2400000000, 0x47, 226},
	{2315625000, 2500000000, 2450000000, 0x48, 226},
	{2363125000, 2500000000, 2500000000, 0x49, 227},
};


struct pll_vco_cntrl_g3 {
	uint32_t vco_fmin;
	uint32_t vco_fmax;
	uint8_t  vco_cntrl;
	uint8_t  P;
};

/* Table 3-10: VCO Ranges in Hz */
struct pll_vco_cntrl_g3 pll_vco_cntrl_tbl[]= {
	{  40000000,   55000000, 0x3F, 8},
	{  52500000,   82500000, 0x39, 8},
	{  80000000,  110000000, 0x2F, 4},
	{ 105000000,  165000000, 0x29, 4},
	{ 160000000,  220000000, 0x1F, 2},
	{ 210000000,  330000000, 0x19, 2},
	{ 320000000,  440000000, 0x0F, 1},
	{ 420000000,  660000000, 0x09, 1},
	{ 630000000, 1149000000, 0x03, 1},
	{1100000000, 1152000000, 0x01, 1},
	{1150000000, 1250000000, 0x01, 1}
};

struct sr_cntrl {
	uint32_t fmin;
	uint32_t fmax;
	uint8_t sr_range;
	uint16_t sr_osc_freq_tgt;
};

/* Table 5-5: VCO Slew rate vs DDL oscillation target */
struct sr_cntrl sr_tbl_g3[]= {
	{  80,  500, 1, 0x384},
	{ 500, 1000, 1, 0x4E2},
	{1000, 1500, 0, 0x7D0}
};

int mipicsi_pll_get_stop_wait (uint16_t mbps, uint8_t *stop_wait)
{


	if (mipicsi_util_is_emulation()) {

		/* hard code */
		*stop_wait = 79;
		return 0;
	} else {
		uint8_t i;
		uint32_t bps = mbps*1000*1000;

		for (i = 0; i < (sizeof(hsfreq_tbl_g3))/
			     (sizeof(struct pll_hsfreq));
		     i++) {
			if ((bps == hsfreq_tbl_g3[i].default_bps) ||
			    ((bps >= hsfreq_tbl_g3[i].min_range) &&
			     (bps < hsfreq_tbl_g3[i].max_range))) {
				*stop_wait = hsfreq_tbl_g3[i].phy_stop_wait;
				return 0;
			}
		}
		pr_info ("%s: Stop Wait not found\n", __func__);
		return -EINVAL;
	}
}

int mipicsi_pll_get_hsfreq (uint16_t mbps, uint8_t *hsfreq)
{
	uint8_t i;
	uint32_t bps = mbps*1000*1000;

	if (mipicsi_util_is_emulation ()) {
		for (i = 0; i < (sizeof(hsfreq_tbl_dc))/(sizeof(struct pll_hsfreq));
		     i++) {
			if ((bps == hsfreq_tbl_dc[i].default_bps) ||
			    ((bps >= hsfreq_tbl_dc[i].min_range) &&
			     (bps < hsfreq_tbl_dc[i].max_range))) {
				*hsfreq = hsfreq_tbl_dc[i].hsfreq;
				return 0;
			}
		}
	} else {
		for (i = 0; i < (sizeof(hsfreq_tbl_g3))/(sizeof(struct pll_hsfreq));
		     i++) {
			if ((bps == hsfreq_tbl_g3[i].default_bps) ||
			    ((bps >= hsfreq_tbl_g3[i].min_range) &&
			     (bps < hsfreq_tbl_g3[i].max_range))) {
				*hsfreq = hsfreq_tbl_g3[i].hsfreq;
				return 0;
			}
		}
	}
	pr_info ("%s: HS Freq not found\n", __func__);
	return -EINVAL;
}

int mipicsi_pll_calc(uint16_t mbps, struct mipicsi_pll *pll)
{
	uint16_t i, M, N, P, N_min, N_max, delta = 0xffff;
	uint32_t fvco, fclkin, fout;
	bool found = false;

	fclkin = REFCLK_MHZ;
	pll->input_freq = REFCLK_MHZ;

	pr_info("%s: PLL calculation for target freq %u\n", __func__, mbps);

	if (mipicsi_util_is_emulation ()) {
		if ((mbps < DC_PLL_MIN_MHZ) || (mbps > DC_PLL_MAX_MHZ))
			return -EINVAL;

		/*
		 * fout = fvco*1/P = M/(P*N)*fclkin
		 * Therefore M = fout*(P*N)/fclkin
		 * Where
		 * M = Feedback Multiplication Ratio
		 * N = Input Frequency Division Ratio
		 * P = Output Frequency Division Ratio
		 *
		 * However the following limits apply
		 * 24MHz >= fclkin/N >= 2MHz
		 * 2000 MHz >= fvco >= 500MHz
		 * 1000 >= M >= 250
		 * 12 >= N >= 1
		 */

		/* Determine valid N values based on fclkin */
		N_min = DC_PLL_N_MIN;
		N_max = DC_PLL_N_MAX;
		for (N = N_min; fclkin/N > 24; N++) {
			N_min = N+1;
		}
		for (N = N_max; fclkin/N < 2; N--) {
			N_max = N-1;
		}

		/* Calculate P based on the valid range for fvco  */
		P = DC_PLL_P_MIN;
		while (((mbps * P) <= DC_PLL_FVCO_MIN) && (P < DC_PLL_P_MAX))
			P *= 2;
		pll->output_div = P;

		/* Calculate M and N */
		for (N = N_min; N <= N_max; N++) {
			M = (mbps * P * N)/(pll->input_freq);
			if ((M >= DC_PLL_M_MIN)
			    && (M <= DC_PLL_M_MAX)) {
				fout = (pll->input_freq*M)/(P*N);
				while ((fout < mbps) && (M <= DC_PLL_M_MAX)) {
					M += 1;
					fout = (pll->input_freq*M)/(P*N);
				}
				if ((fout >= mbps) && (fout - mbps) < delta) {
					pll->loop_div = M;
					pll->input_div = N;
					pll->output_freq = fout;
					delta = fout - mbps;
					if (delta == 0)
						break;
				}
			}
		}

		if (mipicsi_pll_get_hsfreq(mbps, &(pll->hsfreq)) != 0)
			return -EINVAL;

		fvco = pll->output_freq*pll->output_div;
		for (i=0; i<(sizeof(pll_cp_lpf_ctrl_dc))/
			     (sizeof(struct pll_cp_lpf_ctrl)); i++) {
			if (fvco < pll_cp_lpf_ctrl_dc[i].vco_fmax) {
				pll->vco_range    = pll_cp_lpf_ctrl_dc[i].vco_range;
				pll->cp_current   = pll_cp_lpf_ctrl_dc[i].icpctrl;
				pll->lpf_resistor = pll_cp_lpf_ctrl_dc[i].lpfctrl;
				found = true;
				break;
			}
		}
		if (!found)
			return -EINVAL;
		if (pll->output_freq != mbps)
			pr_info("%s: NOTE: Actual PLL output freq set to %u\n",
				__func__, pll->output_freq);

		pr_info("%s: hsfreq=0x%x vco_range=0x%x cp=0x%x lpf=0x%x\n", __func__,
			pll->hsfreq, pll->vco_range, pll->cp_current,
			pll->lpf_resistor);

		pr_info("%s: M=0x%x N=0x%x P=0x%x\n", __func__, pll->loop_div,
			pll->input_div, pll->output_div);
	} else {
		uint32_t hz;

		if ((mbps < G3_PLL_MIN_MHZ) || (mbps > G3_PLL_MAX_MHZ))
			return -EINVAL;

		fout = mbps/2;

		/* Determine valid N values based on fclkin */
		N_min = G3_PLL_N_MIN;
		N_max = G3_PLL_N_MAX;

		for (N = N_min; fclkin/N >= 8; N++) {
			N_min = N+1;
		}
		for (N = N_max; fclkin/N < 2; N--) {
			N_max = N-1;
		}
		pr_info("Nmin=%d Nmax=%d\n", N_min, N_max);

		hz = fout*1000*1000;
		for (i=0; i<(sizeof(pll_vco_cntrl_tbl))/
			     (sizeof(struct pll_vco_cntrl_g3)); i++) {
			if (hz <= pll_vco_cntrl_tbl[i].vco_fmax) {
				pll->vco_cntrl    = pll_vco_cntrl_tbl[i].vco_cntrl;
				P                 = pll_vco_cntrl_tbl[i].P;
				found = true;
				break;
			}
		}
		if (!found)
			return -EINVAL;

		pll->sr_osc_freq_tgt = 0;
		pll->sr_range = 0;
		for (i=0; i<(sizeof(sr_tbl_g3))/(sizeof(struct sr_cntrl)); i++) {
			if (mbps <= sr_tbl_g3[i].fmax) {
				pll->sr_osc_freq_tgt = sr_tbl_g3[i].sr_osc_freq_tgt;
				pll->sr_range = sr_tbl_g3[i].sr_range;
				break;
			}
		}

		/* Calculate M and N */
		for (N = N_min; N <= N_max; N++) {
			M = (mbps/2 * N * P)/fclkin;
			if ((M >= G3_PLL_M_MIN)
			    && (M <= G3_PLL_M_MAX)) {
				fout = (pll->input_freq*M)/(P*N);
				while ((fout < mbps/2) && (M <= G3_PLL_M_MAX)) {
					M += 1;
					fout = (pll->input_freq*M)/(P*N);
				}
				if ((fout >= mbps/2) && (fout - mbps/2) < delta) {
					pll->loop_div = M;
					pll->input_div = N;
					pll->output_freq = fout;
					delta = fout - mbps/2;
					if (delta == 0)
						break;
				}
			}
		}

		if (mipicsi_pll_get_hsfreq(mbps, &(pll->hsfreq)) != 0)
			return -EINVAL;

		pr_info("fout=%d fclkin=%d\n\n", fout,fclkin);
		pr_info("vco_cntrl                 = 0x%x\n", pll->vco_cntrl);
		pr_info("M                         = %d\n", pll->loop_div);
		pr_info("N                         = %d\n\n", pll->input_div);

		if (pll->output_freq != mbps/2)
			pr_info("NOTE: Actual PLL output freq set to %u\n",
				pll->output_freq);
	}

	return 0;
}
