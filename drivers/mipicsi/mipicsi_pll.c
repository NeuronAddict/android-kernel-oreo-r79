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


#define MIPICSI_PLL_M_MIN 250
#define MIPICSI_PLL_M_MAX 1000
#define MIPICSI_PLL_N_MIN 1
#define MIPICSI_PLL_N_MAX 12
#define MIPICSI_PLL_P_MIN 1
#define MIPICSI_PLL_P_MAX 8
#define MIPICSI_PLL_FVCO_MIN 500
#define MIPICSI_PLL_FVCO_MAX 2000



struct pll_freqrange {
	uint32_t min_range;
	uint32_t max_range;
	uint32_t default_bps;
	uint8_t hsfreq;
};

/* Table 5-2: Frequency Ranges and Defaults */
struct pll_freqrange freqrange_dc[] = {
	{80000000, 110250000, 80000000, 0x00},
	{90250000, 152250000, 120000000, 0x01},
	{128250000, 194250000, 160000000, 0x02},
	{171000000, 241500000, 205000000, 0x03},
	{249375000, 301875000, 275000000, 0x04},
	{296875000, 354375000, 325000000, 0x05},
	{403750000, 498750000, 450000000, 0x06},
	{498750000, 603750000, 550000000, 0x07},
	{593750000, 708750000, 650000000, 0x08},
	{688750000, 813750000, 750000000, 0x09},
	{878750000, 1023750000, 950000000, 0x0A},
	{1068750000, 1233750000, 1150000000, 0x0B},
	{1258750000, 1443750000, 1350000000, 0x0C},
	{1448750000, 1653750000, 1550000000, 0x0D},
	{1591250000, 1811250000, 1700000000, 0x0E},
	{1781250000, 2000000000, 1900000000, 0x0F},
	{80000000, 120750000, 90000000, 0x10},
	{99750000, 162750000, 130000000, 0x11},
	{137750000, 204750000, 170000000, 0x12},
	{185250000, 257250000, 220000000, 0x13},
	{273125000, 328125000, 300000000, 0x14},
	{320625000, 393750000, 350000000, 0x15},
	{451250000, 551250000, 500000000, 0x16},
	{546250000, 656250000, 600000000, 0x17},
	{641250000, 761250000, 700000000, 0x18},
	{736250000, 866250000, 800000000, 0x19},
	{926250000, 1076250000, 1000000000, 0x1A},
	{1116250000, 1286250000, 1200000000, 0x1B},
	{1306250000, 1496250000, 1400000000, 0x1C},
	{1496250000, 1706250000, 1600000000, 0x1D},
	{1638750000, 1863750000, 1750000000, 0x1E},
	{1828750000, 2000000000, 1950000000, 0x1F},
	{80000000, 131250000, 100000000, 0x20},
	{109250000, 173250000, 140000000, 0x21},
	{147250000, 215250000, 180000000, 0x22},
	{199500000, 273000000, 235000000, 0x23},
	{356250000, 446250000, 400000000, 0x25},
	{783750000, 918750000, 850000000, 0x29},
	{973750000, 1128750000, 1050000000, 0x2A},
	{1163750000, 1338750000, 1250000000, 0x2B},
	{1353750000, 1548750000, 1450000000, 0x2C},
	{1543750000, 1758750000, 1650000000, 0x2D},
	{1686250000, 1916250000, 1800000000, 0x2E},
	{1876250000, 2000000000, 2000000000, 0x2F},
	{80750000, 141750000, 110000000, 0x30},
	{118750000, 183750000, 150000000, 0x31},
	{156750000, 225750000, 190000000, 0x32},
	{237500000, 275625000, 250000000, 0x33},
	{831250000, 971250000, 900000000, 0x39},
	{1021250000, 1181250000, 1100000000, 0x3A},
	{1211250000, 1391250000, 1300000000, 0x3B},
	{1401250000, 1601250000, 1500000000, 0x3C},
	{1733750000, 1968750000, 1850000000, 0x3E},
	{1781250000, 2000000000, 1900000000, 0x0F},
	{1828750000, 2000000000, 1950000000, 0x1F},
	{1876250000, 2000000000, 2000000000, 0x2F}
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


int mipicsi_pll_calc(uint16_t mbps, struct mipicsi_pll *pll)
{
	uint16_t i, M, N, P, N_min, N_max, fout, delta = 0xffff;
	uint32_t bps, fvco;
	bool found = false;

	if ((mbps < MIPICSI_PLL_MIN_FREQ) || (mbps > MIPICSI_PLL_MAX_FREQ))
		return -EINVAL;

	/* Hardcode input freq for now */
	pll->input_freq = MIPICSI_PLL_INP_FREQ;

	pr_info("%s: PLL calculation for target freq %u\n", __func__, mbps);

#ifdef MNH_EMULATION
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
	N_min = MIPICSI_PLL_N_MIN;
	N_max = MIPICSI_PLL_N_MAX;
	for (N = N_min; pll->input_freq/N > 24; N++) {
		N_min = N+1;
	}
	for (N = N_max; pll->input_freq/N < 2; N--) {
		N_max = N-1;
	}

	/* Calculate P based on the valid range for fvco  */
	P = MIPICSI_PLL_P_MIN;
	while (((mbps * P) <= MIPICSI_PLL_FVCO_MIN) && (P < MIPICSI_PLL_P_MAX))
		P *= 2;
	pll->output_div = P;

	/* Calculate M and N */
	for (N = N_min; N <= N_max; N++) {
		M = (mbps * pll->output_div * N)
			/(pll->input_freq);
		if ((M >= MIPICSI_PLL_M_MIN)
		    && (M <= MIPICSI_PLL_M_MAX)) {
			fout = (pll->input_freq*M)/(P*N);
			while ((fout < mbps) && (M <= MIPICSI_PLL_M_MAX)) {
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

	bps = pll->output_freq*1000*1000;

	for (i = 0; i < (sizeof(freqrange_dc))/(sizeof(struct pll_freqrange));
	     i++) {
		if ((bps == freqrange_dc[i].default_bps) ||
		    ((bps > freqrange_dc[i].min_range) &&
		     (bps < freqrange_dc[i].max_range))) {
			pll->hsfreq = freqrange_dc[i].hsfreq;
			found = true;
			break;
		}
	}

	if (!found)
		return -EINVAL;

	found = false;
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
#endif

	/* TO DO - Add support for Gen 3 */

	if (pll->output_freq != mbps)
		pr_info("%s: NOTE: Actual PLL output freq set to %u\n",
			__func__, pll->output_freq);

	pr_info("%s: hsfreq=0x%x vco_range=0x%x cp=0x%x lpf=0x%x\n", __func__,
		pll->hsfreq, pll->vco_range, pll->cp_current,
		pll->lpf_resistor);

	pr_info("%s: M=0x%x N=0x%x P=0x%x\n", __func__, pll->loop_div,
		 pll->input_div, pll->output_div);

	return 0;
}
