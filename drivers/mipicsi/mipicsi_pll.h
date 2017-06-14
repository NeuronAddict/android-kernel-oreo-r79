#ifndef MIPICSI_PLL_H_
#define MIPICSI_PLL_H_

struct mipicsi_pll {
	uint16_t  output_freq;      /* Active output frequency */
	uint16_t  input_freq;       /* Active input frequency */
	uint16_t  ref_freq;         /* Reference Frequency */
	uint16_t  loop_div;         /* (M) */
	uint8_t   input_div;        /* (N) */
	uint8_t	  output_div;       /* (P) */
	uint8_t   hsfreq;           /* hsfreq */
	uint8_t   vco_range;        /* vcorange (dc)*/
	uint8_t   cp_current;       /* icpctrl (dc)*/
	uint8_t   lpf_resistor;     /* lpfctrl (dc)*/
	uint8_t   vco_cntrl;        /* vco_cntrl (g3)*/
	uint8_t   sr_range;         /* sr_range (g3) */
	uint16_t  sr_osc_freq_tgt;  /* sr_osc_freq_tgt (g3) */
	uint16_t  rx_osc_freq_tgt;  /* rx_osc_freq_tgt (g3) */
};

int mipicsi_pll_get_hsfreq (uint16_t mbps, uint8_t *hsfreq);
int mipicsi_pll_get_stop_wait (uint16_t mbps, uint8_t *stop_wait);
int mipicsi_pll_calc(uint16_t mbps, struct mipicsi_pll *pll);

#endif /* MIPICSI_PLL_H_ */
