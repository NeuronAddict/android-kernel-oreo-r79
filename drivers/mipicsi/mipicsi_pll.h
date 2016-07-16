#ifndef MIPICSI_PLL_H_
#define MIPICSI_PLL_H_

#ifdef MNH_EMULATION
#define MIPICSI_PLL_INP_FREQ 25
#define MIPICSI_PLL_MIN_FREQ 80
#define MIPICSI_PLL_MAX_FREQ 2000
#else
#define MIPICSI_PLL_MIN_FREQ 40
#define MIPICSI_PLL_MAX_FREQ 1250
#endif

struct mipicsi_pll {
	uint16_t  output_freq;      /* Active output frequency */
	uint16_t  input_freq;       /* Active input frequency */
	uint16_t  ref_freq;         /* Reference Frequency */
	uint16_t  hsfreq;           /* hsfreq */
	uint8_t   vco_range;        /* vcorange */
	uint16_t  loop_div;         /* (M) */
	uint8_t   cp_current;       /* icpctrl */
	uint8_t   lpf_resistor;     /* lpfctrl */
	uint8_t   input_div;        /* (N) */
	uint8_t	  output_div;       /* (P) */
};

int mipicsi_pll_calc(uint16_t mbps, struct mipicsi_pll *pll);

#endif /* MIPICSI_PLL_H_ */
