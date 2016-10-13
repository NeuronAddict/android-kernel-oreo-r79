#ifndef MIPICSI_DC_H_
#define MIPICSI_DC_H_


/***************** PHY Related Register Address *****************/

/* Daughtercard Phy Test Codes From Spec */
#define R_CSI2_DCPHY_PLL_BIAS_FCC_VCO     0x10
#define R_CSI2_DCPHY_PLL_CP_LOCK_BYP_ULP  0X11
#define R_CSI2_DCPHY_PLL_LPF_CP_CTRL      0x12
#define R_CSI2_DCPHY_PLL_INPUT_DIV_RAT    0x17
#define R_CSI2_DCPHY_PLL_LOOP_DIV_RAT     0x18
#define R_CSI2_DCPHY_PLL_DIV_RAT_CTRL     0x19
#define R_CSI2_DCPHY_PLL_BYPASS           0x1A
#define R_CSI2_DCPHY_AFE_BYPASS_BANDGAP   0x22
#define R_CSI2_DCPHY_MASTER_SLAVEZ        0x3B
#define R_CSI2_DCPHY_HS_TX_PWR_CTRL_CLK   0x30
#define R_CSI2_DCPHY_HS_RX_CTRL_CLK       0x34
#define R_CSI2_DCPHY_HS_TX_PWR_CTRL_L0    0x40
#define R_CSI2_DCPHY_HS_RX_CTRL_L0        0x44
#define R_CSI2_DCPHY_HS_TX_PWR_CTRL_L1    0x50
#define R_CSI2_DCPHY_HS_RX_CTRL_L1        0x54
#define R_CSI2_DCPHY_BASEDIR_L0           0x4B
#define R_CSI2_DCPHY_BASEDIR_L1           0x5B
#define R_CSI2_DCPHY_HS_TX_PWR_CTRL_L2    0x80
#define R_CSI2_DCPHY_HS_RX_CTRL_L2        0x84
#define R_CSI2_DCPHY_BASEDIR_L2           0x8B
#define R_CSI2_DCPHY_HS_TX_PWR_CTRL_L3    0x90
#define R_CSI2_DCPHY_HS_RX_CTRL_L3        0x94
#define R_CSI2_DCPHY_BASEDIR_L3           0x9B


/* Masks and Values */
#define DC_RX_BASEDIR_VAL       0x03
#define DC_SLAVE_VAL            0x08

#define DC_TX_BASEDIR_VAL       0x02
#define DC_MASTER_VAL           0x0C

#endif /* MIPICSI_DC_H_ */
