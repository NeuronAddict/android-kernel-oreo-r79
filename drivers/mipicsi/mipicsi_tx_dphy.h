#ifndef MIPICSI_TX_DPHY_H_
#define MIPICSI_TX_DPHY_H_

/* DesignWare® Cores MIPI D-PHY v1.2 Tx 4L for TSMC 28-nm HPC/1.8V */

/* Test Codes */
#define R_DPHY_TESTCODE_X_REG 0x0
#define R_DPHY_RDWR_TX_SYS_0 0x1
#define R_DPHY_RDWR_TX_SYS_1 0x2
#define R_DPHY_RDWR_TX_SYS_2 0x3
#define R_DPHY_RDWR_TX_SYS_3 0x4
#define R_DPHY_RDWR_TX_SYS_8 0x9
#define R_DPHY_RD_TX_SYS_0 0x1E
#define R_DPHY_RD_TX_SYS_1 0x1F
#define R_DPHY_RD_TX_SYS_2 0x20
#define R_DPHY_RD_TX_SYS_3 0x21
#define R_DPHY_RD_TX_SYS_4 0x22
#define R_DPHY_RD_CONT_DATA_0 0x23
#define R_DPHY_RD_CONT_DATA_1 0x24
#define R_DPHY_RDWR_TX_SYSTIMERS_0 0x4E
#define R_DPHY_RDWR_TX_SYSTIMERS_1 0x4F
#define R_DPHY_RDWR_TX_SYSTIMERS_2 0x50
#define R_DPHY_RDWR_TX_SYSTIMERS_3 0x51
#define R_DPHY_RDWR_TX_SYSTIMERS_4 0x52
#define R_DPHY_RDWR_TX_SYSTIMERS_5 0x53
#define R_DPHY_RDWR_TX_SYSTIMERS_6 0x54
#define R_DPHY_RDWR_TX_SYSTIMERS_7 0x55
#define R_DPHY_RDWR_TX_SYSTIMERS_8 0x56
#define R_DPHY_RDWR_TX_SYSTIMERS_9 0x57
#define R_DPHY_RDWR_TX_SYSTIMERS_10 0x58
#define R_DPHY_RDWR_TX_SYSTIMERS_11 0x59
#define R_DPHY_RDWR_TX_SYSTIMERS_12 0x5A
#define R_DPHY_RDWR_TX_SYSTIMERS_13 0x5B
#define R_DPHY_RDWR_TX_SYSTIMERS_14 0x5C
#define R_DPHY_RDWR_TX_SYSTIMERS_15 0x5D
#define R_DPHY_RDWR_TX_SYSTIMERS_16 0x5E
#define R_DPHY_RDWR_TX_SYSTIMERS_17 0x5F
#define R_DPHY_RDWR_TX_SYSTIMERS_18 0x60
#define R_DPHY_RDWR_TX_SYSTIMERS_19 0x61
#define R_DPHY_RDWR_TX_SYSTIMERS_20 0x62
#define R_DPHY_RDWR_TX_SYSTIMERS_21 0x63
#define R_DPHY_RDWR_TX_SYSTIMERS_22 0x64
#define R_DPHY_RDWR_TX_SYSTIMERS_23 0x65
#define R_DPHY_RD_TX_SYSTIMERS_0 0x80
#define R_DPHY_RD_TX_SYSTIMERS_1 0x81
#define R_DPHY_RD_TX_SYSTIMERS_2 0x82
#define R_DPHY_RD_TX_SYSTIMERS_3 0x83
#define R_DPHY_RD_TX_SYSTIMERS_4 0x84
#define R_DPHY_RD_TX_SYSTIMERS_5 0x85
#define R_DPHY_RD_TX_SYSTIMERS_6 0x86
#define R_DPHY_RD_TX_SYSTIMERS_7 0x87
#define R_DPHY_RD_TX_SYSTIMERS_8 0x88
#define R_DPHY_RD_TX_SYSTIMERS_9 0x89
#define R_DPHY_RD_TX_SYSTIMERS_10 0x8A
#define R_DPHY_RD_TX_SYSTIMERS_11 0x8B
#define R_DPHY_RD_TX_SYSTIMERS_12 0x8C
#define R_DPHY_RD_TX_TX_STARTUP_OBS_0 0x9C
#define R_DPHY_RDWR_TX_TX_STARTUP_OVR_0 0xB1
#define R_DPHY_RDWR_TX_TX_STARTUP_OVR_1 0xB2
#define R_DPHY_RDWR_TX_DEEMPHASIS_0 0xB3
#define R_DPHY_RDWR_TX_DEEMPHASIS_1 0xB4
#define R_DPHY_RDWR_TX_DEEMPHASIS_2 0xB5
#define R_DPHY_RDWR_TX_DEEMPHASIS_3 0xB6
#define R_DPHY_RDWR_TX_DEEMPHASIS_4 0xB7
#define R_DPHY_RDWR_TX_DEEMPHASIS_5 0xB8
#define R_DPHY_RDWR_TX_DEEMPHASIS_6 0xB9
#define R_DPHY_RDWR_TX_DEEMPHASIS_7 0xBA
#define R_DPHY_RDWR_TX_DEEMPHASIS_8 0xBB
#define R_DPHY_RDWR_TX_DEEMPHASIS_9 0xBC
#define R_DPHY_RDWR_TX_DEEMPHASIS_10 0xBD
#define R_DPHY_RDWR_TX_DEEMPHASIS_11 0xBE
#define R_DPHY_RDWR_TX_DEEMPHASIS_12 0xBF
#define R_DPHY_RDWR_TX_DEEMPHASIS_13 0xC0
#define R_DPHY_RDWR_TX_DEEMPHASIS_14 0xC1
#define R_DPHY_RDWR_TX_DEEMPHASIS_15 0xC2
#define R_DPHY_RDWR_TX_DEEMPHASIS_CONST_0 0xC3
#define R_DPHY_RDWR_TX_DEEMPHASIS_CONST_1 0xC4
#define R_DPHY_RDWR_TX_DEEMPHASIS_CONST_2 0xC5
#define R_DPHY_RDWR_TX_DEEMPHASIS_CONST_3 0xC6
#define R_DPHY_RDWR_TX_DEEMPHASIS_OFF_CONTROL_0 0xC7
#define R_DPHY_RDWR_TX_DEEMPHASIS_OFF_CONTROL_1 0xC8
#define R_DPHY_RDWR_TX_DEEMPHASIS_OFF_CONTROL_2 0xC9
#define R_DPHY_RDWR_TX_BIST_PM_CONTROL 0xCA
#define R_DPHY_RD_TX_DEEMPHASIS_OBS_0 0xCB
#define R_DPHY_RD_TX_DEEMPHASIS_OBS_1 0xCC
#define R_DPHY_RDWR_TX_BIST_3 0x10A
#define R_DPHY_RDWR_TX_BIST_4 0x10B
#define R_DPHY_RDWR_TX_BIST_OVR_2 0x112
#define R_DPHY_RDWR_TX_BIST_OVR_3 0x113
#define R_DPHY_RDWR_TX_BIST_PIPE_TIMER 0x114
#define R_DPHY_RDWR_TX_CHECK_ERROR_TIMER 0x115
#define R_DPHY_RD_TX_BIST_0 0x11E
#define R_DPHY_RD_TX_BIST_1 0x120
#define R_DPHY_RD_TX_BIST_2 0x121
#define R_DPHY_RD_TX_BIST_3 0x122
#define R_DPHY_RD_TX_BIST_4 0x123
#define R_DPHY_RD_TX_BIST_5 0x124
#define R_DPHY_RD_TX_BIST_6 0x125
#define R_DPHY_RD_TX_BIST_7 0x126
#define R_DPHY_RD_TX_BIST_8 0x127
#define R_DPHY_RD_TX_BIST_9 0x128
#define R_DPHY_RD_TX_BIST_10 0x129
#define R_DPHY_RD_TX_BIST_11 0x12A
#define R_DPHY_RD_TX_BIST_12 0x12B
#define R_DPHY_RD_TX_BIST_13 0x12C
#define R_DPHY_RDWR_TX_TX_DUAL_PHY_0 0x133
#define R_DPHY_RD_TX_TX_DUAL_PHY_0 0x148
#define R_DPHY_RDWR_TX_PLL_0 0x15D
#define R_DPHY_RDWR_TX_PLL_1 0x15E
#define R_DPHY_RDWR_TX_PLL_2 0x15F
#define R_DPHY_RDWR_TX_PLL_3 0x160
#define R_DPHY_RDWR_TX_PLL_4 0x161
#define R_DPHY_RDWR_TX_PLL_5 0x162
#define R_DPHY_RDWR_TX_PLL_6 0x163
#define R_DPHY_RDWR_TX_PLL_7 0x164
#define R_DPHY_RDWR_TX_PLL_8 0x165
#define R_DPHY_RDWR_TX_PLL_9 0x166
#define R_DPHY_RDWR_TX_PLL_10 0x167
#define R_DPHY_RDWR_TX_PLL_11 0x168
#define R_DPHY_RDWR_TX_PLL_12 0x169
#define R_DPHY_RDWR_TX_PLL_13 0x16A
#define R_DPHY_RDWR_TX_PLL_14 0x16B
#define R_DPHY_RDWR_TX_PLL_15 0x16C
#define R_DPHY_RDWR_TX_PLL_16 0x16D
#define R_DPHY_RDWR_TX_PLL_17 0x16E
#define R_DPHY_RDWR_TX_PLL_18 0x16F
#define R_DPHY_RDWR_TX_PLL_19 0x170
#define R_DPHY_RDWR_TX_PLL_20 0x171
#define R_DPHY_RDWR_TX_PLL_21 0x172
#define R_DPHY_RDWR_TX_PLL_22 0x173
#define R_DPHY_RDWR_TX_PLL_23 0x174
#define R_DPHY_RDWR_TX_PLL_24 0x175
#define R_DPHY_RDWR_TX_PLL_25 0x176
#define R_DPHY_RDWR_TX_PLL_26 0x177
#define R_DPHY_RDWR_TX_PLL_27 0x178
#define R_DPHY_RDWR_TX_PLL_28 0x179
#define R_DPHY_RDWR_TX_PLL_29 0x17A
#define R_DPHY_RDWR_TX_PLL_30 0x17B
#define R_DPHY_RDWR_TX_PLL_31 0x17C
#define R_DPHY_RD_TX_PLL_0 0x191
#define R_DPHY_RD_TX_PLL_1 0x192
#define R_DPHY_RD_TX_PLL_2 0x193
#define R_DPHY_RD_TX_PLL_3 0x194
#define R_DPHY_RD_TX_PLL_4 0x195
#define R_DPHY_RDWR_TX_CB_0 0x1AA
#define R_DPHY_RDWR_TX_CB_1 0x1AB
#define R_DPHY_RDWR_TX_CB_2 0x1AC
#define R_DPHY_RDWR_TX_CB_3 0x1AD
#define R_DPHY_RDWR_TX_CB_4 0x1AE
#define R_DPHY_RDWR_TX_CB_5 0x1AF
#define R_DPHY_RDWR_TX_CB_6 0x1B0
#define R_DPHY_RD_TX_ANA_CB_0 0x1C4
#define R_DPHY_RD_TX_ANA_CB_1 0x1C5
#define R_DPHY_RD_TX_ANA_CB_2 0x1C6
#define R_DPHY_RDWR_TX_ANA_CB_3 0x1C7
#define R_DPHY_RDWR_TX_DAC_0 0x1DA
#define R_DPHY_RDWR_TX_DAC_1 0x1DB
#define R_DPHY_RDWR_TX_DAC_2 0x1DC
#define R_DPHY_RDWR_TX_DAC_3 0x1DD
#define R_DPHY_RD_TX_DAC_0 0x1F2
#define R_DPHY_RD_TX_DAC_1 0x1F3
#define R_DPHY_RD_TX_DAC_2 0x1F4
#define R_DPHY_RDWR_TX_TERM_CAL_0 0x209
#define R_DPHY_RDWR_TX_TERM_CAL_1 0x20A
#define R_DPHY_RDWR_TX_TERM_CAL_2 0x20B
#define R_DPHY_RD_TX_TERM_CAL_0 0x220
#define R_DPHY_RD_TX_TERM_CAL_1 0x221
#define R_DPHY_RD_TX_TERM_CAL_2 0x222
#define R_DPHY_RDWR_TX_SLEW_0 0x26B
#define R_DPHY_RDWR_TX_SLEW_1 0x26C
#define R_DPHY_RDWR_TX_SLEW_2 0x26D
#define R_DPHY_RDWR_TX_SLEW_3 0x26E
#define R_DPHY_RDWR_TX_SLEW_4 0x26F
#define R_DPHY_RDWR_TX_SLEW_5 0x270
#define R_DPHY_RDWR_TX_SLEW_6 0x271
#define R_DPHY_RDWR_TX_SLEW_7 0x272
#define R_DPHY_RD_TX_SLEW_0 0x280
#define R_DPHY_RDWR_TX_CLKLANE_LANE_0 0x301
#define R_DPHY_RDWR_TX_CLKLANE_LANE_1 0x302
#define R_DPHY_RDWR_TX_CLKLANE_LANE_2 0x303
#define R_DPHY_RDWR_TX_CLKLANE_LANE_3 0x304
#define R_DPHY_RDWR_TX_CLKLANE_LANE_4 0x305
#define R_DPHY_RDWR_TX_CLKLANE_LANE_5 0x306
#define R_DPHY_RDWR_TX_CLKLANE_LANE_6 0x307
#define R_DPHY_RDWR_TX_CLKLANE_LANE_SLEWRATE_0 0x310
#define R_DPHY_RDWR_TX_CLKLANE_LANE_SLEWRATE_1 0x311
#define R_DPHY_RDWR_TX_CLKLANE_LANE_SLEWRATE_2 0x312
#define R_DPHY_RDWR_TX_CLKLANE_LANE_SLEWRATE_3 0x313
#define R_DPHY_RD_TX_CLKLANE_LANE_SLEWRATE_0 0x31A
#define R_DPHY_RD_TX_CLKLANE_LANE_SLEWRATE_1 0x31B
#define R_DPHY_RD_TX_CLKLANE_LANE_SLEWRATE_2 0x31C
#define R_DPHY_RD_TX_CLKLANE_LANE_SLEWRATE_3 0x31D
#define R_DPHY_RD_TX_CLKLANE_LANE_0 0x32B
#define R_DPHY_RD_TX_CLKLANE_LANE_1 0x32C
#define R_DPHY_RD_TX_CLKLANE_AFE_OBS_0 0x354
#define R_DPHY_RD_TX_CLKLANE_AFE_OBS_1 0x355
#define R_DPHY_RD_TX_CLKLANE_AFE_OBS_2 0x356
#define R_DPHY_RD_TX_CLKLANE_AFE_OBS_3 0x357
#define R_DPHY_RDWR_TX_CLKLANE_BIST_0 0x360
#define R_DPHY_RD_TX_CLKLANE_BIST_0 0x36A
#define R_DPHY_RDWR_TX_LANE0_LANE_0 0x501
#define R_DPHY_RDWR_TX_LANE0_LANE_1 0x502
#define R_DPHY_RDWR_TX_LANE0_LANE_2 0x503
#define R_DPHY_RDWR_TX_LANE0_LANE_3 0x504
#define R_DPHY_RDWR_TX_LANE0_LANE_4 0x505
#define R_DPHY_RDWR_TX_LANE0_LANE_5 0x506
#define R_DPHY_RDWR_TX_LANE0_LANE_6 0x507
#define R_DPHY_RDWR_TX_LANE0_LANE_7 0x508
#define R_DPHY_RDWR_TX_LANE0_LANE_8 0x509
#define R_DPHY_RDWR_TX_LANE0_LANE_9 0x50A
#define R_DPHY_RDWR_TX_LANE0_SLEWRATE_0 0x50B
#define R_DPHY_RDWR_TX_LANE0_SLEWRATE_1 0x50C
#define R_DPHY_RDWR_TX_LANE0_SLEWRATE_2 0x50D
#define R_DPHY_RDWR_TX_LANE0_SLEWRATE_3 0x50E
#define R_DPHY_RD_TX_LANE0_SLEWRATE_0 0x51A
#define R_DPHY_RD_TX_LANE0_SLEWRATE_1 0x51B
#define R_DPHY_RD_TX_LANE0_SLEWRATE_2 0x51C
#define R_DPHY_RD_TX_LANE0_SLEWRATE_3 0x51D
#define R_DPHY_RD_TX_LANE0_LANE_0 0x52B
#define R_DPHY_RD_TX_LANE0_LANE_1 0x52C
#define R_DPHY_RD_TX_LANE0_LANE_3 0x52E
#define R_DPHY_RD_TX_LANE0_LANE_4 0x52F
#define R_DPHY_RDWR_TX_LANE0_BIST_0 0x540
#define R_DPHY_RD_TX_LANE0_BIST_0 0x54A
#define R_DPHY_RD_TX_LANE0_AFE_OBS_0 0x554
#define R_DPHY_RD_TX_LANE0_AFE_OBS_1 0x555
#define R_DPHY_RD_TX_LANE0_AFE_OBS_2 0x556
#define R_DPHY_RD_TX_LANE0_AFE_OBS_3 0x557
#define R_DPHY_RDWR_TX_LANE1_LANE_0 0x701
#define R_DPHY_RDWR_TX_LANE1_LANE_1 0x702
#define R_DPHY_RDWR_TX_LANE1_LANE_2 0x703
#define R_DPHY_RDWR_TX_LANE1_LANE_3 0x704
#define R_DPHY_RDWR_TX_LANE1_LANE_4 0x705
#define R_DPHY_RDWR_TX_LANE1_LANE_5 0x706
#define R_DPHY_RDWR_TX_LANE1_LANE_6 0x707
#define R_DPHY_RDWR_TX_LANE1_LANE_7 0x708
#define R_DPHY_RDWR_TX_LANE1_LANE_8 0x709
#define R_DPHY_RDWR_TX_LANE1_SLEWRATE_0 0x70B
#define R_DPHY_RDWR_TX_LANE1_SLEWRATE_1 0x70C
#define R_DPHY_RDWR_TX_LANE1_SLEWRATE_2 0x70D
#define R_DPHY_RDWR_TX_LANE1_SLEWRATE_3 0x70E
#define R_DPHY_RD_TX_LANE1_SLEWRATE_0 0x71A
#define R_DPHY_RD_TX_LANE1_SLEWRATE_1 0x71B
#define R_DPHY_RD_TX_LANE1_SLEWRATE_2 0x71C
#define R_DPHY_RD_TX_LANE1_SLEWRATE_3 0x71D
#define R_DPHY_RD_TX_LANE1_LANE_0 0x72B
#define R_DPHY_RD_TX_LANE1_LANE_1 0x72C
#define R_DPHY_RD_TX_LANE1_LANE_4 0x72F
#define R_DPHY_RDWR_TX_LANE1_BIST_0 0x740
#define R_DPHY_RD_TX_LANE1_BIST_0 0x74A
#define R_DPHY_RD_TX_LANE1_AFE_OBS_4 0x758
#define R_DPHY_RD_TX_LANE1_AFE_OBS_5 0x759
#define R_DPHY_RD_TX_LANE1_AFE_OBS_6 0x75A
#define R_DPHY_RD_TX_LANE1_AFE_OBS_7 0x75B
#define R_DPHY_RDWR_TX_LANE2_LANE_0 0x901
#define R_DPHY_RDWR_TX_LANE2_LANE_1 0x902
#define R_DPHY_RDWR_TX_LANE2_LANE_2 0x903
#define R_DPHY_RDWR_TX_LANE2_LANE_3 0x904
#define R_DPHY_RDWR_TX_LANE2_LANE_4 0x905
#define R_DPHY_RDWR_TX_LANE2_LANE_5 0x906
#define R_DPHY_RDWR_TX_LANE2_LANE_6 0x907
#define R_DPHY_RDWR_TX_LANE2_LANE_7 0x908
#define R_DPHY_RDWR_TX_LANE2_LANE_8 0x909
#define R_DPHY_RDWR_TX_LANE2_SLEWRATE_0 0x90B
#define R_DPHY_RDWR_TX_LANE2_SLEWRATE_1 0x90C
#define R_DPHY_RDWR_TX_LANE2_SLEWRATE_2 0x90D
#define R_DPHY_RDWR_TX_LANE2_SLEWRATE_3 0x90E
#define R_DPHY_RD_TX_LANE2_SLEWRATE_0 0x91A
#define R_DPHY_RD_TX_LANE2_SLEWRATE_1 0x91B
#define R_DPHY_RD_TX_LANE2_SLEWRATE_2 0x91C
#define R_DPHY_RD_TX_LANE2_SLEWRATE_3 0x91D
#define R_DPHY_RD_TX_LANE2_LANE_0 0x92B
#define R_DPHY_RD_TX_LANE2_LANE_1 0x92C
#define R_DPHY_RD_TX_LANE2_LANE_4 0x92F
#define R_DPHY_RDWR_TX_LANE2_BIST_0 0x940
#define R_DPHY_RD_TX_LANE2_BIST_0 0x94A
#define R_DPHY_RD_TX_LANE2_AFE_OBS_4 0x958
#define R_DPHY_RD_TX_LANE2_AFE_OBS_5 0x959
#define R_DPHY_RD_TX_LANE2_AFE_OBS_6 0x95A
#define R_DPHY_RD_TX_LANE2_AFE_OBS_7 0x95B
#define R_DPHY_RDWR_TX_LANE3_LANE_0 0xB01
#define R_DPHY_RDWR_TX_LANE3_LANE_1 0xB02
#define R_DPHY_RDWR_TX_LANE3_LANE_2 0xB03
#define R_DPHY_RDWR_TX_LANE3_LANE_3 0xB04
#define R_DPHY_RDWR_TX_LANE3_LANE_4 0xB05
#define R_DPHY_RDWR_TX_LANE3_LANE_5 0xB06
#define R_DPHY_RDWR_TX_LANE3_LANE_6 0xB07
#define R_DPHY_RDWR_TX_LANE3_LANE_7 0xB08
#define R_DPHY_RDWR_TX_LANE3_LANE_8 0xB09
#define R_DPHY_RDWR_TX_LANE3_SLEWRATE_0 0xB0B
#define R_DPHY_RDWR_TX_LANE3_SLEWRATE_1 0xB0C
#define R_DPHY_RDWR_TX_LANE3_SLEWRATE_2 0xB0D
#define R_DPHY_RDWR_TX_LANE3_SLEWRATE_3 0xB0E
#define R_DPHY_RD_TX_LANE3_SLEWRATE_0 0xB1A
#define R_DPHY_RD_TX_LANE3_SLEWRATE_1 0xB1B
#define R_DPHY_RD_TX_LANE3_SLEWRATE_2 0xB1C
#define R_DPHY_RD_TX_LANE3_SLEWRATE_3 0xB1D
#define R_DPHY_RD_TX_LANE3_LANE_0 0xB2B
#define R_DPHY_RD_TX_LANE3_LANE_1 0xB2C
#define R_DPHY_RD_TX_LANE3_LANE_4 0xB2F
#define R_DPHY_RDWR_TX_LANE3_BIST_0 0xB40
#define R_DPHY_RD_TX_LANE3_BIST_0 0xB4A
#define R_DPHY_RD_TX_LANE3_AFE_OBS_4 0xB58
#define R_DPHY_RD_TX_LANE3_AFE_OBS_5 0xB59
#define R_DPHY_RD_TX_LANE3_AFE_OBS_6 0xB5A
#define R_DPHY_RD_TX_LANE3_AFE_OBS_7 0xB5B

#endif /* MIPICSI_TX_DPHY_H_ */