/**
 * @file    mnh_freq_cooling.h
 * @brief   Frequency cooling header for CPU/IPU/LPDDR frequency changes
 * @author  Intel
 * @date    19 OCT 2016
 * @version 0.1
 */

#include <linux/types.h>
#ifndef __LINUX_MNH_FREQ_COOLING_H
#define __LINUX_MNH_FREQ_COOLING_H

/*******************************************************************************
 *
 *  APIs exposed
 *
 ******************************************************************************/

int mnh_cpu_freq_change(int index);
int mnh_ipu_freq_change(int index);
int mnh_lpddr_freq_change(int index);
int mnh_ipu_reset(void);

#endif

