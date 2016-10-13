/*
 * Copyright (c) 2016, Intel Corporation. All rights reserved.
 *
 * Author: Harish Subramony <harish.subramony@intel.com>
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

#define MNH_TIMER_WIDTH                 32

/* Timer Base Address*/
//#define MNH_TIMER1_REG  	   			0x60083000

/* Lenght of each timer register*/
#define MNH_TIMER_REG_LENGTH      			0x14


/* Timer Config Registers */
#define MNH_TIMER_LOADCOUNT_OFFSET		0x00  
#define MNH_TIMER_LOADCOUNT2_OFFSET 	0xB0  
#define MNH_TIMER_CURRENTVALUE_OFFSET   0x04
#define MNH_TIMER_CONTROLREG_OFFSET     0x08
#define MNH_TIMER_EOI_OFFSET            0x0C
#define MNH_TIMER_INT_STATUS_OFFSET     0x10

#define MNH_TIMERS_INT_STATUS           0xA0
#define MNH_TIMERS_EOI                  0xA4
#define MNH_TIMERS_RAW_INT_STATUS       0xA8
#define MNH_TIMERS_COMP_VERSION         0xAC

//Control Reg bit 0
#define MNH_TIMER_ENABLE                0x01
//Control Reg bit 1
#define MNH_TIMER_MODE_USER_DEFINED     (0x01<<1)
#define MNH_TIMER_MODE_FREE_RUNNING     0x00
//Control Reg bit 2
#define MNH_TIMER_INTERRUPT_MASK        (0x01<<2)


