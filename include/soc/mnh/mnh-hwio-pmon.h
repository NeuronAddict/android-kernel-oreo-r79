/* auto generated: Wednesday, August 17th, 2016 6:17:55pm */
/*
 * Copyright (c) 2016, Intel Corporation. All rights reserved.
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
#ifndef __MNH_HWIO_PMON_
#define __MNH_HWIO_PMON_

#define HWIO_PMON_GC_REGOFF 0x0
#define HWIO_PMON_GC_ADDR(bAddr, regX) (bAddr + HWIO_PMON_GC_REGOFF)
#define HWIO_PMON_GC_RSVD0_FLDMASK (0xfffffff0)
#define HWIO_PMON_GC_RSVD0_FLDSHFT (4)
#define HWIO_PMON_GC_TIMESTAMP_CNT_CLR_FLDMASK (0x8)
#define HWIO_PMON_GC_TIMESTAMP_CNT_CLR_FLDSHFT (3)
#define HWIO_PMON_GC_TIMESTAMP_CNT_EN_FLDMASK (0x4)
#define HWIO_PMON_GC_TIMESTAMP_CNT_EN_FLDSHFT (2)
#define HWIO_PMON_GC_GLBL_CLR_FLDMASK (0x2)
#define HWIO_PMON_GC_GLBL_CLR_FLDSHFT (1)
#define HWIO_PMON_GC_GLBL_EN_FLDMASK (0x1)
#define HWIO_PMON_GC_GLBL_EN_FLDSHFT (0)

#define HWIO_PMON_ST_L_REGOFF 0x4
#define HWIO_PMON_ST_L_ADDR(bAddr, regX) (bAddr + HWIO_PMON_ST_L_REGOFF)
#define HWIO_PMON_ST_L_ST_L_FLDMASK (0xffffffff)
#define HWIO_PMON_ST_L_ST_L_FLDSHFT (0)

#define HWIO_PMON_ST_H_REGOFF 0x8
#define HWIO_PMON_ST_H_ADDR(bAddr, regX) (bAddr + HWIO_PMON_ST_H_REGOFF)
#define HWIO_PMON_ST_H_ST_H_FLDMASK (0xffffffff)
#define HWIO_PMON_ST_H_ST_H_FLDSHFT (0)

#define HWIO_PMON_IE_REGOFF 0x0C
#define HWIO_PMON_IE_ADDR(bAddr, regX) (bAddr + HWIO_PMON_IE_REGOFF)
#define HWIO_PMON_IE_RSVD0_FLDMASK (0xfffffffc)
#define HWIO_PMON_IE_RSVD0_FLDSHFT (2)
#define HWIO_PMON_IE_SNAPSHOT_TIME_INTR_EN_FLDMASK (0x2)
#define HWIO_PMON_IE_SNAPSHOT_TIME_INTR_EN_FLDSHFT (1)
#define HWIO_PMON_IE_TIMESTAMP_OVFL_INTR_EN_FLDMASK (0x1)
#define HWIO_PMON_IE_TIMESTAMP_OVFL_INTR_EN_FLDSHFT (0)

#define HWIO_PMON_IS_REGOFF 0x10
#define HWIO_PMON_IS_ADDR(bAddr, regX) (bAddr + HWIO_PMON_IS_REGOFF)
#define HWIO_PMON_IS_RSVD0_FLDMASK (0xfffffffc)
#define HWIO_PMON_IS_RSVD0_FLDSHFT (2)
#define HWIO_PMON_IS_SNAPSHOT_TIME_INTR_STS_FLDMASK (0x2)
#define HWIO_PMON_IS_SNAPSHOT_TIME_INTR_STS_FLDSHFT (1)
#define HWIO_PMON_IS_TIMESTAMP_OVFL_INTR_STS_FLDMASK (0x1)
#define HWIO_PMON_IS_TIMESTAMP_OVFL_INTR_STS_FLDSHFT (0)

#define HWIO_PMON_TS_L_REGOFF 0x14
#define HWIO_PMON_TS_L_ADDR(bAddr, regX) (bAddr + HWIO_PMON_TS_L_REGOFF)
#define HWIO_PMON_TS_L_TS_L_FLDMASK (0xffffffff)
#define HWIO_PMON_TS_L_TS_L_FLDSHFT (0)

#define HWIO_PMON_TS_H_REGOFF 0x18
#define HWIO_PMON_TS_H_ADDR(bAddr, regX) (bAddr + HWIO_PMON_TS_H_REGOFF)
#define HWIO_PMON_TS_H_TS_H_FLDMASK (0xffffffff)
#define HWIO_PMON_TS_H_TS_H_FLDSHFT (0)

#define HWIO_PMON_PRMU_INTR_REGOFF 0x1C
#define HWIO_PMON_PRMU_INTR_ADDR(bAddr, regX) (bAddr + HWIO_PMON_PRMU_INTR_REGOFF)
#define HWIO_PMON_PRMU_INTR_PRMU_INTR_STS_FLDMASK (0xffffffff)
#define HWIO_PMON_PRMU_INTR_PRMU_INTR_STS_FLDSHFT (0)

#endif // __MNH_HWIO_PMON_
