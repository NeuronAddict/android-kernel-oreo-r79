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
#ifndef __INTEL_HWIO_H_
#define __INTEL_HWIO_H_

#define HW_IN(bAddr,mod,reg)        readl(HWIO_##mod##_##reg##_ADDR(bAddr, 0))
#define HW_INx(bAddr,mod,reg,inst)  readl(HWIO_##mod##_##reg##_ADDR(bAddr, inst))
#define HW_PRTx(bAddr, mod, reg, inst) printk(#mod" "#reg" "#inst" %p\n",\
	HWIO_##mod##_##reg##_ADDR(bAddr, inst))

#define HW_INf(bAddr,mod,reg,fld)                                       \
  ((readl(HWIO_##mod##_##reg##_ADDR(bAddr, 0)) & HWIO_##mod##_##reg##_##fld##_FLDMASK) >> \
   HWIO_##mod##_##reg##_##fld##_FLDSHFT)
#define HW_INxf(bAddr,mod,reg,inst,fld) \
  ((readl(HWIO_##mod##_##reg##_ADDR(bAddr, inst)) & HWIO_##mod##_##reg##_##fld##_FLDMASK) >> \
   HWIO_##mod##_##reg##_##fld##_FLDSHFT)

#define HW_OUT(bAddr,mod,reg,val)       writel(val, HWIO_##mod##_##reg##_ADDR(bAddr, 0))
#define HW_PRT(bAddr, mod, reg)       printk(#mod" "#reg" %p\n",\
	HWIO_##mod##_##reg##_ADDR(bAddr, 0))

#define HW_OUTx(bAddr,mod,reg,inst,val) writel(val, HWIO_##mod##_##reg##_ADDR(bAddr, inst))

#define HW_OUTf(bAddr,mod,reg,fld,val)					\
  writel(								\
	 (readl(HWIO_##mod##_##reg##_ADDR(bAddr, 0)) & ~HWIO_##mod##_##reg##_##fld##_FLDMASK) | \
	 ((val << HWIO_##mod##_##reg##_##fld##_FLDSHFT) & HWIO_##mod##_##reg##_##fld##_FLDMASK), \
	 HWIO_##mod##_##reg##_ADDR(bAddr, 0))

#define HW_OUTxf(bAddr,mod,reg,inst,fld,val)                            \
  writel(								\
	 (readl(HWIO_##mod##_##reg##_ADDR(bAddr, inst)) & ~HWIO_##mod##_##reg##_##fld##_FLDMASK) | \
	 ((val << HWIO_##mod##_##reg##_##fld##_FLDSHFT) & HWIO_##mod##_##reg##_##fld##_FLDMASK), \
	 HWIO_##mod##_##reg##_ADDR(bAddr, inst))


#define HW_IN64(bAddr,mod,reg)        readq(HWIO_##mod##_##reg##_ADDR(bAddr, 0))
#define HW_IN64x(bAddr,mod,reg,inst)  readq(HWIO_##mod##_##reg##_ADDR(bAddr, inst))
#define HW_IN64f(bAddr,mod,reg,fld)                                       \
  ((readq(HWIO_##mod##_##reg##_ADDR(bAddr, 0)) & HWIO_##mod##_##reg##_##fld##_FLDMASK) >> \
   HWIO_##mod##_##reg##_##fld##_FLDSHFT)
#define HW_IN64xf(bAddr,mod,reg,inst,fld) \
  ((readq(HWIO_##mod##_##reg##_ADDR(bAddr, inst)) & HWIO_##mod##_##reg##_##fld##_FLDMASK) >> \
   HWIO_##mod##_##reg##_##fld##_FLDSHFT)

#define HW_OUT64(bAddr,mod,reg,val)       writeq(val, HWIO_##mod##_##reg##_ADDR(bAddr, 0))
#define HW_OUT64x(bAddr,mod,reg,inst,val) writeq(val, HWIO_##mod##_##reg##_ADDR(bAddr, inst))

#define HW_OUT64f(bAddr,mod,reg,fld,val)                                  \
  writeq(\
	 (readq(HWIO_##mod##_##reg##_ADDR(bAddr, 0)) & ~HWIO_##mod##_##reg##_##fld##_FLDMASK) | \
	 ((val << HWIO_##mod##_##reg##_##fld##_FLDSHFT) & HWIO_##mod##_##reg##_##fld##_FLDMASK), \
	 HWIO_##mod##_##reg##_ADDR(bAddr, 0))

#define HW_OUT64xf(bAddr,mod,reg,inst,fld,val)                            \
  writeq(\
	 (readq(HWIO_##mod##_##reg##_ADDR(bAddr, inst)) & ~HWIO_##mod##_##reg##_##fld##_FLDMASK) | \
	 ((val << HWIO_##mod##_##reg##_##fld##_FLDSHFT) & HWIO_##mod##_##reg##_##fld##_FLDMASK), \
	 HWIO_##mod##_##reg##_ADDR(bAddr, inst))


#endif // __INTEL_HWIO_H_
