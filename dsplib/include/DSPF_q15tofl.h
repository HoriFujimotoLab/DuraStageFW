/* ======================================================================== */
/*  TEXAS INSTRUMENTS, INC.                                                 */
/*                                                                          */
/*  NAME                                                                    */
/*      DSPF_q15tofl -- Q15 format to Single Precision IEEE floating point       */
/*      format                                                              */
/*                                                                          */
/*  USAGE                                                                   */
/*                                                                          */
/*    This routine is C Callable and can be called as:                      */
/*                                                                          */
/*      float DSPF_q15tofl(short *x, float *r, int nx);                          */
/*                                                                          */
/*      x     : Input array containing shorts in Q15 format                 */
/*      r     : Output array containing equivalent floats                   */
/*      nx    : Number of values in the x vector                            */
/*                                                                          */
/*                                                                          */
/*  DESCRIPTION                                                             */
/*                                                                          */
/*      This routine converts data in the Q15 format into IEEE Single Preci */
/*      floating point.                                                     */
/*                                                                          */
/*  C CODE                                                                  */
/*                                                                          */
/*      This is the C equivalent for the assembly code.  Note that          */
/*      the assembly code is hand optimized and restrictions may            */
/*      apply.                                                              */
/*                                                                          */
/*  void DSPF_q15tofl(short    *x, float *r, int nx)                             */
/*     {                                                                    */
/*      int i;                                                              */
/*      for (i=0;i<nx;i++)                                                  */
/*           r[i]=(float)x[i]/0x8000;                                       */
/*     }                                                                    */
/*                                                                          */
/*                                                                          */
/* ------------------------------------------------------------------------ */
/*            Copyright (c) 2002 Texas Instruments, Incorporated.           */
/*                           All Rights Reserved.                           */
/* ======================================================================== */
#ifndef DSPF_Q15TOFL_ASM_H_
#define DSPF_Q15TOFL_ASM_H_ 1

void DSPF_q15tofl(const     short * x, float * restrict r, int nx);

#endif
/* ======================================================================== */
/*  End of file: DSPF_q15tofl_h.asm                                              */
/* ------------------------------------------------------------------------ */
/*          Copyright (C) 2002 Texas Instruments, Incorporated.             */
/*                          All Rights Reserved.                            */
/* ======================================================================== */
