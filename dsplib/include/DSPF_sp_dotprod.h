/* ======================================================================== */
/*  TEXAS INSTRUMENTS, INC.                                                 */
/*                                                                          */
/*  NAME                                                                    */
/*      DSPF_sp_dotprod -- Dot Product of 2 Single Precision float vectors       */
/*                                                                          */
/*  USAGE                                                                   */
/*                                                                          */
/*    This routine is C Callable and can be called as:                      */
/*                                                                          */
/*      float DSPF_sp_dotprod(const float *x, const float *y, const int nx);     */
/*                                                                          */
/*      x     : Pointer to array holding the first floating point vector    */
/*      y     : Pointer to array holding the second floating point vector   */
/*      nx    : Number of values in the x & y vectors                       */
/*                                                                          */
/*                                                                          */
/*  DESCRIPTION                                                             */
/*                                                                          */
/*      This routine calculates the dot product of 2 single precision       */
/*  float vectors.                                                          */
/*                                                                          */
/*  C CODE                                                                  */
/*      This is the C equivalent for the assembly code.  Note that          */
/*      the assembly code is hand optimized and restrictions may            */
/*      apply.                                                              */
/*                                                                          */
/*      float dotp(const float *x, const float *y, const int nx)            */
/*      {                                                                   */
/*         int i;                                                           */
/*         float sum = 0;                                                   */
/*                                                                          */
/*         for (i=0; i < nx; i++)                                           */
/*         {                                                                */
/*            sum += x[i] * y[i];                                           */
/*         }                                                                */
/*         return sum;                                                      */
/*      }                                                                   */
/*                                                                          */
/* ------------------------------------------------------------------------ */
/*            Copyright (c) 2002 Texas Instruments, Incorporated.           */
/*                           All Rights Reserved.                           */
/* ======================================================================== */
#ifndef DSPF_SP_DOTPROD_ASM_H_
#define DSPF_SP_DOTPROD_ASM_H_ 1

float DSPF_sp_dotprod(const     float * x, const float * y, const int nx);

#endif
/* ======================================================================== */
/*  End of file: DSPF_sp_dotprod_h.asm                                           */
/* ------------------------------------------------------------------------ */
/*          Copyright (C) 2002 Texas Instruments, Incorporated.             */
/*                          All Rights Reserved.                            */
/* ======================================================================== */
