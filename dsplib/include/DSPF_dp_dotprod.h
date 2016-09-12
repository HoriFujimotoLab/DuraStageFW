/* ======================================================================== */
/*  TEXAS INSTRUMENTS, INC.                                                 */
/*                                                                          */
/*  NAME                                                                    */
/*      DSPF_dp_dotprod -- Dot Product of 2 Double Precision float vectors       */
/*                                                                          */
/*  USAGE                                                                   */
/*                                                                          */
/*    This routine is C Callable and can be called as:                      */
/*                                                                          */
/*      double DSPF_dp_dotprod(const double *x, const double *y, const int nx);  */
/*                                                                          */
/*      x     : Pointer to array holding the first floating point vector    */
/*      y     : Pointer to array holding the second floating point vector   */
/*      nx    : Number of values in the x & y vectors                       */
/*                                                                          */
/*                                                                          */
/*  DESCRIPTION                                                             */
/*                                                                          */
/*      This routine calculates the dot product of 2 Double precision       */
/*  float vectors.                                                          */
/*                                                                          */
/*  C CODE                                                                  */
/*      This is the C equivalent for the assembly code.  Note that          */
/*      the assembly code is hand optimized and restrictions may            */
/*      apply.                                                              */
/*                                                                          */
/*      double DSPF_dp_dotprod(const double *x, const double *y, const int nx)   */
/*      {                                                                   */
/*         int i;                                                           */
/*         double sum = 0;                                                  */
/*                                                                          */
/*         for (i=0; i < nx; i++)                                           */
/*         {                                                                */
/*            sum += x[i] * y[i];                                           */
/*         }                                                                */
/*         return sum;                                                      */
/*      }                                                                   */
/*                                                                          */
/* ------------------------------------------------------------------------ */
/*            Copyright (c) 2003 Texas Instruments, Incorporated.           */
/*                           All Rights Reserved.                           */
/* ======================================================================== */
#ifndef DSPF_DP_DOTPROD_ASM_H_
#define DSPF_DP_DOTPROD_ASM_H_ 1

double DSPF_dp_dotprod(const     double * x, const double * y, const int nx);

#endif
/* ======================================================================== */
/*  End of file: DSPF_dp_dotprod_h.asm                                           */
/* ------------------------------------------------------------------------ */
/*          Copyright (C) 2003 Texas Instruments, Incorporated.             */
/*                          All Rights Reserved.                            */
/* ======================================================================== */
