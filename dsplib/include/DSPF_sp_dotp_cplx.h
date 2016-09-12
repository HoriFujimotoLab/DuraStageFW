/* ======================================================================== */
/*  TEXAS INSTRUMENTS, INC.                                                 */
/*                                                                          */
/*  NAME                                                                    */
/*      DSPF_sp_dotp_cplx -- Complex single precision floating point dot         */
/*      product                                                             */
/*                                                                          */
/*  USAGE                                                                   */
/*                                                                          */
/*    This routine is C Callable and can be called as:                      */
/*                                                                          */
/*      void DSPF_sp_dotp_cplx(const float *x, const float *y, int n, float *    */
/*                        restrict re, float * restrict im);                */
/*                                                                          */
/*      x     : Pointer to array holding the first floating point vector    */
/*      y     : Pointer to array holding the second floating point vector   */
/*      n     : Number of values in the x & y vectors                       */
/*      re    : Pointer to the location storing the real part of the result */
/*      im    : Pointer to the location storing the imaginary part of       */
/*          the result                                                      */
/*                                                                          */
/*                                                                          */
/*  DESCRIPTION                                                             */
/*                                                                          */
/*      This routine calculates the dot product of 2 single precision       */
/*  complex float vectors. The even numbered locations hold the real parts  */
/*  of the complex numbers while the odd numbered locations contain the     */
/*  imaginary portions.                                                     */
/*                                                                          */
/*  TECHNIQUES                                                              */
/*                                                                          */
/*      1.  The loop is unrolled 4 times because the loop carried           */
/*      dependency bound is 4.                                              */
/*                                                                          */
/*      2.  LDDW instructions are used to load two SP floating point        */
/*          values at a time for the x and y arrays.                        */
/*                                                                          */
/*      3.  A load counter avoids all extraneous loads.                     */
/*  ASSUMPTIONS                                                             */
/*                                                                          */
/*      1.  Little Endian is assumed for LDDW instructions.                 */
/*      2.  Since single assignment of registers is not used,               */
/*          interrupts should be disabled before this function is           */
/*          called.                                                         */
/*      3.  Loop counter must be > 0.                                       */
/*  4.  The x and y arrays must be double word aligned.                     */
/*                                                                          */
/*  C CODE                                                                  */
/*      This is the C equivalent for the assembly code.  Note that          */
/*      the assembly code is hand optimized and restrictions may            */
/*      apply.                                                              */
/*                                                                          */
/*      void DSPF_sp_dotp_cplx(const    float* x, const float* y,                */
/*                       int n, float* restrict re,                         */
/*               float* restrict im)                                        */
/*      {                                                                   */
/*                                                                          */
/*          float real=0, imag=0;                                           */
/*          int i=0;                                                        */
/*                                                                          */
/*          for(i=0; i<n; i++)                                              */
/*          {                                                               */
/*              real+=(x[2*i]*y[2*i] - x[2*i+1]*y[2*i+1]);                  */
/*              imag+=(x[2*i]*y[2*i+1] + x[2*i+1]*y[2*i]);                  */
/*          }                                                               */
/*                                                                          */
/*          *re=real;                                                       */
/*          *im=imag;                                                       */
/*     }                                                                    */
/*                                                                          */
/* ------------------------------------------------------------------------ */
/*            Copyright (c) 2002 Texas Instruments, Incorporated.           */
/*                           All Rights Reserved.                           */
/* ======================================================================== */
#ifndef DSPF_SP_DOTP_CPLX_ASM_H_
#define DSPF_SP_DOTP_CPLX_ASM_H_ 1

void DSPF_sp_dotp_cplx(const     float * x, const float * y, int n, float * restrict re, float * restrict im);

#endif
/* ======================================================================== */
/*  End of file: DSPF_sp_dotp_cplx_h.asm                                         */
/* ------------------------------------------------------------------------ */
/*          Copyright (C) 2002 Texas Instruments, Incorporated.             */
/*                          All Rights Reserved.                            */
/* ======================================================================== */
