/* ======================================================================= */
/* TEXAS INSTRUMENTS, INC.                                                 */
/*                                                                         */
/* NAME                                                                    */
/*     DSPF_dp_dotp_cplx -- Complex Double precision floating point dot         */
/*     product                                                             */
/*                                                                         */
/* USAGE                                                                    */
/*                                                                          */
/*    This routine is C Callable and can be called as:                      */
/*                                                                          */
/*      void DSPF_dp_dotp_cplx(const double *x, const double *y, int n, double * */
/*                        restrict re, double * restrict im);               */
/*                                                                          */
/*      x     : Pointer to array holding the first floating point vector    */
/*      y     : Pointer to array holding the second floating point vector   */
/*      n     : Number of values in the x & y vectors                       */
/*      re    : Pointer to the location storing the real                    */
/*              part of the result.                                         */
/*      im    : Pointer to the location storing the imaginary part of       */
/*              the result                                                  */
/*                                                                          */
/*                                                                          */
/* DESCRIPTION                                                              */
/*                                                                          */
/*      This routine calculates the dot product of 2 Double   precision     */
/*  complex float vectors. The even numbered locations hold the real parts  */
/*  of the complex numbers while the odd numbered locations contain the     */
/*  imaginary portions.                                                     */
/*                                                                          */
/* ASSUMPTIONS                                                              */
/*                                                                          */
/*      1.  The value of nx must be  > 0.                                   */
/*                                                                          */
/*                                                                          */
/* C CODE                                                                   */
/*                                                                          */
/*      This is the C equivalent for the assembly code.  Note that          */
/*      the assembly code is hand optimized and restrictions may            */
/*      apply.                                                              */
/*                                                                          */
/*      void DSPF_dp_dotp_cplx(const double* x, const double* y, int n,          */
/*                    double* restrict re, double* restrict im)             */
/*      {                                                                   */
/*                                                                          */
/*          double real=0, imag=0;                                          */
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
/*                                                                          */
/* NOTES                                                                    */
/*                                                                          */
/*  1. Endian: This code is LITTLE ENDIAN.                                  */
/*  2. Interruptibility: This code is interrupt tolerant but not            */
/*     interruptible.                                                       */
/*                                                                          */
/*                                                                          */
/* CYCLES                                                                   */
/*                                                                          */
/*     8*N + 29                                                             */
/*     eg. for N = 128, cycles = 1053.                                      */
/*                                                                          */
/* CODESIZE                                                                 */
/*                                                                          */
/*     352 bytes                                                            */
/* ------------------------------------------------------------------------ */
/*            Copyright (c) 2004 Texas Instruments, Incorporated.           */
/*                           All Rights Reserved.                           */
/* ======================================================================== */
#ifndef DSPF_DP_DOTP_CPLX_H_
#define DSPF_DP_DOTP_CPLX_H_ 1

void DSPF_dp_dotp_cplx(const     double *x, const double *y, int n, double *
                  restrict re, double * restrict im);

#endif
/* ======================================================================== */
/*  End of file:  DSPF_dp_dotp_cplx.h                                            */
/* ------------------------------------------------------------------------ */
/*            Copyright (c) 2004 Texas Instruments, Incorporated.           */
/*                           All Rights Reserved.                           */
/* ======================================================================== */
