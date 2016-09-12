/* ======================================================================= */
/* TEXAS INSTRUMENTS, INC.                                                 */
/*                                                                         */
/* NAME                                                                    */
/*     DSPF_dp_fir_r2 -- Double Precision complex Finite Impulse Response       */
/*     Filter                                                              */
/*                                                                         */
/* USAGE                                                                    */
/*       This routine has the following C prototype:                        */
/*                                                                          */
/*       void DSPF_dp_fir_r2(                                                    */
/*                     const double * restrict x,                           */
/*                     const double * restrict h,                           */
/*                     double      * restrict r,                            */
/*                     int   nh,                                            */
/*                     int   nr                                             */
/*                     )                                                    */
/*            x[nr+nh-1]: Pointer to Input array of size nr+nh-1.           */
/*            h[nh]     : Pointer to coefficient array of size nh.          */
/*                        (in reverse order).                               */
/*            r[nr]     : Pointer to output array of size nr.               */
/*            nh        : Number of coefficients                            */
/*            nr        : Number of output samples.                         */
/*                                                                          */
/* DESCRIPTION                                                              */
/*                                                                          */
/*       Computes a real FIR filter (direct-form) using coefficients        */
/*       stored in vector h[]. The real data input is stored in vector      */
/*       x[]. The filter output result is stored in vector r[]. The         */
/*       filter calculates nr output samples using nh coefficients.         */
/*       The coefficients are expected to be in reverse order.              */
/*                                                                          */
/*  TECHNIQUES                                                              */
/*                                                                          */
/*       1. The outer loop is unrolled four times and inner loop is         */
/*          unrolled twice.                                                 */
/*       2. Register sharing is used to make optimum utilisation of         */
/*          available Registers                                             */
/*       3. Outer loop instructions are executed in parallel with           */
/*          inner loop.                                                     */
/*                                                                          */
/*  ASSUMPTIONS                                                             */
/*                                                                          */
/*       1. nr is a multiple of 2 and greater than or equal to 2.           */
/*       2. nh is a multiple of 2 and greater than or equal to 8.           */
/*       3. Coefficients in array h are expected to be in reverse order.    */
/*       4. x and h should be padded with 4 words at the end.               */
/*                                                                          */
/* C CODE                                                                   */
/*                                                                          */
/*  This is the C equivalent of the assembly code.  Note that               */
/*  the assembly code is hand optimized and restrictions may apply.         */
/*                                                                          */
/*  void DSPF_dp_fir_r2(const double * x, const double * h,                      */
/*                     double *restrict r, int nh, int nr)                  */
/*  {                                                                       */
/*      int i, j;                                                           */
/*      double sum;                                                         */
/*                                                                          */
/*      for (i = 0; i < nr; i++)                                            */
/*      {                                                                   */
/*          sum = 0;                                                        */
/*                                                                          */
/*          for (j = 0; j < nh; j++)                                        */
/*             sum += x[i + j] * h[j];                                      */
/*             r[i] = sum;                                                  */
/*      }                                                                   */
/*  }                                                                       */
/*                                                                          */
/*                                                                          */
/* NOTES                                                                    */
/*                                                                          */
/*   1. Endian: This code is LITTLE ENDIAN.                                 */
/*   2. Interruptibility: This code is interrupt tolerant but not           */
/*      interruptible.                                                      */
/*                                                                          */
/* CYCLES                                                                   */
/*                                                                          */
/*   (8*nh + 10)*ceil(nr/4) + 32                                            */
/*    For nh=24 and nr=62, cycles=3264                                      */
/*                                                                          */
/*                                                                          */
/* CODESIZE                                                                 */
/*                                                                          */
/*  672 bytes                                                               */
/* ------------------------------------------------------------------------ */
/*            Copyright (c) 2004 Texas Instruments, Incorporated.           */
/*                           All Rights Reserved.                           */
/* ======================================================================== */
#ifndef DSPF_DP_FIR_R2_H_
#define DSPF_DP_FIR_R2_H_ 1

void DSPF_dp_fir_r2(
              const double * restrict x,
              const double * restrict h,
              double      * restrict r,
              int   nh,
              int   nr
              );

#endif
/* ======================================================================== */
/*  End of file:  DSPF_dp_fir_r2.h                                               */
/* ------------------------------------------------------------------------ */
/*            Copyright (c) 2004 Texas Instruments, Incorporated.           */
/*                           All Rights Reserved.                           */
/* ======================================================================== */
