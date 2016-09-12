/* ======================================================================= */
/* TEXAS INSTRUMENTS, INC.                                                 */
/*                                                                         */
/* NAME                                                                    */
/*     DSPF_dp_vecmul -- Double Precision vector multiplication                 */
/*                                                                         */
/*  USAGE                                                                   */
/*                                                                          */
/*      This routine is C callable, and has the following C prototype:      */
/*                                                                          */
/*      void DSPF_dp_vecmul   (const double *x,                                  */
/*                        const double *y,                                  */
/*                        double * restrict r,                              */
/*                        int n                                             */
/*                        )                                                 */
/*                                                                          */
/*             x        :  Pointer to first input array                     */
/*             y        :  Pointer to second input array                    */
/*             r        :  Pointer to output array                          */
/*             n        :  Number of elements in arrays                     */
/*                                                                          */
/*  DESCRIPTION                                                             */
/*                                                                          */
/*       This routine performs an element by element Double Precision       */
/*       floating point  multiplication of the vectors x[] and y[] and      */
/*       returns the values in r[].                                         */
/*                                                                          */
/*   TECHNIQUES                                                             */
/*                                                                          */
/*      1. The inner loop is unrolled twice to allow calculation of         */
/*         2 outputs in the kernel. However the stores are executed         */
/*         conditionally to allow 'n' to be any number > 0.                 */
/*                                                                          */
/*   ASSUMPTIONS                                                            */
/*                                                                          */
/*      1. The value of n > 0.                                              */
/*                                                                          */
/*                                                                          */
/*    C CODE                                                                */
/*                                                                          */
/*     This is the C equivalent of the Assembly Code without                */
/*     restrictions.                                                        */
/*                                                                          */
/*     void DSPF_dp_vecmul(const double * x, const double * y,                   */
/*                           double * restrict r, int n)                    */
/*      {                                                                   */
/*        int i;                                                            */
/*                                                                          */
/*        for(i = 0; i < n; i++)                                            */
/*             r[i] = x[i] * y[i];                                          */
/*                                                                          */
/*      }                                                                   */
/*                                                                          */
/*                                                                          */
/*    NOTES                                                                 */
/*                                                                          */
/*       1. Endian: This code is LITTLE ENDIAN.                             */
/*       2. Interruptibility: The code is interrupt tolerant but not        */
/*          interruptible.                                                  */
/*                                                                          */
/*    CYCLES                                                                */
/*                                                                          */
/*         4*Ceil(n/2) + 13                                                 */
/*         eg. for n = 100, cycles = 213                                    */
/*                                                                          */
/*    CODESIZE                                                              */
/*                                                                          */
/*         256 bytes                                                        */
/*                                                                          */
/* ------------------------------------------------------------------------ */
/*            Copyright (c) 2004 Texas Instruments, Incorporated.           */
/*                           All Rights Reserved.                           */
/* ======================================================================== */
#ifndef DSPF_DP_VECMUL_H_
#define DSPF_DP_VECMUL_H_ 1

void DSPF_dp_vecmul       (const double *x,
                  const double *y,
                  double * restrict r,
                  int n
                  );

#endif
/* ======================================================================== */
/*  End of file:  DSPF_dp_vecmul.h                                               */
/* ------------------------------------------------------------------------ */
/*            Copyright (c) 2004 Texas Instruments, Incorporated.           */
/*                           All Rights Reserved.                           */
/* ======================================================================== */
