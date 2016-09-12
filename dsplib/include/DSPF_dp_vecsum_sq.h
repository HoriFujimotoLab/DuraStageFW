/* ======================================================================= */
/* TEXAS INSTRUMENTS, INC.                                                 */
/*                                                                         */
/* NAME                                                                    */
/*     DSPF_dp_vecsum_sq -- Double Precision sum of squares                     */
/*                                                                         */
/*  USAGE                                                                   */
/*                                                                          */
/*      This routine is C callable, and has the following C prototype:      */
/*                                                                          */
/*      double DSPF_dp_vecsum_sq(const double *x,                                */
/*                        int n                                             */
/*                       )                                                  */
/*                                                                          */
/*             x        :  Pointer to input array                           */
/*             n        :  Number of elements in array                      */
/*                                                                          */
/*  DESCRIPTION                                                             */
/*                                                                          */
/*       This routine performs a sum of squares of the elements of the      */
/*       array x and returns the sum.                                       */
/*                                                                          */
/*  TECHNIQUES                                                              */
/*                                                                          */
/*      1. The inner loop is unrolled twice. Hence, 2 registers are used    */
/*         to hold the sum of squares. ADDDPs are staggered.                */
/*                                                                          */
/*  ASSUMPTIONS                                                             */
/*                                                                          */
/*      1. Since loads of 4 doubles beyond the array occur,                 */
/*         a pad must be provided.                                          */
/*                                                                          */
/*  C CODE                                                                  */
/*                                                                          */
/*     This is the C equivalent of the Assembly Code without                */
/*     restrictions. Note that the assembly code is hand optimized          */
/*     and restrictions may apply.                                          */
/*                                                                          */
/*     double DSPF_dp_vecsum_sq(const double *x,int n)                           */
/*     {                                                                    */
/*         int i;                                                           */
/*         double sum=0;                                                    */
/*                                                                          */
/*         for(i = 0;  i < n; i++ )                                         */
/*             sum += x[i]*x[i];                                            */
/*                                                                          */
/*         return sum;                                                      */
/*     }                                                                    */
/*                                                                          */
/*                                                                          */
/*  NOTES                                                                   */
/*                                                                          */
/*    1. Endian: This code is ENDIAN NEUTRAL.                               */
/*    2. Interruptibility: This code is interrupt tolerant                  */
/*                            but not interruptible.                        */
/*                                                                          */
/*  CYCLES                                                                  */
/*                                                                          */
/*    4*Ceil(n/2) + 33                                                      */
/*    eg. for n = 100, cycles = 233                                         */
/*                                                                          */
/*  CODESIZE                                                                */
/*                                                                          */
/*    288 bytes                                                             */
/* ------------------------------------------------------------------------ */
/*            Copyright (c) 2004 Texas Instruments, Incorporated.           */
/*                           All Rights Reserved.                           */
/* ======================================================================== */
#ifndef DSPF_DP_VECSUM_SQ_H_
#define DSPF_DP_VECSUM_SQ_H_ 1

double DSPF_dp_vecsum_sq(const     double *x,
                  int n
                 );

#endif
/* ======================================================================== */
/*  End of file:  DSPF_dp_vecsum_sq.h                                            */
/* ------------------------------------------------------------------------ */
/*            Copyright (c) 2004 Texas Instruments, Incorporated.           */
/*                           All Rights Reserved.                           */
/* ======================================================================== */
