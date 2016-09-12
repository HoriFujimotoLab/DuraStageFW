/* ======================================================================== */
/*  TEXAS INSTRUMENTS, INC.                                                 */
/*                                                                          */
/*  NAME                                                                    */
/*      DSPF_sp_vecsum_sq -- Single Precision sum of squares                     */
/*                                                                          */
/*  USAGE                                                                   */
/*                                                                          */
/*      This routine is C callable, and has the following C prototype:      */
/*                                                                          */
/*      float DSPF_sp_vecsum_sq(const float *x,                                  */
/*                        int n                                             */
/*                       )                                                  */
/*                                                                          */
/*             x        :  Pointer to first input array                     */
/*             n        :  Number of elements in arrays                     */
/*                                                                          */
/*  DESCRIPTION                                                             */
/*                                                                          */
/*       This routine performs a sum of squares of the elements of the      */
/*   array x and returns the sum.                                           */
/*                                                                          */
/*    C CODE                                                                */
/*     This is the C equivalent of the Assembly Code without                */
/*     restrictions.                                                        */
/*                                                                          */
/*     float DSPF_sp_vecsum_sq(const    float *x,int n)                          */
/*     {                                                                    */
/*         int i;                                                           */
/*         float sum=0;                                                     */
/*                                                                          */
/*         for(i=0; i<n; i++)                                               */
/*             sum += x[i]*x[i];                                            */
/*                                                                          */
/*         return sum;                                                      */
/*     }                                                                    */
/*                                                                          */
/*                                                                          */
/* ------------------------------------------------------------------------ */
/*            Copyright (c) 2002 Texas Instruments, Incorporated.           */
/*                           All Rights Reserved.                           */
/* ======================================================================== */
#ifndef DSPF_SP_VECSUM_SQ_ASM_H_
#define DSPF_SP_VECSUM_SQ_ASM_H_ 1

float DSPF_sp_vecsum_sq(const     float * x, int n);

#endif
/* ======================================================================== */
/*  End of file: DSPF_sp_vecsum_sq_h.asm                                         */
/* ------------------------------------------------------------------------ */
/*          Copyright (C) 2002 Texas Instruments, Incorporated.             */
/*                          All Rights Reserved.                            */
/* ======================================================================== */
