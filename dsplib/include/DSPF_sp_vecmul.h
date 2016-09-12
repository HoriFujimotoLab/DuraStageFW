/* ======================================================================== */
/*  TEXAS INSTRUMENTS, INC.                                                 */
/*                                                                          */
/*  NAME                                                                    */
/*      DSPF_sp_vecmul -- Single Precision vector multiplication                 */
/*                                                                          */
/*  USAGE                                                                   */
/*                                                                          */
/*      This routine is C callable, and has the following C prototype:      */
/*                                                                          */
/*      void DSPF_sp_vecmul   (const float *x,                                   */
/*                        const float *x,                                   */
/*                        float * restrict r,                               */
/*                        int n                                             */
/*                       )                                                  */
/*                                                                          */
/*             x        :  Pointer to first input array                     */
/*             y        :  Pointer to second input array                    */
/*             r        :  Pointer to output array                          */
/*             n        :  Number of elements in arrays                     */
/*                                                                          */
/*  DESCRIPTION                                                             */
/*                                                                          */
/*       This routine performs an element by element floating point         */
/*   multiply of the vectors x[] and y[] and returns the values in r[].     */
/*                                                                          */
/*    C CODE                                                                */
/*     This is the C equivalent of the Assembly Code without                */
/*     restrictions.                                                        */
/*                                                                          */
/*     float sp_vecsum_sq(const    float *Åx,int n)                         */
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
#ifndef DSPF_SP_VECMUL_ASM_H_
#define DSPF_SP_VECMUL_ASM_H_ 1

void DSPF_sp_vecmul(const     float * x, const float * y, float * restrict r, int n);

#endif
/* ======================================================================== */
/*  End of file: DSPF_sp_vecmul_h.asm                                            */
/* ------------------------------------------------------------------------ */
/*          Copyright (C) 2002 Texas Instruments, Incorporated.             */
/*                          All Rights Reserved.                            */
/* ======================================================================== */
