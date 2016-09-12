/* ======================================================================== */
/*  TEXAS INSTRUMENTS, INC.                                                 */
/*                                                                          */
/*  NAME                                                                    */
/*      DSPF_sp_iir -- Single Precision IIR filter (used in the VSELP vocoder)   */
/*                                                                          */
/*  USAGE                                                                   */
/*                                                                          */
/*   This routine is C callable, and has the following C prototype:         */
/*                                                                          */
/*      void DSPF_sp_iir    (float* restrict r1,                                 */
/*                      const float*    x,                                  */
/*                      float* restrict r2,                                 */
/*                      const float*    h2,                                 */
/*                      const float*    h1,                                 */
/*                      int nr                                              */
/*                     );                                                   */
/*                                                                          */
/*           r1   :  Delay element values (i/p and o/p)                     */
/*           x    :  Pointer to the input array                             */
/*           r2   :  Pointer to the output array                            */
/*           h2   :  Auto-regressive filter coefficients                    */
/*           h1   :  Moving average filter coefficients                     */
/*           nr   :  Number of output samples                               */
/*                                                                          */
/*  DESCRIPTION                                                             */
/*                                                                          */
/*    The IIR performs an auto-regressive moving-average (ARMA)             */
/*    filter with 4 auto-regressive filter coefficients and 5               */
/*    moving-average filter coefficients for nr output samples.             */
/*    The output vector is stored in two locations. This routine            */
/*    is used as a high pass filter in the VSELP vocoder.                   */
/*                                                                          */
/*  C CODE                                                                  */
/*                                                                          */
/*  This is the C equivalent of the Assembly Code without                   */
/*  restrictions.                                                           */
/*                                                                          */
/*  Note that the assembly code is hand optimized and restrictions          */
/*  may apply.                                                              */
/*                                                                          */
/*  void DSPF_sp_iir (float* restrict r1,                                        */
/*                   const float*    x,                                     */
/*                   float* restrict r2,                                    */
/*                   const float*    h2,                                    */
/*                   const float*    h1,                                    */
/*                   int nr                                                 */
/*                  )                                                       */
/*      {                                                                   */
/*         int i, j;                                                        */
/*         float sum;                                                       */
/*                                                                          */
/*         for (i = 0; i < nr; i++)                                         */
/*         {                                                                */
/*             sum = h2[0] * x[4+i];                                        */
/*                                                                          */
/*             for (j = 1; j <= 4; j++)                                     */
/*                 sum += h2[j] * x[4+i-j] - h1[j] * r1[4+i-j];             */
/*                                                                          */
/*             r1[4+i] = sum;                                               */
/*             r2[i] = r1[4+i];                                             */
/*         }                                                                */
/*     }                                                                    */
/* ------------------------------------------------------------------------ */
/*            Copyright (c) 2002 Texas Instruments, Incorporated.           */
/*                           All Rights Reserved.                           */
/* ======================================================================== */
#ifndef DSPF_SP_IIR_ASM_H_
#define DSPF_SP_IIR_ASM_H_ 1

void DSPF_sp_iir(float     *restrict r1, const float * x, float *restrict r2, const float * h2, const float * h1, int nr);

#endif
/* ======================================================================== */
/*  End of file: DSPF_sp_iir_h.asm                                               */
/* ------------------------------------------------------------------------ */
/*          Copyright (C) 2002 Texas Instruments, Incorporated.             */
/*                          All Rights Reserved.                            */
/* ======================================================================== */
