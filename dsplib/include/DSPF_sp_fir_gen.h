/* ======================================================================== */
/*  TEXAS INSTRUMENTS, INC.                                                 */
/*                                                                          */
/*  NAME                                                                    */
/*      DSPF_sp_fir_gen -- Single Precision Generic FIR filter                   */
/*                                                                          */
/*   USAGE                                                                  */
/*                                                                          */
/*   This routine is C Callable and can be called as:                       */
/*                                                                          */
/*       void sp_firgen(const float *x, const float *h, float * restrict r, */
/*                  int nh, int nr);                                        */
/*                                                                          */
/*       x : Pointer to array holding the input floating point array        */
/*       h : Pointer to array holding the coefficient floating point        */
/*           array                                                          */
/*       y : Pointer to array holding the output floating point array       */
/*       nh: Number of coefficents                                          */
/*       nr: Number of output values                                        */
/*                                                                          */
/*   DESCRIPTION                                                            */
/*                                                                          */
/*       This routine implements a block FIR filter.  There are "nh"        */
/*       filter coefficients, "nr" output samples, and "nh+nr-1"            */
/*       input samples.    The coefficients need to be placed in the "h"    */
/*       array in reverse order {h(nh-1), ... , h(1), h(0)} and the         */
/*       array "x" starts at x(-nh+1) and ends at x(nr-1).  The             */
/*       routine calculates y(0) through y(nr-1) using the following        */
/*       formula:                                                           */
/*                                                                          */
/*       y(n) = h(0)*x(n) + h(1)*x(n-1) + ... + h(nh-1)*x(n-nh+1)           */
/*                                                                          */
/*       where n = {0, 1, ... , nr-1}.                                      */
/*   C CODE                                                                 */
/*       This is the C equivalent for the assembly code.  Note that         */
/*       the assembly code is hand optimized and restrictions may           */
/*       apply.                                                             */
/*                                                                          */
/*       void DSPF_sp_fir_gen(const float *x, const float *h, float * restrict r */
/*                   int nh, int nr)                                        */
/*       {                                                                  */
/*          int i, j;                                                       */
/*          float sum;                                                      */
/*                                                                          */
/*          for(j=0; j < nh; j++)                                           */
/*          {                                                               */
/*             sum = 0;                                                     */
/*             for(i=0; i < nr; i++)                                        */
/*             {                                                            */
/*                 sum += x[i+j] * h[i];                                    */
/*             }                                                            */
/*             r[j] = sum;                                                  */
/*          }                                                               */
/*       }                                                                  */
/* ------------------------------------------------------------------------ */
/*            Copyright (c) 2002 Texas Instruments, Incorporated.           */
/*                           All Rights Reserved.                           */
/* ======================================================================== */
#ifndef DSPF_SP_FIR_GEN_ASM_H_
#define DSPF_SP_FIR_GEN_ASM_H_ 1

void DSPF_sp_fir_gen(const     float * restrict x, const float * restrict h, float * restrict r, int nh, int nr);

#endif
/* ======================================================================== */
/*  End of file: DSPF_sp_fir_gen_h.asm                                           */
/* ------------------------------------------------------------------------ */
/*          Copyright (C) 2002 Texas Instruments, Incorporated.             */
/*                          All Rights Reserved.                            */
/* ======================================================================== */
