/* ======================================================================== */
/*  TEXAS INSTRUMENTS, INC.                                                 */
/*                                                                          */
/*  NAME                                                                    */
/*      DSPF_sp_fir_r2 -- Single Precision complex Finite Impulse Response       */
/*      Filter                                                              */
/*                                                                          */
/* USAGE                                                                    */
/*       This routine has the following C prototype:                        */
/*                                                                          */
/*       void DSPF_sp_fir_r2(                                                    */
/*                     const float * restrict x,                            */
/*                     const float * restrict h,                            */
/*                     float       * restrict r,                            */
/*                     int   nh,                                            */
/*                     int   nr                                             */
/*                     )                                                    */
/*            x[nr+nh-1]: Pointer to Input array of size nr+nh-1.           */
/*            h[nh]:      Pointer to Coefficient array of size nh.          */
/*                        (in reverse order).                               */
/*            r[nr]:      Pointer to Output array od size nr.               */
/*            nh:         Number of coefficients                            */
/*            nr:         Number of output samples.                         */
/*                                                                          */
/* DESCRIPTION                                                              */
/*       Computes a real FIR filter (direct-form) using coefficients        */
/*       stored in vector h[]. The real data input is stored in vector      */
/*       x[]. The filter output result is stored in vector r[]. The         */
/*       filter calculates nr output samples using nh coefficients.         */
/*       The coefficients are expected to be in reverse order.              */
/*                                                                          */
/*  TECHNIQUES                                                              */
/*                                                                          */
/*       The outer loop is unrolled four times and inner loop is            */
/*       unrolled twice.                                                    */
/*                                                                          */
/*  ASSUMPTIONS                                                             */
/*                                                                          */
/*       1. nr is a multiple of 2 and greater than or equal to 2.           */
/*       2. nh is a multiple of 2 and greater than or equal to 8.           */
/*       3. x and h are double word aligned.                                */
/*       4. Coefficients in array h are expected to be in revrse order.     */
/*                                                                          */
/* ------------------------------------------------------------------------ */
/*            Copyright (c) 2002 Texas Instruments, Incorporated.           */
/*                           All Rights Reserved.                           */
/* ======================================================================== */
#ifndef SP_fir_r2_ASM_H_
#define SP_fir_r2_ASM_H_ 1

void DSPF_sp_fir_r2(const     float *restrict x, const float *restrict h, float *restrict r, int nh, int nr);

#endif
/* ======================================================================== */
/*  End of file: DSPF_sp_fir_r2_h.asm                                            */
/* ------------------------------------------------------------------------ */
/*          Copyright (C) 2002 Texas Instruments, Incorporated.             */
/*                          All Rights Reserved.                            */
/* ======================================================================== */
