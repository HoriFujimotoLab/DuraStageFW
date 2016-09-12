/* ======================================================================== */
/*  TEXAS INSTRUMENTS, INC.                                                 */
/*                                                                          */
/*  NAME                                                                    */
/*      DSPF_sp_fir_cplx -- Single Precision complex Finite Impulse Response     */
/*      Filter                                                              */
/*                                                                          */
/* USAGE                                                                    */
/*       This routine has the following C prototype:                        */
/*                                                                          */
/*       void DSPF_sp_fir_cplx(                                                  */
/*                        const float * restrict x,                         */
/*                        const float * restrict h,                         */
/*                        float *       restrict r,                         */
/*                        int   nh,                                         */
/*                        int   nr                                          */
/*                       )                                                  */
/*            x[2*(nr+nh-1)]:Pointer to complex Input array.                */
/*                           The input data pointer x must point to the     */
/*                           (nh)th complex element, i.e. element 2*(nh-1). */
/*            h[2*nh]:       Pointer to complex Coefficient array           */
/*                           (in normal order).                             */
/*            r[2*nr]:       Pointer to complex Output array.               */
/*            nh:            Number of complex coefficients in vector h.    */
/*            nr:            Number of complex output samples to calculate. */
/*                                                                          */
/* DESCRIPTION                                                              */
/*       This function implements the FIR filter for complex input data.    */
/*       The filter has nr output samples and nh coefficients. Each array   */
/*       consists of an even and odd term with even terms representing the  */
/*       real part and the odd terms the imaginary part of the element.     */
/*       The coefficients are expected in normal order.                     */
/*                                                                          */
/*  TECHNIQUES                                                              */
/*                                                                          */
/*       The outer loop is unrolled twice.                                  */
/*                                                                          */
/*  ASSUMPTIONS                                                             */
/*                                                                          */
/*       1. nr is a multiple of 2 and greater than or equal to 2.           */
/*       2. nh is greater than or equal to 5.                               */
/*       3. x and h are double word aligned.                                */
/*       4. x points to 2*(nh-1)th input element.                           */
/*                                                                          */
/* ------------------------------------------------------------------------ */
/*            Copyright (c) 2002 Texas Instruments, Incorporated.           */
/*                           All Rights Reserved.                           */
/* ======================================================================== */
#ifndef DSPF_SP_FIR_CPLX_ASM_H_
#define DSPF_SP_FIR_CPLX_ASM_H_ 1

void DSPF_sp_fir_cplx(const     float *restrict x, const float *restrict h, float *restrict r, int nh, int nr);

#endif
/* ======================================================================== */
/*  End of file: DSPF_sp_fir_cplx_h.asm                                          */
/* ------------------------------------------------------------------------ */
/*          Copyright (C) 2002 Texas Instruments, Incorporated.             */
/*                          All Rights Reserved.                            */
/* ======================================================================== */
