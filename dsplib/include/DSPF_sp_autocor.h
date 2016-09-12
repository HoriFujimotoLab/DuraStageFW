/* ======================================================================== */
/*  TEXAS INSTRUMENTS, INC.                                                 */
/*                                                                          */
/*  NAME                                                                    */
/*      DSPF_sp_autocor -- Single Precision autocorrelation                      */
/*                                                                          */
/* USAGE                                                                    */
/*       This routine has the following C prototype:                        */
/*                                                                          */
/*       void DSPF_sp_autocor                                                    */
/*                     (                                                    */
/*                         float       *  restrict r,                       */
/*                         const short *  restrict x,                       */
/*                         int            nx,                               */
/*                         int            nr                                */
/*                     )                                                    */
/*            r  = pointer to output array of autocorrelation of length nr  */
/*            x  = pointer to input array of length nx+nr. Input data must  */
/*                 be padded with nr consecutive zeros at the beginning.    */
/*            nx = Length of Autocorrelation vector.                        */
/*            nr = Length of lags.                                          */
/*                                                                          */
/*                                                                          */
/* DESCRIPTION                                                              */
/*       This routine performs the autocorrelation of the input array inp.  */
/*       It is assumed that the length of the input array, x, is a multiple */
/*       of 2 and the length of the output array, r, is a multiple of 4.    */
/*       The assembly routine computes 4 output samples at a time.          */
/*                                                                          */
/*       It is assumed that input vector x is padded with nr no of zeros in */
/*       the beginning.                                                     */
/*                                                                          */
/*       x is to be aligned on a double word boundary.                      */
/*                                                                          */
/*                                                                          */
/*  TECHNIQUES                                                              */
/*                                                                          */
/*       The inner loop is unrolled twice and the outer loop is unrolled    */
/*       four times.                                                        */
/*                                                                          */
/*  ASSUMPTIONS                                                             */
/*                                                                          */
/*       1. nx is a multiple of 2 and greater than or equal to 4.           */
/*       2. nr is a multiple of 4 and greater than or equal to 4.           */
/*       3. nx is greater than or equal to nr                               */
/*       4. x  is double word aligned.                                      */
/*                                                                          */
/* ------------------------------------------------------------------------ */
/*            Copyright (c) 2002 Texas Instruments, Incorporated.           */
/*                           All Rights Reserved.                           */
/* ======================================================================== */
#ifndef DSPF_SP_AUTOCOR_ASM_H_
#define DSPF_SP_AUTOCOR_ASM_H_ 1

void DSPF_sp_autocor(float     *restrict r, float *restrict x, int nx, int nr);

#endif
/* ======================================================================== */
/*  End of file: DSPF_sp_autocor_h.asm                                           */
/* ------------------------------------------------------------------------ */
/*          Copyright (C) 2002 Texas Instruments, Incorporated.             */
/*                          All Rights Reserved.                            */
/* ======================================================================== */
