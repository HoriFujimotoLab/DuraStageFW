/* ======================================================================== */
/*  TEXAS INSTRUMENTS, INC.                                                 */
/*                                                                          */
/*  NAME                                                                    */
/*      DSPF_fltoq15 -- IEEE Single Precision floating point to Q15 format       */
/*                                                                          */
/*  USAGE                                                                   */
/*     This routine is C-callable and can be called as:                     */
/*                                                                          */
/*     void DSPF_fltoq15                                                         */
/*     (                                                                    */
/*         const float* restrict x,                                         */
/*         short*       restrict r,                                         */
/*         int         nx                                                   */
/*     );                                                                   */
/*                                                                          */
/*     x[nx] :  Input array contaning values of type float                  */
/*     r[nx] :  Output array contains Q15 equivalents of x[nx]              */
/*     nx    :  Number of elements in both arrays                           */
/*                                                                          */
/*  DESCRIPTION                                                             */
/*     Convert the IEEE floating point numbers stored in vector x[] into    */
/*     Q.15 format numbers stored in vector r[]. Results will be rounded    */
/*     towards negative infinity. All values that exceed the size limit     */
/*     will be saturated to 0x7fff if value is positive and 0x8000 if       */
/*     value is negative.                                                   */
/*                                                                          */
/*  C CODE                                                                  */
/*     void DSPF_fltoq15                                                         */
/*     (                                                                    */
/*         const float* restrict x,                                         */
/*         short*       restrict r,                                         */
/*         int         nx                                                   */
/*     )                                                                    */
/*     {                                                                    */
/*         int i, a;                                                        */
/*                                                                          */
/*         for(i = 0; i < nx; i++)                                          */
/*         {                                                                */
/*             a = floor(32768 * x[i]);                                     */
/*                                                                          */
/*             // saturate to 16-bit //                                     */
/*             if (a>32767)  a =  32767;                                    */
/*             if (a<-32768) a = -32768;                                    */
/*                                                                          */
/*             r[i] = (short) a;                                            */
/*         }                                                                */
/*     }                                                                    */
/* ------------------------------------------------------------------------ */
/*            Copyright (c) 2002 Texas Instruments, Incorporated.           */
/*                           All Rights Reserved.                           */
/* ======================================================================== */
#ifndef DSPF_FLTOQ15_ASM_H_
#define DSPF_FLTOQ15_ASM_H_ 1

void DSPF_fltoq15(const     float * x, short * restrict r, int nx);

#endif
/* ======================================================================== */
/*  End of file: DSPF_fltoq15_h.asm                                              */
/* ------------------------------------------------------------------------ */
/*          Copyright (C) 2002 Texas Instruments, Incorporated.             */
/*                          All Rights Reserved.                            */
/* ======================================================================== */
