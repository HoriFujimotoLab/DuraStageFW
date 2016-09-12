/* ============================================================================*/
/*                                                                             */
/*  TEXAS INSTRUMENTS, INC.                                                    */
/*                                                                             */
/*  NAME                                                                       */
/*      DSPF_sp_biquad: Biquad filter (IIR filter of 2nd order)                     */
/*                                                                             */
/*  USAGE                                                                      */
/*      This routine has following C prototype:                                */
/*                                                                             */
/*           void DSPF_sp_biquad (                                                  */
/*                         float x[],    Pointer to input samples              */
/*                         float b[],    Pointer to Nr coefs b0, b1, b2        */
/*                         float a[],    Pointer to Dr coefs a1, a2            */
/*                         float delay[] Pointer to filter delays              */
/*                         float r[],    Pointer to output samples             */
/*                         int   nx      Number of input/output samples        */
/*                         )                                                   */
/*                                                                             */
/*                                                                             */
/*                                                                             */
/*  DESCRIPTION                                                                */
/*      This routine implements a DF 2 transposed structure of the biquad      */
/*      filter. The transfer function of a biquad can be written               */ 
/*      as:                                                                    */
/*                                                                             */
/*                                   b(0) + b(1)z^(-1) + b(2)z^(-2)            */
/*                       H(Z) =     _________________________________          */
/*                                    1 + a(1)z^(-1) + a(2)z^(-2)              */
/*                                                                             */
/* ============================================================================*/
/*            Copyright (c) 2002 Texas Instruments, Incorporated.              */
/*                           All Rights Reserved.                              */
/* ============================================================================*/

#ifndef DSPF_SP_BIQUAD_ASM
#define DSPF_SP_BIQUAD_ASM 1

void DSPF_sp_biquad     (
                float x[],         //Pointer to input samples
                float b[],         //Pointer to numerator coefficients b[0], b[1] and b[2]
                float a[],         //Pointer to denominator coefficients a[1] and a[2]
                float delay[],     //Pointer to filter delays delay[0] and delay[1]
                float r[],         //Pointer to output samples
                int      nx         //Number of input/output samples
               );
#endif
/* ======================================================================== */
/*  End of file:  DSPF_sp_biquad.h                                               */
/* ------------------------------------------------------------------------ */
/*            Copyright (c) 2002 Texas Instruments, Incorporated.           */
/*                           All Rights Reserved.                           */
/* ======================================================================== */




