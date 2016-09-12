/* ======================================================================== */
/*  TEXAS INSTRUMENTS, INC.                                                 */
/*                                                                          */
/*  NAME                                                                    */
/*      DSPF_sp_dotp_sqr -- Single Precision dot product and sum of square       */
/*                                                                          */
/* USAGE                                                                    */
/*       This routine has the following C prototype:                        */
/*                                                                          */
/*       void DSPF_sp_dotp_sqr(                                                  */
/*                       float G,                                           */
/*                       const float * x,                                   */
/*                       const float * y,                                   */
/*                       float *  restrict r,                               */
/*                       int   nx                                           */
/*                       )                                                  */
/*            G:      Sum of y-squared initial value.                       */
/*            x[nx]:  Pointer to First input array.                         */
/*            y[nx]:  Pointer to Second input array.                        */
/*            r:      Pointer to Output for Accumulation of x[]*y[].        */
/*            nx:     Length of input vectors.                              */
/*                                                                          */
/*                                                                          */
/* DESCRIPTION                                                              */
/*       This routine computes the dot product of x[] and y[] arrays,adding */
/*       it to the value in the location pointed to by r. Additionally, it  */
/*       computes the sum of the squares of the terms in the y array,adding */
/*       it to the argument G. The final value of G is given as the return  */
/*       value of the function.                                             */
/*                                                                          */
/*       x and y should be aligned on a double word boundary.               */
/*                                                                          */
/*                                                                          */
/*  TECHNIQUES                                                              */
/*                                                                          */
/*      The loop is unrolled four times.                                    */
/*                                                                          */
/* ASSUMPTIONS                                                              */
/*                                                                          */
/*      1. nx is a multiple of 4.                                           */
/*      2. x and y should be double word aligned.                           */
/*                                                                          */
/* ------------------------------------------------------------------------ */
/*            Copyright (c) 2002 Texas Instruments, Incorporated.           */
/*                           All Rights Reserved.                           */
/* ======================================================================== */
#ifndef DSPF_SP_DOTP_SQR_ASM_H_
#define DSPF_SP_DOTP_SQR_ASM_H_ 1

float DSPF_sp_dotp_sqr(float     G, const float * x, const float * y, float *restrict r, int nx);

#endif
/* ======================================================================== */
/*  End of file: DSPF_sp_dotp_sqr_h.asm                                          */
/* ------------------------------------------------------------------------ */
/*          Copyright (C) 2002 Texas Instruments, Incorporated.             */
/*                          All Rights Reserved.                            */
/* ======================================================================== */
