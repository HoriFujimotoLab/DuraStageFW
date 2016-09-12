/* ======================================================================= */
/* TEXAS INSTRUMENTS, INC.                                                 */
/*                                                                         */
/* NAME                                                                    */
/*     DSPF_dp_dotp_sqr -- Double Precision dot product and sum of square       */
/*                                                                         */
/*   USAGE                                                                  */
/*                                                                          */
/*       This routine has the following C prototype:                        */
/*                                                                          */
/*       double DSPF_dp_dotp_sqr(                                                */
/*                       double G,                                          */
/*                       const double * x,                                  */
/*                       const double * y,                                  */
/*                       double *  restrict r,                              */
/*                       int   nx                                           */
/*                       )                                                  */
/*            G:      Sum of y-squared initial value.                       */
/*            x[nx]:  Pointer to First input array.                         */
/*            y[nx]:  Pointer to Second input array.                        */
/*            r:      Pointer to Output for Accumulation of x[]*y[].        */
/*            nx:     Length of input vectors.                              */
/*                                                                          */
/*   DESCRIPTION                                                            */
/*                                                                          */
/*       This routine computes the dot product of x[] and y[] arrays,       */
/*   adding it to the value in the location pointed to by r.                */
/*   Additionally, it computes the sum of the squares of the terms          */
/*   in the y array,adding it to the argument G. The final value of         */
/*   G is given as the return value of the function.                        */
/*                                                                          */
/*   TECHNIQUES                                                             */
/*                                                                          */
/*     1. Multiple Assignment was used to reduce loop carry path.           */
/*                                                                          */
/*   ASSUMPTIONS                                                            */
/*                                                                          */
/*     1. There are no special alignment requirements.                      */
/*                                                                          */
/*   C CODE                                                                 */
/*                                                                          */
/*      This is the C equivalent of the assembly code.  Note that           */
/*      the assembly code is hand optimized and restrictions may apply.     */
/*                                                                          */
/*      double DSPF_dp_dotp_sqr(double G, const double * x, const double * y,    */
/*                        double *restrict r, int nx)                       */
/*      {                                                                   */
/*                                                                          */
/*         int i;                                                           */
/*                                                                          */
/*         for (i = 0; i < nx; i++)                                         */
/*         {                                                                */
/*               *r += x[i] * y[i];     // Compute Dot Product //           */
/*               G += y[i] * y[i];      // Compute Square  //               */
/*         }                                                                */
/*                                                                          */
/*         return G;                                                        */
/*      }                                                                   */
/*                                                                          */
/*                                                                          */
/*    NOTES                                                                 */
/*                                                                          */
/*        1. Endian : This code is LITTLE ENDIAN .                          */
/*        2. Interruptibility: This code is interrupt-tolerant but not      */
/*                             interruptible.                               */
/*                                                                          */
/*    CYCLES                                                                */
/*                                                                          */
/*      4*nx + 26                                                           */
/*                                                                          */
/*       For nx=64, cycles=282.                                             */
/*       For nx=30, cycles=146.                                             */
/*                                                                          */
/*    CODESIZE                                                              */
/*                                                                          */
/*      244 bytes                                                           */
/* ------------------------------------------------------------------------ */
/*            Copyright (c) 2004 Texas Instruments, Incorporated.           */
/*                           All Rights Reserved.                           */
/* ======================================================================== */
#ifndef DSPF_DP_DOTP_SQR_H_
#define DSPF_DP_DOTP_SQR_H_ 1

double DSPF_dp_dotp_sqr(
                double G,
                const double * x,
                const double * y,
                double *  restrict r,
                int   nx
                );

#endif
/* ======================================================================== */
/*  End of file:  DSPF_dp_dotp_sqr.h                                             */
/* ------------------------------------------------------------------------ */
/*            Copyright (c) 2004 Texas Instruments, Incorporated.           */
/*                           All Rights Reserved.                           */
/* ======================================================================== */
