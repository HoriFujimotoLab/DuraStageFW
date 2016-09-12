/* ======================================================================= */
/* TEXAS INSTRUMENTS, INC.                                                 */
/*                                                                         */
/* NAME                                                                    */
/*     DSPF_dp_w_vec -- Double Precision weighted sum of vectors                */
/*                                                                         */
/*  USAGE                                                                   */
/*                                                                          */
/*      This routine is C callable, and has the following C prototype:      */
/*                                                                          */
/*        void DSPF_dp_w_vec(const double* x,                                    */
/*                      const double* y,                                    */
/*                      double m,                                           */
/*                      double * restrict r,                                */
/*                      int     nr                                          */
/*                     )                                                    */
/*                                                                          */
/*             x    :  Pointer to first input array                         */
/*             y    :  Pointer to second input array                        */
/*             m    :  Weight factor                                        */
/*             r    :  Output array pointer                                 */
/*             nr   :  Number of elements in arrays                         */
/*                                                                          */
/*  DESCRIPTION                                                             */
/*                                                                          */
/*     This routine is used to obtain the weighted vector sum.              */
/*     Both the inputs and output are Double precision floating             */
/*     point numbers.                                                       */
/*  TECHNIQUES                                                              */
/*                                                                          */
/*     1. The inner loop is unrolled twice.                                 */
/*                                                                          */
/*                                                                          */
/*  ASSUMPTIONS                                                             */
/*                                                                          */
/*      1. The value of nr must be > 0.                                     */
/*                                                                          */
/*                                                                          */
/*  C CODE                                                                  */
/*                                                                          */
/*     This is the C equivalent of the Assembly Code without                */
/*     restrictions.                                                        */
/*                                                                          */
/*     void DSPF_dp_w_vec( const double * x,const double * y, double  m,         */
/*                    double * restrict r,int nr)                           */
/*                                                                          */
/*    {                                                                     */
/*        int i;                                                            */
/*        for (i = 0; i < nr; i++)                                          */
/*            r[i] = (m * x[i]) + y[i];                                     */
/*    }                                                                     */
/*                                                                          */
/*                                                                          */
/*  NOTES                                                                   */
/*                                                                          */
/*    1. Endian: This code is LITTLE ENDIAN.                                */
/*    2. Interruptibility: This code is interrupt tolerant but not          */
/*                         interruptible.                                   */
/*                                                                          */
/*  CYCLES                                                                  */
/*                                                                          */
/*     4*Ceil(n/2) + 32                                                     */
/*     eg. for n = 100, cycles = 232.                                       */
/*                                                                          */
/*  CODESIZE                                                                */
/*                                                                          */
/*    352 bytes                                                             */
/* ------------------------------------------------------------------------ */
/*            Copyright (c) 2004 Texas Instruments, Incorporated.           */
/*                           All Rights Reserved.                           */
/* ======================================================================== */
#ifndef DSPF_DP_W_VEC_H_
#define DSPF_DP_W_VEC_H_ 1

void DSPF_dp_w_vec(const     double* x,
              const double* y,
              double m,
              double * restrict r,
              int     nr
             );

#endif
/* ======================================================================== */
/*  End of file:  DSPF_dp_w_vec.h                                                */
/* ------------------------------------------------------------------------ */
/*            Copyright (c) 2004 Texas Instruments, Incorporated.           */
/*                           All Rights Reserved.                           */
/* ======================================================================== */
