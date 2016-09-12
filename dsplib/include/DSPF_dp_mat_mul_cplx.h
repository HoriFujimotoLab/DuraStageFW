/* ======================================================================= */
/* TEXAS INSTRUMENTS, INC.                                                 */
/*                                                                         */
/* NAME                                                                    */
/*     DSPF_dp_mat_mul_cplx -- Complex matrix multiplication                    */
/*                                                                         */
/*  USAGE                                                                   */
/*                                                                          */
/*       This routine has the following C prototype:                        */
/*                                                                          */
/*       void DSPF_dp_mat_mul_cplx(                                              */
/*                              const double* x,                            */
/*                              int r1,                                     */
/*                              int c1,                                     */
/*                              const double* y,                            */
/*                              int c2,                                     */
/*                              double* restrict r                          */
/*                           )                                              */
/*                                                                          */
/*            x[2*r1*c1]:   Input matrix containing r1*c1 complex           */
/*                          floating point numbers having r1 rows and c1    */
/*                          columns of complex numbers.                     */
/*            r1        :   No. of rows in matrix x.                        */
/*            c1        :   No. of columns in matrix x.                     */
/*                          Also no. of rows in matrix y.                   */
/*            y[2*c1*c2]:   Input matrix containing c1*c2 complex           */
/*                          floating point numbers having c1 rows and c2    */
/*                          columns of complex numbers.                     */
/*            c2        :   No. of columns in matrix y.                     */
/*            r[2*r1*c2]:   Output matrix of c1*c2 complex floating         */
/*                          point numbers having c1 rows and c2 columns of  */
/*                          complex numbers.                                */
/*                                                                          */
/*                          Complex numbers are stored consecutively with   */
/*                          real values stored in even positions and        */
/*                          imaginary values in odd positions.              */
/*                                                                          */
/*  DESCRIPTION                                                             */
/*                                                                          */
/*        This function computes the expression "r = x * y" for the         */
/*        matrices x and y. The columnar dimension of x must match the row  */
/*        dimension of y. The resulting matrix has the same number of rows  */
/*        as x and the same number of columns as y.                         */
/*                                                                          */
/*        Each element of the matrix is assumed to be complex numbers with  */
/*        Real values are stored in even positions and imaginary            */
/*        values in odd positions.                                          */
/*                                                                          */
/*  TECHNIQUES                                                              */
/*                                                                          */
/*        1. Innermost loop is unrolled twice.                              */
/*        2. Outermost loop is executed in parallel with innner loops.      */
/*                                                                          */
/*  ASSUMPTIONS                                                             */
/*                                                                          */
/*        1. r1,r2>=1,c1 should be a multiple of 2 and >=2.                 */
/*        2. x should be padded with 6 words                                */
/*                                                                          */
/*                                                                          */
/*  C CODE                                                                  */
/*                                                                          */
/*  This is the C equivalent of the assembly code.  Note that               */
/*   the assembly code is hand optimized and restrictions may apply.        */
/*                                                                          */
/*        void DSPF_dp_mat_mul_cplx(const double* x, int r1, int c1,             */
/*          const double* y, int c2, double* restrict r)                    */
/*        {                                                                 */
/*            double real, imag;                                            */
/*            int i, j, k;                                                  */
/*                                                                          */
/*            for(i = 0; i < r1; i++)                                       */
/*            {                                                             */
/*              for(j = 0; j < c2; j++)                                     */
/*              {                                                           */
/*                real=0;                                                   */
/*                imag=0;                                                   */
/*                                                                          */
/*                for(k = 0; k < c1; k++)                                   */
/*                {                                                         */
/*                real += (x[i*2*c1 + 2*k]*y[k*2*c2 + 2*j]                  */
/*                -x[i*2*c1 + 2*k + 1] * y[k*2*c2 + 2*j + 1]);              */
/*                                                                          */
/*                imag+=(x[i*2*c1 + 2*k] * y[k*2*c2 + 2*j + 1]              */
/*                   + x[i*2*c1 + 2*k + 1] * y[k*2*c2 + 2*j]);              */
/*                }                                                         */
/*                r[i*2*c2 + 2*j] = real;                                   */
/*                r[i*2*c2 + 2*j + 1] = imag;                               */
/*              }                                                           */
/*            }                                                             */
/*        }                                                                 */
/*                                                                          */
/*  NOTES                                                                   */
/*                                                                          */
/*        1. Real values are stored in even word positions and imaginary    */
/*           values in odd positions.                                       */
/*        2. Endian: This code is LITTLE ENDIAN.                            */
/*        3. Interruptibility: This code is interrupt tolerant but not      */
/*           interruptible.                                                 */
/*                                                                          */
/*  CYCLES                                                                  */
/*                                                                          */
/*        8*r1*c1*c2'+ 18*(r1*c2)+40 WHERE c2'=2*ceil(c2/2)                 */
/*        When r1=3, c1=4, c2=4, cycles = 640                               */
/*        When r1=4, c1=4, c2=5, cycles = 1040                              */
/*                                                                          */
/*  CODESIZE                                                                */
/*                                                                          */
/*        832 bytes                                                         */
/* ------------------------------------------------------------------------ */
/*            Copyright (c) 2004 Texas Instruments, Incorporated.           */
/*                           All Rights Reserved.                           */
/* ======================================================================== */
#ifndef DSPF_DP_MAT_MUL_CPLX_H_
#define DSPF_DP_MAT_MUL_CPLX_H_ 1

void DSPF_dp_mat_mul_cplx(
                       const double* x,
                       int r1,
                       int c1,
                       const double* y,
                       int c2,
                       double* restrict r
                    );

#endif
/* ======================================================================== */
/*  End of file:  DSPF_dp_mat_mul_cplx.h                                         */
/* ------------------------------------------------------------------------ */
/*            Copyright (c) 2004 Texas Instruments, Incorporated.           */
/*                           All Rights Reserved.                           */
/* ======================================================================== */
