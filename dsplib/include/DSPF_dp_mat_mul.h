/* ======================================================================= */
/* TEXAS INSTRUMENTS, INC.                                                 */
/*                                                                         */
/* NAME                                                                    */
/*     DSPF_dp_mat_mul -- Double Precision Matrix Multiplication                */
/*                                                                         */
/* USAGE                                                                    */
/*                                                                          */
/*     This routine has following C prototype:                              */
/*     void DSPF_dp_mat_mul(double *x, int r1, int c1,                           */
/*                     double *y, int c2, double *r)                        */
/*                                                                          */
/*     x      : Pointer to r1 by c1 input matrix.                           */
/*     r1     : Number of rows in x.                                        */
/*     c1     : Number of columns in x.  Also number of rows in y.          */
/*     y      : Pointer to c1 by c2 input matrix.                           */
/*     c2     : Number of columns in y.                                     */
/*     r      : Pointer to r1 by c2 output matrix.                          */
/*                                                                          */
/* DESCRIPTION                                                              */
/*                                                                          */
/*     This function computes the expression "r = x * y" for the matrices   */
/*     x and y.  The column dimension of x must match the row dimension     */
/*     of y.  The resulting matrix has the same number of rows as x and     */
/*     the same number of columns as y.                                     */
/*                                                                          */
/*     The values stored in the matrices are assumed to be Double           */
/*     precision floating point values.                                     */
/*                                                                          */
/*     This code is suitable for dense matrices.  No optimizations are      */
/*     made for sparse matrices.                                            */
/*                                                                          */
/* TECHNIQUES                                                               */
/*                                                                          */
/*     1.    All three loops are unrolled two times                         */
/*     2.    All the prolog stages of the innermost loop (kLoop) are        */
/*           Scheduled in Parallel with outer loop.                         */
/*     3.    Extraneous Loads are allowed in Program                        */
/*     4.    Outer most Loop Instructions are scheduled in Parallel with In *r
/*           Loop Instructions.                                                   */
/*                                                                          */
/* ASSUMPTIONS                                                              */
/*                                                                          */
/*     1.    The arrays 'x', 'y', and 'r' are stored in distinct arrays.    */
/*           That is, in-place processing is not allowed.                   */
/*     2.    All r1, c1, c2 are assumed to be > 1                           */
/*     3.   If r1 is odd, one extra row of x[] matrix is loaded             */
/*     4.   If c2 is odd, one extra col of y[] matrix is loaded.            */
/*     5.   If c1 is odd, one extra col of x[] and  one extra row of        */
/*          y[] array is loaded                                             */
/*                                                                          */
/* C CODE                                                                   */
/*                                                                          */
/*    void DSPF_dp_mat_mul(double *x, int r1, int c1,                            */
/*                    double *y, int c2, double *r)                         */
/*    {                                                                     */
/*        int i, j, k;                                                      */
/*        double sum;                                                       */
/*                                                                          */
/*     //  Multiply each row in x by each column in y.                      */
/*     //  The product of row m in x and column n in y is placed            */
/*     //  in position (m,n) in the result.                                 */
/*                                                                          */
/*        for (i = 0; i < r1; i++)                                          */
/*            for (j = 0; j < c2; j++)                                      */
/*            {                                                             */
/*                sum = 0;                                                  */
/*                for (k = 0; k < c1; k++)                                  */
/*                    sum += x[k + i*c1] * y[j + k*c2];                     */
/*                                                                          */
/*                r[j + i*c2] = sum;                                        */
/*            }                                                             */
/*    }                                                                     */
/*                                                                          */
/* NOTES                                                                    */
/*                                                                          */
/*     1.   Endian: This code is LITTLE ENDIAN .                            */
/*     2.   Interruptibility: This code is interrupt-tolerant but not       */
/*          interruptible.                                                  */
/*                                                                          */
/* CYCLES                                                                   */
/*                                                                          */
/*    (2 * r1' * c1 * c2') + 18*( c2'/2 * r1'/2) + 40                       */
/*    where                                                                 */
/*           r1' = r1 + (r1&1)                                              */
/*           c2' = c2 + (c2&1)                                              */
/*    For r1 = 12, c1 = 14 and c2 = 12,     cycles = 4720                   */
/*                                                                          */
/* CODESIZE                                                                 */
/*                                                                          */
/*    960 Bytes                                                             */
/* ------------------------------------------------------------------------ */
/*            Copyright (c) 2004 Texas Instruments, Incorporated.           */
/*                           All Rights Reserved.                           */
/* ======================================================================== */
#ifndef DSPF_DP_MAT_MUL_H_
#define DSPF_DP_MAT_MUL_H_ 1

void DSPF_dp_mat_mul(double     *x, int r1, int c1,
                double *y, int c2, double *r);

#endif
/* ======================================================================== */
/*  End of file:  DSPF_dp_mat_mul.h                                              */
/* ------------------------------------------------------------------------ */
/*            Copyright (c) 2004 Texas Instruments, Incorporated.           */
/*                           All Rights Reserved.                           */
/* ======================================================================== */
