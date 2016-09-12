/* ======================================================================== */
/*  TEXAS INSTRUMENTS, INC.                                                 */
/*                                                                          */
/*  NAME                                                                    */
/*      DSPF_sp_mat_mul_cplx -- complex matrix multiplication                    */
/*                                                                          */
/*  USAGE                                                                   */
/*       This routine has the following C prototype:                        */
/*                                                                          */
/*       void DSPF_sp_mat_mul_cplx(                                              */
/*                              const float* x,                             */
/*                              int r1,                                     */
/*                              int c1,                                     */
/*                              const float* y,                             */
/*                              int c2,                                     */
/*                              float* restrict r                           */
/*                           )                                              */
/*                                                                          */
/*            x[2*r1*c1]:   Input matrix containing r1*c1 complex           */
/*                          floating point numbers having r1 rows and c1    */
/*                          columns of complex numbers.                     */
/*            r1        :   Number of rows in Matrix x.                     */
/*            c1        :   No. of columns in Matrx x.                      */
/*                          Also no. of rows in Matrix y.                   */
/*            y[2*c1*c2]:   Input matrix containing c1*c2 complex           */
/*                          floating point numbers having c1 rows and c2    */
/*                          columns of complex numbers.                     */
/*            c2        :   No. of columns in Matrix y.                     */
/*            r[2*r1*c2]:   Output matrix of c1*c2 complex floating         */
/*                          point numbers having c1 rows and c2 columns of  */
/*                          complex numbers.                                */
/*                                                                          */
/*                          Complex numbers are stored consecutively with   */
/*                          Real values are stored in even word positions   */
/*                          and imaginary values in odd positions.          */
/*                                                                          */
/*  DESCRIPTION                                                             */
/*        This function computes the expression "r = x * y" for the         */
/*        matrices x and y. The columnar dimension of x must match the row  */
/*        dimension of y. The resulting matrix has the same number of rows  */
/*        as x and the same number of columns as y.                         */
/*                                                                          */
/*        Each element of Matrices are assumed to be complex numbers with   */
/*        Real values are stored in even word positions and imaginary       */
/*        values in odd positions.                                          */
/*                                                                          */
/*  TECHNIQUES                                                              */
/*                                                                          */
/*        Innermost loop is Unrolled twice.                                 */
/*        Two inner loops are collapsed into one loop.                      */
/*        Outermost loop is executed in parallel with innner loops.         */
/*                                                                          */
/*  ASSUMPTIONS                                                             */
/*                                                                          */
/*        c1 >= 4, and r1,r2>=1                                             */
/*        x should be padded with 6 words                                   */
/*        x and y should be double word aligned                             */
/*                                                                          */
/* ------------------------------------------------------------------------ */
/*            Copyright (c) 2002 Texas Instruments, Incorporated.           */
/*                           All Rights Reserved.                           */
/* ======================================================================== */
#ifndef DSPF_SP_MAT_MUL_CPLX_ASM_H_
#define DSPF_SP_MAT_MUL_CPLX_ASM_H_ 1

void DSPF_sp_mat_mul_cplx(const     float* x, int r1, int c1, const float* y, int c2, float* restrict r);

#endif
/* ======================================================================== */
/*  End of file: DSPF_sp_mat_mul_cplx_h.asm                                      */
/* ------------------------------------------------------------------------ */
/*          Copyright (C) 2002 Texas Instruments, Incorporated.             */
/*                          All Rights Reserved.                            */
/* ======================================================================== */
