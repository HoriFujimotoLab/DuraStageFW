/* ======================================================================== */
/*  TEXAS INSTRUMENTS, INC.                                                 */
/*                                                                          */
/*  NAME                                                                    */
/*      DSPF_sp_mat_mul -- Single Precision Matrix Multiplication                */
/*                                                                          */
/* USAGE                                                                    */
/*     This routine has following C prototype                               */
/*                                                                          */
/*         void DSPF_sp_mat_mul                                                  */
/*         (                                                                */
/*             float *x, int r1, int c1,                                    */
/*             float *y,         int c2,                                    */
/*             float *r,                                                    */
/*         );                                                               */
/*                                                                          */
/*     x[]  : Pointer to r1 by c1 input matrix.                             */
/*     r1   : Number of rows in x.                                          */
/*     c1   : Number of columns in x.  Also number of rows in y.            */
/*     y[]  : Pointer to c1 by c2 input matrix.                             */
/*     c2   : Number of columns in y.                                       */
/*     r[]  : Pointer to r1 by c2 output matrix.                            */
/*                                                                          */
/* DESCRIPTION                                                              */
/*     This function computes the expression "r = x * y" for the matrices   */
/*     x and y.  The column dimension of x must match the row dimension     */
/*     of y.  The resulting matrix has the same number of rows as x and     */
/*     the same number of columns as y.                                     */
/*                                                                          */
/*     The values stored in the matrices are assumed to be single precision */
/*     floating point values.                                               */
/*                                                                          */
/*     This code is suitable for dense matrices.  No optimizations are      */
/*     made for sparse matrices.                                            */
/*                                                                          */
/* ------------------------------------------------------------------------ */
/*            Copyright (c) 2002 Texas Instruments, Incorporated.           */
/*                           All Rights Reserved.                           */
/* ======================================================================== */
#ifndef DSPF_SP_MAT_MUL_ASM_H_
#define DSPF_SP_MAT_MUL_ASM_H_ 1

void DSPF_sp_mat_mul(float     * x, int r1, int c1, float * y, int c2, float * r);

#endif
/* ======================================================================== */
/*  End of file: DSPF_sp_mat_mul_h.asm                                           */
/* ------------------------------------------------------------------------ */
/*          Copyright (C) 2002 Texas Instruments, Incorporated.             */
/*                          All Rights Reserved.                            */
/* ======================================================================== */
