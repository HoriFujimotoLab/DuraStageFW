/* ======================================================================== */
/*  TEXAS INSTRUMENTS, INC.                                                 */
/*                                                                          */
/*  NAME                                                                    */
/*      DSPF_sp_mat_trans -- Single Precision matrix transpose                   */
/*                                                                          */
/*  USAGE                                                                   */
/*       This routine has the following C prototype:                        */
/*                                                                          */
/*       void DSPF_sp_mat_trans(                                                 */
/*                            const float *restrict x,                      */
/*                            int rows,                                     */
/*                            int cols,                                     */
/*                            float *restrict r)                            */
/*                                                                          */
/*          x[r1*c1]:  Input matrix containing r1*c1 floating point         */
/*                     numbers having r1 rows and c1 columns.               */
/*          rows    :  Number of rows in Matrix x.                          */
/*                     Also Number of columns in matrix y                   */
/*          cols    :  No. of columns in Matrx x.                           */
/*                     Also no. of rows in Matrix y.                        */
/*          y[c1*c2]:  Output matrix containing c1*r1 floating point        */
/*                     numbers having c1 rows and r1 columns.               */
/*                                                                          */
/*  DESCRIPTION                                                             */
/*                                                                          */
/*    This function transposes the input matrix x[] and writes the          */
/*    result to matrix r[].                                                 */
/*  TECHNIQUES                                                              */
/*                                                                          */
/*    The loop is unrolled twice.                                           */
/*                                                                          */
/*  ASSUMPTIONS                                                             */
/*                                                                          */
/*    rows > 0 and cols > 0                                                 */
/* ------------------------------------------------------------------------ */
/*            Copyright (c) 2002 Texas Instruments, Incorporated.           */
/*                           All Rights Reserved.                           */
/* ======================================================================== */
#ifndef DSPF_SP_MAT_TRANS_ASM_H_
#define DSPF_SP_MAT_TRANS_ASM_H_ 1

void DSPF_sp_mat_trans(const     float *restrict x, int rows, int cols, float *restrict r);

#endif
/* ======================================================================== */
/*  End of file: DSPF_sp_mat_trans_h.asm                                         */
/* ------------------------------------------------------------------------ */
/*          Copyright (C) 2002 Texas Instruments, Incorporated.             */
/*                          All Rights Reserved.                            */
/* ======================================================================== */
