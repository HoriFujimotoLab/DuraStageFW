/* ======================================================================= */
/* TEXAS INSTRUMENTS, INC.                                                 */
/*                                                                         */
/* NAME                                                                    */
/*     DSPF_dp_blk_move -- DSPF_dp_blk_move                                          */
/*                                                                         */
/* USAGE                                                                    */
/*                                                                          */
/*       This routine has the following C prototype:                        */
/*                                                                          */
/*       void DSPF_dp_blk_move(                                                  */
/*                        const double *   x,                               */
/*                        double *restrict r,                               */
/*                        int nx                                            */
/*                       )                                                  */
/*                                                                          */
/*            x[nx]: Pointer to source data to be moved.                    */
/*            r[nx]: Pointer to destination array.                          */
/*            nx:    Number of floats to move.                              */
/*                                                                          */
/* DESCRIPTION                                                              */
/*                                                                          */
/*   This routine moves nx doubles from one memory location                 */
/*       pointed to by x to another pointed to by r.                        */
/*                                                                          */
/*                                                                          */
/* ASSUMPTIONS                                                              */
/*                                                                          */
/*       1. nx is greater than 0.                                           */
/*                                                                          */
/*  C CODE                                                                  */
/*                                                                          */
/*  This is the C equivalent of the assembly code.  Note that               */
/*  the assembly code is hand optimized and restrictions may apply.         */
/*                                                                          */
/*       void DSPF_dp_blk_move(const double * x, double *restrict r, int nx)     */
/*       {                                                                  */
/*           int i;                                                         */
/*           for (i = 0 ; i < nx; i++)                                      */
/*               r[i] = x[i];                                               */
/*       }                                                                  */
/*                                                                          */
/*                                                                          */
/* NOTES                                                                    */
/*                                                                          */
/*      1. Endian: This implementation is Little-Endian.                    */
/*      2. Interruptibility: This code is interrupt-tolerant but not        */
/*                           interruptible.                                 */
/*                                                                          */
/* CYCLES                                                                   */
/*                                                                          */
/*      2*nx+ 8                                                             */
/*      For nx=64, cycles=136.                                              */
/*      For nx=25, cycles=58.                                               */
/*                                                                          */
/* CODESIZE                                                                 */
/*                                                                          */
/*      96 bytes                                                            */
/*                                                                          */
/* ------------------------------------------------------------------------ */
/*            Copyright (c) 2004 Texas Instruments, Incorporated.           */
/*                           All Rights Reserved.                           */
/* ======================================================================== */
#ifndef DSPF_DP_BLK_MOVE_H_
#define DSPF_DP_BLK_MOVE_H_ 1

void DSPF_dp_blk_move(
                 const double *   x,
                 double *restrict r,
                 int nx
                );

#endif
/* ======================================================================== */
/*  End of file:  DSPF_dp_blk_move.h                                             */
/* ------------------------------------------------------------------------ */
/*            Copyright (c) 2004 Texas Instruments, Incorporated.           */
/*                           All Rights Reserved.                           */
/* ======================================================================== */
