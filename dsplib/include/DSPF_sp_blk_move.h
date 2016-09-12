/* ======================================================================== */
/*  TEXAS INSTRUMENTS, INC.                                                 */
/*                                                                          */
/*  NAME                                                                    */
/*      DSPF_sp_blk_move -- Single Precision Block Move                          */
/*                                                                          */
/* USAGE                                                                    */
/*       This routine has the following C prototype:                        */
/*                                                                          */
/*       void DSPF_sp_blk_move                                                   */
/*                     (                                                    */
/*                         const float    *r,                               */
/*                         const float *  restrict r,                       */
/*                         int            nx,                               */
/*                     )                                                    */
/*                                                                          */
/*       x[nx]:   Pointer to source data to be moved.                       */
/*       r[nx]:   Pointer to destination array.                             */
/*       nx:        Number of floats to move.                               */
/*                                                                          */
/*                                                                          */
/* DESCRIPTION                                                              */
/*       This routine moves nx floats from memory location pointed to by x  */
/*       to a separate memory location pointed to by r.                     */
/*                                                                          */
/*                                                                          */
/* TECHNIQUES                                                               */
/* ASSUMPTIONS                                                              */
/* ------------------------------------------------------------------------ */
/*            Copyright (c) 2003 Texas Instruments, Incorporated.           */
/*                           All Rights Reserved.                           */
/* ======================================================================== */
#ifndef DSPF_SP_BLK_MOVE_ASM_H_
#define DSPF_SP_BLK_MOVE_ASM_H_ 1

void DSPF_sp_blk_move(const     float * x, float *restrict r, int nx);

#endif
/* ======================================================================== */
/*  End of file: DSPF_sp_blk_move_h.asm                                          */
/* ------------------------------------------------------------------------ */
/*          Copyright (C) 2003 Texas Instruments, Incorporated.             */
/*                          All Rights Reserved.                            */
/* ======================================================================== */
