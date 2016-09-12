/* ======================================================================== */
/*  TEXAS INSTRUMENTS, INC.                                                 */
/*                                                                          */
/*  NAME                                                                    */
/*      DSPF_sp_maxidx -- Index of Maximum Element of Single Precision Vector    */
/*                                                                          */
/*  USAGE                                                                   */
/*       This routine has the following C prototype:                        */
/*                                                                          */
/*       int DSPF_sp_maxidx(                                                     */
/*                      const float* x,                                     */
/*                      int nx                                              */
/*                    )                                                     */
/*                                                                          */
/*            x :  Pointer to Input array.                                  */
/*            nx:  Number of Inputs in the input Array.                     */
/*            Returns int: Index of Maximum value                           */
/*                                                                          */
/*  DESCRIPTION                                                             */
/*       This routine Finds out the index of maximum number in the input    */
/*       array.                                                             */
/*  TECHNIQUES                                                              */
/*                                                                          */
/*      The loop is unrolled three times.                                   */
/*      Three maximums are maintained in each iteration.                    */
/*      MPY indtructions are  used for move.                                */
/*                                                                          */
/*  ASSUMPTIONS                                                             */
/*                                                                          */
/*      nx is a multiple of 3                                               */
/*      nx >= 3, and nx <= 2^16-1.                                          */
/*                                                                          */
/*                                                                          */
/* ------------------------------------------------------------------------ */
/*            Copyright (c) 2002 Texas Instruments, Incorporated.           */
/*                           All Rights Reserved.                           */
/* ======================================================================== */
#ifndef DSPF_SP_MAXIDX_ASM_H_
#define DSPF_SP_MAXIDX_ASM_H_ 1

int DSPF_sp_maxidx(const     float* x, int nx);

#endif
/* ======================================================================== */
/*  End of file: DSPF_sp_maxidx_h.asm                                            */
/* ------------------------------------------------------------------------ */
/*          Copyright (C) 2002 Texas Instruments, Incorporated.             */
/*                          All Rights Reserved.                            */
/* ======================================================================== */
