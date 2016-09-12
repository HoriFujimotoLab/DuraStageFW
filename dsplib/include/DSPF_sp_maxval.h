/* ======================================================================== */
/*  TEXAS INSTRUMENTS, INC.                                                 */
/*                                                                          */
/*  NAME                                                                    */
/*      DSPF_sp_maxval -- Maximum Element of Single Precision Vector             */
/*                                                                          */
/*  USAGE                                                                   */
/*       This routine has the following C prototype:                        */
/*                                                                          */
/*       float DSPF_sp_maxval(                                                   */
/*                       const float* x,                                    */
/*                       int nx                                             */
/*                      )                                                   */
/*                                                                          */
/*            x :  Pointer to Input array.                                  */
/*            nx:  Number of Inputs in the input Array.                     */
/*            Returns float: Maximum value in the input array               */
/*                                                                          */
/*  DESCRIPTION                                                             */
/*       This routine Finds out the maximum number in the input array.      */
/*                                                                          */
/*                                                                          */
/*  TECHNIQUES                                                              */
/*                                                                          */
/*      The loop is unrolled Six times.                                     */
/*      Six maximums are maintained in each iteration.                      */ 
/*      One of the maximums are calculated using SUBSP in place of CMPGTSP  */
/*                                                                          */
/*  ASSUMPTIONS                                                             */
/*                                                                          */
/*      nx should be multiple of 2 and >= 2.                                */ 
/*      x should be double word aligned.                                    */ 
/*      NAN( Not a Number in Single Precision format) in the input are      */ 
/*      disregarded.                                                        */
/* ------------------------------------------------------------------------ */
/*            Copyright (c) 2002 Texas Instruments, Incorporated.           */
/*                           All Rights Reserved.                           */
/* ======================================================================== */
#ifndef DSPF_SP_MAXVAL_ASM_H_
#define DSPF_SP_MAXVAL_ASM_H_ 1

float DSPF_sp_maxval(const     float* x, int nx);

#endif
/* ======================================================================== */
/*  End of file: DSPF_sp_maxval_h.asm                                            */
/* ------------------------------------------------------------------------ */
/*          Copyright (C) 2002 Texas Instruments, Incorporated.             */
/*                          All Rights Reserved.                            */
/* ======================================================================== */
