/* ======================================================================== */
/*  TEXAS INSTRUMENTS, INC.                                                 */
/*                                                                          */
/*  NAME                                                                    */
/*      DSPF_sp_minval -- Minimum Element of Single Precision Vector             */
/*                                                                          */
/*     USAGE                                                                */
/*       This routine has the following C prototype:                        */
/*                                                                          */
/*       float DSPF_sp_minval(                                                   */
/*                       const float* x,                                    */
/*                       int nx                                             */
/*                      )                                                   */
/*                                                                          */
/*            x :  Pointer to Input array.                                  */
/*            nx:  Number of Inputs in the input Array.                     */
/*            Returns float: Minimum value in the input array               */
/*                                                                          */
/*  DESCRIPTION                                                             */
/*       This routine Finds out the minimum number in the input array.      */
/*                                                                          */
/*                                                                          */
/*  TECHNIQUES                                                              */
/*                                                                          */
/*       The loop is unrolled Six times.                                    */ 
/*       Six minimums are maintained in each iteration.                     */ 
/*       One of the minimums are calculated using SUBSP in place of CMPGTSP */ 
/*                                                                          */
/*  ASSUMPTIONS                                                             */
/*                                                                          */
/*       nx should be multiple of 2 and >= 2.                               */ 
/*       x should be double word aligned.                                   */ 
/*       NAN( Not a Number in Single Precision format) in the input are     */ 
/*       disregarded.                                                       */ 
/*                                                                          */
/*                                                                          */
/* ------------------------------------------------------------------------ */
/*            Copyright (c) 2002 Texas Instruments, Incorporated.           */
/*                           All Rights Reserved.                           */
/* ======================================================================== */
#ifndef DSPF_SP_MINVAL_ASM_H_
#define DSPF_SP_MINVAL_ASM_H_ 1

float DSPF_sp_minval(const     float* x, int nx);

#endif
/* ======================================================================== */
/*  End of file: DSPF_sp_minval_h.asm                                            */
/* ------------------------------------------------------------------------ */
/*          Copyright (C) 2002 Texas Instruments, Incorporated.             */
/*                          All Rights Reserved.                            */
/* ======================================================================== */
