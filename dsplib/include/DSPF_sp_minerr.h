/* ======================================================================== */
/*  TEXAS INSTRUMENTS, INC.                                                 */
/*                                                                          */
/*  NAME                                                                    */
/*      DSPF_sp_minerr -- Searching for a pair of numbers producing maximum      */
/*      dot product                                                         */
/*                                                                          */
/*  USAGE                                                                   */
/*       This routine has the following C prototype:                        */
/*                                                                          */
/*       float DSPF_sp_minerr(const float* GSP0_TABLE, const float* errCoefs,    */
/*                                              int *restrict max_index)    */
/*                                                                          */
/*          GSP0_TABLE[256*9]: GSP0 terms array.                            */
/*          errCoefs[9]:       Array of error coefficients.                 */
/*                             Must be Double word aligned.                 */
/*          max_index:         Index to GSP0_TABLE[max_index], the first    */
/*                             element of the 9-element vector that         */
/*                             resulted in the maximum dot product.         */
/*          return float       Maximum dot product result.                  */
/*                                                                          */
/*  DESCRIPTION                                                             */
/*       Performs a dot product on 256 pairs of 9 element vectors and       */
/*       searches for the pair of vectors which produces the maximum dot    */
/*       product result. This is a large part of the VSELP vocoder          */
/*       codebook search.                                                   */
/*                                                                          */
/*       The function stores the index to the first element of the          */
/*       9-element vector that resulted in the maximum dot product in the   */
/*       memory loaction Pointed by max_index. The maximum dot product      */
/*       value is returned by the function.                                 */
/*  TECHNIQUES                                                              */
/*                                                                          */
/*      The inner loop is totally unrolled.                                 */
/*                                                                          */
/*  ASSUMPTIONS                                                             */
/*                                                                          */
/*      errCoefs must be Double word aligned.                               */
/*                                                                          */
/* ------------------------------------------------------------------------ */
/*            Copyright (c) 2002 Texas Instruments, Incorporated.           */
/*                           All Rights Reserved.                           */
/* ======================================================================== */
#ifndef DSPF_SP_MINERR_ASM_H_
#define DSPF_SP_MINERR_ASM_H_ 1

float DSPF_sp_minerr(const     float* GSP0_TABLE, const float* errCoefs, int *restrict max_index);

#endif
/* ======================================================================== */
/*  End of file: DSPF_sp_minerr_h.asm                                            */
/* ------------------------------------------------------------------------ */
/*          Copyright (C) 2002 Texas Instruments, Incorporated.             */
/*                          All Rights Reserved.                            */
/* ======================================================================== */
