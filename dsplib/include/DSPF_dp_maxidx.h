/* ======================================================================= */
/* TEXAS INSTRUMENTS, INC.                                                 */
/*                                                                         */
/* NAME                                                                    */
/*     DSPF_dp_maxidx -- Index of Maximum Element of Double Precision Vector    */
/*                                                                         */
/*  USAGE                                                                   */
/*                                                                          */
/*       This routine has the following C prototype:                        */
/*                                                                          */
/*       int DSPF_dp_maxidx(                                                     */
/*                      const double* x,                                    */
/*                      int nx                                              */
/*                    )                                                     */
/*                                                                          */
/*            x :  Pointer to Input array.                                  */
/*            nx:  Number of Inputs in the input Array.                     */
/*                                                                          */
/*  DESCRIPTION                                                             */
/*                                                                          */
/*       This routine finds out the index of maximum number in the input    */
/*       array. This function returns the index of the greatest value.      */
/*                                                                          */
/*  TECHNIQUES                                                              */
/*                                                                          */
/*      1. The loop is unrolled three times.                                */
/*      2. Three maximums are maintained in each iteration.                 */
/*      3. MPY instructions are  used for move.                             */
/*                                                                          */
/*  ASSUMPTIONS                                                             */
/*                                                                          */
/*      1. nx is a multiple of 3.                                           */
/*      2. nx >= 3, and nx <= 2^16-1.                                       */
/*                                                                          */
/*  C CODE                                                                  */
/*                                                                          */
/*      int DSPF_dp_maxidx(const double* x, int nx)                              */
/*      {                                                                   */
/*         int index, i;                                                    */
/*         double max;                                                      */
/*                                                                          */
/*         *((int *)&max) = 0x00000000;                                     */
/*         *((int *)&max+1) = 0xfff00000;                                   */
/*                                                                          */
/*         for (i = 0; i < nx; i++)                                         */
/*         if (x[i] > max)                                                  */
/*         {                                                                */
/*            max = x[i];                                                   */
/*            index = i;                                                    */
/*         }                                                                */
/*         return index;                                                    */
/*      }                                                                   */
/*                                                                          */
/*  NOTES                                                                   */
/*                                                                          */
/*     1. Endian: This code is LITTLE ENDIAN.                               */
/*     2. Interruptibility: This code is interrupt tolerant but not         */
/*        interruptible.                                                    */
/*                                                                          */
/*  CYCLES                                                                  */
/*                                                                          */
/*      4*nx/3 + 22                                                         */
/*      For nx=60, cycles=102.                                              */
/*      For nx=30, cycles=62.                                               */
/*                                                                          */
/*  CODESIZE                                                                */
/*                                                                          */
/*      448 bytes                                                           */
/*                                                                          */
/* ------------------------------------------------------------------------ */
/*            Copyright (c) 2004 Texas Instruments, Incorporated.           */
/*                           All Rights Reserved.                           */
/* ======================================================================== */
#ifndef DSPF_DP_MAXIDX_H_
#define DSPF_DP_MAXIDX_H_ 1

int DSPF_dp_maxidx(
               const double* x,
               int nx
             );

#endif
/* ======================================================================== */
/*  End of file:  DSPF_dp_maxidx.h                                               */
/* ------------------------------------------------------------------------ */
/*            Copyright (c) 2004 Texas Instruments, Incorporated.           */
/*                           All Rights Reserved.                           */
/* ======================================================================== */
