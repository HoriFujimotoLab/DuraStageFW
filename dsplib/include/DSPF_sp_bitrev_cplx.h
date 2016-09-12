/* ======================================================================== */
/*  TEXAS INSTRUMENTS, INC.                                                 */
/*                                                                          */
/*  NAME                                                                    */
/*      DSPF_sp_bitrev_cplx -- Bit reversal for Single Precision Complex         */
/*      numbers                                                             */
/*                                                                          */
/*                                                                          */
/*  USAGE                                                                   */
/*                                                                          */
/*     This routine is C Callable and can be called as:                     */
/*                                                                          */
/*     void bitrev_cplx(double *x, short *index, int nx);                   */
/*                                                                          */
/*     x[nx]  : Complex input array to be bit-reversed.                     */
/*              Contains 2*nx floats                                        */
/*     index[]: Array of size ~sqrt(nx) created by the routine              */
/*              bitrev_index to allow the fast implementation of the        */
/*              bit-reversal.                                               */
/*     nx     : Number of elements in array x[]. Must be power of 2.        */
/*                                                                          */
/*  DESCRIPTION                                                             */
/*                                                                          */
/*     This routine performs the bit-reversal of the input array x[],       */
/*     where x[] is an integer array of length nx containing 16-bit         */
/*     complex pairs of data.  This routine requires the index array        */
/*     provided by the program below.  This index should be generated at    */
/*     compile time not by the DSP.                                         */
/*                                                                          */
/*     TI retains all rights, title and interest in this code and only      */
/*     authorizes the use of the bit-reversal code and related table        */
/*     generation code with TMS320-family DSPs manufactured by TI.          */
/*                                                                          */
/*     // ----------------------------------------------------------- //    */
/*     //  This routine calculates the index for bit reversal of      //    */
/*     //  an array of length nx.  The length of the index table is   //    */
/*     //  2^(2*ceil(k/2)) where nx = 2^k.                            //    */
/*     //                                                             //    */
/*     //  In other words, the length of the index table is:          //    */
/*     //      - for even power of radix: sqrt(nx)                    //    */
/*     //      - for odd  power of radix: sqrt(2*nx)                  //    */
/*     // ----------------------------------------------------------- //    */
/*     void bitrev_index(short *index, int nx)                              */
/*     {                                                                    */
/*         int   i, j, k, radix = 2;                                        */
/*         short nbits, nbot, ntop, ndiff, n2, raddiv2;                     */
/*                                                                          */
/*         nbits = 0;                                                       */
/*         i = nx;                                                          */
/*         while (i > 1)                                                    */
/*         {                                                                */
/*             i = i >> 1;                                                  */
/*             nbits++;                                                     */
/*         }                                                                */
/*                                                                          */
/*         raddiv2 = radix >> 1;                                            */
/*         nbot    = nbits >> raddiv2;                                      */
/*         nbot    = nbot << raddiv2 - 1;                                   */
/*         ndiff   = nbits & raddiv2;                                       */
/*         ntop    = nbot + ndiff;                                          */
/*         n2      = 1 << ntop;                                             */
/*                                                                          */
/*         index[0] = 0;                                                    */
/*         for ( i = 1, j = n2/radix + 1; i < n2 - 1; i++)                  */
/*         {                                                                */
/*             index[i] = j - 1;                                            */
/*                                                                          */
/*             for (k = n2/radix; k*(radix-1) < j; k /= radix)              */
/*                 j -= k*(radix-1);                                        */
/*                                                                          */
/*             j += k;                                                      */
/*         }                                                                */
/*         index[n2 - 1] = n2 - 1;                                          */
/*     }                                                                    */
/*                                                                          */
/* ------------------------------------------------------------------------ */
/*            Copyright (c) 2002 Texas Instruments, Incorporated.           */
/*                           All Rights Reserved.                           */
/* ======================================================================== */
#ifndef DSPF_SP_BITREV_CPLX_ASM_H_
#define DSPF_SP_BITREV_CPLX_ASM_H_ 1

void DSPF_sp_bitrev_cplx(double     * x, short * index, int n);

#endif
/* ======================================================================== */
/*  End of file: DSPF_sp_bitrev_cplx_h.asm                                       */
/* ------------------------------------------------------------------------ */
/*          Copyright (C) 2002 Texas Instruments, Incorporated.             */
/*                          All Rights Reserved.                            */
/* ======================================================================== */
