/* ======================================================================= */
/* TEXAS INSTRUMENTS, INC.                                                 */
/*                                                                         */
/* NAME                                                                    */
/*     DSPF_dp_bitrev_cplx -- Bit reversal for Double Precision Complex         */
/*     numbers                                                             */
/*                                                                         */
/* USAGE                                                                    */
/*                                                                          */
/*     This routine is C Callable and can be called as:                     */
/*                                                                          */
/*     void DSPF_dp_bitrev_cplx(double *x, short *index, int nx);                */
/*                                                                          */
/*     x      : Complex input array to be bit-reversed.                     */
/*              Contains 2*nx doubles                                       */
/*     index  : Array of size ~sqrt(nx) created by the routine              */
/*              bitrev_index to allow the fast implementation of the        */
/*              bit-reversal.                                               */
/*     nx     : Number of elements in array x[]. Must be power of 2.        */
/*                                                                          */
/*  DESCRIPTION                                                             */
/*                                                                          */
/*     This routine performs the bit-reversal of the input array x[],       */
/*     where x[] is a double array of length 2*nx containing Double         */
/*     precision floating point complex pairs of data. This routine         */
/*     requires the index array provided by the program below.              */
/*     This index should be generated at compile time not by the DSP.       */
/*                                                                          */
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
/*  ASSUMPTIONS                                                             */
/*                                                                          */
/*     1. nx must be a power of 2.                                          */
/*     2. The table from bitrev_index is already created.                   */
/*     3. The array x is actually an array of 2*nx doubles.                 */
/*                                                                          */
/*  C CODE                                                                  */
/*                                                                          */
/*      This is the C equivalent for the assembly code.  Note that          */
/*      the assembly code is hand optimized and restrictions may            */
/*      apply.                                                              */
/*                                                                          */
/*      void DSPF_dp_bitrev_cplx(double* x, short* index, int nx)                */
/*      {                                                                   */
/*         int    i;                                                        */
/*         short  i0, i1, i2;                                               */
/*         short  j0, j1, j2;                                               */
/*         double xi0r, xi0i, xi1r, xi1i, xi2r, xi2i;                       */
/*         double xj0r, xj0i, xj1r, xj1i, xj2r, xj2i;                       */
/*         short  t;                                                        */
/*         int    a, b, ia, ib, ibs;                                        */
/*         int    mask;                                                     */
/*         int    nbits, nbot, ntop, ndiff, n2, halfn;                      */
/*                                                                          */
/*         nbits = 0;                                                       */
/*         i = nx;                                                          */
/*         while (i > 1)                                                    */
/*         {                                                                */
/*            i = i >> 1;                                                   */
/*            nbits++;                                                      */
/*         }                                                                */
/*                                                                          */
/*         nbot    = nbits >> 1;                                            */
/*         ndiff   = nbits & 1;                                             */
/*         ntop    = nbot + ndiff;                                          */
/*         n2       = 1 << ntop;                                            */
/*         mask    = n2 - 1;                                                */
/*         halfn   = nx >> 1;                                               */
/*                                                                          */
/*         for (i0 = 0; i0 < halfn; i0 += 2)                                */
/*         {                                                                */
/*             b       = i0 & mask;                                         */
/*             a       = i0 >> nbot;                                        */
/*             if (!b) ia = index[a];                                       */
/*             ib      = index[b];                                          */
/*             ibs     = ib << nbot;                                        */
/*                                                                          */
/*             j0      = ibs + ia;                                          */
/*             t       = i0 < j0;                                           */
/*             xi0r     = x[2*i0];                                          */
/*             xi0i     = x[2*i0+1];                                        */
/*             xj0r     = x[2*j0];                                          */
/*             xj0i     = x[2*j0+1];                                           */
/*             if (t)                                                       */
/*             {                                                            */
/*                x[2*i0]   = xj0r;                                         */
/*                x[2*i0+1] = xj0i;                                         */
/*                x[2*j0]   = xi0r;                                         */
/*                x[2*j0+1] = xi0i;                                         */
/*              }                                                           */
/*                                                                          */
/*             i1      = i0 + 1;                                            */
/*             j1      = j0 + halfn;                                        */
/*             xi1r    = x[2*i1];                                           */
/*             xi1i    = x[2*i1+1];                                         */
/*             xj1r    = x[2*j1];                                           */
/*             xj1i    = x[2*j1+1];                                         */
/*                                                                          */
/*             x[2*i1]   = xj1r;                                            */
/*             x[2*i1+1] = xj1i;                                            */
/*             x[2*j1]   = xi1r;                                            */
/*             x[2*j1+1] = xi1i;                                            */
/*                                                                          */
/*             i2      = i1 + halfn;                                        */
/*             j2      = j1 + 1;                                            */
/*                                                                          */
/*             xi2r    = x[2*i2];                                           */
/*             xi2i    = x[2*i2+1];                                         */
/*             xj2r    = x[2*j2];                                           */
/*             xj2i    = x[2*j2+1];                                         */
/*             if (t)                                                       */
/*             {                                                            */
/*             x[2*i2]   = xj2r;                                            */
/*             x[2*i2+1] = xj2i;                                            */
/*             x[2*j2]   = xi2r;                                            */
/*             x[2*j2+1] = xi2i;                                            */
/*             }                                                            */
/*           }                                                              */
/*                                                                          */
/*         }                                                                */
/*                                                                          */
/*                                                                          */
/* NOTES                                                                    */
/*                                                                          */
/*     1. The index table can be generated using the bitrev_index           */
/*        function provided in the dsplib\support\fft directory.            */
/*     2. If nx <= 4K one can use the char (8-bit) data type for            */
/*        the "index" variable. This would require changing the LDH when    */
/*        loading index values in the assembly routine to LDB. This would   */
/*        further reduce the size of the Index Table by half its size.      */
/*                                                                          */
/*     3. Endian: LITTLE ENDIAN configuration used.                         */
/*                                                                          */
/*     4. Interruptibility: This code is interrupt tolerant, but not        */
/*        interruptible.                                                    */
/*                                                                          */
/* CYCLES                                                                   */
/*                                                                          */
/*     5*nx + 33                                                            */
/*     e.g. nx = 128, cycles = 673                                          */
/*                                                                          */
/*  CODESIZE                                                                */
/*                                                                          */
/*     736 bytes                                                            */
/* ------------------------------------------------------------------------ */
/*            Copyright (c) 2004 Texas Instruments, Incorporated.           */
/*                           All Rights Reserved.                           */
/* ======================================================================== */
#ifndef DSPF_DP_BITREV_CPLX_H_
#define DSPF_DP_BITREV_CPLX_H_ 1

void DSPF_dp_bitrev_cplx(double     *x, short *index, int nx);

#endif
/* ======================================================================== */
/*  End of file:  DSPF_dp_bitrev_cplx.h                                          */
/* ------------------------------------------------------------------------ */
/*            Copyright (c) 2004 Texas Instruments, Incorporated.           */
/*                           All Rights Reserved.                           */
/* ======================================================================== */
