/* ======================================================================== */
/*  TEXAS INSTRUMENTS, INC.                                                 */
/*                                                                          */
/*  NAME                                                                    */
/*      DSPF_dp_icfftr2 -- Double Precision Cache optimised radix-2 inverse      */
/*      FFT with complex input                                              */
/*                                                                          */
/* USAGE                                                                    */
/*                                                                          */
/*        This routine is C-callable and can be called as:                  */
/*                                                                          */
/*        void DSPF_dp_icfftr2(int n, double * x, double * w, int n_min)         */
/*                                                                          */
/*        x : input and output sequences (dim-n)      (input/output)        */
/*            x has n complex numbers (2*n DP values).                      */
/*            The real and imaginary values are interleaved in memory.      */
/*            The input is in normal order and output is in bit-reversed    */
/*            Order.                                                        */
/*        w : FFT coefficients (dim-n)                (input)               */
/*            w has n complex numbers (n DP values).                        */
/*            FFT coeficients are in a special sequence so that FFT can     */
/*            be called on smaller input sets multiple times to avoid       */
/*            cache thrashing.                                              */
/*            The real and imaginary values are interleaved in memory.      */
/*        n : FFT size which is a power of 2 and > 4  (input)               */
/*                                                                          */
/*                                                                          */
/* DESCRIPTION                                                              */
/*                                                                          */
/*      This routine is used to compute the Inverse Complex, Radix-2, Fast  */
/*      Fourier Transform of a double precision complex sequence of size    */
/*      n, and a power of 2 in a Cache-friendly way. The routine requires   */
/*      normal order input and normal order coefficents (twiddle factors)   */
/*      in a special sequence and produces results that are in              */
/*      bit-reversed order.                                                 */
/*                                                                          */
/*      The input can be broken into smaller parts and called multiple      */
/*      times to avoid cache thrashing.                                     */
/*                                                                          */
/*      How to use                                                          */
/*                                                                          */
/*      void main(void)                                                     */
/*      {                                                                   */
/*         gen_w_r2(w, N);         // Generate coefficient table            */
/*                                 // in normal order                       */
/*                                 // Function is given in C-CODE section   */
/*                                                                          */
/*        DSPF_dp_icfftr2(N, x, w, 1);  // input in normal order, output in      */
/*                                 // order bit-reversed                    */
/*                                                                          */
/*        bit_rev(x, N)            // Bit reverse the output if             */
/*                                 // normal order output is needed         */
/*                                 // Function is given in C-CODE section   */
/*                                                                          */
/*        divide(x, N);            // scale inverse FFT output              */
/*                                 // result is the same as original        */
/*                                 // input                                 */
/*                                                                          */
/*     }                                                                    */
/*                                                                          */
/*      main Inverse fft of size N can be divided into several steps (where */
/*      number of steps is a power of 2), allowing as much data             */
/*      reuse as possible.                                                  */
/*                                                                          */
/*      For example the following function                                  */
/*      DSPF_dp_icfftr2(N, x, w, 1);                                             */
/*                                                                          */
/*      is equivalent to:                                                   */
/*                                                                          */
/*      DSPF_dp_icfftr2(N, x, w, N/4);                                           */
/*                                                                          */
/*      DSPF_dp_icfftr2(N/4, &x[2 * 0 * (N/4)], &w[N + N/2], 1);                 */
/*      DSPF_dp_icfftr2(N/4, &x[2 * 1 * (N/4)], &w[N + N/2], 1);                 */
/*      DSPF_dp_icfftr2(N/4, &x[2 * 2 * (N/4)], &w[N + N/2], 1);                 */
/*      DSPF_dp_icfftr2(N/4, &x[2 * 3 * (N/4)], &w[N + N/2], 1);                 */
/*                                                                          */
/*      Notice how the 1st icfft function is called on the entire data set. */
/*      It covers the 1st pass of the fft until the butterfly size is       */
/*      N/4. The following 4 ffts do N/4 point ffts, 25% of the original    */
/*      size. These continue down to the end when the buttly is of size     */
/*      2. We use an index of 2* 3/4 *N to the main twiddle factor array    */
/*      for the last 4 calls. This is because the twiddle factor array      */
/*      is composed of successively decimated versions of the main array.   */
/*                                                                          */
/*      The twiddle factor array is composed of log2(N) sets of twiddle     */
/*      factors of size N, N/2, N/4, N/8 etc. The index into this array     */
/*      for each stage of the fft can be calculated by summing these        */
/*      indices up appropriately. For example, if we are dividing the       */
/*      input into 2 parts then index into this array should be N, if we    */
/*      are dividing into 4 parts then index into this array should be      */
/*      N+N/2, if we are dividing into 8 parts index should be N+N/2+N/4.   */
/*                                                                          */
/*      For multiple iffts they can share the same table by calling the     */
/*      small iffts from further down in the twiddle factor array, in the   */
/*      same way as the decomposition works for more data reuse.            */
/*                                                                          */
/*      The functions for creating this special sequence of twiddle         */
/*      factors and bit-reversal are provided in the C CODE section.        */
/*                                                                          */
/*      In general if divide the input into NO_OF_DIV parts we can call     */
/*      the function as follows:                                            */
/*                                                                          */
/*      // Divide the input into NO_OF_DIV parts                            */
/*      DSPF_dp_icfftr2(N, x, w, N/NO_OF_DIV);                                   */
/*                                                                          */
/*                                                                          */
/*      // Find out the index into twiddle factor array                     */
/*      for(w_index=0,j = NO_OF_DIV; j > 1 ; j >>= 1)                       */
/*      {                                                                   */
/*         w_index += j;                                                    */
/*      }                                                                   */
/*      w_index = N * w_index / NO_OF_DIV;                                  */
/*                                                                          */
/*      // Call the Function a subset of inputs                             */
/*      for(i=0; i<NO_OF_DIV; i++)                                          */
/*      {                                                                   */
/*         DSPF_dp_icfftr2(N/NO_OF_DIV, &x[2*i*(N/NO_OF_DIV)], &w[w_index], 1);  */
/*      }                                                                   */
/*                                                                          */
/* TECHNIQUES                                                               */
/*                                                                          */
/*      1. Outer loop instructions are executed in parallel with the        */
/*         inner loop epilog.                                               */
/*                                                                          */
/* ASSUMPTIONS                                                              */
/*                                                                          */
/*      1. Both input x and coefficient w should be aligned on double       */
/*         word boundary.                                                   */
/*      2. n should be greater than 4 and a power of 2.                     */
/*                                                                          */
/* C CODE                                                                   */
/*                                                                          */
/*        This is the C equivalent of the assembly code without             */
/*        restrictions. Note that the assembly code is hand optimized       */
/*        and restrictions may apply.                                       */
/*                                                                          */
/*        void DSPF_dp_icfftr2(int n, double * x, double * w, int n_min)         */
/*        {                                                                 */
/*             int n2, ie, ia, i, j, k, m;                                  */
/*             double rtemp, itemp, c, s;                                   */
/*                                                                          */
/*             n2 = n;                                                      */
/*             ie = 1;                                                      */
/*                                                                          */
/*             for(k = n; k > n_min; k >>= 1)                               */
/*              {                                                           */
/*                n2 >>= 1;                                                 */
/*                ia = 0;                                                   */
/*                for(j=0; j < ie; j++)                                     */
/*                {                                                         */
/*                 for(i=0; i < n2; i++)                                    */
/*                   {                                                      */
/*                      c = w[2*i];                                         */
/*                      s = w[2*i+1];                                       */
/*                      m = ia + n2;                                        */
/*                      rtemp     = x[2*ia]   - x[2*m];                     */
/*                      x[2*ia]   = x[2*ia]   + x[2*m];                     */
/*                      itemp     = x[2*ia+1] - x[2*m+1];                   */
/*                      x[2*ia+1] = x[2*ia+1] + x[2*m+1];                   */
/*                      x[2*m]    = c*rtemp   + s*itemp;                    */
/*                      x[2*m+1]  = c*itemp   - s*rtemp;                    */
/*                      ia++;                                               */
/*                     }                                                    */
/*                  ia += n2;                                               */
/*                 }                                                        */
/*                ie <<= 1;                                                 */
/*                w = w + k;                                                */
/*             }                                                            */
/*         }                                                                */
/*                                                                          */
/*         The follwoing C code is used to generate the coefficient table.  */
/*                                                                          */
/*          #include <math.h>                                               */
/*          // generate real and imaginary twiddle                          */
/*          // table of size n complex numbers (or 2*n numbers)             */
/*                                                                          */
/*          void gen_w_r2(double* w, int n)                                 */
/*          {                                                               */
/*               int i, j=1;                                                */
/*               double pi = 4.0*atan(1.0);                                 */
/*               double e = pi*2.0/n;                                       */
/*               for(j=1; j < n; j <<= 1)                                   */
/*               {                                                          */
/*               for(i=0; i < ( n>>1 ); i += j)                             */
/*                  {                                                       */
/*                  *w++   = cos(i*e);                                      */
/*                  *w++   = -sin(i*e);                                     */
/*                  }                                                       */
/*               }                                                          */
/*          }                                                               */
/*                                                                          */
/*                                                                          */
/*          The follwoing C code is used to bit-reverse the output.         */
/*                                                                          */
/*          bit_rev(double* x, int n)                                       */
/*          {                                                               */
/*            int i, j, k;                                                  */
/*            double rtemp, itemp;                                          */
/*                                                                          */
/*            j = 0;                                                        */
/*            for(i=1; i < (n-1); i++)                                      */
/*            {                                                             */
/*              k = n >> 1;                                                 */
/*               while(k <= j)                                              */
/*               {                                                          */
/*                  j -= k;                                                 */
/*                  k >>= 1;                                                */
/*               }                                                          */
/*               j += k;                                                    */
/*               if(i < j)                                                  */
/*               {                                                          */
/*                  rtemp    = x[j*2];                                      */
/*                  x[j*2]   = x[i*2];                                      */
/*                  x[i*2]   = rtemp;                                       */
/*                  itemp    = x[j*2+1];                                    */
/*                  x[j*2+1] = x[i*2+1];                                    */
/*                  x[i*2+1] = itemp;                                       */
/*               }                                                          */
/*            }                                                             */
/*          }                                                               */
/*                                                                          */
/*      The follwoing C code is used to perform the final scaling           */
/*      of the IFFT:                                                        */
/*                                                                          */
/*      // divide each element of x by n                                    */
/*                                                                          */
/*      divide(double* x, int n)                                            */
/*      {                                                                   */
/*        int i;                                                            */
/*        double inv = 1.0 / n;                                             */
/*                                                                          */
/*         for(i=0; i < n; i++)                                             */
/*          {                                                               */
/*              x[2*i] = inv * x[2*i];                                      */
/*              x[2*i+1] = inv * x[2*i+1];                                  */
/*            }                                                             */
/*       }                                                                  */
/*                                                                          */
/* NOTES                                                                    */
/*                                                                          */
/*      1. The special sequence of twiddle factor array w can be generated  */
/*        using the gen_w_r2 function provided in the previous section.     */
/*                                                                          */
/*      2. Endian:This code is LITTLE ENDIAN.                               */
/*                                                                          */
/*      3. Interruptibility: This code is interrupt-tolerant but not        */
/*         interruptible.                                                   */
/*                                                                          */
/* CYCLES                                                                   */
/*                                                                          */
/*       4 * n * lg(n) + 16 * lg(n) + 34                                    */
/*       eg. IF n = 64,  cycles = 1666                                      */
/*       eg. IF n = 32,  cycles = 754                                       */
/*                                                                          */
/* CODESIZE                                                                 */
/*                                                                          */
/*       1408 bytes                                                         */
/* ------------------------------------------------------------------------ */
/*            Copyright (c) 2004 Texas Instruments, Incorporated.           */
/*                           All Rights Reserved.                           */
/* ======================================================================== */
#ifndef DSPF_DP_ICFFTR2_ASM_H_
#define DSPF_DP_ICFFTR2_ASM_H_ 1

void DSPF_dp_icfftr2(int     n, double * x, double * w, int n_min);

#endif
/* ======================================================================== */
/*  End of file: DSPF_dp_icfftr2_h.asm                                           */
/* ------------------------------------------------------------------------ */
/*          Copyright (C) 2004 Texas Instruments, Incorporated.             */
/*                          All Rights Reserved.                            */
/* ======================================================================== */
