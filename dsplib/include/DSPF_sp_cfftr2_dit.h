/* ======================================================================== */
/*  TEXAS INSTRUMENTS, INC.                                                 */
/*                                                                          */
/*  NAME                                                                    */
/*      DSPF_sp_cfftr2_dit -- Single Precision floating point radix-2 FFT with   */
/*      complex input                                                       */
/*                                                                          */
/*  USAGE                                                                   */
/*          This routine is C-callable and can be called as:                */
/*                                                                          */
/*          void DSPF_sp_cfftr2_dit(float * x, float * w, short n);              */
/*                                                                          */
/*          x : Pointer to complex data input                               */
/*          w : pointer to complex twiddle factor in Bit reverse order      */
/*          N : length of FFT in complex samples, power of 2 such that N>=3 */
/*                                                                          */
/*  DESCRIPTION                                                             */
/*                                                                          */
/*      This routine performs the Decimation-in-Time (DIT) Radix-2 FFT      */
/*      of the input array x.                                               */
/*      x has N complex floating point numbers arranged as successive       */
/*      real and imaginary number pairs. Input array x contains N           */
/*      complex points (N*2 elements). The coefficients for the             */
/*      FFT are passed to the function in array w which contains            */
/*      N/2 complex numbers (N elements) as successive real and             */
/*      imaginary number pairs.                                             */
/*      The FFT Coefficients w are in N/2 bit-reversed order                */
/*      The elements of input array x are in normal order                   */
/*      The assembly routine performs 4 output samples (2 real and 2        */
/*      imaginary) for a pass through inner loop.                           */
/*                                                                          */
/*      Note that (bit-reversed) coefficients for higher order FFT (1024    */
/*      point) can be used unchanged as coefficients for a lower order      */
/*      FFT (512, 256, 128 ... ,2)                                          */
/*                                                                          */
/*      The routine can be used to implement Inverse-FFT by any ONE of      */
/*      the following methods:                                              */
/*                                                                          */
/*      1.Inputs (x) are replaced by their Complex-conjugate values         */
/*        Output values are divided by N                                    */
/*      2.FFT Coefficients (w) are replaced by their Complex-conjugates     */
/*        Output values are divided by N                                    */
/*      3.Swap Real and Imaginary values of input                           */
/*      4.Swap Real and Imaginary values of output                          */
/*                                                                          */
/*  C CODE                                                                  */
/*                                                                          */
/*      This is the C equivalent of the assembly code.  Note that           */
/*      the assembly code is hand optimized and restrictions may            */
/*      apply.                                                              */
/*                                                                          */
/*      void DSPF_sp_cfftr2_dit(float* x, float* w, short n)                     */
/*      {                                                                   */
/*         short n2, ie, ia, i, j, k, m;                                    */
/*         float rtemp, itemp, c, s;                                        */
/*                                                                          */
/*         n2 = n;                                                          */
/*         ie = 1;                                                          */
/*                                                                          */
/*         for(k=n; k > 1; k >>= 1)                                         */
/*         {                                                                */
/*            n2 >>= 1;                                                     */
/*            ia = 0;                                                       */
/*            for(j=0; j < ie; j++)                                         */
/*            {                                                             */
/*               c = w[2*j];                                                */
/*               s = w[2*j+1];                                              */
/*               for(i=0; i < n2; i++)                                      */
/*               {                                                          */
/*                  m = ia + n2;                                            */
/*                  rtemp     = c * x[2*m]   + s * x[2*m+1];                */
/*                  itemp     = c * x[2*m+1] - s * x[2*m];                  */
/*                  x[2*m]    = x[2*ia]   - rtemp;                          */
/*                  x[2*m+1]  = x[2*ia+1] - itemp;                          */
/*                  x[2*ia]   = x[2*ia]   + rtemp;                          */
/*                  x[2*ia+1] = x[2*ia+1] + itemp;                          */
/*                  ia++;                                                   */
/*               }                                                          */
/*               ia += n2;                                                  */
/*            }                                                             */
/*            ie <<= 1;                                                     */
/*         }                                                                */
/*      }                                                                   */
/*                                                                          */
/*                                                                          */
/* ------------------------------------------------------------------------ */
/*            Copyright (c) 2002 Texas Instruments, Incorporated.           */
/*                           All Rights Reserved.                           */
/* ======================================================================== */
#ifndef DSPF_SP_CFFTR2_DIT_ASM_H_
#define DSPF_SP_CFFTR2_DIT_ASM_H_ 1

void DSPF_sp_cfftr2_dit(float     * x, float * w, int n);

#endif
/* ======================================================================== */
/*  End of file: DSPF_sp_cfftr2_dit_h.asm                                        */
/* ------------------------------------------------------------------------ */
/*          Copyright (C) 2002 Texas Instruments, Incorporated.             */
/*                          All Rights Reserved.                            */
/* ======================================================================== */
