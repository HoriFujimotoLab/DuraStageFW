/* ======================================================================= */
/* TEXAS INSTRUMENTS, INC.                                                 */
/*                                                                         */
/* NAME                                                                    */
/*     DSPF_dp_cfftr4_dif -- Double Precision floating point Decimation in      */
/*     Frequency radix-4 FFT with complex input                            */
/*                                                                         */
/*  USAGE                                                                   */
/*                                                                          */
/*  This routine is C Callable and can be called as:                        */
/*                                                                          */
/*  void DSPF_dp_cfftr4_dif(double* x, double* w, short n)                       */
/*                                                                          */
/*  x : Pointer to an array holding the input and output floating           */
/*      point array which contains 'n' complex points                       */
/*  w : Pointer to an array holding the coefficient floating point          */
/*      array which contains 3*n/4 complex numbers                          */
/*  n : Number of complex points in x                                       */
/*                                                                          */
/*                                                                          */
/*  DESCRIPTION                                                             */
/*                                                                          */
/*  This routine implements the DIF (decimation in frequency)               */
/*  complex radix 4 FFT with digit-reversed output and normal               */
/*  order input.  The number of points, 'n', must be a power                */
/*  of 4 {4, 16, 64, 256, 1024, ...}.  This routine is an                   */
/*  in-place routine in the sense that the output is written                */
/*  over the input.  It is not an in-place routine in the                   */
/*  sense that the input is in normal order and the output is               */
/*  in digit-reversed order.                                                */
/*                                                                          */
/*  There must be n complex points (2*n values), and 3*n/4 complex          */
/*  coefficients (3*n/2 values).                                            */
/*                                                                          */
/*  Each real and imaginary input value is interleaved in the               */
/*  'x' array {rx0, ix0, rx1, ix2, ...} and the complex numbers             */
/*  are in normal order.  Each real and imaginary output value              */
/*  is interleaved in the 'x' array and the complex numbers are             */
/*  in digit-reversed  order {rx0, ix0, ...}.  The real and                 */
/*  imaginary values of the coefficients are interleaved in the             */
/*  'w' array {rw0, -iw0, rw1, -iw1, ...} and the complex numbers           */
/*  are in normal order.                                                    */
/*                                                                          */
/*  Note that the imaginary coefficients are negated                        */
/*  {cos(d*0),  sin(d*0),  cos(d*1),   sin(d*1), ...} rather than           */
/*  {cos(d*0), -sin(d*0),  cos(d*1),  -sin(d*1), ...}                       */
/*  where d = 2*PI/n.  The value of w(n,k) is usually written               */
/*  w(n,k) = e^-j(2*PI*k/n) = cos(2*PI*k/n) - sin(2*PI*k/n).                */
/*                                                                          */
/*  The routine can be used to implement an inverse FFT by                  */
/*  performing the complex conjugate on the input complex numbers           */
/*  (negating the imaginary value), and dividing the result by n.           */
/*  Another method to use the FFT to perform an inverse FFT, is to          */
/*  swap the real and imaginary values of the input and the result,         */
/*  and divide the result by n.  In either case, the input is still         */
/*  in normal order and the output is still in digit-reversed order.        */
/*                                                                          */
/*  Note that you can not make the radix 4 FFT into an inverse              */
/*  FFT by using the complex conjugate of the coefficients as               */
/*  you can do with the complex radix 2 FFT.                                */
/*                                                                          */
/*  If you label the input locations from 0 to (n-1) (normal order),        */
/*  the digit-reversed locations can be calculated by reversing the         */
/*  order of the bit pairs of the labels.  For example, for a 1024          */
/*  point FFT, the digit reversed location for                              */
/*  617d = 1001101001b = 10 01 10 10 01  is                                 */
/*  422d = 0110100110b = 01 10 10 01 10  and visa versa.                    */
/*                                                                          */
/*  TECHNIQUES                                                              */
/*                                                                          */
/*  1.  All the three loops are executed as one loop with                   */
/*      conditional instructions.                                           */
/*  2.  The outer loop counter is used as load counterto prevent            */
/*      extraneous loads.                                                   */
/*  3.  If more registers were available, the inner loop could              */
/*      probably be as small as 28 cycles,but the loop was extended         */
/*      to 56 cycles to allow more variables to share registers.            */
/*  4.  The pointer for X and W are maintianed  on both register sides      */
/*      to avoid crosspath Conflicts.                                       */
/*  5.  Variable tctr is used as innerloop Counter.                          */
/*  6.  The variable, K, is used as the outer loop counter.We are           */
/*      finished when n2b = 0.                                              */
/*                                                                          */
/*  ASSUMPTIONS                                                             */
/*                                                                          */
/*  1.  There are no special alignment requirements.                        */
/*                                                                          */
/*  C CODE                                                                  */
/*                                                                          */
/*  This is the C equivalent for the assembly code.  Note that              */
/*  the assembly code is hand optimized and restrictions may                */
/*  apply.                                                                  */
/*                                                                          */
/*  void DSPF_dp_cfftr4_dif(double* x, double* w, short n)                       */
/*  {                                                                       */
/*      short n1, n2, ie, ia1, ia2, ia3, i0, i1, i2, i3, j, k;              */
/*      double r1, r2, r3, r4, s1, s2, s3, s4, co1, co2, co3;               */
/*      double si1, si2, si3;                                               */
/*      n2 = n;                                                             */
/*      ie = 1;                                                             */
/*      for(k=n; k>1; k>>=2)                                                */
/*      {                                                                   */
/*         n1 = n2;                                                         */
/*         n2 >>= 2;                                                        */
/*         ia1 = 0;                                                         */
/*         for(j=0; j<n2; j++)                                              */
/*         {                                                                */
/*             ia2 = ia1 + ia1;                                             */
/*             ia3 = ia1 + ia2;                                             */
/*             co1 = w[ia1*2];                                              */
/*             si1 = w[ia1*2 + 1];                                          */
/*             co2 = w[ia2*2];                                              */
/*             si2 = w[ia2*2 + 1];                                          */
/*             co3 = w[ia3*2];                                              */
/*             si3 = w[ia3*2 + 1];                                          */
/*             ia1 += ie;                                                   */
/*             for(i0=j; i0<n; i0+=n1)                                      */
/*             {                                                            */
/*                 i1 = i0 + n2;                                            */
/*                 i2 = i1 + n2;                                            */
/*                 i3 = i2 + n2;                                            */
/*                 r1 = x[i0*2]   + x[i2*2];                                */
/*                 r3 = x[i0*2]   - x[i2*2];                                */
/*                 s1 = x[i0*2+1] + x[i2*2+1];                              */
/*                 s3 = x[i0*2+1] - x[i2*2+1];                              */
/*                 r2 = x[i1*2]   + x[i3*2];                                */
/*                 r4 = x[i1*2]   - x[i3*2];                                */
/*                 s2 = x[i1*2+1] + x[i3*2+1];                              */
/*                 s4 = x[i1*2+1] - x[i3*2+1];                              */
/*                 x[i0*2]   = r1 + r2;                                     */
/*                 r2        = r1 - r2;                                     */
/*                 r1        = r3 - s4;                                     */
/*                 r3        = r3 + s4;                                     */
/*                 x[i0*2+1] = s1 + s2;                                     */
/*                 s2        = s1 - s2;                                     */
/*                 s1        = s3 + r4;                                     */
/*                 s3        = s3 - r4;                                     */
/*                 x[i1*2]   = co1*r3 + si1*s3;                             */
/*                 x[i1*2+1] = co1*s3 - si1*r3;                             */
/*                 x[i2*2]   = co2*r2 + si2*s2;                             */
/*                 x[i2*2+1] = co2*s2 - si2*r2;                             */
/*                 x[i3*2]   = co3*r1 + si3*s1;                             */
/*                 x[i3*2+1] = co3*s1 - si3*r1;                             */
/*            }                                                             */
/*         }                                                                */
/*         ie <<= 2;                                                        */
/*      }                                                                   */
/*  }                                                                       */
/*                                                                          */
/*  NOTES                                                                   */
/*                                                                          */
/*     1. The twiddle factor array w can be generated by the tw_r4fft       */
/*        function provided in dsplib\support\fft\tw_r4fft.c. The exe       */
/*        file for this function, dsplib\bin\tw_r4fft.exe, can be           */
/*        used dump the twiddle factor array into a file.                   */
/*     2. The function bit_rev in dsplib\support\fft can be used to         */
/*        bit-reverse the output array to convert it into normal order.     */
/*     3. Endian: This code is LITTLE  ENDIAN.                              */
/*     4. Interruptibility: This code is interrupt tolerant but not         */
/*        interruptible.                                                    */
/*                                                                          */
/*  CYCLES                                                                  */
/*                                                                          */
/*    14*n*log4(n) + 46                                                     */
/*    eg. if n = 256, cycles = 14382.                                       */
/*                                                                          */
/*  CODESIZE                                                                */
/*      1344 bytes                                                          */
/* ------------------------------------------------------------------------ */
/*            Copyright (c) 2004 Texas Instruments, Incorporated.           */
/*                           All Rights Reserved.                           */
/* ======================================================================== */
#ifndef DSPF_DP_CFFTR4_DIF_H_
#define DSPF_DP_CFFTR4_DIF_H_ 1

void DSPF_dp_cfftr4_dif(double*     x, double* w, short n);

#endif
/* ======================================================================== */
/*  End of file:  DSPF_dp_cfftr4_dif.h                                           */
/* ------------------------------------------------------------------------ */
/*            Copyright (c) 2004 Texas Instruments, Incorporated.           */
/*                           All Rights Reserved.                           */
/* ======================================================================== */
