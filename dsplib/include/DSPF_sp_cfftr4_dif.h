/* ======================================================================== */
/*  TEXAS INSTRUMENTS, INC.                                                 */
/*                                                                          */
/*  NAME                                                                    */
/*      DSPF_sp_cfftr4_dif -- DSPF_sp_cfftr4_dif                                      */
/*                                                                          */
/*  ÅUSAGEÅ                                                                 */
/*                                                                          */
/*  This routine is C Callable and can be called as:                        */
/*  ÅÅ                                                                      */
/*  ÅÅvoid cfftr4_dif(float* x, float* w, short n)                          */
/*                                                                          */
/*  ÅÅx is pointer to an array holding the input and output floating        */
/*  ÅÅ   point array which contains 'n' complex points                      */
/*  ÅÅw is pointer to an array holding the coefficient floating point       */
/*  ÅÅ   array which contains 3*n/4 complex numbers                         */
/*  ÅÅn is the number of complex points in x                                */
/*                                                                          */
/*  ÅÅIf the routine is not to be used as a C callable function,            */
/*  ÅÅthen you need to initialize values for all of the parameters          */
/*  ÅÅpassed to the function since these are assumed to be in               */
/*  ÅÅregisters as defined by the calling convention of the                 */
/*  ÅÅcompiler, (refer to the TMS320C6x Optimizing C Compiler               */
/*  ÅÅUser's Guide).                                                        */
/*                                                                          */
/*                                                                          */
/*  ÅDESCRIPTION                                                            */
/*                                                                          */
/*  ÅÅThis routine implements the DIF (decimation in frequency)             */
/*  ÅÅcomplex radix 4 FFT with digit-reversed output and normal             */
/*  ÅÅorder input.  The number of points, 'n', must be a power              */
/*  ÅÅof 4 {4, 16, 64, 256, 1024, ...}.  This routine is an                 */
/*  ÅÅin-place routine in the sense that the output is written              */
/*  ÅÅover the input.  It is not an in-place routine in the                 */
/*  ÅÅsense that the input is in normal order and the output is             */
/*  ÅÅin digit-reversed order.                                              */
/*                                                                          */
/*  ÅÅThere must be n complex points (2*n values), and 3*n/4 complex        */
/*  ÅÅcoefficients (3*n/2 values).                                          */
/*                                                                          */
/*  ÅÅEach real and imaginary input value is interleaved in the             */
/*  ÅÅ'x' array {rx0, ix0, rx1, ix2, ...} and the complex numbers           */
/*  ÅÅare in normal order.  Each real and imaginary output value            */
/*  ÅÅis interleaved in the 'x' array and the complex numbers are           */
/*  ÅÅin digit-reversed  order {rx0, ix0, ...}.  The real and               */
/*  ÅÅimaginary values of the coefficients are interleaved in the           */
/*  ÅÅ'w' array {rw0, -iw0, rw1, -iw1, ...} and the complex numbers         */
/*  ÅÅare in normal order.                                                  */
/*                                                                          */
/*  ÅÅNote that the imaginary coefficients are negated                      */
/*  ÅÅ{cos(d*0),  sin(d*0),  cos(d*1),   sin(d*1), ...} rather than         */
/*  ÅÅ{cos(d*0), -sin(d*0),  cos(d*1),  -sin(d*1), ...}                     */
/*  ÅÅwhere d = 2*PI/n.  The value of w(n,k) is usually written             */
/*  ÅÅw(n,k) = e^-j(2*PI*k/n) = cos(2*PI*k/n) - sin(2*PI*k/n).              */
/*                                                                          */
/*  ÅÅThe routine can be used to implement an inverse FFT by                */
/*  ÅÅperforming the complex conjugate on the input complex numbers         */
/*  ÅÅ(negating the imaginary value), and dividing the result by n.         */
/*  ÅÅAnother method to use the FFT to perform an inverse FFT, is to        */
/*  ÅÅswap the real and imaginary values of the input and the result,       */
/*  ÅÅand divide the result by n.  In either case, the input is still       */
/*  ÅÅin normal order and the output is still in digit-reversed order.      */
/*                                                                          */
/*  ÅÅNote that you can not make the radix 4 FFT into an inverse            */
/*  ÅÅFFT by using the complex conjugate of the coefficients as             */
/*  ÅÅyou can do with the complex radix 2 FFT.                              */
/*                                                                          */
/*  ÅÅIf you label the input locations from 0 to (n-1) (normal order),      */
/*  ÅÅthe digit-reversed locations can be calculated by reversing the       */
/*  ÅÅorder of the bit pairs of the labels.  For example, for a 1024        */
/*  ÅÅpoint FFT, the digit reversed location for                            */
/*  ÅÅ617d = 1001101001b = 10 01 10 10 01  is                               */
/*  ÅÅ422d = 0110100110b = 01 10 10 01 10  and visa versa.                  */
/*                                                                          */
/*  ÅÅTI plans to provide an inverse complex radix 4 FFT with digit-        */
/*  ÅÅreversed input and normal order output.  With these 2 routines,       */
/*  ÅÅthe digit-reversed output of the FFT can be the input to the          */
/*  ÅÅinverse FFT without performing digit-reversing on the data.           */
/*                                                                          */
/*  ÅÅTI also plans to provide a complex digit reverse routine.             */
/*                                                                          */
/* ------------------------------------------------------------------------ */
/*            Copyright (c) 2002 Texas Instruments, Incorporated.           */
/*                           All Rights Reserved.                           */
/* ======================================================================== */
#ifndef DSPF_SP_CFFTR4_DIF_ASM_H_
#define DSPF_SP_CFFTR4_DIF_ASM_H_ 1

void DSPF_sp_cfftr4_dif(float     * x, float * w, short n);

#endif
/* ======================================================================== */
/*  End of file: DSPF_sp_cfftr4_dif_h.asm                                        */
/* ------------------------------------------------------------------------ */
/*          Copyright (C) 2002 Texas Instruments, Incorporated.             */
/*                          All Rights Reserved.                            */
/* ======================================================================== */
