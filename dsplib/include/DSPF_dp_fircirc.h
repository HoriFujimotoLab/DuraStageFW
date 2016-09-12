/* ======================================================================= */
/* TEXAS INSTRUMENTS, INC.                                                 */
/*                                                                         */
/* NAME                                                                    */
/*     DSPF_dp_fircirc -- Double Precision Circular FIR algorithm               */
/*                                                                         */
/* USAGE                                                                    */
/*                                                                          */
/*    This routine has following C prototype:                               */
/*                                                                          */
/*    void DSPF_dp_fircirc (double *x, double *h, double *r, int index,          */
/*                     int csize, int nh, int nr)                           */
/*                                                                          */
/*            x[]     :     Input array (circular buffer of 2^(csize+1)     */
/*                          bytes)                                          */
/*                          Must be aligned at 2^(csize+1) byte boundary    */
/*            h[nh]   :     Filter coefficients array                       */
/*                          Must be double-word aligned                     */
/*            r[nr]   :     Output array                                    */
/*            index   :     Offset by which to start reading from the       */
/*                          input array. Must be multiple of 2              */
/*            csize   :     Size of circular buffer x[] is 2^(csize+1)      */
/*                          bytes. Must be 2 <= csize <= 31.                */
/*            nh      :     Number of filter coefficients                   */
/*                          Must be multiple of 2 and >= 4                  */
/*            nr      :     Size of output array                            */
/*                          Must be multiple of 4                           */
/*                                                                          */
/* DESCRIPTION                                                              */
/*                                                                          */
/*    This routine implements a circularly addressed FIR filter.            */
/*    'nh' is the number of filter coefficients. 'nr' is the number         */
/*    of the output samples.                                                */
/*                                                                          */
/* TECHNIQUES                                                               */
/*                                                                          */
/*    1. The outer loop is unrolled 4 times.                                */
/*    2. The inner loop is unrolled 2 times.                                */
/*    3. Regiter sharing is udes to make optimal utilisation of the         */
/*       available registers.                                               */
/*    4. Outerloop instructions and Prolog for next stage are               */
/*       scheduled in the last cycle of Kernel.                             */
/*                                                                          */
/* ASSUMPTIONS                                                              */
/*                                                                          */
/*    1. The circular input buffer x[] must be aligned at a 2^(csize+1)     */
/*       byte boundary. csize must lie in the range 2 <= csize <= 31.       */
/*    2. The number of coefficients  (nh) must be a multiple of 2           */
/*       and greater than or equal to 4.                                    */
/*    3. The number of outputs (nr) must be a multiple of 4 and             */
/*       greater than or equal to 4.                                        */
/*    4. The 'index' (offset to start reading input array) must be          */
/*       mutiple of 2 and less than or equal to (2^(csize-2) - 6)           */
/*    5. The coefficient array is assumed to be in reverse order,           */
/*       i.e. h(nh-1) to h(0) hold coeffs. h0, h1, h2 etc.                  */
/*                                                                          */
/*                                                                          */
/*  C CODE                                                                  */
/*                                                                          */
/*    This is the C equivalent for the assembly code.  Note that            */
/*    the assembly code is hand optimized and restrictions may              */
/*    apply.                                                                */
/*                                                                          */
/*    void DSPF_dp_fircirc  (double x[], double h[], double r[], int index,      */
/*                                  int csize, int nh, int nr)              */
/*      {                                                                   */
/*        int i, j;                                                         */
/*        //Circular Buffer block size = ((2^(csize + 1)) / 8)              */
/*        //floating point numbers                                          */
/*        int mod = (1 << (csize - 2));                                     */
/*        double r0;                                                        */
/*                                                                          */
/*        for (i = 0; i < nr; i++)                                          */
/*        {                                                                 */
/*            r0 = 0;                                                       */
/*            for (j = 0; j < nh; j++)                                      */
/*            {                                                             */
/*             //Operation "% mod" is equivalent to "& (mod -1)"            */
/*             //r0 += x[(i + j + index) % mod] * h[j];                     */
/*                                                                          */
/*               r0 += x[(i + j + index) & (mod - 1)] * h[j];               */
/*            }                                                             */
/*            r[i] = r0;                                                    */
/*        }                                                                 */
/*    }                                                                     */
/*                                                                          */
/* NOTES                                                                    */
/*                                                                          */
/*    1. Endian: This code is LITTLE ENDIAN.                                */
/*    2. Interruptibility: This code is interrupt-tolerant but not          */
/*       interruptible.                                                     */
/*                                                                          */
/* CYCLES                                                                   */
/*                                                                          */
/*    (2*nh + 2) nr + 38                                                    */
/*     For nh = 36 & nr=64, cycles = 4774                                   */
/*                                                                          */
/* CODESIZE                                                                 */
/*                                                                          */
/*     640 bytes                                                            */
/* ------------------------------------------------------------------------ */
/*            Copyright (c) 2004 Texas Instruments, Incorporated.           */
/*                           All Rights Reserved.                           */
/* ======================================================================== */
#ifndef DSPF_DP_FIRCIRC_H_
#define DSPF_DP_FIRCIRC_H_ 1

void DSPF_dp_fircirc     (double *x, double *h, double *r, int index,
                 int csize, int nh, int nr);

#endif
/* ======================================================================== */
/*  End of file:  DSPF_dp_fircirc.h                                              */
/* ------------------------------------------------------------------------ */
/*            Copyright (c) 2004 Texas Instruments, Incorporated.           */
/*                           All Rights Reserved.                           */
/* ======================================================================== */
