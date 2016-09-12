/* ======================================================================== */
/*  TEXAS INSTRUMENTS, INC.                                                 */
/*                                                                          */
/*  NAME                                                                    */
/*      DSPF_sp_w_vec -- Single Precision weighted sum of vectors                */
/*                                                                          */
/*  USAGE                                                                   */
/*                                                                          */
/*      This routine is C callable, and has the following C prototype:      */
/*                                                                          */
/*        void DSPF_sp_w_vec(const float* x,                                     */
/*                      const float * y,                                    */
/*                      float m,                                            */
/*                      float * restrict r,                                 */
/*                      int     nr                                          */
/*                     )                                                    */
/*                                                                          */
/*             x    :  Pointer to first input array                         */
/*             y    :  Pointer to second input array                        */
/*             m    :  Weight factor                                        */
/*             r    :  Output array pointer                                 */
/*             nr   :  Number of elements in arrays                         */
/*                                                                          */
/*  DESCRIPTION                                                             */
/*                                                                          */
/*     This routine is used to obtain the weighted vector sum.              */
/*     Both the inputs and output are single precision floating             */
/*     point numbers.                                                       */
/*    C CODE                                                                */
/*     This is the C equivalent of the Assembly Code without                */
/*     restrictions.                                                        */
/*                                                                          */
/*     void DSPF_sp_w_vec(    const float *    x,                                */
/*                       const float *    y,                                */
/*                       float         m,                                   */
/*                       float *        restrict r,                         */
/*                       int         nr                                     */
/*                    )                                                     */
/*                                                                          */
/*    {                                                                     */
/*        int i;                                                            */
/*        for (i = 0; i < nr; i++)                                          */
/*            r[i] = (m * x[i]) + y[i];                                     */
/*    }                                                                     */
/*                                                                          */
/*                                                                          */
/*                                                                          */
/* ------------------------------------------------------------------------ */
/*            Copyright (c) 2002 Texas Instruments, Incorporated.           */
/*                           All Rights Reserved.                           */
/* ======================================================================== */
#ifndef DSPF_SP_W_VEC_ASM_H_
#define DSPF_SP_W_VEC_ASM_H_ 1

void DSPF_sp_w_vec(const     float * x, const float * y, float m, float * restrict r, int nr);

#endif
/* ======================================================================== */
/*  End of file: DSPF_sp_w_vec_h.asm                                             */
/* ------------------------------------------------------------------------ */
/*          Copyright (C) 2002 Texas Instruments, Incorporated.             */
/*                          All Rights Reserved.                            */
/* ======================================================================== */
