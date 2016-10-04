/* ======================================================================= */
/* DSPF_sp_vecmul.c -- Vector Multiply                                     */
/*              Optimized C Implementation (w/ Intrinsics)                 */
/*                                                                         */
/* Rev 0.0.1                                                               */
/*                                                                         */
/* ----------------------------------------------------------------------- */
/*            Copyright (c) 2009 Texas Instruments, Incorporated.          */
/*                           All Rights Reserved.                          */
/* ======================================================================= */
#pragma CODE_SECTION(DSPF_sp_vecmul, ".text:intrinsic");

#include "DSPF_sp_vecmul.h"

void DSPF_sp_vecmul(const float * x1, const float * x2,
    float *restrict y, const int n)
{
    int i;    
    double x1x0, x3x2;

    _nassert((int)x1 % 8 == 0);
    _nassert((int)x2 % 8 == 0);
    _nassert(n % 2 == 0);
    _nassert(n > 0);

    #pragma MUST_ITERATE(1,,)
    for(i = 0; i < n; i+=2)
    {
        x1x0= _amemd8((void*)(x1 + i));
        x3x2= _amemd8((void*)(x2 + i));

        y[i]     = _itof(_lo(x1x0)) * _itof(_lo(x3x2));
        y[i + 1] = _itof(_hi(x1x0)) * _itof(_hi(x3x2)); 
    }
}

/* ======================================================================= */
/*  End of file:  DSPF_sp_vecmul.c                                         */
/* ----------------------------------------------------------------------- */
/*            Copyright (c) 2009 Texas Instruments, Incorporated.          */
/*                           All Rights Reserved.                          */
/* ======================================================================= */

