/* ======================================================================= */
/* DSPF_sp_dotprod.c -- Dot Product                                        */
/*              Optimized C Implementation (w/ Intrinsics)                 */
/*                                                                         */
/* Rev 0.0.1                                                               */
/*                                                                         */
/* ----------------------------------------------------------------------- */
/*            Copyright (c) 2009 Texas Instruments, Incorporated.          */
/*                           All Rights Reserved.                          */
/* ======================================================================= */
#pragma CODE_SECTION(DSPF_sp_dotprod, ".text:intrinsic");

#include "DSPF_sp_dotprod.h"

float DSPF_sp_dotprod(const float * x, const float * y, const int n)
{
    double x1_x0;
    double y1_y0;
    
    float x0, x1, y0, y1;
    float sum1 = 0.0f;    
    float sum2 = 0.0f;
    int i;

    _nassert(n > 0);
    _nassert(n % 2 == 0);
    _nassert((int)x % 8 == 0);
    _nassert((int)y % 8 == 0);
    
    /* Unrolling by 8 */
    #pragma UNROLL(4)
    for(i = 0; i < n; i+=2)    
    {
        x1_x0 = _amemd8_const ( (void *) (x + i));
        y1_y0 = _amemd8_const ( (void *) (y + i));
        
        x0 = _itof(_lo(x1_x0));
        x1 = _itof(_hi(x1_x0));
        y0 = _itof(_lo(y1_y0));
        y1 = _itof(_hi(y1_y0));
        
        sum1 += x0 * y0;
        sum2 += x1 * y1;
    }
    
    return (sum1 + sum2);
}

/* ======================================================================= */
/*  End of file:  DSPF_sp_dotprod.c                                        */
/* ----------------------------------------------------------------------- */
/*            Copyright (c) 2009 Texas Instruments, Incorporated.          */
/*                           All Rights Reserved.                          */
/* ======================================================================= */

