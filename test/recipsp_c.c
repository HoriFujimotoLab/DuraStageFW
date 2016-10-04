/* ======================================================================= */
/* recipsp_c.c - single precision floating point reciprocal                */
/*              optimized  C implementation (w/ intrinsics) single sample  */
/*                                                                         */
/* Rev 0.0.1                                                               */
/*                                                                         */
/* ----------------------------------------------------------------------- */
/*            Copyright (c) 2010 Texas Instruments, Incorporated.          */
/*                           All Rights Reserved.                          */
/* ======================================================================= */

#include "recipsp_c.h"


float recipsp_c(float a)
{

/*                                   (2 ^ +126)*4          (2 ^ -126)   */
float   X,Y,Zero=0.0,TWO=2.0,Big=8.5070592e37*TWO*TWO,Small=1.17549435e-38 ;

        Y = fabsf(a) ;
        X = _rcpsp(a) ;
        
        X = X*( TWO - a*X ) ;
        X = X*( TWO - a*X ) ;

        if (Y < Small){     
           X = Big ;                /* Div by Small, return MAX    */
              
              if (a < Zero){
                 	X = -Big ;
              }  
        
        }
  		
		if (Y > Big) X = 0 ;            /* Div by MAX, return 0 */
        return (X) ;           /* returns SP FP value = 1.0 / A  */

}

/* ======================================================================== */
/*  End of file: recipsp_c.c                                                */
/* ------------------------------------------------------------------------ */
/*          Copyright (C) 2010 Texas Instruments, Incorporated.             */
/*                          All Rights Reserved.                            */
/* ======================================================================== */

