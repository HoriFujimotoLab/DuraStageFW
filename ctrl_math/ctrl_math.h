/************************************************************************************
CONTROL MATH MODULE
-------------------
Descr.:		mathematic module for control algebra calculations
Boards:		PE-Expert3, C6713A DSP (floating point calculation)
Author:		Thomas Beauduin, University of Tokyo, 2015
*************************************************************************************/
#ifndef	CTRL_MATH_H
#define	CTRL_MATH_H

#include "c67x_fastmath.h"
#include "c67x_dsplib.h"

/*	STATE-SPACE STATE-EQUATION
**	--------------------------
**	DES:	state-equation calculation: dx = Ax + Bu
**	INP:	A,B [0]	: ctrl parameter matrices (row-major array)
**			x,u		: state & input vectors (single array)
**			nrofs/i : number of states / inputs
**	OUT:	dx		: new calculated state vector
*/
void ctrl_math_state(float A[], float x[], float B[], float u[], float *dx, int nrofs, int nrofi);


/*	STATE-SPACE OUTPUT-EQUATION
**	---------------------------
**	DES:	state-equation calculation: y = Cx + Du
**	INP:	C,D [0]	: ctrl parameter matrices (row-major array)
**			x,u		: state & input vectors (single array)
**			s/i/o	: number of states / inputs / outputs
**	OUT:	y		: new calculated output vector
*/
void ctrl_math_output(float C[], float x[], float D[], float u[], float *y, int nrofs, int nrofi, int nrofo);


/*	LINEAR ALGEBRA
**	--------------
**	DES:	conventional linear algebra functions
*/
void ctrl_vec_add(float *x, float *y, float *r, int nr);
void ctrl_vec_mul(float *x, float *y, float *r, int nr);
void ctrl_vec_scale(float *x, float *w, float *r, int nr);
void ctrl_vec_dot(float *x, float *y, float *r, int nr);


/* PRECALC MATH CONST
** ------------------
** DES:		precalculated constants allocated for speed & ease
**			declared in header for general use in ctrl modules
*/
#define	C_SQRT2		((double)1.4142135623730950488016887242097)		// (2)^0.5
#define C_SQRT1_6	((double)0.40824829046386301636621401245098)	// (1/6)^0.5
#define	C_SQRT1_2	((double)0.70710678118654752440084436210485)	// (1/2)^0.5
#define	C_SQRT2_3	((double)0.81649658092772603273242802490196)	// (2/3)^0.5
#define	C_SQRT3_2	((double)1.2247448713915890490986420373529)		// (3/2)^0.5

#endif

