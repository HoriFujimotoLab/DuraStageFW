/************************************************************************************
CONTROL MATH MODULE
-------------------
Descr.:		mathematic module for control algebra calculations
Boards:		PE-Expert3, C6713A DSP (floating point calculation)
Author:		Thomas Beauduin, University of Tokyo, 2015
*************************************************************************************/

#ifndef	CTRL_MATH_H
#define	CTRL_MATH_H

#include <math.h>

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


/*	EFFICIENT MATRIX ALGEBRA
**	-------------------------
**	DES:	linear algebra functions optimized for embedded c
**	INP:	C,D [0]	: ctrl parameter matrices (row-major array)
**			x,u		: state & input vectors (single array)
**			s/i/o	: number of states / inputs / outputs
**	OUT:	y		: new calculated output vector
*/
void ctrl_matrix_prod(float a[], float b[], float *c, int row_a, int col_a, int col_b);
void ctrl_matrix_add(float a[], float b[], float *c, int row, int col);


/* PRECALC MATH CONST
** ------------------
** DES:		precalculated constants allocated for speed & ease
**			declared in header for general use in ctrl modules
*/
#define	NMAX		(8)					// max amount of states
#define	TSQRT2		((double)1.4142135623730950488016887242097)
#define TSQRT1_6	((double)0.40824829046386301636621401245098)
#define	TSQRT1_2	((double)0.70710678118654752440084436210485)
#define	TSQRT2_3	((double)0.81649658092772603273242802490196)
#define	TSQRT3_2	((double)1.2247448713915890490986420373529)

#endif

