/************************************************************************************
TMS320C67x DSP LIBRARY
----------------------
Descr.:		optimized .asm/.c dsp-functions implementation
Author:		Thomas Beauduin
			Hori-Fujimoto Lab, University of Tokyo, 2016
Copyright (c) 2010 Texas Instruments, Incorporated.
http://www.ti.com/tool/SPRC121
*************************************************************************************/
#ifndef	C67X_DSPLIB_H
#define	C67X_DSPLIB_H

/*	MATRIX MULTIPLICATION
**	---------------------
**	DES:	This function computes the expression "r = x * y"
**  ASP:    1. The arrays 'x', 'y', and 'r' are stored in distinct arrays.
**			   That is, in-place processing is not allowed.
**			2. All r1, c1, c2 are assumed to be > 1.
**			3. If r1 is odd, one extra row of x[] matrix is loaded.
**			4. If c2 is odd, one extra col of y[] matrix is loaded.
**			5. If c1 is odd, one extra col of x[] and one extra row of y[] array is loaded
**  MW3:	(v) DSPF_sp_mat_mul: Matrix x Vector
**			(x) DSPF_dp_mat_mul: Matrix x Vector
**			(v) DSPF_sp_mat_mul: Matrix x Matrix
**			(v) DSPF_dp_mat_mul: Matrix x Matrix
**			(x) DSPF_sp_mat_mul: Vector x Vector
**			(x) DSPF_dp_mat_mul: Vector x Vector
*/
void DSPF_dp_mat_mul(
	double	*x,					// pointer to r1 by c1 input matrix.
	int		r1,					// number of rows in x.
	int		c1,					// number of columns in x.
	double	*y,					// pointer to c1 by c2 input matrix.
	int		c2,					// number of columns in y.
	double	*r					// Pointer to r1 by c2 output matrix.
);
void DSPF_sp_mat_mul(float *x, int r1, int c1, float *y, int c2, float *r);


/*	VECTOR MULTIPLICATION
**	---------------------
**	DES:	element by element multiplication of the vectors x[] and y[]
**  ASP:	1. The value of n > 0.
**  MW3:	(v) DSPF_dp_vecmul
**			(x) DSPF_sp_vecmul
*/
void DSPF_dp_vecmul(
	double *x,					// pointer to first input array 
	double *y,					// pointer to second input array 
	double *r,					// pointer to output array
	int	n						// number of elements in arrays
);
void DSPF_sp_vecmul(float *x, float *y, float *r, int n);


/*  WEIGTHED VECTOR ADDITION
**	------------------------
**	DES:	this routine is used to obtain the weighted vector sum.
**  ASP:	1. The value of nr must be > 0.
**  MW3:	(v) DSPF_dp_w_vec
**			(x) DSPF_sp_w_vec
*/
void DSPF_dp_w_vec(
	double *x,					// Pointer to first input array
	double *y,					// Pointer to second input array
	double	m,					// Weight factor
	double *r,					// Output array pointer
	int nr						// Number of elements in arrays
);
void DSPF_sp_w_vec(float *x, float *y, float m, float *r, int nr);


/*  MAXIMUM VALUE
**	-------------
** DES:		find maximum number in the input array.
** ASP:		1. nx should be multiple of 2 and >= 2.
**			2. NAN in the input is disregarded.
*/
double DSPF_dp_maxval(
	double	*x,					// Pointer to Input array.
	int		nx					// Number of Inputs in the input Array.
);
float DSPF_sp_maxval(float* x, int nx);


/*  VECTOR DOTPRODUCT
**	-----------------
**	DES:	This routine calculates the dot product of 2 vectors.
**  MW3:	(x) DSPF_dp_dotprod
**			(x) DSPF_sp_dotprod
double DSPF_dp_dotprod(
	double *x,				// Pointer to array holding first vector
	double *y,				// Pointer to array holding second vector
	int		nx				// Number of values in the x & y vectors
);
float DSPF_sp_dotprod(float *x, float *y, int nx);
*/


/*  MATRIX TRANSPOSE
**	----------------
**	DES:	transpose input matrix x[] into matrix r[].
**  ASP :	1. The number of rows and columns is > 0.
void DSPF_dp_mat_trans(
const double *restrict x,	// Input matrix containing rows*cols
int rows,					// No. of rows in matrix x.
int cols,					// No. of columns in matrix x.
double *restrict r			// Output matrix containing cols*rows.
);
void DSPF_sp_mat_trans(const float *restrict x, int rows, int cols, float *restrict r);
*/


/*	TIME CONVOLUTION
**	-----------------
**	DES:	full-length convolution of real vectors x and h using time-domain techniques
**  ASP:	1. nh is a multiple of 2 and greater than or equal to 4
**          2. nr is a multiple of 4
void DSPF_dp_convol(
	double	*x,					// pointer to input samples
	double	*h,					// pointer to impulse response samples
	double	*r,					// pointer to output samples
	int		nh,					// number of impulse response samples
	int		nr					// number of output samples
);
void DSPF_sp_convol(float *x, float *h, float *r, int nh, int nr);
*/

/*	IIR FILTER
**	----------
**	DES:	The IIR performs an auto-regressive moving-average (ARMA)
**			filter with 4 auto-regressive filter coefficients and 5
**			moving-average filter coefficients for nr output samples.
**  ASP:	1. The value of 'nr' must be > 0.
**			2. Extraneous loads are allowed in the program.
void DSPF_dp_iir(
	double* restrict r1,		// Delay element values(i / p and o / p)
	const double*    x,
	double* restrict r2,
	const double*    h2,
	const double*    h1,
	int nr
);
*/

/*	AUTOCORRELATION
**	---------------
**	DES:	autocorrelation of the input array x
**  ASP:	1. nx is a multiple of 2 and greater than or equal to 4.
**			2. nr is a multiple of 4 and greater than or equal to 4.
**			3. nx is greater than or equal to nr
void DSPF_dp_autocor(
	double	*  restrict r,		// pointer to output array of autocorr length nr
	double  *  restrict x,		// pointer to input array of length nx + nr
	int     nx,					// Length of autocorrelation vector
	int     nr					// Length of lags
);
void DSPF_sp_autocor(float *restrict r, float *restrict x, int nx, int nr);
*/

#endif
