/************************************************************************************
CONTROL MATH MODULE
-------------------
Descr.:		mathematic module for control algebra calculations
Boards:		PE-Expert3, C6713A DSP (floating point calculation)
Author:		Thomas Beauduin, University of Tokyo, 2015
*************************************************************************************/
#include "ctrl_math.h"

#define	NMAX		(4)					// max amount of states

void ctrl_math_state(float A[], float x[], float B[], float u[], float *dx, int nrofs, int nrofi)
{
	float Ax[NMAX] = { 0.0 }; float Bu[NMAX] = { 0.0 };
	DSPF_sp_mat_mul(A, nrofs, nrofs, x, 1, Ax);
	switch (nrofi)
	{
	case 1:		ctrl_vec_scale(B, u, Bu, nrofs);				break;				// SI
	default:	DSPF_sp_mat_mul(B, nrofs, nrofi, u, 1, Bu);		break;				// MI
	}
	ctrl_vec_add(Ax, Bu, &*dx, nrofs);
}


void ctrl_math_output(float C[], float x[], float D[], float u[], float *y, int nrofs, int nrofi, int nrofo)
{
	float Cx[NMAX] = { 0.0 }; float Du[NMAX] = { 0.0 };
	switch (nrofo)
	{
	case 1:		ctrl_vec_dot(C, x, &Cx[0], nrofs);
				switch (nrofi)
				{
				case 1:		Du[0] = D[0] * u[0];						break;		// SISO
				default:	ctrl_vec_dot(D, u, &Du[0], nrofi);			break;		// MISO
				}
	break;
	default:	DSPF_sp_mat_mul(C, nrofo, nrofs, x, 1, Cx);
				switch (nrofi)
				{
				case 1:		ctrl_vec_scale(D, u, Du, nrofo);			break;		// SIMO
				default:	DSPF_sp_mat_mul(D, nrofo, nrofi, u, 1, Du);	break;		// MIMO
				}
	break;
	}
	ctrl_vec_add(Cx, Du, y, nrofs);
}


void ctrl_vec_add(float *x, float *y, float *r, int nr)
{
	int i;
	for (i = 0; i < nr; i++) {
		r[i] = x[i] + y[i];
	}
}
void ctrl_vec_mul(float *x, float *y, float *r, int nr)
{
	int i;
	for (i = 0; i < nr; i++) {
		r[i] = x[i] * y[i];
	}
}
void ctrl_vec_scale(float *x, float *w, float *r, int nr)
{
	int i;
	for (i = 0; i < nr; i++) {
		r[i] = *w * x[i];
	}
}
void ctrl_vec_dot(float *x, float *y, float *r, int nr)
{
	int i; *r = 0.0;
	for (i = 0; i < nr; i++) {
		*r += x[i] * y[i];
	}
}

