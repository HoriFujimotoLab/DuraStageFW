/************************************************************************************
CONTROL MATH MODULE
-------------------
Descr.:		mathematic module for control algebra calculations
Boards:		PE-Expert3, C6713A DSP (floating point calculation)
System:		Yuna single axis ball-screw experimental setup
Author:		Thomas Beauduin, University of Tokyo, 2015
*************************************************************************************/
#include "ctrl_math.h"

//void ctrl_math_state(float A[], float x[], float B[], float u[], float *dx, int nrofs, int nrofi)
//{
//	float C1[NMAX] = { 0.0 }; float C2[NMAX] = { 0.0 };
//	ctrl_matrix_prod(A, x, C1, nrofs, nrofs, 1);
//	ctrl_matrix_prod(B, u, C2, nrofs, nrofi, 1);
//	ctrl_matrix_add(C1, C2, &*dx, nrofs, 1);
//}
//
//
//void ctrl_math_output(float C[], float x[], float D[], float u[], float *y, int nrofs, int nrofi, int nrofo)
//{
//	float C1[NMAX] = { 0.0 }; float C2[NMAX] = { 0.0 };
//	ctrl_matrix_prod(C, x, C1, nrofo, nrofs, 1);
//	ctrl_matrix_prod(D, u, C2, nrofo, nrofi, 1);
//	ctrl_matrix_add(C1, C2, &*y, nrofo, 1);
//}

#define	NMAX		(4)					// max amount of states

//ONLY FOR SI
void ctrl_math_state(float A[], float x[], float B[], float u[], float *dx, int nrofs)
{
	int nrofi = 1;
	float Ax[NMAX] = { 0.0 }; float Bu[NMAX] = { 0.0 };
	ctrl_matrix_prod(A, x, Ax, nrofs, nrofs, nrofi);//DSPF_sp_mat_mul(A, nrofs, nrofs, x, 1, Ax);
	ctrl_vec_scale(B, u, Bu, nrofs);					// SI
	ctrl_vec_add(Ax, Bu, &*dx, nrofs);
}

//ONLY FOR SISO
void ctrl_math_output(float C[], float x[], float D[], float u[], float *y, int nrofs)
{
	//int nrofi = 1; int  nrofo = 1;
	float Cx[NMAX] = { 0.0 }; float Du[NMAX] = { 0.0 };
	ctrl_vec_dot(C, x, &Cx[0], nrofs);
	Du[0] = D[0] * u[0];					
	ctrl_vec_add(Cx, Du, y, nrofs);
}


void ctrl_vec_add(float *x, float *y, float *r, int nr)
{
	int i;
	for (i = 0; i < nr; i++) {
		r[i] = x[i] + y[i];
	}
}
void ctrl_vec_sub(float *x, float *y, float *r, int nr)
{
	int i;
	for (i = 0; i < nr; i++) {
		r[i] = x[i] - y[i];
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

void ctrl_matrix_prod(float a[], float b[], float *c, int row_a, int col_a, int col_b)
{
	int i, j, k;
	for (i = 0; i < row_a; i++){
		for (j = 0; j < col_b; j++){
			float sum = 0;
			for (k = 0; k < col_a; k++){
				sum += a[i*col_a + k] * b[k*col_b + j]; 
			}
			c[i*col_b + j] = sum;
		}
	}
}

void eye_matrix(float *e, int n, float value) {
	int i, j;
	for (i = 0; i < n; i++) {
		for (j = 0; j < n; j++) {
			if (i == j) {
				e[i*n + j] = value;
			}
			else {
				e[i*n + j] = 0;
			}
		}
	}
}


