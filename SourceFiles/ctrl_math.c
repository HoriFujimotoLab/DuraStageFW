/************************************************************************************
CONTROL MATH MODULE
-------------------
Descr.:		mathematic module for control algebra calculations
Boards:		PE-Expert3, C6713A DSP (floating point calculation)
System:		Yuna single axis ball-screw experimental setup
Author:		Thomas Beauduin, University of Tokyo, 2015
*************************************************************************************/
#include "ctrl_math.h"

void ctrl_math_state(float A[], float x[], float B[], float u[], float *dx, int nrofs, int nrofi)
{
	float C1[NMAX] = { 0.0 }; float C2[NMAX] = { 0.0 };
	ctrl_matrix_prod(A, x, C1, nrofs, nrofs, 1);
	ctrl_matrix_prod(B, u, C2, nrofs, nrofi, 1);
	ctrl_matrix_add(C1, C2, &*dx, nrofs, 1);
}


void ctrl_math_output(float C[], float x[], float D[], float u[], float *y, int nrofs, int nrofi, int nrofo)
{
	float C1[NMAX] = { 0.0 }; float C2[NMAX] = { 0.0 };
	ctrl_matrix_prod(C, x, C1, nrofo, nrofs, 1);
	ctrl_matrix_prod(D, u, C2, nrofo, nrofi, 1);
	ctrl_matrix_add(C1, C2, &*y, nrofo, 1);
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


void ctrl_matrix_add(float a[], float b[], float *c, int row, int col)
{
	int i, j;
	for (i = 0; i < row; i++){
		for (j = 0; j < col; j++){
			c[i*col + j] = a[i*col + j] + b[i*col + j];
		}
	}
}

void ctrl_matrix_minus(float a[], float b[], float *c, int row, int col)
{
	int i, j;
	for (i = 0; i < row; i++) {
		for (j = 0; j < col; j++) {
			c[i*col + j] = a[i*col + j] - b[i*col + j];
		}
	}
}

void ctrl_matrix_cnst_mlpy(float *c, float p, int row_c, int col_c) {
	int i, j;
	for (i = 0; i < row_c; i++) {
		for (j = 0; j<col_c; j++) {
			c[row_c*i + j] *= p;
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

/*
void transpose_matrix(float *c, float *c_T,  int row_c, int col_c) {
	int i, j;
	for (i = 0; i < row_c; i++) {
		for (j = 0; j < col_c; j++) {
			c_T[j*row_c + i] = c[i*row_c + j];
		}
	}
}
*/

