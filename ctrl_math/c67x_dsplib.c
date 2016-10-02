/************************************************************************************
DSP LIBRARY
-----------
Descr.:		mathematic module for simulation purposes
Author:		Thomas Beauduin, Wataru Ohnishi
			Hori-Fujimoto Lab, University of Tokyo, 2016
*************************************************************************************
IMPORTANT NOTICE:
THIS CODE IS JUST A COPY-PASTE FROM THE DSP-LIB USER-MANUAL
AND IS ONLY INTENDED FOR SIL (SOFTWARE-IN-THE-LOOP) SIMULATION. 
IT SHOULD NOT BE INCLUDED IN MYWAY PROJECT (INSTEAD TI c67x DSP OPTIMIZED LIBRARY).
*************************************************************************************/
#include "c67x_dsplib.h"

void DSPF_sp_mat_mul(float *x, int r1, int c1, float *y, int c2, float *r)
{
	int i, j, k;
	float sum;
	// Multiply each row in x by each column in y.
	// The product of row m in x and column n in y is placed
	// in position (m,n) in the result.
	for (i = 0; i < r1; i++)
		for (j = 0; j < c2; j++)
		{
			sum = 0;
			for (k = 0; k < c1; k++)
				sum += x[k + i*c1] * y[j + k*c2];
			r[j + i*c2] = sum;
		}
}


void DSPF_sp_w_vec(const float *x, const float *y, float m, float *r, int nr)
{
	int i;
	for (i = 0; i < nr; i++)
		r[i] = (m * x[i]) + y[i];
}

