/************************************************************************************
ADAPTIVE CONTROL MODULE
-------------------
Descr.:		NLMS and Kalman Filter for adaptive control
Boards:		PE-Expert3, C6713A DSP (floating point calculation)
System:		Dura experimental setup
Author:		Shimoda Takaki, University of Tokyo, 2016
*************************************************************************************/
 
#include "adaptive_ctrl.h"

#define sigma_w (0.3) //observation noise variance
#define sigma_v (1e-16) //process noise variance

#define INV2PITS (1.273239544735163e+03) //1/2/pi*FC


void kalman_filter(float *phi, float y, float *theta, float *P) {
	float  P_m[Nd*Nd], kalman_g[Nd], fm[1]; 
	float temp1[Nd*Nd], temp2[Nd], temp3[1], temp4[Nd*Nd], temp5[Nd*Nd];

	eye_matrix(temp1, Nd, sigma_v*sigma_v);
	ctrl_matrix_add(P, temp1, P_m, Nd, Nd); //P^-

	ctrl_matrix_prod(P_m, phi, kalman_g, Nd, Nd, 1);
	ctrl_matrix_prod(phi, kalman_g, temp3, 1, Nd, 1);
	ctrl_matrix_cnst_mlpy(kalman_g, 1 / (temp3[0] + sigma_w*sigma_w), Nd, 1); //kalman gain

	ctrl_matrix_prod(phi, theta, temp3, 1, Nd, 1);
	fm[0] = y - temp3[0];
	ctrl_matrix_prod(kalman_g, fm, temp2, Nd, 1, 1);
	ctrl_matrix_add(theta, temp2, theta, Nd, 1); //update theta

	ctrl_matrix_prod(kalman_g, phi, temp1, Nd, 1, Nd);
	eye_matrix(temp5, Nd, 1);
	ctrl_matrix_minus(temp5, temp1, temp4, Nd, Nd);
	ctrl_matrix_prod(temp4, P_m, P, Nd, Nd, Nd); //update P
}


float dominant_freq(float b, float c) //dominant freq of  z^2 / ( z^2 + b z + c ) 
{
	float  real, imag;
	float fchat;

	float discriminant = b * b - 4.0* 1.0 * c;

	if (discriminant < 0) {
		real = -b *0.50;
		imag = sqrtf(-discriminant) *0.50; //sqrt(D)/2/a
										   //z=u+j*w
										   //log(z)=log(abs(z)) + j*atan2(w,u)
										   //use atan2f(imag, real)
		fchat = atan2f(imag, real)*INV2PITS; //2/pi/ts
		return fchat;
	}

	return 2000; //default
}
