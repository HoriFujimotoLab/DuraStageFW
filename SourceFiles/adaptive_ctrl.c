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

#define INV2PITS (1.591549430918954e+03) //1/2/pi*FC(==10000)

#define RPM2HZ (0.016666666666667) //1/60


void kalman_filter(float *phi, float y, float *theta, float *P) {
	float  kalman_g[Nd], fm[1]; 
	float temp1[Nd*Nd], temp5[Nd*Nd], temp6[Nd], temp8[1];

	//eye_matrix(temp1, Nd, sigma_v*sigma_v);
	//ctrl_matrix_add(P, temp1, P_m, Nd, Nd); //P^-
	//P_m = P, now sigma = 0

	ctrl_matrix_prod(P, phi, kalman_g, Nd, Nd, 1); //[Nd, Nd], [Nd, 1]
	ctrl_vec_dot(phi, kalman_g, temp8, Nd);
	temp8[0] = recipsp(temp8[0] + sigma_w*sigma_w);
	ctrl_vec_scale(kalman_g, temp8, kalman_g, Nd);
	//calculate kalman gain

	ctrl_vec_dot(phi, theta, temp8 ,Nd);
	fm[0] = y - temp8[0];
	ctrl_vec_scale(kalman_g, fm, temp6, Nd);
	ctrl_vec_add(theta, temp6, theta, Nd); 
	//update theta

	ctrl_matrix_prod(kalman_g, phi, temp1, Nd, 1, Nd);
	eye_matrix(temp5, Nd, 1);
	ctrl_vec_sub(temp5, temp1, temp5, Nd*Nd);
	ctrl_matrix_prod(temp5, P, P, Nd, Nd, Nd); 
	//update P
}

//FS=2000 Hz loop
float dominant_freq(float b, float c) //dominant freq of  z^2 / ( z^2 + b z + c ) 
{
	float  real, imag;
	float fchat;

	float discriminant = b * b - 4.0* 1.0 * c;

	if (discriminant < 0) {
		real = -b *0.50;
		imag = sqrtsp(-discriminant) *0.50; //sqrt(D)/2/a
										   //z=u+j*w
										   //log(z)=log(abs(z)) + j*atan2(w,u)
										   //use atan2f(imag, real)
		fchat = atan2sp(imag, real)*INV2PITS; //2/pi/ts*FC
		return fchat;
	}

	return 0.0; //default
}

void calc_new_speed(int *rho, float *omega_new_rpm, float fchat,float omega_sp_rpm, int q) {
	float fTPE;
	fTPE = q*omega_sp_rpm*RPM2HZ;
	if (fTPE > 0) { 
		*rho = (int) (divsp(fchat ,  fTPE)+0.5);
		*omega_new_rpm =divsp(fchat* 60.0, (float)((*rho) * q));
	}
}
