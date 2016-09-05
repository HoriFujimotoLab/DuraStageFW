#pragma once
/************************************************************************************
ADAPTIVE CONTROL MODULE
-------------------
Descr.:		NLMS and Kalman Filter for adaptive control
Boards:		PE-Expert3, C6713A DSP (floating point calculation)
System:		Dura experimental setup
Author:		Shimoda Takaki, University of Tokyo, 2016
*************************************************************************************/

#include "ctrl_math.h"
#include "system_data.h"

/*
DES:Kalman Filter for pararter estimation
IN: phi[k], y[k], theta[k-1], P[k-1];
OUT: theta[k], P[k]

KALMAN FILTER:
	//assumption, state space realization of paramter 
	
	theta[k+1] = theta[k] +  simga_v[k]
	y[k] = phi[k]^T * theta[k] + simga_w[k]

	where

	phi[k]=[-y[k-1], y[k-2], ..., -y[k-Nd]^T
	theta[k]=[a1, a2, ..., aNd]
*/
void kalman_filter(float *phi, float y, float *theta, float *P);

/*
DES:calculate dominant frequency from dominant root of 1/D: d2 + d1 z + z^2
IN: b(=d1), c(=d2)
OUT:dominant frequency
*/
float dominant_freq(float b, float c);

/*
DES:calculate new stable spindle speed [rpm]
*/
void calc_new_speed(int *rho, float *omega_new_rpm, float fchat, float omega_sp_rpm, int q);

