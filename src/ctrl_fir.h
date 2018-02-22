/************************************************************************************
FIR MODULE
-------------------
Descr.:		fir calculation for filtering
Boards:		PE-Expert3, C6713A DSP (floating point calculation)
System:		Dura single axis ball-screw experimental setup
Author:		Shimoda Takaki, University of Tokyo, 2018
*************************************************************************************/

#pragma once

/*
int i: current index
int m: start from i-m  index
int npar: == len(c)
int ndata: == len(u)
float c[]: cofficient vector
float u[]: input vector
return: c[0] * z^(i+m) + ... + c[j] * z^(i+m+j) + ... + c[n-1] * z^(i+m+n-1)
*/
float fir(int i, int m, int npar, int ndata, float c[], float u[]);
