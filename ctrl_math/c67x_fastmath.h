/************************************************************************************
TMS320C67x FASTMATH LIBRARY
---------------------------
Descr.:		optimized .asm/.c implementation of arithmetics
Author:		Thomas Beauduin
Hori-Fujimoto Lab, University of Tokyo, 2016
Copyright (c) 2010 Texas Instruments, Incorporated.
http://www.ti.com/tool/mathlib
*************************************************************************************/
#ifndef	C67X_FASTMATH_H
#define	C67X_FASTMATH_H

/*	SCALAR ARITHMETICS (SP)
**	-----------------------
**	DES:	single precision scalar fast math Kernels.
*/
float atan2sp(float a, float b);
float atansp(float a);
float cossp(float a);
float divsp(float a, float b);
float exp2sp(float a);
float exp10sp(float a);
float expsp(float a);
float log2sp(float a);
float log10sp(float a);
float logsp(float a);
float powsp(float a, float b);
float recipsp(float a);
float rsqrtsp(float a);
float sinsp(float a);
float sqrtsp(float a);

/*	VECTOR ARITHMETICS (SP)
**	-----------------------
**	DES:	single precision vector fast math Kernels.
void atan2sp_v(float *arg, float *arg2, float *output, int size);
void atansp_v(float *arg, float *output, int size);
void cossp_v(float *arg, float *output, int size);
void divsp_v(float *arg, float *arg2, float *output, int size);
void exp2sp_v(float *arg, float *output, int size);
void exp10sp_v(float *arg, float *output, int size);
void expsp_v(float *arg, float *output, int size);
void log2sp_v(float *arg, float *output, int size);
void log10sp_v(float *arg, float *output, int size);
void logsp_v(float *arg, float *output, int size);
void powsp_v(float *arg, float *arg2, float *output, int size);
void recipsp_v(float *arg, float *output, int size);
void rsqrtsp_v(float *arg, float *output, int size);
void sinsp_v(float *arg, float *output, int size);
void sqrtsp_v(float *arg, float *output, int size);
*/

/*	SCALAR ARITHMETICS (DP)
**	-----------------------
**	DES:	Double precision scalar fast math Kernels.
*/
double atan2dp(double a, double b);
double atandp(double a);
double cosdp(double a);
double divdp(double a, double b);
double exp2dp(double a);
double exp10dp(double a);
double expdp(double a);
double log2dp(double a);
double log10dp(double a);
double logdp(double a);
double powdp(double a, double b);
double recipdp(double a);
double rsqrtdp(double a);
double sindp(double a);
double sqrtdp(double a);

/*	VECTOR ARITHMETICS (DP)
**	-----------------------
**	DES:	Double precision vector fast math Kernels.
void atan2dp_v(double *  arg, double *  arg2, double *  output, int size);
void atandp_v(double *arg, double *output, int size);
void cosdp_v(double *arg, double *output, int size);
void divdp_v(double *arg, double *arg2, double *output, int size);
void exp2dp_v(double *arg, double *output, int size);
void exp10dp_v(double *arg, double *output, int size);
void expdp_v(double *arg, double *output, int size);
void log2dp_v(double *arg, double *output, int size);
void log10dp_v(double *arg, double *output, int size);
void logdp_v(double *arg, double *output, int size);
void powdp_v(double *arg, double *arg2, double *output, int size);
void recipdp_v(double *arg, double *output, int size);
void rsqrtdp_v(double *arg, double *output, int size);
void sindp_v(double *arg, double *output, int size);
void sqrtdp_v(double *arg, double *output, int size);
*/

#endif
