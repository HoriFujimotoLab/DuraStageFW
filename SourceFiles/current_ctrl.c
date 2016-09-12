/************************************************************************************
CURRENT CONTROL MODULE
----------------------
Descr.:		Control module for drive current control
Boards:		PE-Expert3, C6713A DSP (float calculation)
System:		S-PMSM motor-type (dq-axis control)
Author:		Thomas Beauduin, University of Tokyo, March 2016
*************************************************************************************/
#include "current_ctrl.h"
#include "ctrl_math.h"
#include "current_ctrl_par.h"

double xff_q[2] = { 0.0, 0.0 }, xfb_d[2] = { 0.0, 0.0 }, xfb_q[2] = { 0.0, 0.0 };

void current_ctrl_zcpi(int axis, float iq_ref, float id_ad, float iq_ad, float *vd_ref, float *vq_ref)
{
	double iq_ff, id_er, iq_er;
	
	iq_ff = Cffi * xff_q[axis] + Dffi * (double)iq_ref;						// input shaping
	xff_q[axis] = Affi * xff_q[axis] + Bffi * (double)iq_ref;
	
	id_er = 0.0 - (double)id_ad;										// dq-axis feedback
	iq_er = iq_ff - (double)iq_ad;
	*vd_ref = (float)(Cfbi * xfb_d[axis] + Dfbi * id_er);
	xfb_d[axis] = Afbi * xfb_d[axis] + Bfbi * id_er;
	*vq_ref = (float)(Cfbi * xfb_q[axis] + Dfbi * iq_er);
	xfb_q[axis] = Afbi * xfb_q[axis] + Bfbi * iq_er;
}


void current_ctrl_dec(float omega_m, float id_ad, float iq_ad, float *vd_ref, float *vq_ref){
	*vd_ref -= (omega_m * Pp * Ls * iq_ad);								// coupling compensation
	*vq_ref += omega_m * Pp * Ke ;				//decoupling(0) and  back-emf compensation
}


void current_ctrl_dtc(float iq_ref, float theta_e, float vdc_ad, float *vu_ref, float *vv_ref, float *vw_ref)
{
	double dtc_err;														// dtc comp voltage
	float ia_ref, ib_ref, iu_ref, iv_ref, iw_ref;						// stationary frame ref

	dtc_err = vhys * vdc_ad;											// deadtime calc
	current_ctrl_dq2ab(0.0, iq_ref, theta_e, &ia_ref, &ib_ref);			// frame transform
	current_ctrl_ab2uvw(ia_ref, ib_ref, &iu_ref, &iv_ref, &iw_ref);		// rot to stationary
	if		(iu_ref >  ihys)	{ *vu_ref += dtc_err; }					// dt comp hysteresis
	else if (iu_ref < -ihys)	{ *vu_ref -= dtc_err; }
	if		(iv_ref >  ihys)	{ *vv_ref += dtc_err; }
	else if (iv_ref < -ihys)	{ *vv_ref -= dtc_err; }
	if		(iw_ref >  ihys)	{ *vw_ref += dtc_err; }
	else if (iw_ref < -ihys)	{ *vw_ref -= dtc_err; }
}


void current_ctrl_reset(void)
{
	int i;
	for (i = 0; i < 2; i++) {
		xff_q[i] = 0.0; xfb_d[i] = 0.0; xfb_q[i] = 0.0;
	}
}


void current_ctrl_uw2ab(float u, float w, float *a, float *b)
{
	*a = TSQRT3_2 * u;
	*b = -TSQRT1_2 * u - TSQRT2 * w;
}


void current_ctrl_ul2ab(float uv, float vw, float *a, float *b)
{
	*a = TSQRT2_3 * uv + TSQRT1_6 * vw;
	*b = TSQRT1_2 * vw;
}


void current_ctrl_ab2dq(float a, float b, float theta_e, float *d, float *q)
{
	*d =  cossp(theta_e) * a + sinsp(theta_e) * b;
	*q = -sinsp(theta_e) * a + cossp(theta_e) * b;
}


void current_ctrl_dq2ab(float d, float q, float theta_e, float *a, float *b)
{
	*a = cossp(theta_e) * d - sinsp(theta_e) * q;
	*b = sinsp(theta_e) * d + cossp(theta_e) * q;

}


void current_ctrl_ab2uvw(float a, float b, float *u, float *v, float *w)
{
	*u = TSQRT2_3 * a;
	*v = -TSQRT1_6 * a + TSQRT1_2 * b;
	*w = -TSQRT1_6 * a - TSQRT1_2 * b;
}

