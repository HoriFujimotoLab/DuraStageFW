/************************************************************************************
CURRENT CONTROL MODULE
----------------------
Descr.:		Control module for drive current control
Boards:		PE-Expert3, C6713A DSP (float calculation)
System:		S-PMSM motor-type (dq-axis control)
Author:		Thomas Beauduin, University of Tokyo, March 2016
*************************************************************************************/
#ifndef	CURRENT_CTRL_H
#define	CURRENT_CTRL_H

#include	"system_data.h"

/*	ZERO-CANCEL PI CTRL
**	-------------------
**	DES:	d/q-axis PI current control with zero-cancel ff
**	INP:	iq_ref	: q-axis reference current (id_ref = 0.0)
**			id/q_ad	: dq-axis measured current 
**	OUT:	vd/q_ref: dq-axis reference voltage 
*/
void current_ctrl_zcpi(float iq_ref, float id_ad, float iq_ad, float *vd_ref, float *vq_ref);


/*	DQ DECOUPLING CTRL
**	------------------
**	DES:	feedforward dq-axis decoupling ctrl
**	INP:	omega_m : measured motor angular velocity
**			id/q_ad	: dq-axis measured current
**	OUT:	vd/q_ref: decoupled dq-axis reference voltage 
*/
void current_ctrl_dec(float omega_m, float id_ad, float iq_ad, float *vd_ref, float *vq_ref);


/*	DEADTIME COMPENSATION CTRL
**	--------------------------
**	DES:	hysteresis compensation of inverter deadtime distortion
**	INP:	iq_ref	  : q-axis reference current (hysteresis input) 
**			theta_e	  : motor electrical angle (axis-transform)
**			vdc_ad	  : measured dc-bus voltage
**	OUT:	vu/v/w_ref: calculated phase voltage references
*/
void current_ctrl_dtc(float iq_ref, float theta_e, float vdc_ad, float *vu_ref, float *vv_ref, float *vw_ref);


/*	REFERENCE FRAMES TRANFORM
**	-------------------------
**	DES:	motor reference frame transformation for FOC
**	INP:	uvw-axis: phase reference frame
**			ab-axis	: stationary reference frame
**	OUT:	dq-axis	: rotating reference frame
*/
void current_ctrl_uw2ab(float u, float w, float *a, float *b);
void current_ctrl_ul2ab(float uv, float vw, float *a, float *b);
void current_ctrl_ab2dq(float a, float b, float theta_e, float *d, float *q);
void current_ctrl_dq2ab(float d, float q, float theta_e, float *a, float *b);
void current_ctrl_ab2uvw(float a, float b, float *u, float *v, float *w);


/*	RESET CONTROL MODULE
**	--------------------
**	DES:	resets module internal counters and variables
**			necessary at measurement init for good startup
*/
void current_ctrl_reset(void);

#endif
