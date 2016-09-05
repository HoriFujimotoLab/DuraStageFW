/************************************************************************************
MOTION CONTROL MODULE
----------------------
Descr.:		Control module for stage motion control
Boards:		PE-Expert3, C6713A DSP (float calculation)
System:		Yuna ball-screw experimental setup
Author:		Thomas Beauduin, University of Tokyo, 2015
*************************************************************************************/
#include "motion_ctrl.h"
#include "ctrl_math.h"
#include "motion_ctrl_par.h"
#include "Rfreqref.h"
#include "Rtimeref.h"

int ref = 0; double t = 0.0;
static float xvpi[2][1] = { 0.0, 0.0 }; //{{XAXIS} , {YAXIS}}
static float xpid[2] = { 0.0 };


void direct_qcurrent_ctrl(int reftype_e, float Aref, float Fref, float *iq_ref){
	switch (reftype_e)
	{
	case 0:	*iq_ref = 0.0;							break;
	case 1:	*iq_ref = Aref;							break;
	case 2:	*iq_ref = Aref*sin(Fref*PI(2)*t);
		t += (TS*1.0e-6);							break;
	case 5:											break;
	}
}

void motion_ctrl_ref(int reftype_e, float Aref, float Fref, float *x_ref)
{
	switch (reftype_e)
	{
	case 0:	*x_ref = 0.0;							break;
	case 1:	*x_ref = Aref;							break;
	case 2:	*x_ref = Aref*sin(Fref*PI(2)*t);
		t += (TS*1.0e-6);							break;
	case 3: *x_ref = Aref*freqref[ref];
		if (ref < (FREQ - 1))	{ ref++; }
		else					{ ref = 0; }		break;
	case 4:	*x_ref = Aref*timeref[ref];
		if (ref < (TIME - 1))	{ ref++; }
		else					{ ref = 0; }		break;
	case 5:											break;
	}
}


void motion_ctrl_vpi(int axis, float vm_ref, float omega_m, float *iq_ref)
{
	float vm_er[1];

	if (fabsf(vm_ref) > VELXLIM) { vm_ref = sign(vm_ref) * VELXLIM; }		// limit speed
	vm_er[0] = vm_ref - omega_m;
	ctrl_math_output(Cvpi[axis], xvpi[axis], Dvpi[axis], vm_er, iq_ref, 1, 1, 1);
	ctrl_math_state(Avpi[axis], xvpi[axis], Bvpi[axis], vm_er, xvpi[axis], 1, 1);
	if (fabsf(*iq_ref) > I_PK) { *iq_ref = sign(*iq_ref) * I_PK; }		// limit torque
}

float motion_ctrl_pos(float x_ref, float x_m) {
	return Kxp*(x_ref - x_m);
}

void motion_ctrl_pack_pos(int axis, float omega_m, float theta_ref, float theta_m, float *iq_ref) { //angular
	float vm_ref;
	vm_ref = motion_ctrl_pos(theta_ref, theta_m); //angular
	if (fabsf(vm_ref) > VELXLIM) { vm_ref = sign(vm_ref) * VELXLIM; }		// limit speed
	motion_ctrl_vpi(axis, vm_ref, omega_m, &(*iq_ref));
}

void motion_ctrl_pid(float x_ref, float x_msr, float *iq_ref)
{
	float x_err[1] = { 0.0 };
	x_err[0] = x_ref - x_msr;
	ctrl_math_output(Cpid[0], xpid, Dpid[0], x_err, iq_ref, 2, 1, 1);
	ctrl_math_state(Apid[0], xpid, Bpid[0], x_err, xpid, 2, 1);
	if (fabsf(*iq_ref) > I_PK) { *iq_ref = sign(*iq_ref) * I_PK; }		// limit torque
}


void motion_ctrl_reset(void)
{
	int i, j;
	for (i = 0; i < 2; i++) {
		xvpi[i][1] = 0.0;
	}
	xpid[0] = 0.0; xpid[1] = 0.0;
	dac_da_out(0, 3, 0);
	for (i = 0; i < Nd; i++) {
		phi_sp[i] = 0;
		theta_par_est[i] =0;
		for (j = 0; j < Nd; j++) {
			if (i==j) P_var[i*Nd +j] =  1e-4;
			else  P_var[i*Nd + j] = 0;
		}
	}
	ctime = 0;
}
