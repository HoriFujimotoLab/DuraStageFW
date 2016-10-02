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
static float xpid[2] = { 0.0, 0.0 };
static float xspvpi[1] = { 0.0 }; //spindle variable state
float sd[2] = { 0, 0 }, td[2] = { 0, 0 }, ud[2] = { 0,0 }, vd[2] = { 0,0 };//disturbance observer states





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

void spindle_motion_ctrl_vpi(float vm_ref, float omega_m, float *t_ref)
{
	float vm_er[1];

	vm_er[0] = vm_ref - omega_m;
	ctrl_math_output(Cspvpi, xspvpi, Dspvpi, vm_er, t_ref, 1, 1, 1);
	ctrl_math_state(Aspvpi, xspvpi, Bspvpi, vm_er, xspvpi, 1, 1);
	if (fabsf(*t_ref) > T_SP) { *t_ref = sign(*t_ref) * T_SP; }		// limit torque
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

float estimated_disturbance(float t_ref, float omega_m) {
	//Torque Command td																		Speed vd
	//			|																									|
	//			|																									|	
	//			------------->[Q(s)]----sd---> - O + <---ud---[Q(s)*(Js+D)]----
	//															   |
	//															   |
	//															   V
	//									OBSEVED DISTURBANCE TORQUE


	//input
	td[1] = t_ref;
	vd[1] = omega_m;

	//disturbance observer
	sd[1] = DOB_A*sd[0] + DOB_B*td[0];
	ud[1] = DOB_C*ud[0] + DOB_D*vd[1] - DOB_E*vd[0];
	
	//update
	sd[0] = sd[1];
	ud[0] = ud[1];
	td[0] = td[1];
	vd[0] = vd[1];

	return ( ud[1] - sd[1] );
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
	torque_command = 0;
	iq_adx = 0; iq_ady = 0;
	omega_sp_ma_rpm = 0;
}
