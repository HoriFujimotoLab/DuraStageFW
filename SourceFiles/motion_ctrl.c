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
static float xvpi[XYMODE][NMAX] = { 0.0 }; //{{XAXIS} , {YAXIS}} stage state
static float xpid[NMAX] = { 0.0 };
static float xspvpi[NMAX] = { 0.0 }; //spindle variable state
static float xq[NMAX] = { 0.0}, xr[NMAX] = {0.0 };//disturbance observer states
static float xstn[NMAX] = { 0.0 }; // notch filter for stage x
static float xqstx[NMAX] = { 0.0 }, xrstx[NMAX] = { 0.0 }; //disturbance observer for stage states
static float xlpf2[NMAX] = { 0.0 }, xhpf2[NMAX] = { 0.0 }; //2 order lpf and hpf for DOB

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
	//Velocity PI control
	vm_er[0] = vm_ref - omega_m;
	ctrl_math_output(Cvpi[axis], xvpi[axis], Dvpi[axis], vm_er, iq_ref, 1);
	ctrl_math_state(Avpi[axis], xvpi[axis], Bvpi[axis], vm_er, xvpi[axis], 1);
	//if (axis == XAXIS) {
	////DOB
	//	dob_stx = estimated_disturbance_stx(*iq_ref, omega_m);
	//	*iq_ref -= dob_stx;
	////notch
	// *iq_ref = notch_stage_x(*iq_ref);
	//}
	if (fabsf(*iq_ref) > I_PK) { *iq_ref = sign(*iq_ref) * I_PK; }		// limit torque
}

void spindle_motion_ctrl_vpi(float vm_ref, float omega_m, float *t_ref)
{
	float vm_sp_er[1];

	vm_sp_er[0] = vm_ref - omega_m;
	ctrl_math_output(Cspvpi, xspvpi, Dspvpi, vm_sp_er, t_ref, 1);
	ctrl_math_state(Aspvpi, xspvpi, Bspvpi, vm_sp_er, xspvpi, 1);
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
	ctrl_math_output(Cpid[0], xpid, Dpid[0], x_err, iq_ref, 2);
	ctrl_math_state(Apid[0], xpid, Bpid[0], x_err, xpid, 2);
	if (fabsf(*iq_ref) > I_PK) { *iq_ref = sign(*iq_ref) * I_PK; }		// limit torque
}

float estimated_disturbance(float t_ref, float omega_m) {
	//Torque Command   td																						Speed   vd 
	//			|																													|
	//			|																													|			
	//			------------->[Q(s)]----sd---> - O + <---ud---[R(S) = Q(s)*(Js)]--------
	//															   |
	//															   |
	//															   V
	//									OBSERVED DISTURBANCE TORQUE
	float sd[IOMAX] , ud[IOMAX];
	//Q Filter
	ctrl_math_output(C_Q, xq, D_Q, &t_ref, sd , 2);
	ctrl_math_state(A_Q, xq, B_Q, &t_ref, xq, 2);
	//R Filter
	ctrl_math_output(C_R, xr, D_R, &omega_m, ud, 2);
	ctrl_math_state(A_R, xr, B_R, &omega_m, xr, 2);
	//observed(estimated) disturbance torque
	return ( ud[0] - sd[0] );
}

float dob_lpf2(float u) {
	float y[IOMAX];
	ctrl_math_output(C_LPF2, xlpf2, D_LPF2, &u, y, 2);
	ctrl_math_state(A_LPF2, xlpf2, B_LPF2, &u, xlpf2, 2);
	return y[0];
}

float dob_hpf2(float u) {
	float y[IOMAX];
	ctrl_math_output(C_HPF2, xhpf2, D_HPF2, &u, y, 2);
	ctrl_math_state(A_HPF2, xhpf2, B_HPF2, &u, xhpf2, 2);
	return y[0];
}

float notch_stage_x(float t_ref) {
	float t_nref[IOMAX];
	ctrl_math_output(C_notch_stage_x, xstn, D_notch_stage_x, &t_ref, t_nref, 2);
	ctrl_math_state(A_notch_stage_x, xstn, B_notch_stage_x, &t_ref, xstn, 2);
	return t_nref[0];
}

float estimated_disturbance_stx(float t_ref, float omega_m) {
	//Torque Command   td																						Speed   vd 
	//			|																													|
	//			|																													|			
	//			------------->[Q(s)]----sd---> - O + <---ud---[R(S) = Q(s)*(Js+D)]-----
	//															   |
	//															   |
	//															   V
	//									OBSERVED DISTURBANCE TORQUE
	float sd[IOMAX], ud[IOMAX];
	//Q Filter
	ctrl_math_output(C_Q_STX, xqstx, D_Q_STX, &t_ref, sd, 2);
	ctrl_math_state(A_Q_STX, xqstx, B_Q_STX, &t_ref, xqstx, 2);
	//R Filter
	ctrl_math_output(C_R_STX, xrstx, D_R_STX, &omega_m, ud, 2);
	ctrl_math_state(A_R_STX, xrstx, B_R_STX, &omega_m, xrstx, 2);
	//observed(estimated) disturbance torque
	return (ud[0] - sd[0]);
}

void motion_ctrl_reset(void)
{
	int i, j;
	for (i = 0; i < NMAX; i++) {
		for(j=0;j<2;j++)	xvpi[i][j] = 0.0;
		xspvpi[i] = 0.0;
		xq[i] = 0.0;
		xr[i] = 0.0;
		xpid[i] = 0.0;
		xstn[i] = 0.0;
		xqstx[i] = 0.0;
		xrstx[i] = 0.0;
		xlpf2[i] = 0.0;
		xhpf2[i] = 0.0;
	}	
	dac_da_out(0, 3, 0);
	for (i = 0; i < Nd; i++) {
		phi_sp[i] = 0;
		theta_par_est[i] =0;
		for (j = 0; j < Nd; j++) {
			if (i==j) P_var[i*Nd +j] = SIGMA_P;
			else  P_var[i*Nd + j] = 0;
		}
	}
	ctime = 0;
	torque_command = 0;
	iq_refx = 0.0; iq_refy = 0.0;
	omega_sp_ma_rpm = 0;
}
