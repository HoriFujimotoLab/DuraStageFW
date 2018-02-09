/************************************************************************************
MOTION CONTROL MODULE
----------------------
Descr.:		Control module for stage motion control
Boards:		PE-Expert3, C6713A DSP (float calculation)
System:		Yuna ball-screw experimental setup
Author:		Thomas Beauduin, University of Tokyo, 2015
*************************************************************************************/
#include "ctrl_motion.h"
#include "ctrl_math.h"
#include "ctrl_motion_par.h"

//FOR IDENTIFICATION
//#include "data/FILE.h"

int ref = 0;
double t = 0.0;

//INTERNAL STATE
static float xvpi[XYMODE][NMAX] = {0.0}; //{{XAXIS} , {YAXIS}} stage state
static float xpid[NMAX] = {0.0};
static float xspvpi[NMAX] = {0.0}; //spindle variable state

void direct_qcurrent_ctrl(int reftype_e, float Aref, float Fref, float *iq_ref)
{
	switch (reftype_e)
	{
	case 0:
		*iq_ref = 0.0;
		break;
	case 1:
		*iq_ref = Aref;
		break;
	case 2:
		*iq_ref = Aref * sin(Fref * PI(2) * t);
		t += (TS * 1.0e-6);
		break;
	case 5:
		break;
	}
}

void motion_ctrl_vpi(int axis, float vm_ref, float omega_m, float *iq_ref)
{
	float vm_er[1];

	if (fabsf(vm_ref) > VELXLIM)
	{
		vm_ref = sign(vm_ref) * VELXLIM;
	} // limit speed
	//Velocity PI control
	vm_er[0] = vm_ref - omega_m;
	ctrl_math_output(Cvpi[axis], xvpi[axis], Dvpi[axis], vm_er, iq_ref, 1);
	ctrl_math_state(Avpi[axis], xvpi[axis], Bvpi[axis], vm_er, xvpi[axis], 1);
	if (fabsf(*iq_ref) > I_PK)
	{
		*iq_ref = sign(*iq_ref) * I_PK;
	} // limit torque
}

void spindle_motion_ctrl_vpi(float vm_ref, float omega_m, float *t_ref)
{
	float vm_sp_er[1];

	vm_sp_er[0] = vm_ref - omega_m;
	ctrl_math_output(Cspvpi, xspvpi, Dspvpi, vm_sp_er, t_ref, 1);
	ctrl_math_state(Aspvpi, xspvpi, Bspvpi, vm_sp_er, xspvpi, 1);
	if (fabsf(*t_ref) > T_SP)
	{
		*t_ref = sign(*t_ref) * T_SP;
	} // limit torque
	  //*t_ref *= -1; //ISSUE01: TORQUE GAIN IS INVERSED.
}

float motion_ctrl_pos(float x_ref, float x_m)
{
	return Kxp * (x_ref - x_m);
}

void motion_ctrl_pack_pos(int axis, float omega_m, float theta_ref, float theta_m, float *iq_ref)
{ //angular
	float vm_ref;
	vm_ref = motion_ctrl_pos(theta_ref, theta_m); //angular
	if (fabsf(vm_ref) > VELXLIM)
	{
		vm_ref = sign(vm_ref) * VELXLIM;
	} // limit speed
	motion_ctrl_vpi(axis, vm_ref, omega_m, &(*iq_ref));
}

void motion_ctrl_pid(float x_ref, float x_msr, float *iq_ref)
{
	float x_err[1] = {0.0};
	x_err[0] = x_ref - x_msr;
	ctrl_math_output(Cpid[0], xpid, Dpid[0], x_err, iq_ref, 2);
	ctrl_math_state(Apid[0], xpid, Bpid[0], x_err, xpid, 2);
	if (fabsf(*iq_ref) > I_PK)
	{
		*iq_ref = sign(*iq_ref) * I_PK;
	} // limit torque
}

void motion_ctrl_reset(void)
{
	int i, j;
	for (i = 0; i < NMAX; i++)
	{
		for (j = 0; j < 2; j++)
			xvpi[i][j] = 0.0;
		xspvpi[i] = 0.0;
		xpid[i] = 0.0;
	}
	dac_da_out(0, 3, 0);
	torque_command = 0;
	iq_refx = 0.0;
	iq_refy = 0.0;
	omega_sp_ma_rpm = 0;
}
