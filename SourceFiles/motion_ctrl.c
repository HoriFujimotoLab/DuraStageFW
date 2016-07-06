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
static float xvpi[1] = { 0.0 };
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


void motion_ctrl_vpi(float vm_ref, float omega_m, float *iq_ref)
{
	float vm_er[1] = { 0.0 };
	vm_er[0] = vm_ref - omega_m;
	ctrl_math_output(Cvpi[0], xvpi, Dvpi[0], vm_er, iq_ref, 1, 1, 1);
	ctrl_math_state(Avpi[0], xvpi, Bvpi[0], vm_er, xvpi, 1, 1);
	if (fabsf(*iq_ref) > I_PK) { *iq_ref = sign(*iq_ref) * I_PK; }		// limit torque
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
	xvpi[0] = 0.0;
	xpid[0] = 0.0; xpid[1] = 0.0;

}
