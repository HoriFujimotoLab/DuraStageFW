/************************************************************************************
MOTION CONTROL MODULE
----------------------
Descr.:		Control module for stage motion control
Boards:		PE-Expert3, C6713A DSP (float calculation)
System:		Yuna ball-screw experimental setup
Author:		Thomas Beauduin, University of Tokyo, 2015
*************************************************************************************/
#ifndef	MOTION_CTRL_H
#define	MOTION_CTRL_H

#include	"system_data.h"

/*	---------------------
**	DES:	generate q current reference trajectory
**	INP:	reftype_e	: reference type (enumerate)
**			Aref	: amplitude of reference signal
**	OUT:	iq_ref	: calculated motion reference
*/
void direct_qcurrent_ctrl(int reftype_e, float Aref, float Fref, float *iq_ref);

/*	MOTION REF GENERATION
**	---------------------
**	DES:	generate reference trajectory
**	INP:	iqtype_e	: reference type (enumerate)
**			Aref	: amplitude of reference signal
**	OUT:	x_ref	: calculated motion reference
*/
void motion_ctrl_ref(int reftype_e, float Aref, float Fref, float *x_ref);


/*	PI ANGULAR VELOCITY CTRL
**	------------------------
**	DES:	pi control of motor velocity (loop shaped)
**	INP:	vm_ref	: angular velocity reference
**			omega_m	: angular velocity measurement
**	OUT:	iq_ref	: calculated current reference
*/
void motion_ctrl_vpi(float vm_ref, float omega_m, float *iq_ref);


/*	PID ANGULAR POSITION CTRL
**	-------------------------
**	DES:	pid control of motor angle (pole placement)
**	INP:	xm_ref	: reference angular position
**			theta_m	: measured angular position
**	OUT:	iq_ref	: calculated current reference
*/
void motion_ctrl_pid(float x_ref, float x_msr, float *iq_ref);


/*	RESET CONTROL MODULE
**	--------------------
**	DES:	resets module internal counters and variables
**			necessary at measurement init for good startup
*/
void motion_ctrl_reset(void);


#endif
