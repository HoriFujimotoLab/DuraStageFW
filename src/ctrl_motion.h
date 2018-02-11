/************************************************************************************
MOTION CONTROL MODULE
----------------------
Descr.:		Control module for stage motion control
Boards:		PE-Expert3, C6713A DSP (float calculation)
System:		Dura ball-screw experimental setup
Author:		Shimoda Takaki, University of Tokyo, 2018
*************************************************************************************/
#pragma once

#include "system_data.h"

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
void motion_ctrl_vpi(int axis, float vm_ref, float omega_m, float *iq_ref);

/*SPINDLE 	PI ANGULAR VELOCITY CTRL
**	------------------------
**	DES:	pi control of spindle motor velocity (loop shaped)
**	INP:	vm_ref	: angular velocity reference
**			omega_m	: angular velocity measurement
**	OUT:t_ref	: calculated torque reference
*/
void spindle_motion_ctrl_vpi(float vm_ref, float omega_m, float *t_ref);

/*	P POSITION CTRL
**	------------------------
**	DES:	p control of motor posistion
**	INP:	x_ref	: position reference
**			x_m	: position measurement
**	OUT:	velocity reference	
*/
float motion_ctrl_pos(float x_ref, float x_m);

/*	P-PI POSITION CTRL PACKAGE
**	------------------------
**	DES:	p-pi control of motor angle
**	INP:	axis: X or Y axis
				omega_m: angular velocity mesurement
				theta_ref: angular position reference
				theta_m: angular position mesurement
**	OUT:	iq_ref	: calculated current reference
*/
void motion_ctrl_pack_pos(int axis, float omega_m, float theta_ref, float theta_m, float *iq_ref);

/*	PID ANGULAR POSITION CTRL
**	-------------------------
**	DES:	pid control of motor angle (pole placement)
**	INP:	xm_ref	: reference angular position
**			theta_m	: measured angular position
**	OUT:	iq_ref	: calculated current reference
*/
void motion_ctrl_pid(float x_ref, float x_msr, float *iq_ref);

/* Prefilter Structure Feedforward
**
** DES:   Prefilter for input shaping
** INP:   x_ref: step value
** OUT:   x_ref_ff: shaped input
*/
float motion_ctrl_prefilter(float x_ref);

/*	RESET CONTROL MODULE
**	--------------------
**	DES:	resets module internal counters and variables
**			necessary at measurement init for good startup
*/
void motion_ctrl_reset(void);
