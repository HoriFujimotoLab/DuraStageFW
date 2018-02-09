/************************************************************************************
MOTOR ROTARY-ENCODER MODULE
---------------------------
Descr.:		Hardware module for motor encoder (00672)
Boards:		MWPE-Expert3, MWPE-PIOS (PIO expansion board)
Sensor:		20bit absolute rotary encoder (Tamagawa)
Author:		Thomas Beauduin, University of Tokyo, March 2016
*************************************************************************************/

#pragma once

#include "system_data.h"

extern float theta_mx;

/*	INIT ENCODER & BOARD
**	--------------------
**	DES:	initiate pios board and encoder by register set/clear
**			necessary at program init before encoder reading
*/
void motor_enc_init(int axis);


/*	READ MOTOR ENCODER DATA
**	-----------------------
**	DES:	returns the read and processed motor enc data from pios board
**  IN :    axis   :    selection of axis						[-]		{X,Y:0,1}
**	OUT:	theta_e:	theta electric for park transformation	[rad]	{0,2pi}
**			theta_m:	theta mechanic for position control		[rad]	{-LR/2, LR/2}
**			omega_m:	omega mechanic for decoupling control	[rad/s] {-2pi*fs, 2pi*fs}
**			omega_ma:	omega averaged (time) for vel control	[rad/s]	{-2pi*fs, 2pi*fs}
**	DSP:	calctime:	average calculation time				8 [ms]  (vers. 29/11/15)
*/
void motor_enc_elec(int axis, float *theta_e);
void motor_enc_read(int axis, float *theta_m, float *omega_m, float *omega_ma);


/*	RESET ENCODER MODULE
**	--------------------
**	DES:	resets encoder module internal counters and variables
**			necessary at measurement init to calibrate setup encoders
**  IN :    axis:  selection of axis {X,Y:0,1}
*/
void motor_enc_reset(int axis);


/*	CHECK ENCODER STATUS
**	--------------------
**	DES:	check run/error status of motor encoder
**  IN :    axis:  selection of axis			[-]	{X,Y:0,1}
**	OUT:	encoder status for error handling	[-]	{RUN=1, ERROR=0}
*/
void motor_enc_status(int axis, unsigned int *status);

/*	READ MOTOR ENCODER DATA(SPINDLE)
**	-----------------------
**	DES:	returns the read and processed motor enc data from pios board
**  IN :    axis   :    selection of axis						[-]		{X,Y:0,1}
**	OUT:	theta_e:	theta electric for park transformation	[rad]	{0,2pi}
**			theta_m:	theta mechanic for position control		[rad]	{-LR/2, LR/2}
**			omega_m:	omega mechanic for decoupling control	[rad/s] {-2pi*fs, 2pi*fs}
**			omega_ma:	omega averaged (time) for vel control	[rad/s]	{-2pi*fs, 2pi*fs}
**	DSP:	calctime:	average calculation time				8 [ms]  (vers. 29/11/15)
*/
void motor_enc_spindle(int *count_old, int *count, float *omega_old, float *omega, float *omega_ma, float *theta_m, int *r_count);

