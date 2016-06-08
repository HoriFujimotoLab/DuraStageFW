/************************************************************************************
MOTOR POWER-ELEC MODULE
-----------------------
Descr.:		hardware module for motor drive
Boards:		MWPE-Expert3, MWPE-PEV (PEV expansion board)
PWelec:		MWINV-9R122A	(9.1kVA power module)
Author:		Thomas Beauduin, University of Tokyo, March 2016
*************************************************************************************/

#ifndef	MOTOR_PEM_H
#define	MOTOR_PEM_H

#include "system_data.h"

/*	INIT PEV BOARD ADC
**	------------------
**	DES:	initiate pev board and adc settings
**			sensor ranges set and offset calculated
*/
void motor_adc_init(void);


/*	INIT INVERTER
**	-------------
**	DES:	initiate inverter settings and gate signal
**			modulation type set & pwm function started
*/
void motor_inv_init(void);


/*	READ MOTOR ADC DATA
**	-------------------
**	DES:	returns the read and processed motor adc data from pev board
**	INP:	grp_ad: ad convertion group number [-] {0 or 1}
**	OUT:	ad0:	grp0: - [-] {0,-}, grp1: - [-] {0,1}
**			ad1:	grp0: - [-] {0,-}, grp1: - [-] {0,1}
**			ad2:	grp0: - [-] {0,-}, grp1: - [-] {0,1}
**			ad3:	grp0: - [-] {0,-}, grp1: - [-] {0,1}
*/
void motor_adc_read(int grp_ad, float *ad0, float *ad1, float *ad2, float *ad3);


/*	SET INVERTER OUTPUT
**	-------------------
**	DES:	modulates reference signal and creates gate signal
**			modulation type: S-PWM for speed & SVM for power
**	INP:	vx_ref:		phase reference voltage	[V]	{-200, 200}
**			vdc_ad:		measured dc-bus voltage	[V] {   0, 400}
*/
void motor_inv_pwm(int axis, float vu_ref, float vv_ref, float vw_ref, float vdc_ad);


#endif
