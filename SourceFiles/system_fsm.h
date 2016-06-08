/************************************************************************************
SYSTEM FINIT-STATE-MACHINE MODULE
---------------------------------
Descr.:		finit-state-machine (fsm) for system protection
Boards:		MWPE-Expert3, MWPE-DIO (MWPE-PEV dio connect)
Sensor:		Digital input from inverter & emergency switches
Author:		Thomas Beauduin, University of Tokyo, 2015
*************************************************************************************/

#ifndef	SYSTEM_FSM_H
#define	SYSTEM_FSM_H

/*	HARDWARE MODULES INCLUSION
**	--------------------------
**	DES:	include the required hardware modules headers
**			necessary for the particular setup configuration
*/
#include	"motor_pem.h"
#include	"motor_enc.h"
#include	"stage_adc.h"
#include	"stage_lin.h"
#include	"system_data.h"
#include	<mwio3.h>


/*	CONTROL MODULES INCLUSION
**	-------------------------
**	DES:	include the required control modules headers
**			necessary for the particular experiment configuration
*/
#include	"current_ctrl.h"
#include	"motion_ctrl.h"
#include	<math.h>


/*	INIT SYSTEM PROTECTION
**	----------------------
**	DES:	initiate dio board & protection flags
**			necessary at program start before interrupt init
*/
void system_fsm_init(void);


/*	PROCES SYSTEM MODE
**	------------------
**	DES:	finit-state-machine mode processing 
**			defines the system state and sets mode flag
*/
void system_fsm_mode(void);

#endif
