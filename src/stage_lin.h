/************************************************************************************
STAGE LINEAR-SCALE MODULE
-------------------------
Descr.:		Hardware module for linear-scale of table
Boards:		MWPE-Expert3, MWPE-FPGAA (custom board)
Sensor:		1nm incremental linear scale (magnescale)
Author:		Thomas Beauduin, University of Tokyo, April 2015
*************************************************************************************/
#pragma once

#include "system_data.h"

/*	INIT ENCODER & BOARD
**	--------------------
**	DES:	initiate fpga board and encoder by register set/clear
**			necessary at program init before encoder reading
*/
void stage_lin_init(void);

/*	READ LINEAR-SCALE DATA
**	-----------------------
**	DES:	returns the processed incremental data from linear scale
**  IN :    axis :  X axis:0, Y axis:1
**	OUT:	pos_t:	linear table position	[mm]
**			vel_t:	linear table velocity	[rad/s]
**			vel_ta:	averaged velocity (lpf)	[rad/s]
*/
void stage_lin_read(int axis, float *pos_t, float *vel_t, float *vel_ta);

void stage_lin_nano_read(int axis, int *pos_nano);

/*	READ LINEAR-SCALE STATUS
**	------------------------
**	DES:	returns the status of the sensor data acquisition
**	OUT:	status:	sensor error status [-] {ERR = 1}
*/
void stage_lin_status(int *status);

/*	READ LINEAR-SCALE STATUS
**	------------------------
**	DES:	returns the data count registers to 0
**			necessary at stage home referencing 
*/
void stage_lin_reset(void);

/*	NOTES:
**	home:	sticker close to motor (analog msr)
**			home position resolution: 0.4 um
**			max. home sensing speed: 150mm/s
*/
