/************************************************************************************
SETUP SENSOR ADC MODULE
-----------------------
Descr.:		Hardware module adc of stage sensors
Boards:		MWPE3-C6713A, MWPE3-ADC (14 bit)
Sensor:		inverter I/V sensors (myway), torque sensor (unipulse)
Author:		Thomas Beauduin, University of Tokyo, March 2016
*************************************************************************************/

#pragma once

#include "system_data.h"

/*	INIT CONVERSION & BOARD
**	-----------------------
**	DES:	initiate adc board & calculate offsets
**			necessary at program init before reading
*/
void setup_adc_init(void);

/*	READ SENSOR ADC DATA
**	---------------------
**	DES:	returns the processed adc sensor data
**	OUT:	ad0: grp0: vdc [V] {0,400}, grp1: vdc [V] {0,400}, grp2: tor [Nm] {0,20}
**			ad1: grp0: idc [A] {0,50},  grp1: idc [A] {0,50},  grp2: tp1 [%]  {0,1}
**			ad2: grp0: iux [A] {0,50},  grp1: iux [A] {0,50},  grp2: tp2 [%]  {0,1}
**			ad3: grp0: iwx [A] {0,50},  grp1: iuy [A] {0,50},  grp2: tp3 [%]  {0,1}
*/
void setup_adc_read(int grp_ad, float *ad0, float *ad1, float *ad2, float *ad3);

/*
INIT CONVERSION & BOARD
**	-----------------------
**	DES:	initiate dac board 
**			necessary at program init before reading
*/
void setup_dac_init(void);

/*
INIT CONVERSION & BOARD
**	-----------------------
**	DES:	initiate PEV board (ENCORDER)
**			necessary at program init before reading
*/
void setup_spindle_enc_init(void);
