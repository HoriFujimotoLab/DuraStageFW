/************************************************************************************
MULTI INTERRUPT FIRMWARE FOR DURA STAGE
VER MULTI CHATTER AVOIDANCE
-------------------------
Descr.:		main file for multiple interrupt experiments
Boards:		PE-Expert3, C6713A DSP + System Hardware
System:		Main-file of modular firmware structure
Author:		Shimoda Takaki, the University of Tokyo, 2018
*************************************************************************************/

#pragma once

#include "system_fsm.h"

/*
Initilize system interrupt functions
*/
void system_init(void);

/*
PWM edge interrupt for current control
*/
interrupt void system_cint5(void);

/*
Timer 0 interrupt for motion control
*/
interrupt void system_tint0(void);

/*
Timer 1 interrupt for motion control
*/
interrupt void system_tint1(void);
