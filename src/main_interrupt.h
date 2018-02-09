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
