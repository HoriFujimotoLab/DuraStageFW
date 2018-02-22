/************************************************************************************
ILC MODULE
-------------------
Descr.:		c-code implementation of iterative learning control
Boards:		PE-Expert3, C6713A DSP (floating point calculation)
System:		Dura single axis ball-screw experimental setup
Author:		Shimoda Takaki, University of Tokyo, 2018
*************************************************************************************/

#pragma once

/*
record current error
int i: current index
float e: current error
return: 
*/
void record_error_ilc(int i, float e);

/*
record additive force
int i: current index
float e: current force
return: 
*/
void record_force_ilc(int i, float f);

/*
calculate ilc 
return: 
*/
void set_ilc();

/*
return ilc output
int i: current index
return: 
*/
float ilc(int i);

/*
reset ilc
return:
*/
void reset_ilc();
