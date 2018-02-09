/************************************************************************************
MULTI INTERRUPT FIRMWARE FOR DURA STAGE
VER MULTI CHATTER AVOIDANCE
-------------------------
Descr.:		main file for multiple interrupt experiments
Boards:		PE-Expert3, C6713A DSP + System Hardware
System:		Main-file of modular firmware structure
Author:		Shimoda Takaki, the University of Tokyo, 2017
*************************************************************************************/

#include "main_interrupt.h"

void main(void)
{
	// SYSTEM INIT
	int_disable();
	system_init();
	int_enable();

	//SYSTEM RUN
	while (1)
	{
		system_fsm_mode();
	}
}
