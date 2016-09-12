/************************************************************************************
PROCESSOR-IN-THE-LOOP
---------------------
Descr.:		
System:		
Author:		Thomas Beauduin, University of Tokyo, 2015
*************************************************************************************/
#include "system_fsm.h"

// TIMER COMP
#define ALPHA (0.2)
#define	PEV_BDN	0	
#define	INV_CH	0
float t0 = 0.0, t0_a = 0.0;

interrupt void system_tint0(void);

void main(void)
{
	int_disable();

	// TIMER INIT
	timer0_init(TS);
	timer0_init_vector(system_tint0);
	timer0_start();
	timer0_enable_int();
	watch_init();
	watch_data();

	// TEST CODE INIT
	system_fsm_init();


	// INVERTER INIT
	pev_inverter_init(PEV_BDN, INV_CH, FC, DT);
	pev_inverter_set_uvw(PEV_BDN, INV_CH, 0.0, 0.0, 0.0);
	wait(TC);
	pev_inverter_start_pwm(PEV_BDN, INV_CH);

	int_enable();

	//SYSTEM RUN
	while (1) { system_fsm_mode(); }
}


void system_tint0(void)
{
	// TEST CODE
	// ---------

	/*
	ctrl_current_uw2ab(iu0_ad, iw0_ad, &ia0_ad, &ib0_ad);
	ctrl_current_ab2dq(ia0_ad, ib0_ad, theta_e, &id0_ad, &iq0_ad);
	//ctrl_current_pzcpi(Ai_pzcpi, Bi_pzcpi, Ci_pzcpi, Di_pzcpi, iq0_ref, id0_ad, iq0_ad, &vd0_ref, &vq0_ref);
	ctrl_current_zcpi(Affi, Bffi, Cffi, Dffi, Afbi, Bfbi, Cfbi, Dfbi, iq0_ref, id0_ad, iq0_ad, &vd0_ref, &vq0_ref);
	ctrl_current_dec(vel_ma, id0_ad, iq0_ad, &vd0_ref, &vq0_ref);
	ctrl_current_dq2ab(vd0_ref, vq0_ref, theta_e, &va0_ref, &vb0_ref);
	ctrl_current_ab2uvw(va0_ref, vb0_ref, &vu0_ref, &vv0_ref, &vw0_ref);
	hardw_inv_pwm(0, vu0_ref, vv0_ref, vw0_ref, vdc0_ad);
	*/

	// put the code you want to measure performance in here
	// not important what is being calculated, output is always 0



	// TIMER COUNT
	// -----------
	t0 = timer0_read();
	t0_a = ALPHA * (t0 * 17.7777e-3) + (1 - ALPHA) * t0_a;
	if (msr >= 0 && msr < NROFT) { msr++; }
	watch_data_8ch();
}


