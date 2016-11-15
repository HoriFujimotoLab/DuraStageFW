/************************************************************************************
MULTI INTERRUPT FIRMWARE
-------------------------
Descr.:		main file for multiple interrupt experiments
Boards:		PE-Expert3, C6713A DSP + System Hardware
System:		Main-file of modular firmware structure
Author:		Thomas Beauduin, University of Tokyo, 2016
*************************************************************************************/

#include "system_fsm.h"

void system_init(void);
interrupt void system_tint0(void);
interrupt void system_cint5(void);

void main(void)
{
	// SYSTEM INIT
	int_disable();
	system_init();
	int_enable();

	//SYSTEM RUN
	while (1) { system_fsm_mode(); }
}

void system_tint0(void) {
	//x-axis current
	motor_enc_elec(XAXIS, &theta_ex); //x-axis
	setup_adc_read(XAXIS, &vdc_adx, &idc_adx, &iu_adx, &iw_adx); 
																 //x-axis motor/stage
	motor_enc_read(XAXIS, &theta_mx, &omega_mx, &omega_max);
	stage_lin_nano_read(XAXIS, &x_nano_linx);

	if (sysmode_e == SYS_RUN) {
		if (msr >= 0 && msr < NROFT) {
			motion_ctrl_ref(5, Aref, Fref, &iq_refx); //system identification
			msr++;
		}
		else {
			iq_refx = 0.0;
		}
	}

	if (sysmode_e == SYS_INI || sysmode_e == SYS_RUN) {
		//X-AXIS
		current_ctrl_uw2ab(iu_adx, iw_adx, &ia_adx, &ib_adx);
		current_ctrl_ab2dq(ia_adx, ib_adx, theta_ex, &id_adx, &iq_adx);
		current_ctrl_zcpi(XAXIS, iq_refx, id_adx, iq_adx, &vd_refx, &vq_refx); //dq current control
		current_ctrl_dec(omega_max, id_adx, iq_adx, &vd_refx, &vq_refx);
		current_ctrl_dq2ab(vd_refx, vq_refx, theta_ex, &va_refx, &vb_refx);
		current_ctrl_ab2uvw(va_refx, vb_refx, &vu_refx, &vv_refx, &vw_refx);
		motor_inv_pwm(XAXIS, vu_refx, vv_refx, vw_refx, vdc_adx);
	}
	else {
		motor_inv_pwm(XAXIS, 0, 0, 0, vdc_adx);
		motor_inv_pwm(YAXIS, 0, 0, 0, vdc_ady);
	}

	watch_data_8ch();
}


void system_cint5(void)	{}

void system_init(void)
{
	// SENSORS
	watch_init();
	motor_adc_init();
	setup_adc_init();
	motor_enc_init(XAXIS); //X-axis
	motor_enc_init(YAXIS); //Y-axis
	stage_lin_init();
	setup_dac_init();
	setup_spindle_enc_init();

	// DRIVE CTRL
	int5_init_vector(system_cint5);
	motor_inv_init();
	int5_enable_int();

	// CURRENT CONTROL
	// MOTION CTRL
	timer0_init(TQ);
	timer0_init_vector(system_tint0);
	timer0_start();
	timer0_enable_int();

	system_fsm_init();
}
