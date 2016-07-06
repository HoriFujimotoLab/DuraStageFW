/************************************************************************************
MULTI INTERRUPT FIRMWARE
-------------------------
Descr.:		main file for multiple interrupt experiments
Boards:		PE-Expert3, C6713A DSP + System Hardware
System:		Main-file of modular firmware structure
Author:		Thomas Beauduin, University of Tokyo, 2016
*************************************************************************************/

/*
NOTE:
Now y-axis motor is not being used.
*/

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


void system_tint0(void)
{
	unsigned int regs[2];

	// SENSOR READ
	motor_enc_read(0, &theta_mx, &omega_mx, &omega_max);
	//motor_enc_read(1, &theta_my, &omega_my, &omega_may); //y-axis
	setup_adc_read(2, &torque_ad, &temp1, &temp2, &temp3);

	// MULTI-INT ON
	regs[0] = CSR;
	regs[1] = IRP;
	int_enable();

	// MOTION CTRL
	if (sysmode_e == SYS_STP) {}
	if (sysmode_e == SYS_INI) {}
	if (sysmode_e == SYS_RUN)
	{
		if (msr >= 0 && msr < NROFT) {
			motion_ctrl_ref(reftype_e, Aref, Fref, &xref); 
			msr++;
			//motion_ctrl_pid(xref, theta_mx, &iq_refx); //too dangerous now to activate 
		}
		else { motion_ctrl_ref(REF_OFF, Aref, Fref, &xref); }
		//iq_refx = xref;	// open-loop ???
	}

	// MULTI-INT OFF
	int_disable();
	CSR = regs[0];
	IRP = regs[1];
	watch_data_8ch();
}

void system_cint5(void)
{
	// SENSOR READ
	motor_enc_elec(0, &theta_ex);
	//motor_enc_elec(1, &theta_ey); //y-axis
	setup_adc_read(0, &vdc_adx, &idc_adx, &iu_adx, &iw_adx);
	//setup_adc_read(1, &vdc_ady, &idc_ady, &iu_ady, &iw_ady); //y-axis

	//mode iq=Aref*sin(Fref:2*pi*t)
	if (test7 != 0) {
		direct_qcurrent_ctrl(reftype_e, Aref, Fref, &iq_refx);
	}

	// CURRENT CONTROL - X-AXIS
	if (sysmode_e == SYS_INI || sysmode_e == SYS_RUN)
	{
		current_ctrl_uw2ab(iu_adx, iw_adx, &ia_adx, &ib_adx);
		current_ctrl_ab2dq(ia_adx, ib_adx, theta_ex, &id_adx, &iq_adx);
		current_ctrl_zcpi(iq_refx, id_adx, iq_adx, &vd_refx, &vq_refx); //dq current control
		current_ctrl_dec(omega_max, id_adx, iq_adx, &vd_refx, &vq_refx);
		current_ctrl_dq2ab(vd_refx, vq_refx, theta_ex, &va_ref, &vb_ref);
		current_ctrl_ab2uvw(va_ref, vb_ref, &vu_ref, &vv_ref, &vw_ref);
		motor_inv_pwm(0, vu_ref, vv_ref, vw_ref, vdc_adx);
	}
	else { motor_inv_pwm(0, 0, 0, 0, vdc_adx); }
}


void system_init(void)
{
	// SENSORS
	watch_init();
	motor_adc_init();
	setup_adc_init();
	motor_enc_init(0);
	//motor_enc_init(1); //if not using y axis

	// DRIVE CTRL
	int5_init_vector(system_cint5);
	motor_inv_init();
	int5_enable_int();

	// MOTION CTRL
	timer0_init(TS);
	timer0_init_vector(system_tint0);
	timer0_start();
	timer0_enable_int();
	system_fsm_init();
}
