/************************************************************************************
MULTI INTERRUPT FIRMWARE
-------------------------
Descr.:		main file for multiple interrupt experiments
Boards:		PE-Expert3, C6713A DSP + System Hardware
System:		Main-file of modular firmware structure
Author:		Shimoda Takaki, University of Tokyo, 2018
*************************************************************************************/

#include "system_fsm.h"
#include "input.h"

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
	while (1)
	{
		system_fsm_mode();
	}
}

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

	timer0_init(TS); // MOTION CTRL
	timer0_init_vector(system_tint0);
	timer0_start();
	timer0_enable_int();

	system_fsm_init();
}

void system_cint5(void)
{
	motor_enc_elec(XAXIS, &theta_ex);							 //x-axis
	setup_adc_read(XAXIS, &vdc_adx, &idc_adx, &iu_adx, &iw_adx); //y-axis
	motor_enc_elec(YAXIS, &theta_ey);							 //y-axis
	setup_adc_read(YAXIS, &vdc_ady, &idc_ady, &iu_ady, &iw_ady); //y-axis

	if (sysmode_e == SYS_INI || sysmode_e == SYS_RUN)
	{
		//X-AXIS
		current_ctrl_uw2ab(iu_adx, iw_adx, &ia_adx, &ib_adx);
		current_ctrl_ab2dq(ia_adx, ib_adx, theta_ex, &id_adx, &iq_adx);
		current_ctrl_zcpi(XAXIS, iq_refx, id_adx, iq_adx, &vd_refx, &vq_refx); //dq current control
		current_ctrl_dec(omega_max, id_adx, iq_adx, &vd_refx, &vq_refx);
		current_ctrl_dq2ab(vd_refx, vq_refx, theta_ex, &va_refx, &vb_refx);
		current_ctrl_ab2uvw(va_refx, vb_refx, &vu_refx, &vv_refx, &vw_refx);
		motor_inv_pwm(XAXIS, vu_refx, vv_refx, vw_refx, vdc_adx);
		//Y-AXIS
		current_ctrl_uw2ab(iu_ady, iw_ady, &ia_ady, &ib_ady);
		current_ctrl_ab2dq(ia_ady, ib_ady, theta_ey, &id_ady, &iq_ady);
		current_ctrl_zcpi(YAXIS, iq_refy, id_ady, iq_ady, &vd_refy, &vq_refy); //dq current control
		current_ctrl_dec(omega_may, id_ady, iq_ady, &vd_refy, &vq_refy);
		current_ctrl_dq2ab(vd_refy, vq_refy, theta_ey, &va_refy, &vb_refy);
		current_ctrl_ab2uvw(va_refy, vb_refy, &vu_refy, &vv_refy, &vw_refy);
		motor_inv_pwm(YAXIS, vu_refy, vv_refy, vw_refy, vdc_ady);
	}
	else
	{
		motor_inv_pwm(XAXIS, 0, 0, 0, vdc_adx);
		motor_inv_pwm(YAXIS, 0, 0, 0, vdc_ady);
	}
}

void system_tint0(void)
{
	unsigned int regs[2];

	// MULTI-INT ON
	regs[0] = CSR;
	regs[1] = IRP;
	int_enable();

	//X read
	motor_enc_read(XAXIS, &theta_mx, &omega_mx, &omega_max);
	stage_lin_read(XAXIS, &x_linx, &v_linx, &v_linx_ma);
	//Y read
	motor_enc_read(YAXIS, &theta_my, &omega_my, &omega_may);
	stage_lin_read(YAXIS, &x_liny, &v_liny, &v_liny_ma);

	// MOTION CTRL
	if (sysmode_e == SYS_STP)
	{
	}
	if (sysmode_e == SYS_INI)
	{
	}
	if (sysmode_e == SYS_RUN)
	{
		if (msr >= 0 && msr < NOI * NROFS)
		{
			iq_refx = I_PK * refvec[(msr++) % NROFS];
		}
		else
		{
			iq_refx = 0.0;
		}
	}

	// MULTI-INT OFF
	int_disable();
	CSR = regs[0];
	IRP = regs[1];

	watch_data_8ch();
}
