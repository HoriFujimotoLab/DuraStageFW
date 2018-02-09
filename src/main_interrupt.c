#include "main_interrupt.h"

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

	watch_data_8ch();
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
	//Spindle read
	motor_enc_spindle(&count_old_sp, &count_sp, &omega_old_sp, &omega_sp, &omega_sp_ma, &theta_sp, &r_count_sp);
	//NO UPMILLING
	if (omega_sp_ma < 0)
	{
		omega_sp_ma = omega_sp_ma_temp;
	}
	omega_sp_ma_temp = omega_sp_ma;

	//X
	v_mx = RAD2M * omega_max;
	//Y
	v_my = RAD2M * omega_may;
	//Spindle
	omega_sp_ma_rpm = RADPS2RPM * omega_sp_ma;
	omega_sp_ref_rpm_ma = ALPHAMA_FIRST * omega_sp_ref_rpm + (1 - ALPHAMA_FIRST) * omega_sp_ref_rpm_ma;

	// MOTION CTRL
	if (sysmode_e == SYS_STP)
	{
	}
	if (sysmode_e == SYS_INI)
	{
	}
	if (sysmode_e == SYS_RUN)
	{
		switch (cmode)
		{

		/*
		//spindle torque control
		case TORQUE_MODE:
			dac_da_out(0, 3, DA_GAIN_TORQUE * torque_command);
			break;

		//spindle speed control via torque
		case SPINDLE_OMEGA_MODE:
			spindle_motion_ctrl_vpi(omega_sp_ref_rpm_ma * RPM2RADPS, omega_sp_ma, &torque_command);
			dac_da_out(0, 3, DA_GAIN_TORQUE * torque_command);
			break;

		//spindle speed (no LPF) control via torque
		case DIRECT_SPINDLE_OMEGA_MODE:
			spindle_motion_ctrl_vpi(omega_sp_ref_rpm * RPM2RADPS, omega_sp_ma, &torque_command);
			dac_da_out(0, 3, DA_GAIN_TORQUE * torque_command);
			break;
		*/

		//stage velocity control
		case VEL_MODE:
			if (xymode == XMODE)
			{
				motion_ctrl_vpi(XAXIS, vm_refx * M2RAD, omega_max, &iq_refx);
			}
			else if (xymode == YMODE)
			{
				motion_ctrl_vpi(YAXIS, vm_refy * M2RAD, omega_may, &iq_refy);
			}
			//spindle control also
			spindle_motion_ctrl_vpi(omega_sp_ref_rpm_ma * RPM2RADPS, omega_sp_ma, &torque_command);
			dac_da_out(0, 3, DA_GAIN_TORQUE * torque_command);
			break;

		//stage position incremental mode
		case INC_MODE:
			if (xymode == XMODE)
			{
				if (fabsf(incx) > 0)
					theta_m_ref_linx = incx + x_linx;
				motion_ctrl_vpi(XAXIS, motion_ctrl_pos(theta_m_ref_linx, x_linx) * M2RAD, omega_max, &iq_refx);
				incx = 0;
			}
			else if (xymode == YMODE)
			{
				if (fabsf(incy) > 0)
					theta_m_ref_liny = incy + x_liny;
				motion_ctrl_vpi(YAXIS, motion_ctrl_pos(theta_m_ref_liny, x_liny) * M2RAD, omega_may, &iq_refy);
				incy = 0;
			}
			break;

		default: //stop
			iq_refx = 0;
			iq_refy = 0;
			dac_da_out(0, 3, 0);
			break;
		}
	}

	// MULTI-INT OFF
	int_disable();
	CSR = regs[0];
	IRP = regs[1];
}
