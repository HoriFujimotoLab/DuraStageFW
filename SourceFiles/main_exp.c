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
interrupt void system_tint1(void);
interrupt void system_cint5(void);

float time7 = 0;

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
	motor_enc_elec(XAXIS, &theta_ex); //x-axis
	setup_adc_read(XAXIS, &vdc_adx, &idc_adx, &iu_adx, &iw_adx); //y-axis
	motor_enc_elec(YAXIS, &theta_ey); //y-axis
	setup_adc_read(YAXIS, &vdc_ady, &idc_ady, &iu_ady, &iw_ady); //y-axis

	if (sysmode_e == SYS_INI || sysmode_e == SYS_RUN) {
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
	else {
		motor_inv_pwm(XAXIS, 0, 0, 0, vdc_adx);
		motor_inv_pwm(YAXIS, 0, 0, 0, vdc_ady);
	}

		//X read
	motor_enc_read(XAXIS, &theta_mx, &omega_mx, &omega_max);
	stage_lin_read(XAXIS, &x_linx, &v_linx, &v_linx_ma);
		//Y read
	motor_enc_read(YAXIS, &theta_my, &omega_my, &omega_may);
	
	//Acceleraton
	setup_adc_read(2, &torque_ad, &aspx, &aspy, &aspz);

	//disturbace observer
	observed_disturbance = estimated_disturbance(0, v_linx);

	//adaptive
	if (kmode > 0) {
		aspx_hf = dob_hpf2(observed_disturbance);
		//aspx_hf = dob_lpf2(dob_hpf2(observed_disturbance));
		////debug
		//time7 += TQ * 1.0e-6;
		//aspx_hf = sinsp(PI(2) * 1500 * time7);
		//aspx = omega_sp_ref_rpm_ma - omega_sp*RADPS2RPM;
		kalman_filter(phi_sp, aspx_hf, theta_par_est, P_var);
		//record phi=[-aspx[k-1], -aspx[k-2]]
		phi_sp[1] = phi_sp[0]; //aspx[k-2]
		phi_sp[0] = -aspx_hf; //aspx[k-1]
	}

	if (msr >= 0) msr++;

	watch_data_8ch();
}

void system_tint1(void)
{
	unsigned int regs[2];

	// MULTI-INT ON
	regs[0] = CSR;
	regs[1] = IRP;
	int_enable();

	//SAFETY
	if ((is_drive > 0) && (omega_sp_ma_rpm < 2000)) vm_refx = -1.0e-4; //go back in case of sudden spindle stop
	//X-AXIS
	v_mx = RAD2M* omega_max;
	//Y-AXIS
	v_my = RAD2M* omega_may;
	stage_lin_read(YAXIS, &x_liny, &v_liny, &v_liny_ma);
	//SPINDLE
	motor_enc_spindle(&count_old_sp, &count_sp, &omega_old_sp, &omega_sp, &omega_sp_ma, &theta_sp, &r_count_sp);
	omega_sp_ma_rpm = RADPS2RPM * omega_sp_ma;
	omega_sp_ref_rpm_ma = ALPHAMA_FIRST*omega_sp_ref_rpm + (1 - ALPHAMA_FIRST)*omega_sp_ref_rpm_ma;
	//ADAPTIVE
	fchat_a = dominant_freq(theta_par_est[0], theta_par_est[1]);
	//IS_CONTACT
	epsilon_sp = aspx*aspx - contact_threshold;
	if (epsilon_sp > 0)	kmode = 1;

	// MOTION CTRL
	if (sysmode_e == SYS_STP) {}
	if (sysmode_e == SYS_INI) {}
	if (sysmode_e == SYS_RUN)
	{
		switch (cmode) {
			//case MAIN_MODE_V: //const v
			//	motion_ctrl_vpi(XAXIS, vm_refx*M2RAD, omega_max, &iq_refx);
			//	dac_da_out(0, 3, DA_GAIN_RPM *  omega_sp_ref_rpm_ma);
			//	flag_threshold = epsilon_sp - threshold;
			//	if (flag_threshold > 0) cmode = ADAPTIVE_MODE_V;
			//	break;

			//case ADAPTIVE_MODE_V:
			//	calc_new_speed(&rho_sp, &omega_sp_new_rpm, fchat_a, omega_sp_ma_rpm, Qn);
			//	if ( (omega_sp_ref_rpm  < 1000) && (kmode > 0)) 		cmode = MAIN_MODE_V;
			//	motion_ctrl_vpi(XAXIS, vm_refx*M2RAD, omega_max, &iq_refx);
			//	dac_da_out(0, 3, DA_GAIN_RPM *  omega_sp_new_rpm);
			//	break;

			//case SHIMODA_MAIN_MODE_V:
			//	motion_ctrl_vpi(XAXIS, vm_refx*M2RAD, omega_max, &iq_refx);
			//	dac_da_out(0, 3, DA_GAIN_RPM *  omega_sp_ref_rpm_ma);
			//	flag_threshold = epsilon_sp - threshold;
			//	if (flag_threshold > 0) cmode = SHIMODA_A_MODE_V;
			//	break;

			//case SHIMODA_A_MODE_V:
			//	calc_new_speed(&rho_sp, &omega_sp_new_rpm, fchat_a, omega_sp_ma_rpm / beta_SHIMODA, Qn);
			//	if ((omega_sp_ref_rpm  < 1000) && (kmode > 0)) 		cmode = MAIN_MODE_V;
			//	motion_ctrl_vpi(XAXIS, vm_refx*M2RAD, omega_max, &iq_refx);
			//	dac_da_out(0, 3, DA_GAIN_RPM *  omega_sp_new_rpm * beta_SHIMODA);
			//	//if (omega_sp_ref_rpm < 100) cmode = MAIN_MODE;
			//	break;
			//	//scan mode
			//	//const feed per tooth

			//case CHATTER_TEST_MODE:
			//	motion_ctrl_vpi(XAXIS, vm_refx*M2RAD, omega_max, &iq_refx);
			//	if (chatter_rpm > 1000){
			//		dac_da_out(0, 3, DA_GAIN_RPM *  chatter_rpm);
			//	}
			//	else dac_da_out(0, 3, DA_GAIN_RPM *  omega_sp_ref_rpm_ma);
			//	break;

			//case MAIN_MODE:
			//	if ((omega_sp_ma_rpm < 2000) && (vm_refx > 0)) vm_refx = -0.0001;
			//	vm_refx = Qn*omega_sp_ma_rpm*RPM2HZ*feedpertooth;
			//	motion_ctrl_vpi(XAXIS, vm_refx*M2RAD, omega_max, &iq_refx);
			//	dac_da_out(0, 3, DA_GAIN_RPM *  omega_sp_ref_rpm_ma);
			//	flag_threshold = epsilon_sp - threshold;
			//	if (flag_threshold > 0) cmode = ADAPTIVE_MODE;
			//	break;

			//case ADAPTIVE_MODE:
			//	if (((omega_sp_ma_rpm < 2000) || (omega_sp_ref_rpm < 2000)) && (vm_refx > 0)) {
			//		vm_refx = -0.0001;
			//		cmode = MAIN_MODE;
			//	}
			//	vm_refx = Qn*omega_sp_ma_rpm*RPM2HZ*feedpertooth;
			//	motion_ctrl_vpi(XAXIS, vm_refx*M2RAD, omega_max, &iq_refx);
			//	dac_da_out(0, 3, DA_GAIN_RPM *  omega_sp_new_rpm);
			//	//if (omega_sp_ref_rpm < 100) cmode = MAIN_MODE;
			//	//time7 += (TS*1.0e-6);
			//	//if (time7 > 5) cmode = SHIMODA_A_MODE;
			//	break;

			//case SHIMODA_MAIN_MODE:
			//	if ((omega_sp_ma_rpm < 2000) && (vm_refx > 0))	 vm_refx = -0.0001;
			//	vm_refx = Qn*omega_sp_ma_rpm*RPM2HZ*feedpertooth;
			//	motion_ctrl_vpi(XAXIS, vm_refx*M2RAD, omega_max, &iq_refx);
			//	dac_da_out(0, 3, DA_GAIN_RPM *  omega_sp_ref_rpm_ma);
			//	flag_threshold = epsilon_sp - threshold;
			//	if (flag_threshold > 0) cmode = SHIMODA_A_MODE;
			//	break;
			//case SHIMODA_A_MODE:
			//	if (((omega_sp_ma_rpm < 2000) || (omega_sp_ref_rpm < 2000)) && (vm_refx > 0)) {
			//		vm_refx = -0.0001;
			//		cmode = MAIN_MODE;
			//	}
			//	vm_refx = Qn*omega_sp_ma_rpm*RPM2HZ*feedpertooth;
			//	motion_ctrl_vpi(XAXIS, vm_refx*M2RAD, omega_max, &iq_refx);
			//	dac_da_out(0, 3, DA_GAIN_RPM *  omega_sp_new_rpm * beta_SHIMODA);
			//	//if (omega_sp_ref_rpm < 100) cmode = MAIN_MODE;
			//	break;
			//	//scan mode

		//const v
		case DOB_MAIN_MODE_V:
			//X-AXIS for feed
			motion_ctrl_vpi(XAXIS, vm_refx*M2RAD, omega_max, &iq_refx);
			//Y-AXIS for servo-lock	
			if (vm_refx == 0.0) {
				theta_m_ref_liny = x_liny;
			}
			motion_ctrl_vpi(YAXIS, motion_ctrl_pos(theta_m_ref_liny, x_liny)*M2RAD, omega_may, &iq_refy);
			//Adaptive
			if ((kmode > 0) && (aspx_hf*aspx_hf - threshold > 0)) 	flag_threshold = 1;
			if (flag_threshold > 0) {
				calc_new_speed(&rho_sp, &omega_sp_new_rpm, fchat_a, omega_sp_ma_rpm, Qn);
				if (omega_sp_ref_rpm > 2000) omega_sp_ref_rpm_ma = omega_sp_new_rpm;
			}
			//Spindle for cutting
			spindle_motion_ctrl_vpi(omega_sp_ref_rpm_ma*RPM2RADPS, omega_sp_ma, &torque_command);
			dac_da_out(0, 3, DA_GAIN_TORQUE * torque_command);	
			break;

			//spindle torque control
		case	TORQUE_MODE:
			dac_da_out(0, 3, DA_GAIN_TORQUE * torque_command);
			break;

			//spindle speed control via torque
		case SPINDLE_OMEGA_MODE:
			spindle_motion_ctrl_vpi(omega_sp_ref_rpm_ma*RPM2RADPS, omega_sp_ma, &torque_command);
			dac_da_out(0, 3, DA_GAIN_TORQUE * torque_command);
			break;

			//spindle speed(no LPF) control via torque
		case DIRECT_SPINDLE_OMEGA_MODE:
			spindle_motion_ctrl_vpi(omega_sp_ref_rpm*RPM2RADPS, omega_sp_ma, &torque_command);
			dac_da_out(0, 3, DA_GAIN_TORQUE * torque_command);
			break;

			//stage velocity control
		case VEL_MODE:
			motion_ctrl_vpi(XAXIS, vm_refx*M2RAD, omega_max, &iq_refx);
			motion_ctrl_vpi(YAXIS, vm_refy*M2RAD, omega_may, &iq_refy);
			break;

			//stage position incremental mode
		case INC_MODE:
			if (xymode == XMODE) {
				if (fabsf(incx) > 0)  theta_m_ref_linx = incx + x_linx;
				motion_ctrl_vpi(XAXIS, motion_ctrl_pos(theta_m_ref_linx, x_linx)*M2RAD, omega_max, &iq_refx);
				incx = 0;
			}
			else if (xymode == YMODE) {
				if (fabsf(incy) > 0) theta_m_ref_liny = incy + x_liny;
				motion_ctrl_vpi(YAXIS, motion_ctrl_pos(theta_m_ref_liny, x_liny)*M2RAD, omega_may, &iq_refy);
				incy = 0;
			}
			break;


			//case POS_MODE:
			////stage position control full closed
			//if (xymode == XMODE) motion_ctrl_vpi(XAXIS, motion_ctrl_pos(theta_m_ref_linx, x_linx)*M2RAD, omega_max, &iq_refx);
			//else if (xymode == YMODE) motion_ctrl_vpi(YAXIS, motion_ctrl_pos(theta_m_ref_liny, x_liny)*M2RAD, omega_may, &iq_refy);
			//break;

		case 0: //stop
			iq_refx = 0;
			iq_refy = 0;
			dac_da_out(0, 3, 0);
			break;

		default:
			break;
		}
		/*
		//return to O
		if (cmode == OPOS_MODE) {
		motion_ctrl_pack_pos(XAXIS, omega_max, (LIMXMAX + LIMXMIN)*M2RAD / 2.0, theta_mx, &iq_refx);
		motion_ctrl_pack_pos(YAXIS, omega_may, (LIMYMAX +LIMYMIN)*M2RAD / 2.0, theta_my, &iq_refy);
		}
		*/
		/*
		if ((x_mx > LIMXMAX) || (x_mx < LIMXMIN)) {
		motion_ctrl_pack_pos(XAXIS, omega_max, (LIMXMAX+ LIMXMIN)*M2RAD /2.0, theta_mx, &iq_refx);
		isoverrun = ISOVRRN;
		}
		if ((x_my > LIMYMAX) || (x_my <LIMYMIN)) {
		motion_ctrl_pack_pos(YAXIS, omega_may, (LIMYMAX +LIMYMIN)*M2RAD / 2.0, theta_my, &iq_refy);
		isoverrun = ISOVRRN;
		}
		*/
	}

	// MULTI-INT OFF
	int_disable();
	CSR = regs[0];
	IRP = regs[1];

	//watch_data_8ch(); //watch data @ this frequency FS
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
	timer1_init(TS);
	timer0_init_vector(system_tint0);
	timer1_init_vector(system_tint1);
	timer0_start();
	timer1_start();
	timer0_enable_int();
	timer1_enable_int();

	system_fsm_init();
}
