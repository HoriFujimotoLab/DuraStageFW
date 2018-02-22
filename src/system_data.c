/************************************************************************************
SYSTEM GLOBAL DATA MODULE
-------------------------
Descr.:		Header with experimental setup global data
Boards:		PE-Expert3, MWPE-C6713A DSP, MW-WAVE2 acquisition
System:		Motor Bench experimental setup (S-PMSM drives)
Author:		Thomas Beauduin, University of Tokyo, March 2016
*************************************************************************************/
#include "system_data.h"

// GLOBAL SYSTEM VAR
int msr = -1, cnt = 0;   // measurement counters
int set = -1, calib = 0; // calibration counters
float time = 0.0, t0 = 0.0, t0_a = 0.0;
enum mode sysmode_e = SYS_STP;

// GLOBAL COMMAND VAR
float Aref = 0.0, Fref = 0.0;
enum ref reftype_e = REF_OFF;

// GLOBAL CTRL VAR
float vref = 0.0, xref = 0.0;
float uptc = 0.0, yptc = 0.0;
float vd_refx = 0.0, vq_refx = 0.0, id_refx = 0.0, iq_refx = 0.0;
float vd_refy = 0.0, vq_refy = 0.0, id_refy = 0.0, iq_refy = 0.0;
enum fb fbtype_e = FB_OFF;
enum ff fftype_e = FF_OFF;
enum dob dobtype_e = DOB_OFF;
enum fric frictype_e = FRIC_OFF;
float omega_sp_ref_rpm = 0.0, omega_sp_ref_rpm_ma = 0.0; //spindle speed ref [rpm]
float vm_refx = 0.0, vm_refy = 0.0;
float theta_m_refx = 0.0, theta_m_refy = 0.0;
float incx = 0.0, incy = 0.0;

// GLOBAL MSR VAR
float idc_adx, theta_ex;
float iu_adx, iw_adx, ia_adx, ib_adx, id_adx, iq_adx;
float vu_adx, vw_adx, va_adx, vb_adx, vd_adx, vq_adx;
float vdc_adx, vdc_ady, idc_ady, theta_ey;
float iu_ady = 0.0, iw_ady = 0.0, ia_ady = 0.0, ib_ady = 0.0, id_ady = 0.0, iq_ady = 0.0;
float vu_ady = 0.0, vw_ady = 0.0, va_ady = 0.0, vb_ady = 0.0, vd_ady = 0.0, vq_ady = 0.0;
float va_refx = 0.0, vb_refx = 0.0, vu_refx = 0.0, vv_refx = 0.0, vw_refx = 0.0;
float va_refy = 0.0, vb_refy = 0.0, vu_refy = 0.0, vv_refy = 0.0, vw_refy = 0.0;

// SPINDLE VAR
int count_sp, count_old_sp, r_count_sp;
float omega_sp, omega_sp_ma, omega_sp_ma_rpm, omega_old_sp, theta_sp, omega_sp_ma_2;
float omega_sp_new_rpm;
float omega_sp_ma_temp;
float torque_command, observed_disturbance, observed_disturbance_ma;

//MOTOR VAR
float theta_mox, theta_mdx, omega_mx, omega_max;
float theta_my, theta_moy, theta_mdy, omega_my, omega_may;
float x_mx, x_my, v_mx, v_my;
float x_linx, x_liny, v_linx, v_liny, v_linx_ma, v_liny_ma;
float theta_m_ref_liny, theta_m_ref_linx;
float dob_stx;
int x_nano_linx;
float sigma_w, sigma_v;

//EXPERIMENTAL VAR
float e_theta_mx = 0;
float simulated_disturbance = 0;
float theta_m_refx_ff = 0;

//ACCEL VAR
float torque_ad, aspx, aspy, aspz;

//SYSTEM VAR
int is_drive;
int cmode, xymode, kmode;
int watch = 0;

//ILC VAR
float f_ff = 0;
int flag_ILC = 0;

//DEBUG par
float aux1, aux2, aux3, aux4, aux5, aux6, aux7;
