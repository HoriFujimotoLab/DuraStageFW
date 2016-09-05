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
int msr = -1, cnt = 0;				// measurement counters
int set = -1, calib = 0;			// calibration counters
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
float omega_sp_ref_rpm = 0.0, omega_sp_ref_rpm_ma=0.0; //spindle speed ref [rpm]
float vm_refx=0.0, vm_refy = 0.0;
float theta_m_refx = 0.0, theta_m_refy = 0.0;
float incx = 0.0, incy = 0.0;

// GLOBAL MSR VAR
float idc_adx, theta_ex;
float iu_adx, iw_adx, ia_adx, ib_adx, id_adx, iq_adx;
float vu_adx, vw_adx, va_adx, vb_adx, vd_adx, vq_adx;
float vdc_adx, vdc_ady, idc_ady, theta_ey;
float iu_ady = 0.0, iw_ady = 0.0, ia_ady = 0.0, ib_ady = 0.0, id_ady = 0.0, iq_ady=0.0;
float vu_ady = 0.0, vw_ady = 0.0, va_ady = 0.0, vb_ady = 0.0, vd_ady = 0.0, vq_ady = 0.0;
float va_refx = 0.0, vb_refx = 0.0, vu_refx = 0.0, vv_refx = 0.0, vw_refx = 0.0;
float va_refy = 0.0, vb_refy = 0.0, vu_refy = 0.0, vv_refy = 0.0, vw_refy = 0.0;

int count_sp = 0.0, count_old_sp = 0.0, r_count_sp=0.0;
float omega_sp = 0.0, omega_sp_ma=0.0, omega_sp_ma_rpm=0.0, omega_old_sp = 0.0, theta_sp=0.0;
float  omega_sp_new_rpm = 0.0;

float theta_mox = 0.0, theta_mdx = 0.0, omega_mx = 0.0, omega_max = 0.0; //theta_mx,
float theta_my = 0.0, theta_moy = 0.0, theta_mdy = 0.0, omega_my = 0.0, omega_may = 0.0;
float x_mx = 0.0, x_my = 0.0, v_mx = 0.0, v_my = 0.0;
float torque_ad = 0.0, aspx = 0.0, aspy = 0.0, aspz = 0.0;

int test = 0.0, test1 = 0.0, test2 = 0.0, test3 = 0.0, test4 = 1, test5 = 0.0, test6 = 0.0, test7 = 0.0; //for debugging 
int cmode = 0, xymode = 0, kmode=0;
float aux1 = 0.0, aux2 = 0.0, aux3 = 0.0, aux4 = 0.0, aux5 = 0.0, aux6 = 0.0, aux7=0.0;
float fchat_a = 0.0, fchat_a_ma=0.0; 
float threshold = 50, flag_threshold = 0, contact_threshold = 25, beta_SHIMODA=0.975;

float phi_sp[Nd] = { 0.0, 0.0 }, theta_par_est[Nd] = { 0.0, 0.0 }, P_var[Nd*Nd] = {1e-4, 0.0, 0.0, 1e-4};
float  aspx_hf = 0.0, epsilon_sp=0.0, epsilon_sp_max=0.0;
int rho_sp = 0;

float feedpertooth=0.0;

int ontimer = 0;
float limit_time = 0.0, ctime=0.0;

int isoverrun = 0;





