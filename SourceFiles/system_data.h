/************************************************************************************
SYSTEM GLOBAL DATA MODULE
-------------------------
Descr.:		Header with experimental setup global data
Boards:		PE-Expert3, MWPE-C6713A DSP, MW-WAVE2 acquisition
System:		Motor Bench experimental setup (S-PMSM drives)
Author:		Thomas Beauduin, University of Tokyo, March 2016
*************************************************************************************/
#ifndef	SYSTEM_DATA_H
#define	SYSTEM_DATA_H

// MOTOR PAR
#define		Pp		(4.0)					// pole pairs amount	[-]
#define		Kt		    (0.412805442470582)		 // dq-Axis torque constant		[Nm/A] = CatalogValue/sqrt(3)
#define		Ke		(0.412805442470582)		// dq-Axis voltage constant [V/(rad/s)]
#define		Rs		(4.15)		// Stator resistance	[Ohm] /phase
#define		Ls		(0.004880)		// Stator inductance	[H] =Lq=Ld
#define		Ktx	(0.715)

#define		OVC_LIM	(50.48)					// overcurrent limit	[A] 
#define		OVV_LIM	(380.0)					// overvorltage limit	[V]
#define		OVS_LIM	(450.0)					// overspeed limit		[rad/s]
//#define		OVS_LIN_LIM	(0.1)					// overspeed limit		[m/s]
#define		I_PK	(25.0)					// ctrl out saturation	[A]
//#define       T_SP (24.0)                    //spindle torque satuation [Nm] 
#define       T_SP (12.0)

// INV PAR
#define		FC		(10000.0)				// carrier frequency	[Hz]
#define		TC		(100.0)                  // carrier sampling		[us]
#define		DT		(3500.0)				// inv dead time		[ns]

//CURRENT CONTROL AND MOTION SENSOR PAR
#define		FQ (5000)
#define		TQ (200.0)		//current control sampling [us]

// MOTION CONTROL PAR
#define		FS		(1000.0)          		// system frequency		[Hz]
#define		TS		(1000.0)   				// system sampling		[us]
#define		NROFT	(100000)               // total number of msr	[-]			
#define		XAXIS (0) //stage x axis
#define		YAXIS  (1) //stage y axis

// MATH PAR
#define		PI(n)	(3.14159265358979323846 * (n))
#define		sign(a) (((a)<0) ? -1 : ((a)>0))

//spindle encorders
#define PEV_BDN 0
#define RADPS2RPM (9.549296585513700)
#define RPM2RADPS (0.104719755119660)

//adaptive
#define Nd ((int) 2) //size of parameter
#define Qn ((int) 4) //number of flutes
#define CONTACT_THRESHOLD (100.0) //m^2/s^2
#define SIGMA_W (0.1) //observation noise variance
#define SIGMA_V (1.0e-6) //process noise variance
#define SIGMA_P (1.0e-2) //initial value of P

//modes
#define POS_MODE (1)
#define VEL_MODE (2)
#define QCRNT_MODE (3)
#define OPOS_MODE (4)
#define INC_MODE (5)
#define MAIN_MODE (6)
#define SCAN_MODE (7)
#define ADAPTIVE_MODE (8)
#define SHIMODA_A_MODE (9)
#define SHIMODA_MAIN_MODE (10)
#define EXPERIMENTAL_MODE (11)
#define MAIN_MODE_V (12)
#define ADAPTIVE_MODE_V (13)
#define SHIMODA_MAIN_MODE_V (14)
#define SHIMODA_A_MODE_V (15)
#define CHATTER_TEST_MODE (16)
#define TORQUE_MODE (17)
#define  SPINDLE_OMEGA_MODE (18)
#define DIRECT_SPINDLE_OMEGA_MODE (19)
#define DOB_MAIN_MODE_V (20)
#define DOB_ADAPTIVE_MODE_V (21)

//limit
#define MAX_ACC (92)

//XY modes
#define XMODE (0)
#define YMODE (1)
#define XYMODE (2)

//STAGE LIMIT [m]
#define LIMXMIN  (-0.47)
#define LIMXMAX  (0.0074)
#define ISOVRRN (1)

//STAGE VELOCITY LIMIT [rad/s]
#define VELXLIM (0.5235987755982287)
#define VELYLIM (0.5235987755982287)

//CONSTANTS
#define RAD2M (0.001909859317103)  //R[m/rad]
#define M2RAD (5.235987755982287e+02) //1/R [rad/m]
#define RPM2HZ (0.016666666666667)

#define DA_GAIN_RPM (0.0024920) //1/400 , 400 rpm/V *0.997
#define DA_GAIN_TORQUE (0.416666666666667) //1/2.4 2.4Nm/V

//MA FILTERS
//TS
#define ALPHASP		(0.715390456663971)					// 200 Hz LPF @1000*10^-6 sampling
#define ALPHAMA_FIRST (0.006263487375222)				//1 Hz LPF for 1000 usec
#define SPCNT2RADPS (0.0958737992428526)					// 2 * PI(1) /SPCNTPREV/(1000*10^-6)
//TQ
#define  ALPHA		(0.222232320828211	)						// 200 Hz LPF @200 usec sampling
#define INV2PITS (7.957747154594767e+02)					//1/2/pi*FC(==5000)

// SYSTEM VAR
extern int msr, cnt, set, calib;
extern float time, t0, t0_a;
extern enum mode {
	SYS_STP = 0x01, SYS_INI = 0x02,
	SYS_RUN = 0x04, SYS_ERR = 0x08
} sysmode_e;

// COMMAND VAR
extern float Aref, Fref;
extern enum ref {
	REF_OFF = 0, REF_CST = 1, REF_SIN = 2,
	REF_FDI = 3, REF_TDI = 4, REF_EXT = 5
} reftype_e;

// CTRL VAR
extern float vref, xref;
extern float uptc, yptc;
extern float vd_refx, vq_refx, id_refx, iq_refx;
extern float vd_refy, vq_refy, id_refy, iq_refy;
extern enum fb {
	FB_OFF = 0, FB_VPI = 1, FB_PPI = 2,
	FB_PID = 3, FB_SFB = 4, FB_HINF = 5
} fbtype_e;
extern enum ff {
	FF_OFF = 0, FF_ACC = 1, 
	FF_CIN = 2, FF_PTC = 3
} fftype_e;
extern enum dob {
	DOB_OFF = 0, DOB_RGB = 1
} dobtype_e;
extern enum fric {
	FRIC_OFF = 0, FRIC_COU = 1,
	FRIC_GMS = 2, FRIC_PRO = 3
} frictype_e;
extern float omega_sp_ref_rpm, omega_sp_ref_rpm_ma;
extern float vm_refx, vm_refy;
extern float theta_m_refx, theta_m_refy;
extern float incx, incy;

// MSR VAR
extern float idc_adx, theta_ex; //vdc_adx, 
extern float iu_adx, iw_adx, ia_adx, ib_adx, id_adx, iq_adx;
extern float vu_adx, vw_adx, va_adx, vb_adx, vd_adx, vq_adx;
extern float vdc_adx, vdc_ady, idc_ady, theta_ey;
extern float iu_ady, iw_ady, ia_ady, ib_ady, id_ady, iq_ady;
extern float vu_ady, vw_ady, va_ady, vb_ady, vd_ady, vq_ady;
extern float va_refx, vb_refx, vu_refx, vv_refx, vw_refx;
extern float va_refy, vb_refy, vu_refy, vv_refy, vw_refy;

extern int count_sp, count_old_sp, r_count_sp;
extern float omega_sp, omega_sp_ma, omega_sp_ma_rpm, omega_old_sp, theta_sp, omega_sp_ma_2;
extern float omega_sp_new_rpm;

extern float  theta_mox, theta_mdx, omega_mx, omega_max; //theta_mx,
extern float theta_my, theta_moy, theta_mdy, omega_my, omega_may;
extern float x_mx, x_my, v_mx, v_my;
extern float torque_ad, aspx, aspy, aspz;
extern float x_linx, x_liny, v_linx, v_liny, v_linx_ma, v_liny_ma;
extern float theta_m_ref_liny , theta_m_ref_linx ;
extern float dob_stx;
extern int  x_nano_linx;
extern float sigma_w, sigma_v;

extern float torque_command, observed_disturbance, observed_disturbance_ma;

extern int is_drive;
extern float pre_linx;

extern int test, test1, test2, test3, test4, test5, test6, test7, pin;
extern int cmode, xymode, kmode;
extern float aux1, aux2, aux3, aux4, aux5, aux6, aux7;
extern float fchat_a, fchat_a_ma;
extern float threshold, flag_threshold, contact_threshold, beta_SHIMODA;

extern float phi_sp[Nd], theta_par_est[Nd], P_var[Nd*Nd];
extern float aspx_hf, epsilon_sp, epsilon_sp_max;
extern int rho_sp;
extern float chatter_rpm;

extern float feedpertooth;

extern int ontimer;
extern float limit_time, ctime;

extern int isoverrun;

#endif


