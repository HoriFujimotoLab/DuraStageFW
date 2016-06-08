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
#define		Kt		(0.564)					// Torque constant		[Nm/A]
#define		Ke		(0.172668006987608)		// Voltage constant		[Nm/V]
#define		Rs		(0.156666666666667)		// Stator resistance	[Ohm]
#define		Ls		(0.002280000000000)		// Stator inductance	[H]
#define		OVC_LIM	(34.0)					// overcurrent limit	[A]
#define		OVV_LIM	(380.0)					// overvorltage limit	[V]
#define		OVS_LIM	(450.0)					// overspeed limit		[rad/s]
#define		I_PK	(25.0)					// ctrl out saturation	[A]

// INV PAR
#define		FC		(10000.0)				// carrier frequency	[Hz]
#define		TC		(100.0)                  // carrier sampling		[us]
#define		DT		(3500.0)				// inv dead time		[ns]

// STAGE PAR
#define		TS		(400.0)   				// system sampling		[us]
#define		FS		(2500.0)          		// system frequency		[Hz]
#define		NROFT	(50000.0)               // total number of msr	[-]			

// MATH PAR
#define		PI(n)	(3.14159265358979323846 * (n))
#define		sign(a) (((a)<0) ? -1 : ((a)>0))

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

// MSR VAR
extern float idc_adx, theta_ex; //vdc_adx, 
extern float iu_adx, iw_adx, ia_adx, ib_adx, id_adx, iq_adx;
extern float vu_adx, vw_adx, va_adx, vb_adx, vd_adx, vq_adx;
extern float vdc_ady, idc_ady, theta_ey;
extern float iu_ady, iw_ady, ia_ady, ib_ady, id_ady, iq_ady;
extern float vu_ady, vw_ady, va_ady, vb_ady, vd_ady, vq_ady;
extern float va_ref, vb_ref, vu_ref, vv_ref, vw_ref;

extern float  theta_mox, theta_mdx, omega_mx, omega_max; //theta_mx,
extern float theta_my, theta_moy, theta_mdy, omega_my, omega_may;
extern float torque_ad, temp1, temp2, temp3;
extern int test;

#endif
