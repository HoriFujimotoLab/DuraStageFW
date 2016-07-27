/************************************************************************************
MOTOR ROTARY-ENCODER MODULE
---------------------------
Descr.:		Hardware module for motor encoder (00672)
Boards:		MWPE-Expert3, MWPE-PIOS (PIO expansion board)
Sensor:		20bit absolute rotary encoder (Tamagawa)
Author:		Thomas Beauduin, University of Tokyo, March 2016
*************************************************************************************/

#include "motor_enc.h"
#include <math.h>
#include <mwio3.h>

float theta_mx = 0.0;

// MODULE PAR
#define DAT_ADDR		0xA0070000					// myway defined data address
#define RST_ADDR		0xA0071000					// myway defined reset address
#define	ENC_RES			(1048576.0)					// 20bit resolution [cnt/turn]
#define ALPHA			(0.715390456663971)						// recursive IIR MAF factor //400Hz@5*10^-4 sampling
const int pios_bdn[2] = {0, 1};						// PIOS board dip switch numbers
const int enc_dir[2] = { -1, -1 };					// encoder rotation direction
const float enc_off[2] = {2.363769, 2.187501416};	// setup encoder-rotor offsets

// MODULE VAR
static int nrofr[2] = { 0, 0 };						// number of turns [-]
static double theta_msr[2] = { 0.0, 0.0 };			// measured theta [rad]

//SPINDLE ENCORDER
#define SPCNT2RADPS (0.191747598485705)  // 2 * PI(1) /SPCNTPREV/TS
#define SPCNTPREV (65536.0) //cnt/turn 2^16
#define MAXSPCNT (1.073741823500000e+09) //INTMAX/2

void motor_enc_init(int axis)
{
	pios_pio_set_bit(pios_bdn[axis], 0);									// bit0 DS40
	pios_pio_clr_bit(pios_bdn[axis], 1);									// bit1 INRQ
	pios_pio_set_bit(pios_bdn[axis], 2);									// bit2 RQC0
	pios_pio_set_bit(pios_bdn[axis], 3);									// bit3 RQC1
	pios_pio_clr_bit(pios_bdn[axis], 5);									// bit5 ABSMD (20bit)
	pios_pio_clr_bit(pios_bdn[axis], 6);									// bit6 RESET
}


void motor_enc_elec(int axis, float *theta_e)
{
	// LOCAL VAR
	unsigned int data_msr;													// measured data [cnt]

	// READ DATA
	pios_pio_set_bit(pios_bdn[axis], 4);									// bit4 RQSTB
	(*(volatile int*)(RST_ADDR + ((pios_bdn[axis]) << 14))); wait(3);		// reset flag
	pios_pio_clr_bit(pios_bdn[axis], 4);									// bit4 RQSTB
	data_msr = (*(volatile int*)(DAT_ADDR + ((pios_bdn[axis]) << 14)) & 0x000FFFFF);
	theta_msr[axis] = (double)data_msr / ENC_RES * PI(2) * enc_dir[axis];	// [cnt] to [rad]

	// ELEC
	*theta_e = (float)(theta_msr[axis] * Pp) - enc_off[axis];				// mech to elec
	while (*theta_e > PI(2)) { *theta_e -= PI(2); }							// value limitation:
	while (*theta_e < 0)	 { *theta_e += PI(2); }							// {0, 2pi}
}


void motor_enc_read(int axis, float *theta_m, float *omega_m, float *omega_ma)
{
	// LOCAL VAR
	double diff = 0.0;														// position diff   [rad]
	int i = 0;																// rev jump index  [-]

	// MECH
	diff = theta_msr[axis] - (double)(*theta_m - nrofr[axis] * PI(2));		// theta time difference
	if (diff >  PI(1)){ nrofr[axis]--; i--; }								// revolution calc
	if (diff < -PI(1)){ nrofr[axis]++; i++; }								// note: max vel pi*fs
	*theta_m = (float)(theta_msr[axis] + nrofr[axis] * PI(2));				// full screw pos calc
	*omega_m = (float)((diff + i*PI(2)) * FS);								// full screw vel calc

	// FILT
	*omega_ma = ALPHA * *omega_m + (1.0 - ALPHA) * *omega_ma;				// resursive iir maf
}


void motor_enc_status(int axis, unsigned int *status)
{
	*status = (((*(volatile int*)(DAT_ADDR + ((pios_bdn[axis]) << 14))) & 0x80000000) >> 31);
}


void motor_enc_reset(int axis)
{
	nrofr[axis] = 0; 														// reset number of rev
}

void motor_enc_spindle(int *count_old, int *count, float *omega_old, float *omega, float *omega_ma, float *theta_m, int *r_count) {
	/**************************角速度演算（計数法）ishibashi***************************/
	//note unsigned int 0 ～ 4,294,967,295
	//int pin;
	int count_d;

	*count_old = *count; /*１サンプル前のカウント値*/
	*omega_old = *omega;                              /**omegaの１サンプル前の更新*/

	*count = pev_abz_read(PEV_BDN);				/* Read value of ABZ encoder's count	*/
	//pin = pev_abz_in_pin(PEV_BDN);			/* Check status of ABZ encoder's signal */

	count_d = *count- *count_old; //diff

	/*
	if (sign((int)*omega) != sign(count_d) ) {                     //Z interruption: when spindle speed(almost constant, at least there is no posibbility of servo use) >0, count_d should be >0 unless Z intteruption
		*omega = (float) (count_d + SPCNTPREV) * SPCNT2RADPS;										//omega = 2 * PI(1) / 8000 * (count_d + 8000) / 0.001; //8000???
		*theta_m += (float)(count_d + SPCNTPREV) * SPCNT2RADPS;    //スピンドルモータの位相/*パルス数から角速度変換
	}
	*/
	                                        /*z信号がこない場合*/
	*omega = (float) (count_d)* SPCNT2RADPS;              /*パルス数から角速度変換*/
	*theta_m += (float) count_d  * SPCNT2RADPS;          //スピンドルモータの位相
	
	//reset
	if ((*count > MAXSPCNT) || (*count < -MAXSPCNT)) {
		pev_abz_clear(PEV_BDN);
		*count = 0;
	}

	while (*theta_m > PI(2)) {
		*theta_m -= PI(2);
		r_count += 1;
	}							// value limitation:
	while (*theta_m < 0) {
		*theta_m += PI(2);
		r_count -= 1;
	}							// {0, 2pi}

	//rpm = RADPS2RPM**omega;                            /**omegaからrpmの算出*/

	*omega_ma = ALPHA * *omega + (1.0 - ALPHA) * *omega_ma;

/************************************************************************/
}