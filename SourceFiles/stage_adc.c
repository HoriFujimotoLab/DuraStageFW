/************************************************************************************
STAGE SENSOR ADC MODULE
-----------------------
Descr.:		Hardware module adc of setup sensors
Boards:		MWPE3-C6713A, MWPE3-ADC (14 bit)
Sensor:		torque sensor (unipulse), other (temp)
Author:		Thomas Beauduin, University of Tokyo, March 2016
*************************************************************************************/

#include	"setup_adc.h"
#include	<mwio3.h>

// MODULE PAR
#define		ADC_BDN	0					// adc board number
#define		NROFM	(500.0)				// number of offset msr

// ADC RANGES
#define		R_IS	(50.0)				// Hioki current sensor [A]  (31.25A/5V)
#define		R_VS	(400.0)				// DC voltage sensor	[V]  (400V/5V)
#define		R_TS	(20.0)				// Unipulse T sensor	[Nm] (100%/5V)
#define		R_SV	(1.0)				// Servo Analyzer		[%]  (100%/5V)
#define		R_EX	(31.25)

// ADC OFFSETS (see notes)
#define		O_VDCX	(0.0)				// ch2 adc offset: vdc x
#define		O_VDCY	(0.0)				// ch2 adc offset: vdc y

// MODULE VAR
float ad_avg[12] = { 0.0 };				// MW-ADC average offsets


void setup_adc_init(void)
{
	// LOCAL VAR
	int i = 0, j = 0;													// loop counters
	float ad0, ad1, ad2, ad3;											// measured values
	float sum0 = 0.0, sum1 = 0.0, sum2 = 0.0, sum3 = 0.0;				// value summation

	// RANGE SET						
	adc_ad_init(ADC_BDN);												// init ADC board
	adc_ad_set_range(ADC_BDN, 0, R_VS, R_IS, R_IS, R_IS);				// grp 0 range settings
	adc_ad_set_range(ADC_BDN, 1, R_VS, R_IS, R_IS, R_IS);				// grp 1 range settings
	adc_ad_set_range(ADC_BDN, 2, R_TS, R_SV, R_SV, R_SV);				// grp 2 range settings
	
	// AVG CALC
	for (i = 0; i < 3; i++){
		adc_ad_set_offset(ADC_BDN, i, 0.0, 0.0, 0.0, 0.0);				// initial offset
		for (j = 0; j < NROFM; j++){
			adc_ad_in_grp(ADC_BDN, i, &ad0, &ad1, &ad2, &ad3);			// ADC group x read
			sum0 -= ad0; sum1 -= ad1; sum2 -= ad2; sum3 -= ad3;			// offset neg sum calc
		}
		ad_avg[4*i] = sum0 / NROFM; ad_avg[4*i+1] = sum1 / NROFM;		// channel avg calc
		ad_avg[4*i+2] = sum2 / NROFM; ad_avg[4*i+3] = sum3 / NROFM;
		sum0 = 0.0; sum1 = 0.0; sum2 = 0.0; sum3 = 0.0; j = 0;
	}
	
	// OFFSET SET
	adc_ad_set_offset(ADC_BDN, 0, O_VDCX, ad_avg[1], ad_avg[2], ad_avg[3]);		
	adc_ad_set_offset(ADC_BDN, 1, O_VDCY, ad_avg[5], ad_avg[6], ad_avg[7]);
	adc_ad_set_offset(ADC_BDN, 2, ad_avg[8], ad_avg[9], ad_avg[10], ad_avg[11]);
}


void setup_adc_read(int grp_ad, float *ad0, float *ad1, float *ad2, float *ad3)
{
	adc_ad_in_grp(ADC_BDN, grp_ad, &*ad0, &*ad1, &*ad2, &*ad3);
	//if (grp_ad == 0) { *ad2 = -*ad2; *ad3 = -*ad3; }					// hioki cabling inverse
}


/* OFFSET PROCEDURE:
** offset cause: a) sensors offset, b) adc-board offset
** offset cases: 
** case 1)	remove both offsets (init data=0):
**			msr & calculate offset at init (ec, ls, ex sensors)
** case 2)  remove board offset only, info in sensor offset:
**			dedicated experiment for adc-board (lc sensor)
**			sampling (30min), removal of bnc cables, offset msr
*/

