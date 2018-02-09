/************************************************************************************
MOTOR POWER-ELEC MODULE
-----------------------
Descr.:		hardware module for motor drive
Boards:		MWPE-Expert3, MWPE-PEV (PEV expansion board)
PWelec:		MWINV-9R122A	(9.1kVA power module)
Author:		Thomas Beauduin, University of Tokyo, March 2016
*************************************************************************************/

#include "motor_pem.h"
#include <math.h>
#include <mwio3.h>

// MODULE PAR
#define PEV_BDN 0   // pev board number		 [-]
#define TPRE 0.0	// interrupt offset		 [us]  (1.0e6/FS/2.0)
#define NROFM (500) // number of offset msr  [-]

// INV ADC RANGES
#define R_SS (1.0) // sys safety sensors (100%/5V)

// MODULE VAR
float pe_avg[8] = {0.0}; // MW-PEV average offsets

void motor_adc_init(void)
{
	// LOCAL VAR
	int i = 0, j = 0;									  // loop counters
	float ad0, ad1, ad2, ad3;							  // measured values
	float sum0 = 0.0, sum1 = 0.0, sum2 = 0.0, sum3 = 0.0; // value summation

	// RANGE SET
	pev_init(PEV_BDN);									  // init PEV board
	pev_ad_set_range(PEV_BDN, 0, R_SS, R_SS, R_SS, R_SS); // grp 0 range settings
	pev_ad_set_range(PEV_BDN, 1, R_SS, R_SS, R_SS, R_SS); // grp 1 range settings

	// AVG CALC
	for (i = 0; i < 2; i++)
	{
		for (j = 0; j < NROFM; j++)
		{
			pev_ad_start(PEV_BDN, i); // ADC group x start
			while (pev_ad_in_st(PEV_BDN, i) != 0)
			{
			}												   // ADC group x wait
			pev_ad_in_grp(PEV_BDN, i, &ad0, &ad1, &ad2, &ad3); // ADC group x read
			sum0 += ad0;
			sum1 += ad1;
			sum2 += ad2;
			sum3 += ad3; // offset sum calc
		}
		pe_avg[4 * i] = sum0 / NROFM;
		pe_avg[4 * i + 1] = sum1 / NROFM; // channel avg calc
		pe_avg[4 * i + 2] = sum2 / NROFM;
		pe_avg[4 * i + 3] = sum3 / NROFM;
		sum0 = 0.0;
		sum1 = 0.0;
		sum2 = 0.0;
		sum3 = 0.0;
		j = 0;
	}
}

void motor_adc_read(int grp_ad, float *ad0, float *ad1, float *ad2, float *ad3)
{
	float adf0, adf1, adf2, adf3;
	pev_ad_start(PEV_BDN, grp_ad); // ADC group x conv start
	while (pev_ad_in_st(PEV_BDN, grp_ad) != 0)
	{
	}															// ADC group x conv wait
	pev_ad_in_grp(PEV_BDN, grp_ad, &adf0, &adf1, &adf2, &adf3); // ADC group x data read
	if (grp_ad == 0)
	{ // grp 0 offset processing
		*ad0 = adf0 - pe_avg[0];
		*ad1 = adf1 - pe_avg[1]; // NO CNT
		*ad2 = adf2 - pe_avg[2];
		*ad3 = adf3 - pe_avg[3]; // NO CNT
	}
	else
	{ // grp 1 offset processing
		*ad0 = adf0 - pe_avg[4];
		*ad1 = adf1 - pe_avg[6]; // NO CNT
		*ad2 = adf2 - pe_avg[5];
		*ad3 = adf3 - pe_avg[7]; // NO CNT
	}
}

void motor_inv_init(void)
{
	pev_inverter_init(PEV_BDN, 0, FC, DT); // init inv para
	pev_inverter_init(PEV_BDN, 1, FC, DT);
	pev_inverter_set_uvw(PEV_BDN, 0, 0.0, 0.0, 0.0); // set modulation type
	pev_inverter_set_uvw(PEV_BDN, 1, 0.0, 0.0, 0.0);
	pev_inverter_set_syncint(PEV_BDN, TPRE); // set inv interrupt offset
	pev_inverter_enable_up_int5(PEV_BDN);	// inv interrupt settings
	wait(TC);								 // wait 1 carrier cycle [us]
	pev_inverter_start_pwm(PEV_BDN, 0);		 // start gate signal (pwm)
	pev_inverter_start_pwm(PEV_BDN, 1);
}

void motor_inv_pwm(int axis, float vu_ref, float vv_ref, float vw_ref, float vdc_ad)
{
	float mu_ref, mv_ref, mw_ref; // phase modulation index
	double vdc_idx;				  // dc-bus voltage index
	vdc_idx = 2.0 / vdc_ad;
	mu_ref = mwlimit((float)(vu_ref * vdc_idx), 1.0);
	mv_ref = mwlimit((float)(vv_ref * vdc_idx), 1.0);
	mw_ref = mwlimit((float)(vw_ref * vdc_idx), 1.0);
	pev_inverter_set_uvw(PEV_BDN, axis, mu_ref, mv_ref, mw_ref); // spwm output to inverter
}
