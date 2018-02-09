/************************************************************************************
SYSTEM FINIT-STATE-MACHINE MODULE
---------------------------------
Descr.:		finit-state-machine (fsm) for system protection
Boards:		MWPE-Expert3, MWPE-DIO (MWPE-PEV dio connect)
Sensor:		Digital input from inverter & emergency switches
Author:		Shimoda Takaki, University of Tokyo, 2018
*************************************************************************************/
#include "system_fsm.h"

// SYSTEM CNT
#define HWX_DI 0x0001 // hardware error			(DI0)
#define THX_DI 0x0002 // thermal error			(DI1)
#define HWY_DI 0x0010 // emergency switch			(DI4)
#define THY_DI 0x0020 // emergency switch			(DI5)

#define INI_SW 0x0100 // system init switch		(DI8)
#define RUN_SW 0x0200 // system run switch		(DI9)
#define ERR_SW 0x0400 // error override switch	(DI10)
#define OIL_SW 0x0800 // oil pump switch			(DI11)

#define RST_DO 0x0001 // inv reset relay output	(DO1)
#define OIL_DO 0x0002 // Oilpump relay output		(DO2)
//10 seconds move side to side

#define FWE_LED 0x0100 // 1. firmware error		(DO8)
#define OVC_LED 0x0200 // 2. overcurrent error		(DO9)
#define OVV_LED 0x0400 // 3. overvoltage error		(DO10)
#define OVS_LED 0x0800 // 4. overspeed error		(DO11)
#define HWE_LED 0x1000 // 5. inv hardware error	(DO12) //if inv power is not supplied
#define SSE_LED 0x2000 // 6. setup sensor error	(DO13)
#define OVL_LED 0x4000 // 7. over load error	(DO14)

// MODULE PAR
//#define		PEV_BDN		0				// pev board number
#define SWT_PRT 0.2 // switch protection time

// MODULE VAR & FUNC
int din, don, err = 0;
int firmerr = 0, senserr = 0;
void system_fsm_err(void);
void system_fsm_reset(void);

void system_fsm_mode(void)
{
	din = pev_pio_in(PEV_BDN);
	don = 0;

	switch (sysmode_e)
	{
	case SYS_STP:
		if (INI_SW == (din & INI_SW))
		{											// init switch on
			pev_pio_out(PEV_BDN, don);				// reset relays
			pev_inverter_start_pwm(PEV_BDN, XAXIS); // drive inv's on
			pev_inverter_start_pwm(PEV_BDN, YAXIS);
			sysmode_e = SYS_INI;
		}
		if (INI_SW == (din & INI_SW) && RUN_SW == (din & RUN_SW))
		{ // avoid direct init
			err = err | FWE_LED;
			sysmode_e = SYS_ERR; // firmware init err
		}
		break;

	case SYS_INI:
		system_fsm_err();		   // check for errors
		pev_pio_out(PEV_BDN, don); // switch relays
		if (err != 0)
		{										   // error detected
			pev_inverter_stop_pwm(PEV_BDN, XAXIS); // error mode change
			pev_inverter_stop_pwm(PEV_BDN, YAXIS);
			sysmode_e = SYS_ERR;
		}
		if (INI_SW != (din & INI_SW))
		{										   // ini switch off
			system_fsm_reset();					   // reset firmware
			pev_inverter_stop_pwm(PEV_BDN, XAXIS); // revert to stp mode
			pev_inverter_stop_pwm(PEV_BDN, YAXIS);
			sysmode_e = SYS_STP;
		}
		if (RUN_SW == (din & RUN_SW))
		{ // run switch on
			sysmode_e = SYS_RUN;
		}
		break;

	case SYS_RUN:
		system_fsm_err();		   // check for errors
		pev_pio_out(PEV_BDN, don); // switch relays
		if (err != 0)
		{										   // error detected
			pev_inverter_stop_pwm(PEV_BDN, XAXIS); // error mode change
			pev_inverter_stop_pwm(PEV_BDN, YAXIS);
			sysmode_e = SYS_ERR;
		}
		if (OIL_SW == (din & OIL_SW))
		{
			pev_pio_set_bit(PEV_BDN, OIL_DO);
		} // oil pump on
		else
		{
			pev_pio_clr_bit(PEV_BDN, OIL_DO);
		} // oil pump off
		if (RUN_SW != (din & RUN_SW))
		{
			sysmode_e = SYS_INI;
		} // firmware reset
		break;

	case SYS_ERR:
		pev_pio_out(PEV_BDN, err);								  // system error LED
		if (RUN_SW != (din & RUN_SW) && INI_SW != (din & INI_SW)) // hardware reset
		{
			system_fsm_reset();
			err = 0;
			sysmode_e = SYS_STP; // firmware reset
			don = don | RST_DO;
			pev_pio_out(PEV_BDN, don); // reset relay on
		}
		break;
	}
	led_out(sysmode_e); // system mode LED
}

void system_fsm_err(void)
{
	if (firmerr != 0)
	{
		err = err | FWE_LED;
	} // 1. firmware error
	switch (xymode)
	{
	case XMODE:
		if (fabsf(iu_adx) > OVC_LIM)
		{
			err = err | OVC_LED;
		} // 2. overcurrent
		if (fabsf(iw_adx) > OVC_LIM)
		{
			err = err | OVC_LED;
		} // 2.
		if (fabsf(idc_adx) > OVC_LIM)
		{
			err = err | OVC_LED;
		} // 2.
		if (fabsf(vdc_adx) > OVV_LIM)
		{
			err = err | OVV_LED;
		} // 3. overvoltage
		if (fabsf(omega_max) > OVS_LIM)
		{
			err = err | OVS_LED;
		} // 4. overspeed
		if (HWX_DI == (din & HWY_DI))
		{
			err = err | HWE_LED;
		} // 5. hardware error
		if (THX_DI == (din & THY_DI))
		{
			err = err | HWE_LED;
		} // 5.
		break;
	case YMODE:
		if (fabsf(iu_ady) > OVC_LIM)
		{
			err = err | OVC_LED;
		} // 2.
		if (fabsf(iw_ady) > OVC_LIM)
		{
			err = err | OVC_LED;
		} // 2.
		if (fabsf(idc_ady) > OVC_LIM)
		{
			err = err | OVC_LED;
		} // 2.
		if (fabsf(vdc_ady) > OVV_LIM)
		{
			err = err | OVV_LED;
		} // 3.
		if (fabsf(omega_may) > OVS_LIM)
		{
			err = err | OVS_LED;
		} // 4.
		if (HWY_DI == (din & HWY_DI))
		{
			err = err | HWE_LED;
		} // 5.
		if (THY_DI == (din & THY_DI))
		{
			err = err | HWE_LED;
		} // 5.
		break;
	}
}

void system_fsm_reset(void)
{
	// CTRL RESET
	current_ctrl_reset();
	motion_ctrl_reset();

	// HARD RESET
	motor_enc_reset(0);
	motor_enc_reset(1);
}

void system_fsm_init(void)
{
	pev_inverter_stop_pwm(PEV_BDN, 0);
	pev_inverter_stop_pwm(PEV_BDN, 1);
	sysmode_e = SYS_STP;

	//fail safe
	din = pev_pio_in(PEV_BDN);
	if (INI_SW == (din & INI_SW) || RUN_SW == (din & RUN_SW))
	{ // avoid direct init
		err = err | FWE_LED;
		sysmode_e = SYS_ERR; // firmware init err
	}
}
