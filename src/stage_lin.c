/************************************************************************************
STAGE LINEAR-SCALE MODULE
-------------------------
Descr.:		Hardware module for linear-scale of table
Boards:		MWPE-Expert3, MWPE-FPGAA (custom board: 08934-C2-xxx)
Sensor:		1nm incremental linear scale (magnescale)
Author:		Thomas Beauduin, University of Tokyo, April 2015
*************************************************************************************/

#include	"stage_lin.h"
#include	<mwio3.h>									

// FPGA ADDR
#define	INI_ADDR	((volatile unsigned int*)0xA002000C)		// init address     (RW bit0)
#define	ERR_ADDR	((volatile unsigned int*)0xA0020000)		// error address    (R bit6..0)
#define	RST_ADDR	((volatile unsigned int*)0xA0020008)		// reset address    (RW bit0)
#define	CLR_ADDR	((volatile unsigned int*)0xA0020020)		// clear address    (W bit6..2)
#define	CON_ADDR	((volatile unsigned int*)0xA0020024)		// convert address  (W only 1 write)
#define	DAT0_ADDR	((volatile unsigned int*)0xA0020028)		// data address		(R 32bit)
#define	DAT2_ADDR	((volatile unsigned int*)0xA0020030)		// data address		(R 32bit)
#define	STA_ADDR	((volatile unsigned int*)0xA0020024)		// status address	(R bit6..0:busy, bit7:err)

// linear scale direction X:-1, Y:+1
float LIN_DIR[2] = { -1.0, 1.0 };
// MODULE VAR
int read_err = 0, hard_err = 0;


void stage_lin_init(void)
{
	*RST_ADDR = 0; *RST_ADDR = 1;								// reset registers
	*RST_ADDR = 1; *RST_ADDR = 0;								// reset hardware  
	*INI_ADDR = 0; *CLR_ADDR = 0x04;							// counter register init
}


void stage_lin_read(int axis, float *pos_t, float *vel_t, float *vel_ta)
{
	// LOCAL VAR
	int data_cnt = 0;											// msr counter [cnt]
	int i = 0;													// loop index  [-]
	float temp = 0;												// temp data buffer

	// READ DATA
	*CON_ADDR = 1;												// conversion start
	while (((*STA_ADDR & 0x02) == 1) && (i <= 5)) { i++; }		// wait for rdy status
	if (i >= 5) { read_err = 1; }								// over-time error
	if (axis == 0){ data_cnt = *DAT0_ADDR; }				    // read data register X axis
	if (axis == 1){ data_cnt = *DAT2_ADDR; }				    // read data register Y axis
	
	// POS & VEL
	temp = *pos_t;												// previous msr   [m]
	*pos_t =(float) LIN_DIR[axis] * data_cnt * 1.0e-9;						// table position [m]
	*vel_t = (*pos_t - temp) * FS;								// table velocity [m/s]
	*vel_ta = ALPHA * *vel_t + (1 - ALPHA) * *vel_ta;			// resursive maf  [m/s]
}

void stage_lin_nano_read(int axis, int *pos_nano)
{
	// LOCAL VAR
	int data_cnt = 0;											// msr counter [cnt]
	int i = 0;													// loop index  [-]

																// READ DATA
	*CON_ADDR = 1;												// conversion start
	while (((*STA_ADDR & 0x02) == 1) && (i <= 5)) { i++; }		// wait for rdy status
	if (i >= 5) { read_err = 1; }								// over-time error
	if (axis == 0) { data_cnt = *DAT0_ADDR; }				    // read data register X axis
	if (axis == 1) { data_cnt = *DAT2_ADDR; }				    // read data register Y axis

																// POS & VEL
	*pos_nano = (int)LIN_DIR[axis] * data_cnt;						// table position [nm]
}


void stage_lin_status(int *status)
{
	hard_err = *ERR_ADDR;										// hardware error
	if (read_err == 1 || hard_err == 1) { *status = 1; }		// fpga read error
	else								{ *status = 0; }
}


void stage_lin_reset(void)
{
	*CLR_ADDR = 0x04;											// set data_addr to 0
}
