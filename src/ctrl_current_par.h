/************************************************************************************
CURRENT CONTROL PARAMETER
----------------------
Descr.:		Control module for drive current control
Boards:		PE-Expert3, C6713A DSP (float calculation)
System:		DURA, S-PMSM motor-type (dq-axis control)
Author:		Shimoda Takaki, University of Tokyo, March 2018
*************************************************************************************/

//PZC PI
//Bw:1000 Hz(2*pi*1000 rad/s), Ts: 10 kHz (100 us)
//with 1000 Hz LPF Input Shaping
float Afbi = 1.00000000000000000000e+00;
float Bfbi = 2.00000000000000000000e+00;
float Cfbi = 1.30376095123976300000e+00;
float Dfbi = 3.19657052502761410000e+01;

float Affi = 5.21885552778623580000e-01;
float Bffi = 5.00000000000000000000e-01;
float Cffi = 7.27635469800950620000e-01;
float Dffi = 2.39057223610688240000e-01;

//Dead-Time SP
float ihys = 5.0000000000e-02;
float vhys = 2.8800000000e-02;
