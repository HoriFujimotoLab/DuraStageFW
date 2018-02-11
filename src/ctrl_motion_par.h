/************************************************************************************
MOTION CONTROL PARAMETER
----------------------
Descr.:		Control module for drive current control
Boards:		PE-Expert3, C6713A DSP (float calculation)
System:		Dura, Ball-screw-driven stage control
Author:		Shimoda Takaki, University of Tokyo, March 2018
*************************************************************************************/

//Velocity PI Pole Assignment: Bw 20Hz, Ts 1 kHz
float Avpi[2][1] = {{1.00000000000000000000e+00}, {1.00000000000000000000e+00}};
float Bvpi[2][1] = {{5.00000000000000000000e-01}, {5.00000000000000000000e-01}};
float Cvpi[2][1] = {{5.40990092213619270000e-01}, {5.40990092213619270000e-01}};
float Dvpi[2][1] = {{1.90696052089545360000e+00}, {1.90696052089545360000e+00}};

float A_StageLPF[4] = {9.90897487419981290000e-01, -2.81916400125604150000e-01, 5.00000000000000000000e-01, 0.00000000000000000000e+00};
float B_StageLPF[2] = {2.00000000000000000000e+00, 0.00000000000000000000e+00};
float C_StageLPF[2] = {7.78663875520966940000e-01, -1.40726703839911330000e+00};
float D_StageLPF[1] = {0.00000000000000000000e+00};

//Postion P based on Velocity PI, Bw:5 Hz, Ts1kHz
float Kxp = PI(10);

//Spindle velocity PI PZC control Bw:40Hz, Ts 1kHz
float Aspvpi[1] = {1.00000000000000000000e+00};
float Bspvpi[1] = {6.25000000000000000000e-02};
float Cspvpi[1] = {1.20637157897849790000e-01};
float Dspvpi[1] = {1.25663706143591720000e+00};

/*
//Pole Assigned Position PID: Bw 18 Hz, Ts 2 kHz, Gain Margin 10 dB, Phase Margin 40 degrees
float Apid[2][2] = {
	{1.7877412762836360e+00, -7.8774127628363588e-01},
	{1.0000000000000000e+00, 0.0000000000000000e+00},
};
float Bpid[2][1] = {
	{3.2000000000000000e+01},
	{0.0000000000000000e+00},
};
float Cpid[1][2] = {
	{-8.5554638999512918e+00, 8.5839634206202504e+00},
};
float Dpid[1][1] = {
	{1.5836149446374882e+03},
};
*/

//H infinity Loop Shaping PID: Bw 18 Hz, Ts 2 kHz, Gain Margin 12 dB, Phase Margin 40 degrees
float Apid[2][2] = {
	{1.4946071513535852e+00, -4.9460715135358529e-01},
	{1.0000000000000000e+00, 0.0000000000000000e+00},
};
float Bpid[2][1] = {
	{6.4000000000000000e+01},
	{0.0000000000000000e+00},
};
float Cpid[1][2] = {
	{-3.5575397727840482e+01, 3.5830035432516993e+01},
};
float Dpid[1][1] = {
	{5.0797579430351443e+03},
};

//H infinity Loop Shaping Second-order Prefilter: Bw 14.541 Hz, Ts 2 kHz, Overshoot = 15 %
float A_C_FF[4] = {1.93542867251606650000e+00, -9.37449187631321230000e-01, 1.00000000000000000000e+00, 0.00000000000000000000e+00};
float B_C_FF[2] = {6.25000000000000000000e-02, 0.00000000000000000000e+00};
float C_C_FF[2] = {3.18063724713027180000e-02, 5.05539447449518570000e-04};
float D_C_FF[1] = {5.05128778813689520000e-04};

/*
//H infinity Loop Shaping Second-order Prefilter: Bw 13 Hz, Ts 2 kHz, Overshoot = 5 %
float A_C_FF[4] = {1.94257793683596590000e+00, -9.44181194920592710000e-01, 1.00000000000000000000e+00, 0.00000000000000000000e+00};
float B_C_FF[2] = {3.12500000000000000000e-02, 0.00000000000000000000e+00};
float C_C_FF[2] = {5.05677596120281820000e-02, 7.15935604142138230000e-04};
float D_C_FF[1] = {4.00814521156700680000e-04};
*/
