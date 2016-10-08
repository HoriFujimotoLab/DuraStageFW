//MOTION CTRL PAR:
//Velocity PI: Bw 40Hz, Ts 1 kHz
float Avpi[2][1] = { { 1.00000000000000000000e+00 },{ 1.00000000000000000000e+00 } };
float Bvpi[2][1] = { { 5.00000000000000000000e-01 },{ 5.00000000000000000000e-01 } };
float Cvpi[2][1] = { { 9.84219236928830730000e-01 },{ 9.84219236928830730000e-01 } };
float Dvpi[2][1] = { { 3.51506870331725140000e+00 },{ 3.51506870331725140000e+00 } };


//Postion P based on Velocity PI, Bw:0.5 Hz, Ts1kHz
float Kxp = 3.14;

//Spindle velocity PI control Bw:80Hz, Ts 1kHz
float Aspvpi[1] = { 1.00000000000000000000e+00 };
float Bspvpi[1] = { 1.25000000000000000000e-01 };
float Cspvpi[1] = { 1.07367070529084780000e-01 };
float Dspvpi[1] = { 1.55822995618053730000e+00 };

//Position PID: Bw 40Hz, Ts 2kHz
float Apid[2][2] = {
	{ 1.601349668864234, -0.601349668864234 },
	{ 1.000000000000000, 0.0000000000000000 },
};
float Bpid[2][1] = {
	{ 64.0000000000000 },
	{ 0.00000000000000 },
};
float Cpid[1][2] = {
	{ -18.408379271595891, 18.560148270902872 },
};
float Dpid[1][1] = {
	{ 3.755247742061939e+03 },
};


//Disturbance Observer: 100us sampling, nominal plant: 1/Js, LPF 2000 Hz 2 order
float	A_Q[4] = { 5.69219086672058670000e-01,-3.24010368631773180000e-01,2.50000000000000000000e-01,0.00000000000000000000e+00 };
float	B_Q[2] = { 1.00000000000000000000e+00,0.00000000000000000000e+00 };
float	C_Q[2] = { 3.57739556469564290000e-01,6.16175796065279950000e-01 };
float	D_Q[1] = { 0.00000000000000000000e+00 };

float	A_R[4] = { 5.69219086672058670000e-01,-3.24010368631773180000e-01,2.50000000000000000000e-01,0.00000000000000000000e+00 };
float	B_R[2] = { 4.00000000000000000000e+00,0.00000000000000000000e+00 };
float	C_R[2] = { -4.91534862079756610000e-04,-5.67367519264141770000e+00 };
float	D_R[1] = { 1.10899262505564950000e+01 };

//notch Filter for stage x control : resonance at 100 Hz, Bw:50Hz, Fs:1kHz, 2 order
float	A_notch_stage_x[4] = { 1.39680224666742080000e+00,-7.26542528005360900000e-01,1.00000000000000000000e+00,0.00000000000000000000e+00 };
float	B_notch_stage_x[2] = { 5.00000000000000000000e-01,0.00000000000000000000e+00 };
float	C_notch_stage_x[2] = { -3.81966011211908540000e-01,4.72135954952365600000e-01 };
float	D_notch_stage_x[1] = { 8.63271264016353350000e-01 };

//DOB for Stage x: FS:1000 Hz, Bw:40 Hz, 2 order
float	A_Q_STX[4] = { 1.55553535834357830000e+00,-6.04922562764271140000e-01,1.00000000000000000000e+00,0.00000000000000000000e+00 };
float	B_Q_STX[2] = { 2.50000000000000000000e-01,0.00000000000000000000e+00 };
float	C_Q_STX[2] = { 1.07031930645427710000e-01,9.05168870373438960000e-02 };
float	D_Q_STX[1] = { 0.00000000000000000000e+00 };

float	A_R_STX[4] = { 1.55553535834357830000e+00,-6.04922562764271140000e-01,1.00000000000000000000e+00,0.00000000000000000000e+00 };
float	B_R_STX[2] = { 1.00000000000000000000e+00,0.00000000000000000000e+00 };
float	C_R_STX[2] = { 6.14351906303447490000e-01,-5.37456189188626920000e-01 };
float	D_R_STX[1] = { 4.01045201153208910000e-01 };











