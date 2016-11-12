//MOTION CTRL PAR:
//Velocity PI: Bw 40Hz, Ts 1 kHz
float Avpi[2][1] = { { 1.00000000000000000000e+00 },{ 1.00000000000000000000e+00 } };
float Bvpi[2][1] = { { 5.00000000000000000000e-01 },{ 5.00000000000000000000e-01 } };
float Cvpi[2][1] = { { 9.84219236928830730000e-01 },{ 9.84219236928830730000e-01 } };
float Dvpi[2][1] = { { 3.51506870331725140000e+00 },{ 3.51506870331725140000e+00 } };


//Postion P based on Velocity PI, Bw:1 Hz, Ts1kHz
float Kxp = PI(2);

//Spindle velocity PI control Bw:40Hz, Ts 1kHz
float Aspvpi[1] = { 1.00000000000000000000e+00 };
float Bspvpi[1] = { 6.25000000000000000000e-02 };
float Cspvpi[1] = { 1.20637157897849790000e-01 };
float Dspvpi[1] = { 1.25663706143591720000e+00 };

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

//Disturbance Observer: 250us sampling, nominal plant: 1/(Ms+D), LPF 2000 Hz 2 order
float	A_Q[4] = { 8.64278365275446270000e-02,-5.97581674146558180000e-02,3.12500000000000000000e-02,0.00000000000000000000e+00 };
float	B_Q[2] = { 2.00000000000000000000e+00,0.00000000000000000000e+00 };
float	C_Q[2] = { 4.10512776792965220000e-01,1.51062484189171830000e+00 };
float	D_Q[1] = { 0.00000000000000000000e+00 };

float	A_R[4] = { 8.64278365275446270000e-02,-5.97581674146558180000e-02,3.12500000000000000000e-02,0.00000000000000000000e+00 };
float	B_R[2] = { 4.09600000000000000000e+03,0.00000000000000000000e+00 };
float	C_R[2] = { -5.99206421126804000000e+02,-2.81075316532771510000e+03 };
float	D_R[1] = { 3.09777771972444510000e+06 };


//HPF and LPF for band path filter, Fs:4000 Hz, bwlpf:1000 Hz, bwhpf:2000.
float	A_HPF2[4] = { -4.55280381897048340000e-01,-2.07280226140322180000e-01,2.50000000000000000000e-01,0.00000000000000000000e+00 };
float	B_HPF2[2] = { 1.00000000000000000000e+00,0.00000000000000000000e+00 };
float	C_HPF2[2] = { -3.66168040040502190000e-01,5.65626954972870850000e-01 };
float	D_HPF2[1] = { 1.49134918659508060000e-01 };

float	A_LPF2[4] = { 2.28743344294052410000e-01,-1.04647035117654800000e-01,1.25000000000000000000e-01,0.00000000000000000000e+00 };
float	B_LPF2[2] = { 1.00000000000000000000e+00,0.00000000000000000000e+00 };
float	C_LPF2[2] = { 4.37021765256110860000e-01,1.54815542079649720000e+00 };
float	D_LPF2[1] = { 1.96084383773913710000e-01 };

//2000 Hz notch filter for DOB, Fs:10 kz, interference: 2000 Hz, bw:10 Hz
float	A_notch_2000[4] = { 6.16098452011502600000e-01,-9.93736471541614490000e-01,1.00000000000000000000e+00,0.00000000000000000000e+00 };
float	B_notch_2000[2] = { 1.25000000000000000000e-01,0.00000000000000000000e+00 };
float	C_notch_2000[2] = { -1.54358007478219150000e-02,4.99513005070939540000e-02 };
float	D_notch_2000[1] = { 9.96868235771120380000e-01 };

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











