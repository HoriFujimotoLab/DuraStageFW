//MOTION CTRL PAR:
//Velocity PI: Bw 80Hz, Ts 2 kHz
float Avpi[2][1] = { { 1.00000000000000000000e+00 },{ 1.00000000000000000000e+00 } };
float Bvpi[2][1] = { { 5.00000000000000000000e-01 },{ 5.00000000000000000000e-01 } };
float Cvpi[2][1] = { { 4.92109618464414920000e-01 },{ 4.92109618464414920000e-01 } };
float Dvpi[2][1] = { { 3.51506870331725140000e+00 },{ 3.51506870331725140000e+00 } };

//Postion P based on Velocity PI, Bw:0.5 Hz, Ts2kHz
float Kxp = 3.14;

//Spindle velocity PI control Bw:80Hz, Ts 2kHz
float Aspvpi[1] = { 1.00000000000000000000e+00 };
float Bspvpi[1] = { 6.25000000000000000000e-02 };
float Cspvpi[1] = { 1.07367070529083010000e-01 };
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


//Disturbance Observer
float DOB_A = 2.078796e-01;
float DOB_B = 7.921204e-01;
float DOB_C = 2.078796e-01;
float DOB_D = 2.373395e+05;
float DOB_E = 2.373394e+05;






