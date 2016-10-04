//MOTION CTRL PAR:
//Velocity PI: Bw 80Hz, Ts 1 kHz
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


//Disturbance Observer: 125us sampling, nominal plant: 1/Js, LPF 2000 Hz
float DOB_A = 2.078796e-01;
float DOB_B = 7.921204e-01;
float DOB_C = 2.078796e-01;
float DOB_D = 1.964459e+01;
float DOB_E = 1.964459e+01;










