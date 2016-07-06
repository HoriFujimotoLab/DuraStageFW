//MOTOR CTRL PARAMETERS:
//FOR DURA

//Zero cancell FF
//no FF
float	Affi = 0.0;
float	Bffi = 0.0;
float	Cffi = 0.0;
float	Dffi = 1.0;

//Pole-Place PI
//Bw:191 Hz(1200 rad/s), Ts: 10 kHz (100us)
//電流PI制御器と速度PI制御器の各行列

float	Afbi = 1.00000000000000000000e+00;
float	Bfbi = 5.00000000000000000000e-01;
float	Cfbi = 6.03185789489240070000e-01;
float	Dfbi = 3.76991118430775220000e+00;

//Dead-Time SP
float	ihys	=	5.0000000000e-02;
float	vhys	=	2.8800000000e-02;

