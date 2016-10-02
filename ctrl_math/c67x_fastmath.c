/************************************************************************************
FAST MATH LIBRARY
------------------
Descr.:		arithmetic module for simulation purposes
Author:		Thomas Beauduin, Wataru Ohnishi
			Hori-Fujimoto Lab, University of Tokyo, 2016
*************************************************************************************
IMPORTANT NOTICE:
THIS CODE IS JUST A COPY-PASTE FROM THE FASTMATH-LIB USER-MANUAL
AND IS ONLY INTENDED FOR SIL (SOFTWARE-IN-THE-LOOP) SIMULATION. 
IT SHOULD !NOT! BE INCLUDED IN MYWAY PROJECT (INSTEAD TI c67x DSP OPTIMIZED LIBRARY).
*************************************************************************************/
#include "c67x_fastmath.h"
#include <math.h>


float sinsp(float x)
{
	float y = 0.0;
	y = sinf(x);
	return y;
}

double sindp(double x)
{
	double y = 0.0;
	y = sin(x);
	return y;
}

float cossp(float x)
{
	float y = 0.0;
	y = cosf(x);
	return y;
}

