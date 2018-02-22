/************************************************************************************
FIR MODULE
-------------------
Descr.:		fir calculation for filtering
Boards:		PE-Expert3, C6713A DSP (floating point calculation)
System:		Dura single axis ball-screw experimental setup
Author:		Shimoda Takaki, University of Tokyo, 2018
*************************************************************************************/

#include "ctrl_fir.h"

float fir(int i, int m, int npar, int ndata, float c[], float u[])
{
    int j;       // index of c
    int k;       // index of input vector u
    float y = 0; //return

    for (j = 0; j < npar; j++)
    {
        k = i + m + j;
        if ((k >= 0) && (k < ndata))
        {
            y += c[j] * u[k];
        }
    }

    return y;
}
