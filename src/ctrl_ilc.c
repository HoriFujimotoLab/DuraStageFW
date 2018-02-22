/************************************************************************************
ILC MODULE
-------------------
Descr.:		c-code implementation of iterative learning control
Boards:		PE-Expert3, C6713A DSP (floating point calculation)
System:		Dura single axis ball-screw experimental setup
Author:		Shimoda Takaki, University of Tokyo, 2018
*************************************************************************************/

#include "ctrl_fir.h"
#include "ctrl_ilc.h"
#include "ctrl_ilc_par.h"
#include "system_data.h"

static float e_ILC[NDATAILC] = {0.0};
static float f_ILC[NDATAILC] = {0.0}; // sum of fILC, fFB, fE
static float f_ILC_next[NDATAILC] = {0.0};

void record_error_ilc(int i, float e)
{
    e_ILC[i] = e;
}

void record_force_ilc(int i, float f)
{
    f_ILC[i] += f;
}

void set_ilc()
{
    int i;
    for (i = 0; i < NDATAILC; i++)
    {
        f_ILC[i] += fir(i, MZEPETC, NPARZEPETC, NDATAILC, cZEPETC, e_ILC);
    }
    for (i = 0; i < NDATAILC; i++)
    {
        f_ILC_next[i] = fir(i, MILC, NPARILC, NDATAILC, cILC, f_ILC);
    }
    for (i = 0; i < NDATAILC; i++)
    {
        f_ILC[i] = f_ILC_next[i];
    }
}

float ilc(int i)
{
    return f_ILC[i];
}

void reset_ilc()
{
    int i;
    for (i = 0; i < NDATAILC; i++)
    {
        e_ILC[i] = 0;
        f_ILC[i] = 0;
        f_ILC_next[i] = 0;
    }
}
