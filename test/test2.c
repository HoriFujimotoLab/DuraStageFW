#include<stdio.h>

#define SPCNT2RADPS (0.766990393942821) // 2 * PI(1) /SPCNTPREV/(125*10^-6)
void test6(){
  int count_d=400;
  float omega;
  omega = (float) count_d*SPCNT2RADPS;
  printf("%f\n",omega);
}

