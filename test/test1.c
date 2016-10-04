//#include<math.h>
#include <stdio.h>

#include "DSPF_sp_vecmul.h"
#include "DSPF_sp_dotprod.h"
#include "atan2sp_c.h"
#include "divsp_c.h"
#include "recipsp_c.h"
#include "sqrtsp_c.h"

void ctrl_matrix_add(float a[], float b[], float *c, int row, int col)
{
	int i, j;
	for (i = 0; i < row; i++){
		for (j = 0; j < col; j++){
			c[i*col + j] = a[i*col + j] + b[i*col + j];
		}
	}
}

void ctrl_matrix_minus(float a[], float b[], float *c, int row, int col)
{
	int i, j;
	for (i = 0; i < row; i++){
		for (j = 0; j < col; j++){
			c[i*col + j] = a[i*col + j] - b[i*col + j];
		}
	}
}

void ctrl_matrix_prod(float a[], float b[], float *c, int row_a, int col_a, int col_b)
{
  int i, j, k;
  for (i = 0; i < row_a; i++){
    for (j = 0; j < col_b; j++){
      float sum = 0;
      for (k = 0; k < col_a; k++){
	sum += a[i*col_a + k] * b[k*col_b + j]; 
      }
      c[i*col_b + j] = sum;
    }
  }
}

void eye_matrix(float *e, int n, float value) {
  int i, j;
  for (i = 0; i < n; i++) {
    for (j = 0; j < n; j++) {
      if (i == j) {
	e[i*n + j] = value;
      }
      else {
	e[i*n + j] = 0;
      }
    }
  }
}

void ctrl_matrix_cnst_mlpy(float *c, float p, int row_c, int col_c){
  int i, j;
  for (i=0; i < row_c; i++){
    for(j=0; j<col_c; j++){
      c[row_c*i+j] *= p;
    }
  } 
}

void print_matrix(float *c, int row_c, int col_c){
  int i, j;
  for (i=0; i < row_c; i++){
    printf("\t");
    for(j=0; j<col_c; j++){
      printf("%f\t", c[row_c*i+j]);
    }
    printf("\n");
  } 
}

#define Nd ((int) 2)
#define sigma_w (1e-2)
#define sigma_v (1e-8)

#define myu (0.1)
#define Ld ((int) 4)
#define INV2PITS (1.591549430918954e+03) //1/2/pi*10000

float dominant_freq(float b, float c)
{
	float  real, imag;
	float fchat;

	float discriminant = b * b - 4.0* 1.0 * c;

	if (discriminant < 0) {
		real = -b *0.50;
		imag = sqrtsp_c(-discriminant) *0.50; //sqrt(D)/2/a
										   //z=u+j*w
										   //log(z)=log(abs(z)) + j*atan2(w,u)
										   //use atan2f(imag, real)
		fchat = atan2sp_c(imag, real)*INV2PITS; //2/pi/ts*FC
		return fchat;
	}

	return 2000.0; //default
}

void test1(){
  float a[]={1, 0, 0, 1}, b[]={2, 1}; //eye(2)
  int row_a=2,col_a=2, row_b=2, col_b=1;

  float c[row_a*col_b];

  ctrl_matrix_prod(a,b,c,row_a,col_a,col_b);

  int i;
  for (i=0; i < (row_a*col_b); i++){
    printf("c[%d] : %f\n",i, c[i]);
  }
}

void test2(){
  int n=3;
  float c[n*n];

  eye_matrix(c,n, 1);
  print_matrix(c, n ,n);

}



//in: phi[k], y[k], theta[k-1], P[k-1];
//out: theta[k], P[k]
void kalman_filter(float *phi, float y, float *theta, float *P) {
	float  P_m[Nd*Nd], kalman_g[Nd], fm[1]; 
	float temp1[Nd*Nd], temp2[Nd], temp3[1], temp4[Nd*Nd], temp5[Nd*Nd];

	eye_matrix(temp1, Nd, sigma_v*sigma_v);
	ctrl_matrix_add(P, temp1, P_m, Nd, Nd); //P^-

	ctrl_matrix_prod(P_m, phi, kalman_g, Nd, Nd, 1);
	ctrl_matrix_prod(phi, kalman_g, temp3, 1, Nd, 1);
	ctrl_matrix_cnst_mlpy(kalman_g, 1 / (temp3[0] + sigma_w*sigma_w), Nd, 1); //kalman gain

	ctrl_matrix_prod(phi, theta, temp3, 1, Nd, 1);
	fm[0] = y - temp3[0];
	ctrl_matrix_prod(kalman_g, fm, temp2, Nd, 1, 1);
	ctrl_matrix_add(theta, temp2, theta, Nd, 1); //update theta

	ctrl_matrix_prod(kalman_g, phi, temp1, Nd, 1, Nd);
	eye_matrix(temp5, Nd, 1);
	ctrl_matrix_minus(temp5, temp1, temp4, Nd, Nd);
	ctrl_matrix_prod(temp4, P_m, P, Nd, Nd, Nd); //update P
}



void test3(){
  float phi[2]={1, 0};
  float y=2;

  float theta[2]={0, 0};
  float P[4]={1, 0, 0 ,1};
  
  kalman_filter(phi, y, theta, P);
  
  print_matrix(theta, 2, 1);
  print_matrix(P, 2, 2);

  float fchat;
  fchat = dominant_freq(theta[0], theta[1]);
  printf("fchat = %f",fchat);
}

void test4(){
  float fchat;
  fchat = dominant_freq(-0.2, 1);
  printf("fchat = %f",fchat);
}

void test5(){
  float a[2]={1, 2}, b[2]={3, 1};
  float c[1];

  ctrl_matrix_prod(a,b,c,1,2,1);
  print_matrix(c,1,1);
}




void kalman_filter_2(float *phi, float y, float *theta, float *P) {
	float  kalman_g[Nd], fm; 
	float temp1[Nd*Nd], temp5[Nd*Nd], temp6[Nd], temp7;
	int i;

	//eye_matrix(temp1, Nd, sigma_v*sigma_v);
	//ctrl_matrix_add(P, temp1, P_m, Nd, Nd); //P^-
	//P_m = P, now sigma = 0

	ctrl_matrix_prod(P, phi, kalman_g, Nd, Nd, 1); //[Nd, Nd], [Nd, 1]
	temp7 = recipsp_c(DSPF_sp_dotprod(phi, kalman_g, Nd) + sigma_w*sigma_w);
	for (i = 0; i < Nd; i++) {
		temp6[i] = temp7;
	}
	DSPF_sp_vecmul(kalman_g, temp6, kalman_g, Nd); //calculate kalman gain

	fm = y - DSPF_sp_dotprod(phi, theta, Nd);
	for (i = 0; i < Nd; i++) {
		temp6[i] = fm;
	}
	DSPF_sp_vecmul(kalman_g, temp6, temp6, Nd);
	ctrl_matrix_add(theta, temp6, theta, Nd, 1); //update theta

	ctrl_matrix_prod(kalman_g, phi, temp1, Nd, 1, Nd);
	eye_matrix(temp5, Nd, 1);
	ctrl_matrix_minus(temp5, temp1, temp5, Nd, Nd);
	ctrl_matrix_prod(temp5, P, P, Nd, Nd, Nd); //update P
}

void test7(){
  float phi[2]={1, 0};
  float y=2;

  float theta[2]={0, 0};
  float P[4]={1, 0, 0 ,1};

  float fchat;
  
  kalman_filter(phi, y, theta, P);
  
  print_matrix(theta, 2, 1);
  print_matrix(P, 2, 2);

  fchat = dominant_freq(theta[0], theta[1]);
  printf("fchat = %f",fchat);

  float phi2[2]={1, 0};
  float y2=2;

  float theta2[2]={0, 0};
  float P2[4]={1, 0, 0 ,1};
  
  kalman_filter_2(phi2, y2, theta2, P2);
  
  print_matrix(theta2, 2, 1);
  print_matrix(P2, 2, 2);

  fchat = dominant_freq(theta2[0], theta2[1]);
  printf("fchat = %f",fchat);

}

int main(){
  printf("s");
  test7();
}
