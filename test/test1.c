#include<math.h>
#include <stdio.h>

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


#define Nd ((int) 2)
#define sigma_w (1e-2)
#define sigma_v (1e-8)

#define myu (0.1)
#define Ld ((int) 4)

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

#define INV2PITS (1.591549430918954e+03) //1/2/pi*10000

float dominant_freq(float b, float c)
{ 
  float discriminant;
  float  real, imag;
  float fchat;
  
  discriminant = b * b - 4 * 1.0 * c;

  if (discriminant < 0){ 
    real = -b * 0.50;
    imag = sqrtf(-discriminant) * 0.50; //sqrt(D)/2/a
    //z=u+j*w
    //log(z)=log(abs(z)) + j*atan2(w,u)
    //use atan2f(imag, real)
    fchat = atan2f(imag, real)*INV2PITS; //2/pi/ts
    return fchat;
  }

  return 2000.0; //default
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

void NLMS_filter(){
  float ahat;

  ctrl_matrix_prod(P_m, phi, kalman_g, Nd, 1, 1);
}

int main(){
  test3();
}
