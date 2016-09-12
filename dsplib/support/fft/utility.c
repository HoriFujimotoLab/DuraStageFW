/* ======================================================================== */
/*                                                                          */
/*  TEXAS INSTRUMENTS, INC.                                                 */
/*                                                                          */
/*  FILE                                                                    */
/*      utility.c                                                           */
/*                                                                          */
/*  FUNCTIONS                                                               */
/*      void tw_genSPxSPfft(float*w, int N)                                 */
/*      void tw_genr2fft(float*w, int N)                                    */
/*      void bit_rev(float* x, int n)                                       */
/*      void tw_genr4fft(float*w, int N)                                    */
/*      R4DigitRevIndexTableGen(int n, int * count,                         */
/*                    unsigned short *IIndex, unsigned short *JIndex)       */
/*      digit_reverse(double *yx, unsigned short *JIndex,                   */
/*              unsigned short *IIndex, int count)                          */
/*                                                                          */
/*  DESCRIPTION                                                             */
/*      utility functions used with single precision fft analysis.          */
/*                                                                          */
/* ======================================================================== */

#include<math.h>

#define PI 3.14159265358979323846

/* ======================================================================== */
/*                                                                          */
/*  TEXAS INSTRUMENTS, INC.                                                 */
/*                                                                          */
/*  NAME                                                                    */
/*      tw_genSPxSPfft                                                      */
/*                                                                          */
/*  USAGE                                                                   */
/*      This function has the prototype:                                    */
/*                                                                          */
/*      void tw_genSPxSPfft(float*w, int N)                                 */
/*                                                                          */
/*      w              : Pointer to twiddle factor array returned by the    */
/*                       routine. w should be of size 2*N floats.           */
/*      n              : Dimension of the FFT                               */
/*                                                                          */
/*  DESCRIPTION                                                             */
/*      This routine calculates the twiddle factor array for use in the     */
/*      sp_fftSPxSP   routine.                                              */
/*                                                                          */
/* ======================================================================== */
void tw_genSPxSPfft(float * w, int n)                                      
{                                                                  
     int i, j, k;                                                        
     double x_t, y_t, theta1, theta2, theta3;                         
                                                                       
     for (j=1, k=0; j <= n>>2; j = j<<2)                              
     {                                                                
         for (i=0; i < n>>2; i+=j)                                    
         {                                                            
             theta1 = 2*PI*i/n;                                       
             x_t = cos(theta1);                                       
             y_t = sin(theta1);                                       
             w[k]   =  (float)x_t;                                    
             w[k+1] =  (float)y_t;                                    
                                                                      
             theta2 = 4*PI*i/n;                                       
             x_t = cos(theta2);                                       
             y_t = sin(theta2);                                       
             w[k+2] =  (float)x_t;                                    
             w[k+3] =  (float)y_t;                                    
                                                                      
             theta3 = 6*PI*i/n;                                       
             x_t = cos(theta3);                                       
             y_t = sin(theta3);                                       
             w[k+4] =  (float)x_t;                                    
             w[k+5] =  (float)y_t;                                    
             k+=6;                                                    
         }                                                            
     }                                                                
}

/* ======================================================================== */
/*                                                                          */
/*  TEXAS INSTRUMENTS, INC.                                                 */
/*                                                                          */
/*  NAME                                                                    */
/*      tw_genr2fft                                                         */
/*                                                                          */
/*  USAGE                                                                   */
/*      This function has the prototype:                                    */
/*                                                                          */
/*      void tw_genr2fft(float*w, int N)                                    */
/*                                                                          */
/*      w              : Pointer to twiddle factor array returned by the    */
/*                       routine. w should be of size N floats.             */
/*      n              : Dimension of the FFT                               */
/*                                                                          */
/*  DESCRIPTION                                                             */
/*      This routine calculates the twiddle factor array for use in the     */
/*      tw_genr2fft routine.  After generation, the array still needs to    */
/*      be bit-reversed. (see bit_rev)                                      */
/*                                                                          */
/* ======================================================================== */
void tw_genr2fft(float* w, int n)                                      
{                                                              
   int i;                                                      
   float pi = 4.0*atan(1.0);                                   
   float e = pi*2.0/n;                                         
                                                               
    for(i=0; i < ( n>>1 ); i++)                                
    {                                                          
       w[2*i]   = cos(i*e);                                    
       w[2*i+1] = sin(i*e);                                    
    }                                                          
} 

/* ======================================================================== */
/*                                                                          */
/*  TEXAS INSTRUMENTS, INC.                                                 */
/*                                                                          */
/*  NAME                                                                    */
/*      bit_rev                                                             */
/*                                                                          */
/*  USAGE                                                                   */
/*      This function has the prototype:                                    */
/*                                                                          */
/*      void bit_rev(float* x, int n)                                       */
/*                                                                          */
/*      w              : Pointer to the input array                         */
/*      n              : Size of the array (number of complex floats)       */
/*                                                                          */
/*  DESCRIPTION                                                             */
/*      Used to bit reverse the given array. Used in the tw_genr2fft        */
/*      routine to bit_rev the twiddle factor array as well as to bit_rev   */
/*      the output.  NOTE:  Remember that the size of the twiddle factor    */
/*      array is N/2 complex floats, and the size of the output array is    */
/*      N complex floats (where N is the dimension of the fft).             */
/*                                                                          */
/* ======================================================================== */
void bit_rev(float* x, int n)                                     
{                                                              
  int i, j, k;                                                 
  float rtemp, itemp;                                          
                                                               
  j = 0;                                                       
  for(i=1; i < (n-1); i++)                                     
  {                                                            
     k = n >> 1;                                               
     while(k <= j)                                             
     {                                                         
        j -= k;                                                
        k >>= 1;                                               
     }                                                         
     j += k;                                                   
     if(i < j)                                                 
     {                                                         
        rtemp    = x[j*2];                                     
        x[j*2]   = x[i*2];                                     
        x[i*2]   = rtemp;                                      
        itemp    = x[j*2+1];                                   
        x[j*2+1] = x[i*2+1];                                   
        x[i*2+1] = itemp;                                      
     }                                                         
   }                                                           
} 


/* ======================================================================== */
/*                                                                          */
/*  TEXAS INSTRUMENTS, INC.                                                 */
/*                                                                          */
/*  NAME                                                                    */
/*      tw_genr4fft                                                         */
/*                                                                          */
/*  USAGE                                                                   */
/*      This function has the prototype:                                    */
/*                                                                          */
/*      void tw_genr4fft(float*w, int N)                                    */
/*                                                                          */
/*      w              : Pointer to twiddle factor array returned by the    */
/*                       routine. w should be of size (3/2)*N floats.       */
/*      n              : Dimension of the FFT                               */
/*                                                                          */
/*  DESCRIPTION                                                             */
/*      This routine calculates the twiddle factor array for use in the     */
/*      tw_genr4fft routine.                                                */
/*                                                                          */
/* ======================================================================== */            
void tw_genr4fft(float *w, int N)
{
    double delta = 2*3.14159265359/N;
    int i;
    for(i = 0; i < 3*N/4; i++)
    {
        w[2*i+1] = sin(i * delta);
        w[2*i] = cos(i * delta);
    }
}                                                           
              
/* ======================================================================== */
/*                                                                          */
/*  TEXAS INSTRUMENTS, INC.                                                 */
/*                                                                          */
/*  NAME                                                                    */
/*      R4DigitRevIndexTableGen                                             */
/*                                                                          */
/*  USAGE                                                                   */
/*      This function has the prototype:                                    */
/*                                                                          */
/*      R4DigitRevIndexTableGen(int n, int * count,                         */
/*                    unsigned short *IIndex, unsigned short *JIndex)       */
/*                                                                          */
/*      n              : Dimension of fft                                   */
/*      count          : Pointer to count                                   */
/*      IIndex         : Pointer to IIndex, of size 4*n                     */
/*      JIndex         : Pointer to JIndex, of size 4*n                     */
/*                                                                          */
/*  DESCRIPTION                                                             */
/*      Used to generate index tables for a digit reversal function         */
/*        for a radix-4 FFT.  Used in conjunction with digit_reverse.       */
/*                                                                          */
/* ======================================================================== */
void R4DigitRevIndexTableGen(int n, int * count, unsigned short *IIndex, unsigned short *JIndex)
{
 	int j, n1, k, i;
 	j = 1;
	n1 = n - 1;
	*count = 0;

	for(i=1; i<=n1; i++)
	{
		if(i < j)
		{
			IIndex[*count] = (unsigned short)(i-1);
			JIndex[*count] = (unsigned short)(j-1);
			*count = *count + 1;
		}
					  
		k = n >> 2;

		while(k*3 < j)
		{
			j = j - k*3;
			k = k >> 2;

		}
		j = j + k;
	}
}

/* ======================================================================== */
/*                                                                          */
/*  TEXAS INSTRUMENTS, INC.                                                 */
/*                                                                          */
/*  NAME                                                                    */
/*      digit_reverse                                                       */
/*                                                                          */
/*  USAGE                                                                   */
/*      This function has the prototype:                                    */
/*                                                                          */
/*      digit_reverse(double *yx, unsigned short *JIndex,                   */
/*              unsigned short *IIndex, int count)                          */
/*                                                                          */
/*      yx             : Array to be digit reverse                          */
/*      IIndex         : assigned in R4DigitRevIndexTableGen                */
/*      JIndex         : assigned in R4DigitRevIndexTableGen                */
/*      count          : assigned in R4DigitRevIndexTableGen                */
/*                                                                          */
/*  DESCRIPTION                                                             */
/*      Digit reversal function for a radix-4 FFT.  The output of the fft   */
/*         needs to be digit reversed.                                      */
/*                                                                          */
/* ======================================================================== */
void digit_reverse(double *yx, unsigned short *JIndex, unsigned short *IIndex, int count)
{   

   /* NOTE TRICK -- The value pointed at by yx (*yx) is 
      declared to be of size double... thus both the real and imaginary part
      of each pair of transformed value are exchanged at the same time! */

	int i;
	unsigned short I, J;
	double YXI, YXJ;

	for (i = 0; i<count; i++)
	{
		I = IIndex[i];
		J = JIndex[i];
		YXI = yx[I];
		YXJ = yx[J];
		yx[J] = YXI;
		yx[I] = YXJ;
	}

}

