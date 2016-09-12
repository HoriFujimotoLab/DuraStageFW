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
/*      void bit_rev(float *x, int n);                                      */
/*                                                                          */
/*      x              : Array to be bit-reversed.                          */
/*      n              : Number of complex array elements to bit-reverse.   */
/*                                                                          */
/*  DESCRIPTION                                                             */
/*      This routine bit reverses the floating point array x which          */
/*      is considered to be an array of complex numbers with the even       */
/*      numbered elements being thr real parts of the complex numbers       */
/*      while the odd numbered elements being the imaginary parts of the    */
/*      complex numbers. This function is made use of in sp_icfftr2_dif     */
/*      to bit-reverse the twiddle factor array generated using             */
/*      tw_genr2fft.c.                                                      */
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
