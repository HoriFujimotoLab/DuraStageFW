/* ======================================================================= */
/* TEXAS INSTRUMENTS, INC.                                                 */
/*                                                                         */
/* NAME                                                                    */
/*     DSPF_sp_ifftSPxSP -- Single Precision floating point mixed radix         */
/*     inverse FFT with complex input                                      */
/*                                                                         */
/*  USAGE                                                                   */
/*      This routine is C-callable and can be called as:                    */
/*                                                                          */
/*                                                                          */
/*      void DSPF_sp_ifftSPxSP                                                   */
/*      (                                                                   */
/*          int N,                                                          */
/*          float * ptr_x,                                                  */
/*          float * ptr_w,                                                  */
/*          float * ptr_y,                                                  */
/*          unsigned char * brev,                                           */
/*          int n_min,                                                      */
/*          int offset,                                                     */
/*          int n_max                                                       */
/*      );                                                                  */
/*                                                                          */
/*      N      :  Length of ifft in complex samples, power of 2 such that   */
/*                N >=8 and N<= 16385.                                      */
/*      ptr_x  :  Pointer to complex data input (normal order).             */
/*      ptr_w  :  Pointer to complex twiddle factor (see below).            */
/*      ptr_y  :  Pointer to complex output data (normal order).            */
/*      brev   :  Pointer to bit reverse table containing 64 entries.       */
/*      n_min  :  Smallest ifft butterfly used in computation used for      */
/*                decomposing ifft into sub iffts, see notes.               */
/*      offset :  Index in complex samples of sub-ifft from start of main   */
/*                ifft.                                                     */
/*      n_max  :  size of main ifft in complex samples.                     */
/*                                                                          */
/*                                                                          */
/*  DESCRIPTION                                                             */
/*                                                                          */
/*      The benchmark performs a mixed radix forwards ifft using a special  */
/*      sequence of coefficients generated in the following way:            */
/*                                                                          */
/*      // generate vector of twiddle factors for optimized algorithm //    */
/*      void tw_gen(float * w, int N)                                       */
/*      {                                                                   */
/*          int j, k;                                                       */
/*          double x_t, y_t, theta1, theta2, theta3;                        */
/*          const double PI = 3.141592654;                                  */
/*                                                                          */
/*          for (j=1, k=0; j <= N>>2; j = j<<2)                             */
/*          {                                                               */
/*              for (i=0; i < N>>2; i+=j)                                   */
/*              {                                                           */
/*                  theta1 = 2*PI*i/N;                                      */
/*                  x_t = cos(theta1);                                      */
/*                  y_t = sin(theta1);                                      */
/*                  w[k]   =  (float)x_t;                                   */
/*                  w[k+1] =  (float)y_t;                                   */
/*                                                                          */
/*                  theta2 = 4*PI*i/N;                                      */
/*                  x_t = cos(theta2);                                      */
/*                  y_t = sin(theta2);                                      */
/*                  w[k+2] =  (float)x_t;                                   */
/*                  w[k+3] =  (float)y_t;                                   */
/*                                                                          */
/*                  theta3 = 6*PI*i/N;                                      */
/*                  x_t = cos(theta3);                                      */
/*                  y_t = sin(theta3);                                      */
/*                  w[k+4] =  (float)x_t;                                   */
/*                  w[k+5] =  (float)y_t;                                   */
/*                  k+=6;                                                   */
/*               }                                                          */
/*          }                                                               */
/*      }                                                                   */
/*                                                                          */
/*      This redundant set of twiddle factors is size 2*N float samples.    */
/*      The function is accurate to about 130dB of signal to noise ratio    */
/*      to the DFT function below:                                          */
/*                                                                          */
/*      void dft(int N, float x[], float y[])                               */
/*      {                                                                   */
/*          int k,i, index;                                                 */
/*          const float PI = 3.14159654;                                    */
/*          float * p_x;                                                    */
/*          float arg, fx_0, fx_1, fy_0, fy_1, co, si;                      */
/*                                                                          */
/*          for(k = 0; k<N; k++)                                            */
/*          {                                                               */
/*              p_x = x;                                                    */
/*              fy_0 = 0;                                                   */
/*              fy_1 = 0;                                                   */
/*              for(i=0; i<N; i++)                                          */
/*              {                                                           */
/*                  fx_0 = p_x[0];                                          */
/*                  fx_1 = p_x[1];                                          */
/*                  p_x += 2;                                               */
/*                  index = (i*k) % N;                                      */
/*                  arg = 2*PI*index/N;                                     */
/*                  co = cos(arg);                                          */
/*                  si = -sin(arg);                                         */
/*                  fy_0 += ((fx_0 * co) - (fx_1 * si));                    */
/*                  fy_1 += ((fx_1 * co) + (fx_0 * si));                    */
/*              }                                                           */
/*              y[2*k] = fy_0;                                              */
/*              y[2*k+1] = fy_1;                                            */
/*          }                                                               */
/*      }                                                                   */
/*                                                                          */
/*      The function takes the table and input data and calculates the      */
/*      ifft producing the time domain data in the Y array. The output is   */
/*      scaled by a scaling factor of 1/N.                                  */
/*                                                                          */
/*      As the ifft allows every input point to effect every output point   */
/*      in a cache based system such as the c6711, this causes cache        */
/*      thrashing. This is mitigated by allowing the main ifft of size N    */
/*      to be divided into several steps, allowing as much data reuse as    */
/*      possible.                                                           */
/*                                                                          */
/*      For example the following function:                                 */
/*                                                                          */
/*      DSPF_sp_ifftSPxSP(1024, &x[0],&w[0],y,brev,4,  0,1024);                  */
/*                                                                          */
/*      is equvalent to:                                                    */
/*                                                                          */
/*      DSPF_sp_ifftSPxSP(1024,&x[2*0],  &w[0] ,   y,brev,256,  0,1024;          */
/*      DSPF_sp_ifftSPxSP(256, &x[2*0],  &w[2*768],y,brev,4,    0,1024;          */
/*      DSPF_sp_ifftSPxSP(256, &x[2*256],&w[2*768],y,brev,4,  256,1024;          */
/*      DSPF_sp_ifftSPxSP(256, &x[2*512],&w[2*768],y,brev,4,  512,1024;          */
/*      DSPF_sp_ifftSPxSP(256, &x[2*768],&w[2*768],y,brev,4,  768,1024;          */
/*                                                                          */
/*      Notice how the 1st ifft function is called on the entire 1K data    */
/*      set it covers the 1st pass of the ifft until the butterfly size is  */
/*      256. The following 4 iffts do 256 pt iffts 25% of the size. These   */
/*      continue down to the end when the buttefly is of size 4. They use   */
/*      an index the main twiddle factor array of 0.75*2*N. This is         */
/*      because the twiddle factor array is composed of successively        */
/*      decimated versions of the main array.                               */
/*                                                                          */
/*      N not equal to a power of 4 can be used, i.e. 512. In this case to  */
/*      decompose the ifft the following would be needed :                  */
/*                                                                          */
/*      DSPF_sp_ifftSPxSP(512, &x[0],&w[0],y,brev,2,  0,512);                    */
/*                                                                          */
/*      is equvalent to:                                                    */
/*                                                                          */
/*      DSPF_sp_ifftSPxSP(512, &x[2*0],  &w[0] ,   y,brev,128,  0,512)           */
/*      DSPF_sp_ifftSPxSP(128, &x[2*0],  &w[2*384],y,brev,4,    0,512)           */
/*      DSPF_sp_ifftSPxSP(128, &x[2*128],&w[2*384],y,brev,4,  128,512)           */
/*      DSPF_sp_ifftSPxSP(128, &x[2*256],&w[2*384],y,brev,4,  256,512)           */
/*      DSPF_sp_ifftSPxSP(128, &x[2*384],&w[2*384],y,brev,4,  384,512)           */
/*                                                                          */
/*      The twiddle factor array is composed of log4(N) sets of twiddle     */
/*      factors, (3/4)*N, (3/16)*N, (3/64)*N, etc.  The index into this     */
/*      array for each stage of the ifft is calculated by summing these     */
/*      indices up appropriately.                                           */
/*                                                                          */
/*      For multiple iffts they can share the same table by calling the     */
/*      small iffts from further down in the twiddle factor array. In the   */
/*      same way as the decomposition works for more data reuse.            */
/*                                                                          */
/*      Thus, the above decomposition can be summarized for a general N,    */
/*      radix "rad" as follows:                                             */
/*                                                                          */
/*      DSPF_sp_ifftSPxSP(N,  &x[0],      &w[0],      y,brev,N/4,0,    N)        */
/*      DSPF_sp_ifftSPxSP(N/4,&x[0],      &w[2*3*N/4],y,brev,rad,0,    N)        */
/*      DSPF_sp_ifftSPxSP(N/4,&x[2*N/4],  &w[2*3*N/4],y,brev,rad,N/4,  N)        */
/*      DSPF_sp_ifftSPxSP(N/4,&x[2*N/2],  &w[2*3*N/4],y,brev,rad,N/2,  N)        */
/*      DSPF_sp_ifftSPxSP(N/4,&x[2*3*N/4],&w[2*3*N/4],y,brev,rad,3*N/4,N)        */
/*                                                                          */
/*      As discussed previously, N can be either a power of 4 or 2.         */
/*      If N is a power of 4, then rad = 4, and if N is a power of 2 and    */
/*      not a power of 4, then rad = 2. "rad" is used to control how many   */
/*      stages of decomposition are performed. It is also used to           */
/*      determine whether a radix-4 or radix-2 decomposition should be      */
/*      performed at the last stage. Hence when "rad" is set to "N/4" the   */
/*      first stage of the transform alone is performed and the code        */
/*      exits. To complete the IFFT, four other calls are required to       */
/*      perform N/4 size IFFTs. In fact, the ordering of these 4 IFFTs      */
/*      amongst themselves does not matter and hence from a cache           */
/*      perspective, it helps to go through the remaining 4 IFFTs in        */
/*      exactly the opposite order to the first. This is illustrated as     */
/*      follows:                                                            */
/*                                                                          */
/*      DSPF_sp_ifftSPxSP(N,  &x[0],      &w[0],      y,brev,N/4,0,    N)        */
/*      DSPF_sp_ifftSPxSP(N/4,&x[2*3*N/4],&w[2*3*N/4],y,brev,rad,3*N/4,N)        */
/*      DSPF_sp_ifftSPxSP(N/4,&x[2*N/2],  &w[2*3*N/4],y,brev,rad,N/2,  N)        */
/*      DSPF_sp_ifftSPxSP(N/4,&x[2*N/4],  &w[2*3*N/4],y,brev,rad,N/4,  N)        */
/*      DSPF_sp_ifftSPxSP(N/4,&x[0],      &w[2*3*N/4],y,brev,rad,0,    N)        */
/*                                                                          */
/*      In addition this function can be used to minimize call overhead,    */
/*      by completing the IFFT with one function call invocation as shown   */
/*      below:                                                              */
/*                                                                          */
/*      DSPF_sp_ifftSPxSP(N,  &x[0],      &w[0],      y, brev, rad, 0, N)        */
/*                                                                          */
/*                                                                          */
/*  TECHNIQUES                                                              */
/*                                                                          */
/*      1. A special sequence of coeffs. used as generated above            */
/*         produces the ifft. This collapses the inner 2 loops in the       */
/*         taditional Burrus and Parks implementation Fortran Code.         */
/*                                                                          */
/*      2. The revised IFFT uses a redundant sequence of twiddle            */
/*         factors to allow a linear access through the data. This linear   */
/*         access enables data and instruction level parallelism.           */
/*                                                                          */
/*      3.The data produced by the ifftSPxSP ifft is in normal form, the    */
/*         whole data array is written into a new output buffer.            */
/*                                                                          */
/*      4. The ifftSPxSP butterfly is bit reversed, i.e. the inner 2        */
/*         points of the butterfly are corssed over, this has the effect    */
/*         of making the data come out in bit reversed rather than          */
/*         ifftSPxSP digit reversed order. This simplifies the last pass    */
/*         of the loop. A simple table is used to do the bit reversal out   */
/*         of place.                                                        */
/*                                                                          */
/*      unsigned char brev[64] =                                            */
/*      {                                                                   */
/*          0x0, 0x20, 0x10, 0x30, 0x8, 0x28, 0x18, 0x38,                   */
/*          0x4, 0x24, 0x14, 0x34, 0xc, 0x2c, 0x1c, 0x3c,                   */
/*          0x2, 0x22, 0x12, 0x32, 0xa, 0x2a, 0x1a, 0x3a,                   */
/*          0x6, 0x26, 0x16, 0x36, 0xe, 0x2e, 0x1e, 0x3e,                   */
/*          0x1, 0x21, 0x11, 0x31, 0x9, 0x29, 0x19, 0x39,                   */
/*          0x5, 0x25, 0x15, 0x35, 0xd, 0x2d, 0x1d, 0x3d,                   */
/*          0x3, 0x23, 0x13, 0x33, 0xb, 0x2b, 0x1b, 0x3b,                   */
/*          0x7, 0x27, 0x17, 0x37, 0xf, 0x2f, 0x1f, 0x3f                    */
/*      };                                                                  */
/*                                                                          */
/*                                                                          */
/*  ASSUMPTIONS                                                             */
/*      1. N must be a power of 2 and N >= 8  N <= 16384 points.            */
/*                                                                          */
/*      2. Complex time data x and twiddle facotrs w are aligned on         */
/*         double word boundares. Real values are stored in even word       */
/*         positions and imaginary values in odd positions.                 */
/*                                                                          */
/*      3. All data is in single precision floating point format. The       */
/*         complex frequency data will be returned in linear order.         */
/*                                                                          */
/*      4. This code is interupt tolerant, interupts are disabled on        */
/*         entry to each loop and reanlbed on exit out of the loop.         */
/*                                                                          */
/*      5. x must be padded with 16 words at the end.                       */
/*                                                                          */
/*                                                                          */
/*  C CODE                                                                  */
/*                                                                          */
/*      This is the C equivalent of the assembly code without               */
/*      restrictions.                                                       */
/*                                                                          */
/*      Note that the assembly code is hand optimized and restrictions may  */
/*      apply.                                                              */
/*                                                                          */
/*      void DSPF_sp_ifftSPxSP                                                   */
/*      (                                                                   */
/*          int N,                                                          */
/*          float * ptr_x,                                                  */
/*          float * ptr_w,                                                  */
/*          float * ptr_y,                                                  */
/*          unsigned char * brev,                                           */
/*          int n_min,                                                      */
/*          int offset,                                                     */
/*          int n_max                                                       */
/*      );                                                                  */
/*      {                                                                   */
/*          int  i, j, k, l1, l2, h2, predj;                                */
/*          int  tw_offset, stride, ifft_jmp;                               */
/*                                                                          */
/*          float x0, x1, x2, x3,x4,x5,x6,x7;                               */
/*          float xt0, yt0, xt1, yt1, xt2, yt2, yt3;                        */
/*          float yt4, yt5, yt6, yt7;                                       */
/*          float si1,si2,si3,co1,co2,co3;                                  */
/*          float xh0,xh1,xh20,xh21,xl0,xl1,xl20,xl21;                      */
/*          float x_0, x_1, x_l1, x_l1p1, x_h2 , x_h2p1, x_l2, x_l2p1;      */
/*          float xl0_0, xl1_0, xl0_1, xl1_1;                               */
/*          float xh0_0, xh1_0, xh0_1, xh1_1;                               */
/*          float *x,*w;                                                    */
/*          int   k0, k1, j0, j1, l0, radix;                                */
/*          float * y0, * ptr_x0, * ptr_x2;                                 */
/*                                                                          */
/*          radix = n_min;                                                  */
/*                                                                          */
/*          stride = N; // N is the number of complex samples //            */
/*          tw_offset = 0;                                                  */
/*          while (stride > radix)                                          */
/*          {                                                               */
/*              j = 0;                                                      */
/*              ifft_jmp = stride + (stride>>1);                            */
/*              h2 = stride>>1;                                             */
/*              l1 = stride;                                                */
/*              l2 = stride + (stride>>1);                                  */
/*              x = ptr_x;                                                  */
/*              w = ptr_w + tw_offset;                                      */
/*                                                                          */
/*              for (i = 0; i < N; i += 4)                                  */
/*              {                                                           */
/*                  co1 = w[j];                                             */
/*                  si1 = w[j+1];                                           */
/*                  co2 = w[j+2];                                           */
/*                  si2 = w[j+3];                                           */
/*                  co3 = w[j+4];                                           */
/*                  si3 = w[j+5];                                           */
/*                                                                          */
/*                  x_0    = x[0];                                          */
/*                  x_1    = x[1];                                          */
/*                  x_h2   = x[h2];                                         */
/*                  x_h2p1 = x[h2+1];                                       */
/*                  x_l1   = x[l1];                                         */
/*                  x_l1p1 = x[l1+1];                                       */
/*                  x_l2   = x[l2];                                         */
/*                  x_l2p1 = x[l2+1];                                       */
/*                                                                          */
/*                  xh0  = x_0    + x_l1;                                   */
/*                  xh1  = x_1    + x_l1p1;                                 */
/*                  xl0  = x_0    - x_l1;                                   */
/*                  xl1  = x_1    - x_l1p1;                                 */
/*                                                                          */
/*                  xh20 = x_h2   + x_l2;                                   */
/*                  xh21 = x_h2p1 + x_l2p1;                                 */
/*                  xl20 = x_h2   - x_l2;                                   */
/*                  xl21 = x_h2p1 - x_l2p1;                                 */
/*                                                                          */
/*                  ptr_x0 = x;                                             */
/*                  ptr_x0[0] = xh0 + xh20;                                 */
/*                  ptr_x0[1] = xh1 + xh21;                                 */
/*                                                                          */
/*                  ptr_x2 = ptr_x0;                                        */
/*                  x += 2;                                                 */
/*                  j += 6;                                                 */
/*                  predj = (j - ifft_jmp);                                 */
/*                  if (!predj) x += ifft_jmp;                              */
/*                  if (!predj) j = 0;                                      */
/*                                                                          */
/*                  xt0 = xh0 - xh20;                                       */
/*                  yt0 = xh1 - xh21;                                       */
/*                  xt1 = xl0 - xl21;                                       */
/*                  yt2 = xl1 - xl20;                                       */
/*                  xt2 = xl0 + xl21;                                       */
/*                  yt1 = xl1 + xl20;                                       */
/*                                                                          */
/*                  ptr_x2[l1  ] = xt1 * co1 - yt1 * si1;                   */
/*                  ptr_x2[l1+1] = yt1 * co1 + xt1 * si1;                   */
/*                  ptr_x2[h2  ] = xt0 * co2 - yt0 * si2;                   */
/*                  ptr_x2[h2+1] = yt0 * co2 + xt0 * si2;                   */
/*                  ptr_x2[l2  ] = xt2 * co3 - yt2 * si3;                   */
/*                  ptr_x2[l2+1] = yt2 * co3 + xt2 * si3;                   */
/*              }                                                           */
/*                                                                          */
/*              tw_offset += ifft_jmp;                                      */
/*              stride = stride>>2;                                         */
/*          }// end while //                                                */
/*                                                                          */
/*          j = offset>>2;                                                  */
/*                                                                          */
/*          ptr_x0 = ptr_x;                                                 */
/*          y0 = ptr_y;                                                     */
/*          //l0 = _norm(n_max) - 17;    get size of ifft //                */
/*          l0=0;                                                           */
/*                                                                          */
/*          for(k=30;k>=0;k--)                                              */
/*              if( (n_max & (1 << k)) == 0 )                               */
/*                  l0++;                                                   */
/*          else                                                            */
/*              break;                                                      */
/*                                                                          */
/*          l0=l0-17;                                                       */
/*          if (radix <= 4) for (i = 0; i < N; i += 4)                      */
/*          {                                                               */
/*              // reversal computation //                                  */
/*              j0 = (j     ) & 0x3F;                                       */
/*              j1 = (j >> 6);                                              */
/*              k0 = brev[j0];                                              */
/*              k1 = brev[j1];                                              */
/*              k = (k0 << 6) +  k1;                                        */
/*              k = k >> l0;                                                */
/*              j++;        // multiple of 4 index //                       */
/*                                                                          */
/*              x0   = ptr_x0[0];  x1 = ptr_x0[1];                          */
/*              x2   = ptr_x0[2];  x3 = ptr_x0[3];                          */
/*              x4   = ptr_x0[4];  x5 = ptr_x0[5];                          */
/*              x6   = ptr_x0[6];  x7 = ptr_x0[7];                          */
/*              ptr_x0 += 8;                                                */
/*                                                                          */
/*              xh0_0  = x0 + x4;                                           */
/*              xh1_0  = x1 + x5;                                           */
/*              xh0_1  = x2 + x6;                                           */
/*              xh1_1  = x3 + x7;                                           */
/*                                                                          */
/*              if (radix == 2)                                             */
/*              {                                                           */
/*                  xh0_0 = x0;                                             */
/*                  xh1_0 = x1;                                             */
/*                  xh0_1 = x2;                                             */
/*                  xh1_1 = x3;                                             */
/*              }                                                           */
/*                                                                          */
/*              yt0  = xh0_0 + xh0_1;                                       */
/*              yt1  = xh1_0 + xh1_1;                                       */
/*              yt4  = xh0_0 - xh0_1;                                       */
/*              yt5  = xh1_0 - xh1_1;                                       */
/*                                                                          */
/*              xl0_0  = x0 - x4;                                           */
/*              xl1_0  = x1 - x5;                                           */
/*              xl0_1  = x2 - x6;                                           */
/*              xl1_1  = x3 - x7;                                           */
/*                                                                          */
/*              if (radix == 2)                                             */
/*              {                                                           */
/*                  xl0_0 = x4;                                             */
/*                  xl1_0 = x5;                                             */
/*                  xl1_1 = x6;                                             */
/*                  xl0_1 = x7;                                             */
/*              }                                                           */
/*                                                                          */
/*              yt2  = xl0_0 + xl1_1;                                       */
/*              yt3  = xl1_0 - xl0_1;                                       */
/*              yt6  = xl0_0 - xl1_1;                                       */
/*              yt7  = xl1_0 + xl0_1;                                       */
/*                                                                          */
/*              if (radix == 2)                                             */
/*              {                                                           */
/*                  yt7  = xl1_0 - xl0_1;                                   */
/*                  yt3  = xl1_0 + xl0_1;                                   */
/*              }                                                           */
/*                                                                          */
/*              y0[k] = yt0; y0[k+1] = yt1;                                 */
/*              k += n_max>>1;                                              */
/*              y0[k] = yt2; y0[k+1] = yt3;                                 */
/*              k += n_max>>1;                                              */
/*              y0[k] = yt4; y0[k+1] = yt5;                                 */
/*              k += n_max>>1;                                              */
/*              y0[k] = yt6; y0[k+1] = yt7;                                 */
/*          }                                                               */
/*      }                                                                   */
/*                                                                          */
/*                                                                          */
/*  NOTES                                                                   */
/*                                                                          */
/*      Configuration is LITTLE ENDIAN.                                     */
/*                                                                          */
/*                                                                          */
/*  CYCLES                                                                  */
/*                                                                          */
/*      cycles = 3 * ceil(log4(N)-1) * N  + 21*ceil(log4(N)-1) + 2*N + 44   */
/*      e.g. N = 1024,  cycles = 14464                                      */
/*      e.g. N = 512,   cycles = 7296                                       */
/*      e.g. N = 256,   cycles = 2923                                       */
/*      e.g. N = 128,   cycles = 1515                                       */
/*      e.g. N = 64,    cycles = 598                                        */
/*                                                                          */
/*                                                                          */
/*  CODESIZE                                                                */
/*      1504 bytes                                                          */
/*                                                                          */
/*                                                                          */
/* ------------------------------------------------------------------------ */
/*            Copyright (c) 2003 Texas Instruments, Incorporated.           */
/*                           All Rights Reserved.                           */
/* ======================================================================== */
#ifndef DSPF_SP_IFFTSPXSP_H_
#define DSPF_SP_IFFTSPXSP_H_ 1

void DSPF_sp_ifftSPxSP
(
    int N,
    float * ptr_x,
    float * ptr_w,
    float * ptr_y,
    unsigned char * brev,
    int n_min,
    int offset,
    int n_max
);

#endif
/* ======================================================================== */
/*  End of file:  DSPF_sp_ifftSPxSP.h                                            */
/* ------------------------------------------------------------------------ */
/*            Copyright (c) 2003 Texas Instruments, Incorporated.           */
/*                           All Rights Reserved.                           */
/* ======================================================================== */
