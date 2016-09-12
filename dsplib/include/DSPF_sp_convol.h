/* =======================================================================      */
/*                                                                              */     
/*  TEXAS INSTRUMENTS, INC.                                                     */
/*                                                                              */
/*  NAME                                                                         */
/*      DSPF_sp_convol: Convolution(Floating Point Version)                          */ 
/*                                                                              */
/*  USAGE                                                                       */
/*      This routine is C callable, and has the following C prototype:          */
/*                                                                              */
/*      void DSPF_sp_convol                                                          */  
/*             (                                                                */
/*                       float *x, //x: Pointer to input samples//              */          
/*                       float *h, //h: Pointer to impulse response samples//   */
/*                       float *r, //r: Pointer to output samples//             */
/*                       int   nh, //nh: Number of impulse response samples//   */
/*                       int   nr  //nr: Number of output samples//             */
/*             )                                                                */
/*                                                                              */
/*              x = pointer to real input vector of size = nr+nh-1              */
/*                  a typically contains input data (x) padded with             */
/*                  consecutive nh - 1  zeros at the beginning and end.         */
/*              h = pointer to real input vector of size nh in forward order.   */
/*                  h typically contains the filter coefs.                      */
/*              r = pointer to real output vector of size nr                    */
/*              nh= number of elements in vector b. NOTE: nh <= nr  nh is       */
/*                  typically noted as m in convol formulas. nh must be a       */
/*                  MULTIPLE of 2                                               */
/*              nr= number of elements in vector r. nr must be a MULTIPLE of 4  */
/*                                                                              */
/*              If routine is not to be used as a C callable function then      */
/*              you need to initialize values for all of the values passed      */
/*              as these are assumed to be in registers as defined by the       */
/*              calling convention of the compiler, (refer to the C compiler    */
/*              reference guide).                                               */
/*                                                                              */
/*  DESCRIPTION                                                                 */
/*              This fucntion calculates the full-lenght convolution of real    */
/*              vectors x and h using time-domain techniques. The result is     */
/*              placed in real vector r.                                        */
/*                                                                              */
/*              It is assumed that input vector x is padded with nh-1 no of     */
/*              zeros in the beginning and end.                                 */
/*                                                                              */
/*              It is assumed that the length of the input vector h, nh, is a   */
/*              multiple of 2 and the length of the output vector r, nr, is a   */    
/*              multiple of 4. nh is greater thanor equal to 4 and nr is greater*/
/*              than or equal to nh. The routine computes 4 output              */
/*              samples at a time.                                              */
/*                                                                              */
/*              x and h are assumed to be aligned on a double word boundary     */
/*                                                                              */
/* -----------------------------------------------------------------------      */
/*            Copyright (c) 2002 Texas Instruments, Incorporated.               */
/*                           All Rights Reserved.                               */
/* =======================================================================      */

#ifndef DSPF_SP_CONVOL_ASM
#define DSPF_SP_CONVOL_ASM 1

void DSPF_sp_convol
        (                                                                   
             float *x, /*x: Pointer to input samples*/                                   
             float *h, /*h: Pointer to impulse response samples*/           
             float *r, /*r: Pointer to output samples*/                       
             int   nh, /*nh: Number of impulse response samples*/            
             int   nr  /*nr: Number of output samples*/                     
        );

#endif
/* ======================================================================== */
/*  End of file:  DSPF_sp_convol.h                                               */
/* ------------------------------------------------------------------------ */
/*            Copyright (c) 2002 Texas Instruments, Incorporated.           */
/*                           All Rights Reserved.                           */
/* ======================================================================== */
