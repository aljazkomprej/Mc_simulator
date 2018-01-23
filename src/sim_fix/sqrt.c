/** \file sqrt.c
 * \brief Calculation of square root and absolute value of complex number.
 * \ingroup MCONTROL
 *
 * $URL: file:///C:/Users/drevensekd/Documents/TECES/Code/ELMOS/elmos_foc_repo/branches/SSCM/motor_control/sqrt.c $
 * $Rev: 63 $
 * \author Dusan Drevensek, TECES (Initial version 20151209)
 * $Author: drevensekd $
 * $Date: 2016-01-26 13:54:43 +0100 (tor, 26 jan 2016) $
*/

#include <stdint.h>
#include <stdlib.h>
#include "divider.h"
#include "sqrt.h"
#include "h430_mul_mac.h"
#include "measure_time.h"
#include "constants.h"


/** \brief Function calculates square root of 32-bit unsigned number.
\author Dusan Drevensek
\param square  Input to the sqrt function.
\return Function returns the absolute value.

\details
Function firstly estimates initial value.

This is followed by 3 iterations, which is sufficient to get correct result
of the absolute value with the precision of 1 digit.

\todo Optimize the calculation of initial guess!

\todo There is further possibility to optimize the code by inline usage of 
divider and using 'dead' cycles while waiting for the result of division.
*/
uint16_t Sqrt32_16(uint32_t square)
{
  uint16_t result;
  
  if( square > 4294836225U )
  { /* special case because of troubles with division */
    result=65535U;
  }
  else if( square > 0U)
  { /* calculate only when square is not zero */
    uint32_t tmp;
 
    tmp=square;
    result=1;
    
    /* calculation of initial value  */
    /* this takes 219 cycles */
    while ( tmp > 1 )
    {
      tmp >>= 2U;
      result = (result << 1U) + 1U;
    }
    
    
    /* iterative calculation if sqrt */
    /* one iteration takes 51 cycles */
    
    result=(result+divider_udiv( square, result))>>1;


    result=(result+divider_udiv( square, result))>>1;
    result=(result+divider_udiv( square, result))>>1;
  }
  else
  {
    result=0U;
  }
  return result;
}



/** \brief Function calculates absolute value of the complex number.
\author Dusan Drevensek
\param *cin  Pointer to complex value structure.
\return Function returns the absolute value.

\details
Function firstly calculates the square ( re^2+im^2 ), 
and estimates initial value of the result as
result0= ( abs(re) + abs(im) ) / sqrt(2) .

This is followed by 3 iterations, which is sufficient to get correct result
of the absolute value with the precision of 1 digit.

\todo There is further possibility to optimize the code by inline usage of 
divider and using 'dead' cycles while waiting for the result of division.
*/
uint16_t Complex16_Abs(complex16_t *cin)
{
  uint32_t square;
  uint16_t result;
    
  /* calculating the square */
  Multiply_S16_S16(cin->re, cin->re);
  MultiplyAccumulate_S16_S16(cin->im, cin->im);
  square=(uint32_t)GetAccumulator();
  
  // calculations to this point take 41-19=22 cycles 
  
  /* calculate initial value */
  Multiply_S16_S16( abs(cin->re), (int16_t)(32768./SQRT2) );
  MultiplyAccumulate_S16_S16( abs(cin->im), (int16_t)(32768./SQRT2) );
  result=GetAccumulatorAsrSat( 15 );
  
  if(result > 0)
  {
    /* iteration for precise result */
    result=(result+divider_udiv( square, result))>>1;
    result=(result+divider_udiv( square, result))>>1;
    result=(result+divider_udiv( square, result))>>1;
  }
  else
  {
    result=0;
  }
  return result;
}