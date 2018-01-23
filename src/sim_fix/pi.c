/** \file pi.c
 * \brief Pi regulator module
 * \ingroup MCONTROL
 *
 * $Rev: 91 $
 * \author Dusan Drevensek, TECES (Initial version 20151108)
 * $Author: drevensekd $
 * $Date: 2016-04-26 15:50:33 +0200 (tor, 26 apr 2016) $
*/


#include <stdio.h>
#include <stdint.h>
#include "pi.h"

int32_t __ssat(int32_t in, uint8_t bits)
{
    int32_t out;
    int32_t max;

    max=1L<<(bits-1);

    if ( in > max - 1 )
    {
        out=max-1;
    }
    else
    {
        if (in < -max)
        {
            out=-max;
        }
        else
        {
            out=in;
        }
    }

    return out;
}


/** \brief Function sets proportional and integral gain of the pi regulator.
\author Dusan Drevensek (Initial version 20151120)
\par *pi Pointer to regulator structure.
\par kp Proportional gain.
\par Ts_Ti Integral gain, which is expressed as Ts/Ti*32768..
\return Nothing.
*/
void pi_regulator_set_kpki(pi_regulator_t *pi, uint16_t kp, uint16_t Ts_Ti)
{
  pi->kp=kp;
  pi->ki=Ts_Ti;

  pi->kpki=(uint16_t)((((uint32_t)pi->kp * (uint32_t)pi->ki))/32768UL);

}

/** \brief Function sets upper and lower output limits of the pi regulator.
\author Dusan Drevensek (Initial version 20151120)
\par *pi Pointer to regulator structure.
\par min_limit Lower limit of integrator output.
\par max_limit Upper limit of integrator output.
\return Nothing.
*/
void pi_regulator_set_limits(pi_regulator_t *pi,
                             int16_t min_limit, int16_t max_limit)
{
  pi->max_limit=max_limit;
  pi->min_limit=min_limit;
}


/** \brief Function sets initial condition of the pi regulator.
\author Dusan Drevensek (Initial version 20151216)
\par *pi Pointer to regulator structure.
\par initial value.
\return Nothing.
*/
void pi_regulator_set_initial_condition(pi_regulator_t *pi,
                             int16_t initial_value)
{
  pi->yout=initial_value;
  pi->delta_limit=0;
  pi->integrator=initial_value*32768L;
}


/** \brief Function fully initializes the pi regulator.
\author Dusan Drevensek (Initial version 20151120)
\par *pi Pointer to regulator structure.
\par kp Proportional gain.
\par Ts_Ti Integral gain, which is expressed as Ts/Ti*32768.
\par min_limit Lower limit of integrator output.
\par max_limit Upper limit of integrator output.
\return Nothing.
*/
void pi_regulator_init(pi_regulator_t *pi, uint16_t kp, uint16_t Ts_Ti,
                       int16_t min_limit, int16_t max_limit, uint16_t error_gain_in)
{
  uint16_t error_gain;

  error_gain=error_gain_in;

  pi_regulator_set_kpki(pi, kp, Ts_Ti);
  pi_regulator_set_limits(pi, min_limit, max_limit);

  if(error_gain>256U) /* limit error gain */
  {
    error_gain=256U;
  }
  pi->error_gain=error_gain;

  pi->integrator=0L;  /* initial value of the integrator */
}


/** \brief Function calculates pi regulator.
\author Dusan Drevensek (Initial version 20151110)
\par *pi Pointer to regulator structure.
\par err Control error.
\return Calculated value of pi regulator.

\details
Block diagram of the PI regulator implementation is shown on the picture bellow.
\image html anti_windup_pi_regulator.png

Fixed point PI regulator accuracy is influenced by roundings. In order to keep the
integrator inside PI regulator integrating, the product kp*ki must be greater than 32768
( kp*ki > 32768 ! ).

*/
int16_t pi_regulator(pi_regulator_t *pi, int16_t err)
{
  int16_t err_sat;
  int16_t out_before_limit;
  int32_t tmp32;
  int16_t prop; /* proportional signal */

  /* multiply and saturate error */
  err_sat=(int16_t)__ssat( ((int32_t)err * (int32_t)pi->error_gain), 16)  ;

  prop = __ssat((((int32_t)err_sat * (int32_t)pi->kp ) / 32768L ), 16);

#if 0
  tmp32=pi->integrator;
  tmp32 += ( (int32_t)prop - (int32_t)pi->delta_limit ) * (int32_t)pi->ki;
  pi->integrator=tmp32;
#else

  tmp32 = __ssat(( (int32_t)prop - (int32_t)pi->delta_limit ) * (int32_t)pi->ki , 30);
  tmp32 = __ssat( tmp32 + pi->integrator, 31);
  //tmp32 =  tmp32 + pi->integrator;
  pi->integrator = tmp32;
#endif

  out_before_limit = (int16_t)__ssat( (( tmp32 ) / (32768L)) + prop, 16);


  /* saturate the output if needed */
  if( out_before_limit > pi->max_limit )  /* upper limit active */
  {
    pi->yout=pi->max_limit;
    pi->delta_limit=out_before_limit - pi->max_limit;
  }
  else if( out_before_limit < pi->min_limit ) /* lower limit active */
  {
    pi->yout=pi->min_limit;
    pi->delta_limit=out_before_limit - pi->min_limit;
  }
  else
  {   /* no limiting */
    pi->yout=out_before_limit;
    pi->delta_limit=0;
  }

  return pi->yout;
}


/** \brief Function calculates pi regulator version with long integration time.
\author Dusan Drevensek (Initial version 20151207)
\par *pi Pointer to regulator structure.
\par err Control error.
\return Calculated value of pi regulator.
*/
int16_t pi_regulator_long_Ti(pi_regulator_t *pi, int16_t err)
{
  int16_t err_sat;
  int16_t out_before_limit;
	int32_t tmp32;

  /* multiply and saturate error */
  err_sat=(int16_t)__ssat(  ((int32_t)err * (int32_t)pi->error_gain) , 16 );

	tmp32=pi->integrator;
	tmp32 += (int32_t)err_sat * (int32_t)pi->kpki;
	tmp32 += (int32_t)pi->delta_limit * (int32_t)pi->ki;
  pi->integrator=__ssat( tmp32, 32-2);
	out_before_limit = (int16_t)( ( (tmp32/128) + ( (int32_t)err_sat * (int32_t)pi->kp ) ) / 32768L);

  /* saturate the output if needed */
  if( out_before_limit > pi->max_limit )  /* upper limit active */
  {
    pi->yout=pi->max_limit;
    pi->delta_limit=pi->max_limit-out_before_limit;
  }
  else if( out_before_limit < pi->min_limit ) /* lower limit active */
  {
    pi->yout=pi->min_limit;
    pi->delta_limit=pi->min_limit-out_before_limit;
  }
  else
  {   /* no limiting */
    pi->yout=out_before_limit;
    pi->delta_limit=0;
  }

  return pi->yout;
}
