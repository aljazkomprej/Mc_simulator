/** \file adc.c
 * \brief Analog to digital conversion module
 * \ingroup MCONTROL
 *
 * $Rev: 94 $
 * \author Dusan Drevensek, TECES (Initial version 20151029)
 * $Author: drevensekd $
 * $Date: 2016-04-28 15:55:45 +0200 (čet, 28 apr 2016) $
 * \todo Evaluation board doesn't allow the measurement of dc-link
 *       without turning on at list one upper MOSFET in inverter.
 *       For new hardwares it would be desirable to enable dc-link
 *       voltage measurement and include it here in the module.
*/

#include <stdio.h>
#include <stdint.h>
#include "adc.h"
#include "h430_mul_mac.h"
#include "parameters.h"
#include "svpwm.h"
//#include "errors.h"
#include "motor_control.h"



/** \brief ADC conversion data structure. */
adc_t adc;




void init_adc_variables(void)
{
  adc.iu.gain=ADC_IU_GAIN;
  adc.iu.offset=ADC_IU_OFFSET;
  adc.iu.adc_digits=ADC_IU_OFFSET;

  adc.iv.gain=ADC_IV_GAIN;
  adc.iv.offset=ADC_IV_OFFSET;
  adc.iv.adc_digits=ADC_IV_OFFSET;
}


void CalculateCurrentsSingleShunt(void)
{
  uint8_t sec;
  uint16_t mod;
  int16_t sum_offset;

  static const int8_t sscm_k11[]={ 0,-1,-1, 0, 1, 1};
  static const int8_t sscm_k12[]={ 1, 1, 0,-1,-1, 0};
  static const int8_t sscm_k21[]={ 1, 1, 0,-1,-1, 0};
  static const int8_t sscm_k22[]={-1, 0, 1, 1, 0,-1};

  sec=sv.sector-1U;
  mod=sv.region;

#define ADC_I_OFFSET (ADC_IU_OFFSET)

  if( mod == 1U) /* small signal region */
  {
     adc.iv.adc_digits = +(int16_t)adc.i1_digits ;
     adc.iu.adc_digits = 3*ADC_I_OFFSET - (int16_t)adc.i2_digits - (int16_t)adc.i1_digits ;

  }
  else  //if( mod == 0U )
  { /* normal pwm pattern */
    /* iu1=k11*i1+k12*i2 */

    adc.iu.adc_digits = adc.i1_digits * (int16_t)sscm_k11[sec]
    + (int16_t)adc.i2_digits * (int16_t)sscm_k12[sec]
    + (int16_t)ADC_I_OFFSET * ((1-(int16_t)sscm_k11[sec]-(int16_t)sscm_k12[sec]));

    adc.iv.adc_digits = adc.i1_digits * (int16_t)sscm_k21[sec]
    + (int16_t)adc.i2_digits * (int16_t)sscm_k22[sec]
    + (int16_t)ADC_I_OFFSET * ((1-(int16_t)sscm_k21[sec]-(int16_t)sscm_k22[sec]));
  }
}


