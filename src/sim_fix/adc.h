/** \file adc.h
 * \brief Header for Analog to digital conversion module
 * \ingroup MCONTROL
 *
 * $Rev: 91 $
 * \author Dusan Drevensek, TECES (Initial version 20151029)
 * $Author: drevensekd $
 * $Date: 2016-04-26 15:50:33 +0200 (tor, 26 apr 2016) $
*/

#ifndef ADC_H
#define ADC_H

#include "h430_mul_mac.h"


/* Do not change the size of the structure, otherwise the needed alignment may fail */
typedef struct
{
  /** \brief Digits of the adc conversion as read from the AD. */
  uint16_t adc_digits;

  /** \brief Value of the quantitiy presented in fixed point. */
  int16_t value;

  /** \brief Gain of the signal (to get proper value of the quantity). */
  int16_t gain;

  /** \brief Offset od the adc digits (for bipolar quantities). */
  uint16_t offset;
} adc_quantity_t;


typedef struct
{
  uint16_t i1_digits;
  uint16_t i2_digits;


  /** \brief U phase current. */
  adc_quantity_t iu;

  /** \brief V phase current. */
  adc_quantity_t iv;

  adc_quantity_t iw;

  /** \brief DC link voltage. */
  adc_quantity_t udc;

  /** \brief MCU temperature. */
  adc_quantity_t temperature;

  /** \brief U phase voltage. */
  adc_quantity_t uu;

  /** \brief V phase voltage. */
  adc_quantity_t uv;

  /** \brief W phase voltage. */
  adc_quantity_t uw;


  //uint8_t k11[6],k12[6],k21[6],k22[6];

} adc_t;



extern adc_t adc;

#ifdef __cplusplus
extern "C" {
#endif

static void calculate_adc_quantity(adc_quantity_t *adc_quantity);
void saradc_init_pwm(void);
void init_adc_variables(void);
void CalculateCurrentsSingleShunt(void);

#ifdef __cplusplus
}
#endif

/*******  inline functions ********/

/** \brief Function translates adc digits into a quantity represented
     by fixed point number
\author Dusan Drevensek (TECES) 201510
\param *adc_quantity Pointer to adc_quantity_t for specific conversion
\return Function returns nothing

\details
Functions calculates fixedpoint representation of the quantity converted
by adc. The result is stored inside the structure,
which is given as a parameter.
*/
static inline void calculate_adc_quantity(adc_quantity_t *adc_quantity)
{
  int16_t dval;

  dval=(int16_t)(adc_quantity->adc_digits - adc_quantity->offset);
  adc_quantity->value=Mulmacs_S16_S16_AsrSat(MUL, dval, adc_quantity->gain, 10);
}




#endif /* ADC_H */




