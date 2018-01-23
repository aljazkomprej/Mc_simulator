/** \file parameters.h
 * \ingroup MCONTROL
 * \brief Header with most parameter constants. Here you can change motor
 * and control parameters
 *
 * $URL: $
 * $Rev:  $
 * \author Dusan Drevensek, TECES (Initial version 20151116)
 * $Author: drevensekd $
 * $Date: $
*/


#ifndef PARAMETERS_H
#define PARAMETERS_H

#include "scaling.h"

/****************   MOTOR PARAMETERS   *****************************/

/** \brief Number of pole pairs. */
#define MOTOR_POLEPAIRS  5

/** \brief Stator resistance [Ohm]. */
#define MOTOR_STATOR_RESISTANCE  0.09 // 0.22

/** \brief D-axis stator inductance [H]. */
#define MOTOR_D_AXIS_INDUCTANCE  35e-6

/** \brief Q-axis stator inductance [H]. */
#define MOTOR_Q_AXIS_INDUCTANCE 35e-6

/** \brief Motor flux [Vs]. */
#define MOTOR_FLUX  1.4e-3


/*******************************************************************/
/***************** CONTROL PARAMETERS ******************************/
/*******************************************************************/


/*******************************************************************/
/******************** OPEN LOOP PARAMETERS *************************/

/** \brief The speed for switching to closed loop contro. */
#define MC_OPEN_LOOP_MAX_SPEED 500

/** \brief Current magnitude in open loop control. */
#define MC_OPEN_LOOP_CURRENT 4000

/** \brief Acceleration in open loop control. */
#define MC_OPEN_LOOP_ACCELERATION 2000



/*******************************************************************/
/************ SPEED TRAJECTORY GENERATOR PARAMETERS ****************/

/** \brief Acceleration for the speed ramp. */
#define MC_SPEED_ACCELERATION  10

/** \brief Deceleration for the speed ramp. */
#define MC_SPEED_DECCELERATION  10




/*******************************************************************/
/******************* SPEED CONTROL PARAMETERS **********************/

/** \brief Proportional gain for PI speed controller. */
#define MC_PI_SPEED_KP  6000

/** \brief Integral gain for PI speed controller. */
#define MC_PI_SPEED_KI  10


/*******************************************************************/
/***************** CURRENT CONTROL PARAMETERS **********************/

/** \brief Proportional gain for d-axis  PI current regulator. */
#define MC_PI_D_KP  1200  // 5000

/** \brief Integral gain for d-axis PI current regulator. */
#define MC_PI_D_KI  6000  // 15000


/** \brief Proportional gain for q-axis PI current regulator. */
#define MC_PI_Q_KP  MC_PI_D_KP

/** \brief Proportional gain for q-axis PI current regulator. */
#define MC_PI_Q_KI  MC_PI_D_KI

/** \brief Maximal q-component current Amperes. */
#define MC_MAXIMAL_Q_CURRENT_FLOAT    5.

/** \brief Maximal q-compoent current in fixed point. */
#define MC_MAXIMAL_Q_CURRENT ((int16_t)(MC_MAXIMAL_Q_CURRENT_FLOAT/K16_CURRENT))

/*******************************************************************/
/******************** FLUX OBSERVER PARAMETERS *********************/


/** \brief Proportional gain for PI regulator in flux observer. */
#define MC_FOBS_KP  10000

/** \brief Integral gain for PI regulator in flux observer. */
#define MC_FOBS_KI  1000 // 3




/*******************************************************************/
/************************* PLL PARAMETERS **************************/

/** \brief Proportional gain for PI regulator in PLL. */
#define MC_PLL_KP   8000 // 4500

/** \brief Integral gain for PI regulator in PLL. */
#define MC_PLL_KI   1000 // 2000





/*******************************************************************/
/***************** INVERTER PARAMETERS *****************************/
/*******************************************************************/


/** \brief Oscilator clock [Hz] */
#define OSC_CLOCK_FLOAT     48e6

/** \brief PWM Prescaler : Osc. clock will be divied by (PWM_PRESCALER + 1) */
#define PWM_PRESCALER 3


/** \brief PWM clock frequency [Hz] as a floating point value. */
#define PWM_CLOCK_FLOAT     (OSC_CLOCK_FLOAT/(PWM_PRESCALER+1))

/** \brief PWM carrier frequency [Hz] as a floating point value. */
#define PWM_FREQUENCY_FLOAT 10e3

/** \brief Minimal pulse width [s] as a floating point value. */
#define MIN_PULSE_FLOAT  2e-6

/** \brief Dead time [s] as a floating point value. */
#define PWM_DEADTIME_FLOAT  1.e-6

/** \brief Motor control sampling time as a floating point value. */
#define SAMPLING_TIME_FLOAT (1./PWM_FREQUENCY_FLOAT)




/* Calculated innverter parameter from floating point values */

/** \brief Minimal pulse width in clock cycles of the PWM clock. */
#define MIN_PULSE  ((uint16_t)(PWM_CLOCK_FLOAT*MIN_PULSE_FLOAT))

/** \brief Maximal value of PWM counter in symetrical up/down counting mode. */
#define PWM_UD_COUNTER_MAX ((uint16_t)(PWM_CLOCK_FLOAT/(2.*PWM_FREQUENCY_FLOAT)))

/** \brief Number of deadtime-cycles. */
#define PWM_DEADTIME  ((uint16_t)(PWM_DEADTIME_FLOAT*PWM_CLOCK_FLOAT))


/* protection limits */
#define OVER_CURRENT_LIMIT_FLOAT  8.


#define OVER_CURRENT_LIMIT ((int16_t)(OVER_CURRENT_LIMIT_FLOAT/K16_CURRENT))

/*******************************************************************/
/***************** AD CONVERTER  PARAMETERS ************************/
/*******************************************************************/

/** \brief U-phase current offset in digits of the AD converter. */
#define ADC_IU_OFFSET   2048U

/** \brief U-phase current gain. */
#define ADC_IU_GAIN     8854



/** \brief V-phase current offset in digits of the AD converter. */
#define ADC_IV_OFFSET   2048U

/** \brief V-phase current gain. */
#define ADC_IV_GAIN     8854



/** \brief DC-link offset in digits of the AD converter. */
#define ADC_UDC_OFFSET   0U

/** \brief DC-link gain. */
#define ADC_UDC_GAIN     3297



/** \brief U-phase voltage offset in digits of the AD converter. */
#define ADC_UU_OFFSET   2206U

/** \brief U-phase voltage gain. */
#define ADC_UU_GAIN     2000U



/** \brief V-phase voltage offset in digits of the AD converter. */
#define ADC_UV_OFFSET   2195U

/** \brief V-phase voltage gain. */
#define ADC_UV_GAIN     2000U



/** \brief MCU temperature offset. */
#define ADC_TEMPERATURE_OFFSET  2593U

/** \brief MCU temperature gain. */
#define ADC_TEMPERATURE_GAIN    -18928


#endif /* PARAMETERS_H */
