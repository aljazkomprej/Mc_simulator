/* motor_control.h DD20151116 */



/**
 * @defgroup MCONTROL Motor control
 *
 * This module provides functions needed for Field oriented control of
 * permanent magnet synchronous machine (PMSM).
 *
 * @{
 *    - main.c , main.h
 *    - motor_control.c, motor_control.h
 *    - park.c, park.h
 *    - pi.c, pi.h
 *    - filter.c, filter.h
 *    - svpwm.c, svpwm.h
 *    - trigonometry.c, trigonometry.h
 *    - adc.c, adc.h
 *    - state_machine.c, state_machine.h
 *    - terminal.c, terminal.h
 *    - measure_time.c, measure_time.h
 * @}
 */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

//#include "saradc_ctrl_bf.h"

#include "structs.h"
#include "pi.h"
#include "pll.h"

typedef struct
{
  uint16_t polepairs;
  uint16_t Rs;        /**< stator resistance */
  uint16_t Ld;        /**< d-axis inductance */
  uint16_t Lq;        /**< q-axis inductance */
  uint16_t flux;      /**< motor flux */
} motor_parameters_t;


typedef struct
{
    complex16_t bemf;  /**< Calculated back EMF. */
    complex16_t flux_ab_1; /**< Stator flux calculated from the voltage model */
    complex16_t flux_ab_2; /**< Stator flux calculated from the current model */
    complex16_t flux_dq_2; /**< Stator flux calculated from the current model */
    complex32_t integrator32; /**< 32-bit value of integrator for better precision */
    complex16_t volt_compensation; /**< Voltage compensation from PI regulator */
    complex16_t flux_ab;        /**< Calculated active flux */

    pi_regulator_t pi_a;    /**< Alpha axis regulator   */
    pi_regulator_t pi_b;    /**< Beta axis regulator */

    /** \brief Proportional gain of PI regulator in flux observer. */
    uint16_t pi_kp;

    /** \brief  Integral gain of PI regulator in flux observer. */
    uint16_t pi_ki;

    complex16_t urs;

} flux_observer_t;



/** \brief Motor control structure, which contains all important motor control variables. */
typedef struct
{
  /** \brief Rotor angle structure. */
  //angle_t angle;

  /** \brief Stator current in stator reference frame. */
  complex16_t iab;

  /** \brief Stator current in a dq reference frame. */
  complex16_t idq;

  /** \brief Command value for the stator current in a dq reference frame. */
  complex16_t idq_cmd;

  /** \brief Stator voltage in a dq reference frame. */
  complex16_t udq;

  /** \brief Stator voltage in a stationary reference frame. */
  complex16_t uab;

  complex16_t udq_duty;

  /* \brief Current error vector. */
  complex16_t eidq;

  /** \brief Stator voltage duty cycle in a stator reference frame */
  complex16_t uab_duty;

  pi_regulator_t pi_d;
  pi_regulator_t pi_q;

  /** \brief Speed regulator structure. */
  pi_regulator_t pi_speed;

  /** \brief Speed command. */
  int16_t we_command;

  pll_t pll;

  /** \brief This points to the active angle structure. */
  angle_t *angle_ptr;

  /** \brief Flux observer structure. */
  flux_observer_t flux_observer;

  int16_t test_current;

} motor_control_t;


/** \brief Structure for motor control parameters */
typedef struct
{
  /** \brief Proportional gain for d-axis current regulator */
  uint16_t pi_d_kp;

  /** \brief Integral gain for d-axis current regulator */
  uint16_t pi_d_ki;

  /** \brief Proportional gain for q-axis current regulator */
  uint16_t pi_q_kp;

  /** \brief Integral gain for q-axis current regulator */
  uint16_t pi_q_ki;

  uint16_t pi_speed_kp;
  uint16_t pi_speed_ki;

} mc_parameters_t;


typedef struct
{
  //saradc_ctrl_l0_current_t L0_current;
  //saradc_ctrl_l1_current_t L1_current;
  uint8_t fl_interrupt_execution; /* set to 1 if when FL interrupt executes */
  uint8_t sl_interrupt_execution; /* set to 1 if when SL interrupt executes */
  uint16_t slow_interrupt_counter;

  //complex16_t v1;
} mc_test_t;


/* Variable declarations */
extern motor_control_t mc;
extern mc_parameters_t mc_par;
extern motor_parameters_t motor_par;
extern mc_test_t mc_test;

#ifdef __cplusplus
extern "C" {
#endif

/* Function declarations  */
void motor_control_isr(int16_t irq);
int16_t motor_control_init(void);
void init_motor_parameters(motor_parameters_t *mp);
int16_t generate_square_signal(int16_t amplitude, uint16_t period);

/* from flux observer */
void flux_observer_gain_reinit(flux_observer_t *fobs, uint16_t kp, uint16_t ki);
void init_flux_observer(flux_observer_t *fobs);
int16_t integrator_flux(int32_t *integrator32, int16_t input, int16_t sampling_time);
void flux_observer(motor_control_t *mc,motor_parameters_t *mp);


/* speed regulator */
void speed_regulator_gain_reinit(motor_control_t *mc, mc_parameters_t *mcp, uint16_t kp, uint16_t ki);

#ifdef __cplusplus
}
#endif

#endif
