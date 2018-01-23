/* svpwm.h DD20151111 */

#ifndef SVPWM_H
#define SVPWM_H

#include <stdbool.h>
#include "structs.h"
#include "adc.h"

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;

typedef int8_t s8;
typedef int16_t s16;
typedef int32_t s32;

#define MATH_ANGLE_MAX                65536

//#define SQRT3 ( 1.7320508075688772935274463415059 )
#define MATH_ARCTAN_LUT_LENGTH_BITS 8u
#define MATH_ARCTAN_LUT_LENGTH      (( 1 << MATH_ARCTAN_LUT_LENGTH_BITS ) + 1 )

#define MATH_SINE_LUT_LENGTH_BITS 6u
#define MATH_SINE_LUT_LENGTH      ( 1 << MATH_SINE_LUT_LENGTH_BITS )
#define MATH_SINE_LUT_IDX_MSK     ( 0x3F )




#define F_CPU 48000000u                                                                                                                         /**< core clock frequency [Hz] */
#define F_CPU_MHZ ( F_CPU / 1000000u )                                                                                                          /**< core clock frequency [MHz] */

#define PWM_OUT_FREQ     20000u                                                                                                                   /**< frequency of PWM output signal [Hz] */
#define PWM_OUT_FREQ_KHZ ( PWM_OUT_FREQ / 1000u )                                                                                                 /**< frequency of PWM output signal [kHz] */

#define PWM_DEADTIME_INITVAL ( (u16) ( 0.5 * (double) F_CPU_MHZ ) )                                                                               /**< deadtime between HS and LS switch [register DEAD_TIME of block PWMN] */

#define PWM_PERIOD_DURATION_TICKS 2400/*( F_CPU_MHZ * 1000u / PWM_OUT_FREQ_KHZ )*/                                                                        /**< amount of timer ticks per PWM period */

#define PWM_SINGLE_SHUNT_MAX_CMP_VAL ( PWM_PERIOD_DURATION_TICKS - 1u )                                                                           /**< maximum timer compare value (single shunt operation) */
#define PWM_SINGLE_SHUNT_MIN_CMP_VAL ( ADC_THIRD_PWM_EDGE + 1u )
#define PWM_SVM_CNT_LIM_MARGIN             20u
#define PWM_SVM_MIN_TIME                   ( (u16) ( 3.0 * (double) F_CPU_MHZ ) )
#define PWM_SVM_STIME_OFFSET_LOWA          ( 0u )
#define _PWM_SINGLE_SHUNT_CMP_MEDIAN ((( PWM_SINGLE_SHUNT_MAX_CMP_VAL - PWM_SINGLE_SHUNT_MIN_CMP_VAL ) / 2u ) + PWM_SINGLE_SHUNT_MIN_CMP_VAL ) /**< middle between PWM_SINGLE_SHUNT_MAX_CMP_VAL and PWM_SINGLE_SHUNT_MIN_CMP_VAL */
#define PWM_SINGLE_SHUNT_CMP_MEDIAN ( _PWM_SINGLE_SHUNT_CMP_MEDIAN + (u16) ((double) (( PWM_SINGLE_SHUNT_MAX_CMP_VAL - PWM_SINGLE_SHUNT_MIN_CMP_VAL ) / 2u ) * 0.1547 ))                                                                                         /**< move currents sample time to the edge (max. value is PWM_SVM_MIN_TIME/2), if 0 is used it will be sampled in the middle PWM_SVM_MIN_TIME/2*/


#define ADC_GATE_DRIVER_PROPAGATION_DLY ((u16) ( 0 * (double) F_CPU_MHZ )) //0.4 for real applications                                                                /**< propagation delay through the gate driver, i.e. delay between the MotCU PWM signals and the actual gate driver output [us] */

#define ADC_SAMPLE_EXTENSION_CLK_CYCLES ( 3 )                                                                                                  /**< the sampling time of the ADC is extended by that amount of (ADC) clock cycles / 2 (6 MHz) -> only for all IMMEDIATE_SAMPLES (user measurements and VSUP) */

#define ADC_SAMPLE_DELAY_SHUNT_VOLTAGE ((u16) ( 1.21 * (double) F_CPU_MHZ ))                                                                   /**< time that is waited before a shunt voltage is sampled [us] */
#define ADC_SAMPLE_CLK_CYCLES          ((u16) ( 0.166 * (double) F_CPU_MHZ ))                                                                  /**< duration of an ADC sample phase during phase current measurement [us], only used for calculation of measurement timings -> ADC_SAMPLE_EXTENSION_CLK_CYCLES does _not_ have an impact here! */
#define ADC_CONVERSION_CLK_CYCLES      ((u16) ( 1.4375 * (double) F_CPU_MHZ ))                                                                 /**< duration of an AD conversion, until next conversion can be started [us] */

#if 0
/* single shunt */
#define ADC_FIRST_PWM_EDGE        ((u16) ( 0 ))
#define ADC_FIRST_CURRENT_SAMPLE  ((u16) ( ADC_GATE_DRIVER_PROPAGATION_DLY + PWM_DEADTIME_INITVAL + ADC_SAMPLE_DELAY_SHUNT_VOLTAGE ))
#define ADC_SECOND_PWM_EDGE       ((u16) ( ADC_FIRST_CURRENT_SAMPLE - ADC_GATE_DRIVER_PROPAGATION_DLY + ADC_SAMPLE_CLK_CYCLES ))
#define ADC_SECOND_CURRENT_SAMPLE ((u16) ( ADC_SECOND_PWM_EDGE + ADC_GATE_DRIVER_PROPAGATION_DLY + PWM_DEADTIME_INITVAL + ADC_SAMPLE_DELAY_SHUNT_VOLTAGE ))
#define ADC_THIRD_PWM_EDGE        ((u16) ( ADC_SECOND_PWM_EDGE * 2u ))
#else
#define ADC_FIRST_PWM_EDGE        ((u16) ( 0 ))
#define ADC_FIRST_CURRENT_SAMPLE  ((u16) ( 4.0 * (double) F_CPU_MHZ ))
#define ADC_SECOND_PWM_EDGE       ((u16) ( 5.0 * (double) F_CPU_MHZ ))
#define ADC_SECOND_CURRENT_SAMPLE ((u16) ( ADC_SECOND_PWM_EDGE +  ADC_FIRST_CURRENT_SAMPLE ))
#define ADC_THIRD_PWM_EDGE        ((u16) ( ADC_SECOND_PWM_EDGE * 2u ))
#endif // 0





/*============================ TYPE DEFINITIONS ==============================*/


/* structure for dead-time compensation */
typedef struct
{
    uint16_t dead_time;	/* dead-time */
    int16_t i_knee;	/* current value of the knee */
    int16_t k;
} dtcomp_t;

/* structure for space vector modulator */
typedef struct
{
  /** \brief Value for compare register 1. */
  uint16_t t1;

  /** \brief Value for compare register 2. */
  uint16_t t2;

    /** \brief Value for compare register 3. */
  uint16_t t3;

  uint16_t k1, k2;	/* constants */

  /** \brief Minimal pulse width. */
  uint16_t tmin;

  /** \brief Maximal pulse width. */
  uint16_t tmax;

  /** \brief Period of the PWM timer. */
  uint16_t period;

  /** \brief Dead-time compensation structure. */
  dtcomp_t dtcomp;

  // compare values for sawtooth signal
  uint16_t t1up,t2up,t3up;
  uint16_t t1down,t2down,t3down;

  /** \brief Minimal fundamental vector width for SSCM */
  int16_t tmin_sscm;



  /** \brief Times for current measuring points. */
  uint16_t m1,m2;

  /** \brief This contains the channel selection for udc measurement.  */
  uint8_t udc_meas_channel;

  uint8_t enable_short_meas_sequence;

  /** \brief Measuring point for udc. */
  uint16_t m_udc;

  uint8_t sector;
  uint8_t region;

} svpwm_t;

typedef struct
{
  s16 v_alpha;                                /**< momentary value of output voltage vector in alpha-beta-coordinates, alpha [PWM counter value] */
  s16 v_beta;                                 /**< momentary value of output voltage vector in alpha-beta-coordinates, beta [PWM counter value] */

  s16 v_u;                                    /**< momentary value of output voltage at motor phase U [PWM counter value]*/
  s16 v_v;                                    /**< momentary value of output voltage at motor phase U [PWM counter value]*/
  s16 v_w;                                    /**< momentary value of output voltage at motor phase U [PWM counter value]*/

  s16 svm_length;
  u16 svm_phi;
  u16 svm_phi2;
  u16 svm_sector;
  u16 svm_i1_samp_time_up;                    /**< sample time value for pwm first half period */
  u16 svm_i2_samp_time_down;                  /**< sample time value for pwm second half period */
  u16 svm_t0_up;
  u16 svm_t1_up;
  u16 svm_t2_up;
  u16 svm_t0_down;
  u16 svm_t1_down;
  u16 svm_t2_down;
  u16 time_v3;
  u16 time_v5;

  /*for testing*/
  u16 ua;
  u16 va;
  u16 wa;
  s16 u_vu;
  s16 u_wv;
  s16 u_uw;

  u16 svm_stored_sector;

} foc_ctrl_loop_data_t;

typedef enum
{
  MC_PHASE_U = 1u,
  MC_PHASE_V = 2u,
  MC_PHASE_W = 3u
} mc_motor_phase_t;

//typedef enum { false, true } bool;

typedef struct
{
  s16 pwm0_cmp_val;                                          /**< actual compare value of phase U */
  s16 pwm1_cmp_val;                                          /**< actual compare value of phase V */
  s16 pwm2_cmp_val;                                          /**< actual compare value of phase W */
  s16 pwm0_dwn_val;                                          /**< actual compare value of phase U */
  s16 pwm1_dwn_val;                                          /**< actual compare value of phase V */
  s16 pwm2_dwn_val;                                          /**< actual compare value of phase W */

  s16 min_cmp_val;                                           /**< mininmum of pwm0_cmp_val, pwm1_cmp_val, pwm2_cmp_val */
  s16 max_cmp_val;                                           /**< maxinmum of pwm0_cmp_val, pwm1_cmp_val, pwm2_cmp_val */

  bool pwm_cmp_val_overflow_flag;                            /**< flag: compare values can not be set, abs(max_cmp_val - min_cmp_val) is too big; overflow */

  s16 curr1_offset_val;                                      /**< ADC value of the offset of either the sum current OPA or the first phase current OPA [ADC LSB] */
  s16 curr2_offset_val;                                      /**< ADC value of the offset of the second phase current OPA [ADC LSB] */

  s16 last_phase_voltage_order;

  mc_motor_phase_t highest_phase;                          /**< holds the motor phase for which the FOC computed the highest value (at any given time) [enum] */

} mc_data_t;



/* VARIABLE DECLARATIONS */

/** \brief Space vector modulator structure */
extern svpwm_t sv;
extern mc_data_t mc_data;
extern foc_ctrl_loop_data_t foc_data;

/*========================== FUNCTION DECLARATIONS ===========================*/

#ifdef __cplusplus
extern "C" {
#endif

void init_svpwm(svpwm_t *pwm, uint16_t period);
void svpwm(svpwm_t *pwm, const complex16_t *d);
void svpwm_sscm(svpwm_t *pwm, const complex16_t *d);
void foc_svm_improved ( foc_ctrl_loop_data_t * foc_data, s16 a, s16 b, u16 svm_switch );

void mc_svm_extract_phase_currents( void );

void mc_isr_handler ( int16_t u, int16_t v, int16_t w );
void mc_current_calc (void);

#ifdef __cplusplus
}
#endif

#endif /* SVPWM_H */
