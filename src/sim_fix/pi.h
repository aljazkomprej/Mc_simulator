/* pi.h  DD20151108 */

#ifndef PI_H
#define PI_H

typedef struct
{
  /* proportional gain */
  uint16_t kp;

  /* integrator gain = sampling time / integration time constant */
  uint16_t ki;

  uint16_t kpki;

  int16_t yout;		    /**< Regulator output from the last step. */

  /* upper limit */
  int16_t max_limit;

  /* lower limit*/
  int16_t min_limit;

  int16_t delta_limit;    /**< Difference between regulator input and the output limit */

  /* error multiplication */
  uint16_t error_gain;

  /* internal value of integrator */
  int32_t integrator;

} pi_regulator_t;


#ifdef __cplusplus
extern "C" {
#endif

void pi_regulator_init(pi_regulator_t *pi, uint16_t kp, uint16_t Ts_Ti,
                       int16_t min_limit, int16_t max_limit, uint16_t error_gain_in);



void pi_regulator_set_kpki(pi_regulator_t *pi, uint16_t kp, uint16_t Ts_Ti);
void pi_regulator_set_limits(pi_regulator_t *pi,
                             int16_t min_limit, int16_t max_limit);
void pi_regulator_set_initial_condition(pi_regulator_t *pi,
                             int16_t initial_value);


int16_t pi_regulator(pi_regulator_t *pi, int16_t err);
int16_t pi_regulator_long_Ti(pi_regulator_t *pi, int16_t err);


#ifdef __cplusplus
}
#endif


#endif /* PI_H */
