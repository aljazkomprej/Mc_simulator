/* pll.h D20151123 */

#ifndef PLL_H
#define PLL_H

#include "structs.h"
#include "pi.h"


typedef struct
{
    int16_t kp;
    int16_t ki;
  
    /** \brief Pi regulator, which control the phase of input signal and output signal to zero. */
    pi_regulator_t pi;

    /** \brief Angle structure containing also sin/cos values. */
    angle_t angle;

    /** \brief Electric angular speed. */
    int16_t we;

    int16_t phase_error;

    int32_t integrator32;
} pll_t;


#ifdef __cplusplus
extern "C" {
#endif

void pll_init(pll_t *pll);
void pll_calculate(pll_t *pll, const complex16_t *in);
void pll_gain_reinit(pll_t *pll, uint16_t kp, uint16_t ki);
#ifdef __cplusplus
}
#endif

#endif
