/* pll.c DD20151123 */

#include <stdint.h>

#include "parameters.h"
#include "structs.h"
#include "pi.h"
#include "pll.h"
#include "h430_mul_mac.h"
#include "scaling.h"
#include "trigonometry.h"

#define PI_LIMIT 20000


void pll_gain_reinit(pll_t *pll, uint16_t kp, uint16_t ki)
{
  pll->kp=kp;
  pll->ki=ki;
  pi_regulator_set_kpki(&pll->pi, kp, ki);
}

void pll_init(pll_t *pll)
{
  pll->kp=MC_PLL_KP;
  pll->ki=MC_PLL_KI;

  pi_regulator_init(&pll->pi, pll->kp, pll->ki, -PI_LIMIT, PI_LIMIT, 10);

  /* set angle, sin & cos */
  pll->angle.angle=0;
  pll->angle.cos=32767;
  pll->angle.sin=0;
  pll->integrator32=0L;
  pll->phase_error=0;
}

/** \brief PLL calculation
\todo phase error gain depends on flux magnitude !
\todo Check the constant for integrator gain! Make it dependable on sampling time!
*/
void pll_calculate(pll_t *pll, const complex16_t *in)
{
    /* there is no normalization of input singnal */
    /* dynamics of PLL will depend on input signal level! */

    /* phase discriminator */
    //pll->phase_error=muls_macs_asr_sat( -in->re, pll->angle.sin, in->im, pll->angle.cos, 14); // duty +, +
    pll->phase_error=muls_macs_asr_sat( -in->re, pll->angle.sin, in->im, pll->angle.cos, 12); // duty +, +
    //pll->phase_error=( (double)(-in->re)*pll->angle.sin + (double)in->im * pll->angle.cos )/4096.; // duty +, +


    pll->we=pi_regulator(&pll->pi, pll->phase_error);

    /* intagrator */
    /* load accumulator */
    SetAccumulator(pll->integrator32);

    /* integrate */
    //multiply_accumulate_s16_s16( pll->we , SAMPLING_TIME);
    MultiplyAccumulate_S16_S16( pll->we , 7772);

    /* store accumulator */
    pll->integrator32=GetAccumulator();

    /* get return value */
    pll->angle.angle=GetAccumulatorAsr(15); /* xxx preveri .. popravi!  gain?? */

    /* calc sin/cos*/
    pll->angle.cos=cos_lt(pll->angle.angle);
    pll->angle.sin=sin_lt(pll->angle.angle);
}
