/** \file flux_obs.c
 * \brief Flux observer module
 * \ingroup MCONTROL
 *
 * $URL: file:///C:/Users/drevensekd/Documents/TECES/Code/ELMOS/elmos_foc_repo/branches/SSCM/motor_control/flux_obs.c $
 * $Rev: 63 $
 * \author Dusan Drevensek, TECES (Initial version 201511)
 * $Author: drevensekd $
 * $Date: 2016-01-26 13:54:43 +0100 (tor, 26 jan 2016) $
*/
#include "parameters.h"
#include "scaling.h"
#include "motor_control.h"
#include "park.h"
#include "pi.h"
#include "sqrt.h"


#include "h430_mul_mac.h"

/** \brief Output limit of the PI regululator inside the flux observer. */ 
#define FOBS_PI_LIMIT 2000

/** \brief Error gain of the PI regulator in flux observer. */
#define FOBS_PI_ERR_GAIN 1 


/** \brief Function reinitializes parameters of the PI regulator inside  the
           flux observer.
\author Dusan Drevensek (TECES) 20151201
\par *fobs Pointer to flux observe structure.
\par kp Proportional gain of the PI regulator.
\par ki Integral gain of the PI regulator.
\retutn Nothing.

\details
The purpose of this function is to be called during on the fly change
of the flux observer parameters.
*/
void flux_observer_gain_reinit(flux_observer_t *fobs, uint16_t kp, uint16_t ki)
{
  fobs->pi_kp=kp;
  fobs->pi_ki=ki;
    
  pi_regulator_set_kpki(&fobs->pi_a, fobs->pi_kp, fobs->pi_ki);
  pi_regulator_set_kpki(&fobs->pi_b, fobs->pi_kp, fobs->pi_ki);  
}

/** \brief Initialization of the flux observer.
\author Dusan Drevensek (TECES) 20151123
\par *fobs Pointer to the flux observer structure.
\return Nothing.

\details
Function initializes the PI regulator inside flux observer and
zeroes some internal variables.
*/
void init_flux_observer(flux_observer_t *fobs)
{
  fobs->pi_kp=MC_FOBS_KP;
  fobs->pi_ki=MC_FOBS_KI;
  
  pi_regulator_init(&fobs->pi_a, fobs->pi_kp , fobs->pi_ki , -FOBS_PI_LIMIT, FOBS_PI_LIMIT,  FOBS_PI_ERR_GAIN );
  pi_regulator_init(&fobs->pi_b, fobs->pi_kp , fobs->pi_ki , -FOBS_PI_LIMIT, FOBS_PI_LIMIT,  FOBS_PI_ERR_GAIN );
  
  fobs->bemf.re=0;
  fobs->bemf.im=0;
  
  fobs->integrator32.re=0;
  fobs->integrator32.im=0;
  
}



/** \brief Function integrates voltage into the flux.
\author Dusan Drevensek (TECES) 20151123
\par *integrator32 This is the content of the integrator. This parameter serves
                   as input and output at the same time.
\par input The voltage, that has to be integrated.
\par sampling_time Sampling time in fixed point format.
\return Nothing.

\details
Function does discrete integration using rectangle rule.
Scaling factors for the time, voltage and flux are taken into account here.
*/
int16_t integrator_flux(int32_t *integrator32, int16_t input, 
                        int16_t sampling_time)
{
    int16_t result;

    /* load accumulator */
    SetAccumulator(*integrator32);

    /* integrate */
    MultiplyAccumulate_S16_S16(input , sampling_time);

    /* store accumulator */
    *integrator32=GetAccumulator();

    /* get return value */
    result=GetAccumulatorAsrSat(14U); 
    return result;
}


/** \brief Active flux observer calculation
\author Dusan Drevensek (TECES)
\par *mc Pointer to motor_control_t structure.
\par *mp Pointer to motor parameters.
\return Nothing

\details
Function calculates the active flux. The block diagram of calculation is shown
on the picture bellow. The result is stored inside the mc structure.


\image html active_flux_observer.png

\warning
Prior to calling of this function the stator current must be transformed
into the dq reference frame, since it is expected for mc->idq to already have 
the correct value! Since the dq current is also needed for the current 
regulator, some CPU cycles are saved this way.

*/
void flux_observer(motor_control_t *mc,motor_parameters_t *mp)
{

    /* BEMF calculation */
    mc->flux_observer.bemf.re=mc->uab.re - Mulmacs_S16_U16_AsrSat(MUL, mc->iab.re, mp->Rs, 15U);
    mc->flux_observer.bemf.im=mc->uab.im - Mulmacs_S16_U16_AsrSat(MUL, mc->iab.im, mp->Rs, 15U);

    /* BEMF integration -> flux */
    
    mc->flux_observer.flux_ab_1.re=integrator_flux(&mc->flux_observer.integrator32.re,
                              mc->flux_observer.bemf.re - mc->flux_observer.volt_compensation.re,
                              SAMPLING_TIME);
    mc->flux_observer.flux_ab_1.im=integrator_flux(&mc->flux_observer.integrator32.im,
                        mc->flux_observer.bemf.im - mc->flux_observer.volt_compensation.im,
                                                   SAMPLING_TIME);

    mc->flux_observer.flux_ab_1.abs=Complex16_Abs(&mc->flux_observer.flux_ab_1);
    
    /*********** calculation of current compensation loop ****************/

    /* calculate fluxes */
    mc->flux_observer.flux_dq_2.re=motor_par.flux+Mulmacs_S16_U16_AsrSat(MUL, mc->idq.re, mp->Ld, 15U);
    mc->flux_observer.flux_dq_2.im=Mulmacs_S16_U16_AsrSat(MUL, mc->idq.im, mp->Lq, 15U);

    /* transform fluxes back to ab */
    transf_dq_ab_fix16(&mc->flux_observer.flux_dq_2,&mc->flux_observer.flux_ab_2,&mc->pll.angle);

    /* pi regulator for cancelation of the offset */
    mc->flux_observer.volt_compensation.re=pi_regulator_long_Ti(&mc->flux_observer.pi_a, mc->flux_observer.flux_ab_1.re - mc->flux_observer.flux_ab_2.re);
    mc->flux_observer.volt_compensation.im=pi_regulator_long_Ti(&mc->flux_observer.pi_b, mc->flux_observer.flux_ab_1.im - mc->flux_observer.flux_ab_2.im);

    /* Adding 'Lq' component of the flux to form the active flux */
    mc->flux_observer.flux_ab.re=mc->flux_observer.flux_ab_1.re - Mulmacs_S16_U16_AsrSat(MUL, mc->iab.re, mp->Lq, 15U);
    mc->flux_observer.flux_ab.im=mc->flux_observer.flux_ab_1.im - Mulmacs_S16_U16_AsrSat(MUL, mc->iab.im, mp->Lq, 15U);
    
    /* calculate flux magnitude */
    mc->flux_observer.flux_ab.abs=Complex16_Abs(&mc->flux_observer.flux_ab);

}

