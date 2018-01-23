//#include <fstream>
//#include <iostream>

#include <stdio.h>
#include "test_c.h"
#include "motor_control.h"
//#include "test_flux_obs.h"
#include "parameters.h"
#include "scaling.h"
#include "park.h"
#include "h430_mul_mac.h"
#include "sqrt.h"
#include "pi.h"


#include <math.h>

#include "pmsm.h"

using namespace std;

motor_control_t mc;
mc_parameters_t mc_par;
motor_parameters_t motor_par;

int16_t udc;

extern "C" void init_motor_parameters(motor_parameters_t *mp);

#define MAX_DUTY 17000





/** \brief Initialization of variables for motor control.
* \author Dusan Drevensek TECES (Initial version 20151117)
* \return Function returns nonzero in a case of error
*
* \details
* The function must be called prior to starting the motor control!
*/
int16_t motor_control_init(void)
{
  //init_test_pin();

  mc.angle_ptr=&mc.pll.angle;

  udc=12./K16_VOLTAGE;

  mc.udq.re=0;
  mc.udq.im=0;

  mc.uab.re=0;
  mc.uab.im=0;

  mc.udq_duty.re=0;
  mc.udq_duty.im=0;


  mc.idq_cmd.re=0;
  mc.idq_cmd.im=0;

  mc.we_command=4000;
  mc.test_current=0;

  mc.pll.angle.angle=0;

  //init_svpwm(&sv, PWM_UD_COUNTER_MAX);

#define MAX_DUTY 17000
  mc_par.pi_d_kp=MC_PI_D_KP;
  mc_par.pi_d_ki=MC_PI_D_KI;

  mc_par.pi_q_kp=MC_PI_Q_KP;
  mc_par.pi_q_ki=MC_PI_Q_KI;

  pi_regulator_init(&mc.pi_d, mc_par.pi_d_kp, mc_par.pi_d_ki, -MAX_DUTY, MAX_DUTY, 6);
  pi_regulator_init(&mc.pi_q, mc_par.pi_q_kp, mc_par.pi_q_ki, -MAX_DUTY, MAX_DUTY, 6);

//#define MAX_ISQ 3000
  mc_par.pi_speed_kp=MC_PI_SPEED_KP;
  mc_par.pi_speed_ki=MC_PI_SPEED_KI;

  pi_regulator_init(&mc.pi_speed, mc_par.pi_speed_kp, mc_par.pi_speed_ki,
                    -MC_MAXIMAL_Q_CURRENT, MC_MAXIMAL_Q_CURRENT, 50); // 10


  init_motor_parameters(&motor_par);
  init_flux_observer(&mc.flux_observer);
  pll_init(&mc.pll);

  //open_loop_init(&spd, MC_OPEN_LOOP_ACCELERATION, MC_OPEN_LOOP_MAX_SPEED , MC_OPEN_LOOP_CURRENT);
  //speed_ramp_initialization(&spd, MC_SPEED_ACCELERATION, MC_SPEED_DECCELERATION);

  return 0;
}


/*==============================================================================
:* FUN:	void calculate_voltages_from_duty_cycle(int udc, COMPLEX_INT *usab_duty, COMPLEX_INT *usab )
:*
:* DES:	Function calculates stator voltage vector from the duty cycle vector
:*       and dc-link voltage.
:*
:* PRM:	int udc : dc link voltage
:*       COMPLEX_INT *usab_duty : pointer to duty cycle vector
:*			COMPLEX_INT *usab : pointer to stator voltage vector (output)
:*
:* RET:	none
:*
:* VER: 1.0 DD 20090804
:*============================================================================*/
void calculate_uab_from_duty_cycle(const int16_t udc, complex16_t *uab_duty, complex16_t *uab)
{
  uab->re=Mulmacs_S16_S16_AsrSat(MUL, uab_duty->re  , udc, 15);
  uab->im=Mulmacs_S16_S16_AsrSat(MUL, uab_duty->im  , udc, 15);
}




/** \brief Current regulator in dq reference frame.
\author Dusan Drevensek
\date 20151224

\details
Function is intended to be used for current control.
Since it contains two regulators (d and q axis) and cycle to cycle
limit calculations, it is encapsuled into a single function for better code
organization and readability.
*/
static inline void current_regulator(void)
{
  /* error calculation for each axis */
  mc.eidq.re=mc.idq_cmd.re - mc.idq.re;
  mc.eidq.im=mc.idq_cmd.im - mc.idq.im;

  /* d-axis current regulator */
  mc.udq_duty.re=pi_regulator(&mc.pi_d, mc.eidq.re );

  { /* limit calculation for the q-axis current regulator */
    int16_t limit_pi_q;
    uint32_t limit_pi_q_tmp;

    Multiply_S16_S16(MAX_DUTY, MAX_DUTY);
    MultiplyAccumulate_S16_S16(mc.udq_duty.re, -mc.udq_duty.re);
    limit_pi_q_tmp=GetAccumulator();
    limit_pi_q=Sqrt32_16( limit_pi_q_tmp );
    pi_regulator_set_limits(&mc.pi_q, -limit_pi_q, limit_pi_q);
  }

  /* q-axis current regulator */
  mc.udq_duty.im=pi_regulator(&mc.pi_q, mc.eidq.im );
}




void test_motor_control_wo_pwm(void)
{
    uint32_t i;
    int16_t integ16;
    int32_t integrator32;
    double Ts;
    cmplx_dbl_t cs; // cos, sin
    cmplx_dbl_t uab; // voltage
    cmplx_dbl_t iab; // current
    double angle_flt;
    Pmsm mot;


    ofstream dfile;
    dfile.open("mcwopwm.dat");

    Ts=100e-6;

    angle_flt=0.;
    iab.re=0; iab.im=0;


    /************ initialization *************/

    /* init PMSM model */
    mot.Set_Sampling_Time(Ts);
    mot.Set_Parameters(MOTOR_STATOR_RESISTANCE,MOTOR_D_AXIS_INDUCTANCE,MOTOR_Q_AXIS_INDUCTANCE,MOTOR_FLUX,MOTOR_POLEPAIRS,0.0001,0.0002);


    init_motor_parameters(&motor_par);
    init_flux_observer(&mc.flux_observer);
    pll_init(&mc.pll);

    mc.angle_ptr=&mc.pll.angle;

    motor_control_init();

    /* set stator current command */
    mc.idq_cmd.re=0./K16_CURRENT;
    mc.idq_cmd.im=5./K16_CURRENT;


    for(i=0;i<50000;i++)
    {
        // set stator current variable for mc
        mc.iab.re=mot.Get_isa()/K16_CURRENT;
        mc.iab.im=mot.Get_isb()/K16_CURRENT;

        // set udc


        // transform iab -> idq
        transf_ab_dq_fix16(&mc.iab, &mc.idq, mc.angle_ptr);


        // flux observer
        flux_observer(&mc, &motor_par);

        // pll
        pll_calculate(&mc.pll,&mc.flux_observer.flux_ab);

        // current regulator
        current_regulator();

        /* transform the voltage into stationary reference frame */
        transf_dq_ab_fix16(&mc.udq_duty, &mc.uab_duty, mc.angle_ptr);

        /* calculate uab */
        calculate_uab_from_duty_cycle(udc, &mc.uab_duty, &mc.uab);


        // transformation of voltages to floating point for motor simulation
        uab.re=mc.uab.re*K16_VOLTAGE;
        uab.im=mc.uab.im*K16_VOLTAGE;

        // PMSM model calculation
        mot.Calculate(uab.re,uab.im, 0.);


        dfile << i << " "
          << mc.uab.re << " " << mc.uab.im << " "
          << mc.udq.re << " " << mc.udq.im << " "
          << mc.udq_duty.re << " " << mc.udq_duty.im << " "
          << mc.iab.re << " " << mc.iab.im << " "
          << mc.idq.re << " " << mc.idq.im << " "
          << mc.flux_observer.flux_ab.re << " " << mc.flux_observer.flux_ab.im << " " << mc.flux_observer.flux_ab.abs << " "
          << mc.flux_observer.flux_ab_1.re << " " << mc.flux_observer.flux_ab_1.im << " "
          << mc.flux_observer.flux_ab_2.re << " " << mc.flux_observer.flux_ab_2.im << " "
          << mc.flux_observer.volt_compensation.re  << " " << mc.flux_observer.volt_compensation.im << " "
          << mc.flux_observer.bemf.re  << " " << mc.flux_observer.bemf.im << " "
          << mc.pll.angle.angle << " " << mc.pll.we << " " << mc.pll.phase_error << " "
          << mc.flux_observer.urs.re << " " << mc.flux_observer.urs.im << " "

          << endl;

    }

    dfile.close();

}
