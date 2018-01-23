#include <stdio.h>
#include "test_c.h"
#include "motor_control.h"
//#include "test_flux_obs.h"
#include "parameters.h"
#include "scaling.h"

#include <math.h>

#if 0
motor_control_t mc;
mc_parameters_t mc_par;
motor_parameters_t motor_par;
#endif


void init_motor_parameters(motor_parameters_t *mp)
{
    mp->Rs=MOTOR_STATOR_RESISTANCE/K16_IMPEDANCE;
    mp->Ld=MOTOR_D_AXIS_INDUCTANCE/K16_INDUCTANCE;
    mp->Lq=MOTOR_Q_AXIS_INDUCTANCE/K16_INDUCTANCE;
    mp->flux=MOTOR_FLUX/K16_FLUX;
    mp->polepairs=MOTOR_POLEPAIRS;
}


void test_flux_observer(void)
{
    FILE *fp;
    uint16_t i;
    int16_t integ16;
    int32_t integrator32;
    double Ts;
    double u0; // voltage magnitude
    cmplx_dbl_t cs; // cos, sin
    cmplx_dbl_t uab; // voltage
    cmplx_dbl_t iab; // current
    double angle_flt;
    double freq;

    fp = fopen ("fobs.dat", "w");

    Ts=100e-6;

    u0=5.;
    freq=100.;

    angle_flt=0.;
    iab.re=0; iab.im=0;


    /* initialization */
    init_motor_parameters(&motor_par);
    init_flux_observer(&mc.flux_observer);
    pll_init(&mc.pll);

    /* set stator current */
    mc.uab.re=0./K16_VOLTAGE;
    mc.uab.im=5./K16_VOLTAGE;

    /* set stator current */
    mc.iab.re=0./K16_CURRENT;
    mc.iab.im=5./K16_CURRENT;


    integrator32=0L;
    integ16=integrator_flux(&integrator32, 6827, 4915);
    printf("intg; i16=%d, u32=%d\n",integ16, integrator32);

    integ16=integrator_flux(&integrator32, 6827, 4915);
    printf("intg; i16=%d, u32=%d\n",integ16, integrator32);

    flux_observer(&mc, &motor_par);

    printf("\nbemf=%d / %d \n",mc.flux_observer.bemf.re,mc.flux_observer.bemf.im);


    for(i=0;i<1000;i++)
    {
        angle_flt+=Ts*freq*2.*M_PI;
        cs.re=cos(angle_flt);
        cs.im=sin(angle_flt);
        uab.re=u0*cs.re;
        uab.im=u0*cs.im;

        // transformation of voltages to fixed point
        mc.uab.re=uab.re/K16_VOLTAGE;
        mc.uab.im=uab.im/K16_VOLTAGE;

        mc.iab.re=iab.re/K16_CURRENT;
        mc.iab.im=iab.im/K16_CURRENT;

        flux_observer(&mc, &motor_par);
        pll_calculate(&mc.pll,&mc.flux_observer.flux_ab);

        fprintf(fp,"%u %d %d ",i,mc.uab.re, mc.uab.im);
        fprintf(fp,"%d %d ",mc.iab.re, mc.iab.im);
        fprintf(fp,"%d %d ",mc.flux_observer.flux_ab.re, mc.flux_observer.flux_ab.im);
        fprintf(fp,"%u %d ",mc.pll.angle.angle,mc.pll.we);

        fprintf(fp,"\n");


    }

    fclose(fp);

}
