/* test_svpwm.c */

#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include "svpwm.h"
#include "test_svpwm.h"




svpwm_t pwm;
complex16_t uin;

void test_svpwm(void)
{
    FILE *fp;
    uint16_t i;
    int16_t xmin, xmax;
    uint16_t points;
    double angle, dangle;
    uint16_t angle_fix;
    double ang_min,ang_max;
    int16_t u0;

    printf("\nSVPWM_SSCM test \n");
    fp = fopen ("sscm.dat", "w");

    init_svpwm(&pwm, 1000);

    u0=10490;

    uin.re=8660;
    uin.im=5000;
    points=36000;
    dangle=2*M_PI/points;
    ang_min=0.;
    ang_max=M_PI;
    angle=ang_min;


    svpwm_sscm(&pwm,&uin);
    printf("\n%5u %5u %5u %5u %5u %5u | %5u %5u ",
           pwm.t1up,pwm.t1down,pwm.t2up,pwm.t2down,pwm.t3up,pwm.t3down,pwm.m1,pwm.m2);

    for(i=0;i<points;i++)
    {
        angle+=dangle;
        uin.re=u0*cos(angle);
        uin.im=u0*sin(angle);
        angle_fix=angle*65536/(2.*M_PI);
        svpwm_sscm(&pwm,&uin);
        fprintf(fp,"\n%5u  %5u %5u %5u %5u %5u %5u   %5u %5u %5u",
                angle_fix,pwm.t1up,pwm.t1down,pwm.t2up,pwm.t2down,pwm.t3up,pwm.t3down,pwm.m1,pwm.m2,pwm.m_udc);
    }


    fclose(fp);
}

