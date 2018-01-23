/* test_pi.c */

#include <stdio.h>
#include <stdint.h>

#include "pi.h"
#include "test_pi.h"

static pi_regulator_t pi_t;

static int16_t result;


void test_pi(void)
{
    uint16_t i;
    FILE *fp;


    fp = fopen("pi.dat", "wt");

    printf("\nPi regulator test \n");
    #define PI_LIMIT 32000
//    pi_regulator_init(&pi_t, 32767, 32768/10, -PI_LIMIT, PI_LIMIT, 20);
    pi_regulator_init(&pi_t, 32767/10, 32768/10, -PI_LIMIT, PI_LIMIT, 1);

    for(i=1; i<1460; i++)
    {
        if(i==14)
        {
            i=14;
        }

        result=pi_regulator(&pi_t,-3200);
        printf("i=%3u   pires=%5d integ=%5d \n",i,result,pi_t.integrator/65536L);

        fprintf(fp,"%3u  %5d %10ld \n",i,result,pi_t.integrator);

    }

    fclose(fp);
}

void test_pi_cl(void)
{
    uint16_t i;
    double tao = 500e-6;
    double Ts=100e-6;
    double yfilt=0.;

    printf("\nPi regulator test \n");
    #define PI_LIMIT 30000
    pi_regulator_init(&pi_t, 32767, 32768/10, -PI_LIMIT, PI_LIMIT, 20);

    for(i=1; i<40; i++)
    {
        yfilt=


        result=pi_regulator(&pi_t,1000);
        printf("i=%3u   pires=%5d\n",i,result);
    }

}
