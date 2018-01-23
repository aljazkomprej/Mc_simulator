/* test_pll.c */


#include <fstream>
#include <iostream>
//#include <cmath>

#include <stdio.h>
#include <stdint.h>

#include "pll.h"
#include "test_pll.h"
//#include "trigonometry.h"

using namespace std;


extern "C" int16_t cos_lt(uint16_t angle);
extern "C" int16_t sin_lt(uint16_t angle);
extern "C" void pll_calculate(pll_t *pll, const complex16_t *in);


static pll_t pll;

static int16_t result;
complex16_t psiab;


void test_pll(void)
{
    uint16_t i;
    uint16_t angle;
    int16_t dangle;

    dangle=1000;

    ofstream dfile;

    dfile.open("pll.dat");

    pll_init(&pll);

#define K_SIGNAL 0.01

    angle=0;
    psiab.re=cos_lt(angle)*K_SIGNAL;
    psiab.im=sin_lt(angle)*K_SIGNAL;

    for(i=1; i<4000; i++)
    {
        if(i==2200) dangle-=500;
        angle+=dangle;
        psiab.re=cos_lt(angle)*K_SIGNAL;
        psiab.im=sin_lt(angle)*K_SIGNAL;
        pll_calculate(&pll,&psiab);

        printf("i=%3u  ang0=%5u angpll=%5u  we=%6d  ph_err=%6d (%6d/%6d)\n",i,angle,pll.angle.angle,pll.we,pll.phase_error,
        pll.angle.cos,pll.angle.sin);

        dfile << i << " " << angle << " " << pll.angle.angle << " " << pll.we << " " << pll.phase_error << " "
         << psiab.re << " "  << psiab.im << endl ;

    }

    dfile.close();
}
