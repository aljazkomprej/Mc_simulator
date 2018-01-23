#include "poly.h"
#include "h430_mul_mac.h"


void init_poly3(poly3_t *poly3, int16_t xmin, int16_t xmax, int16_t asr,
                int16_t coef0, int16_t coef1, int16_t coef2, int16_t coef3 )
{
    poly3->xmin=xmin;
    poly3->xmax=xmax;
    poly3->asr=asr;
    poly3->coef[0]=coef0;
    poly3->coef[1]=coef1;
    poly3->coef[2]=coef2;
    poly3->coef[3]=coef3;
}

int16_t poly3(poly3_t *poly3, int16_t x)
{
    int16_t result;
    int16_t tmp;

    if(x<poly3->xmin)
    {
        x=poly3->xmin;
    }
    else if(x > poly3->xmax)
    {
        x=poly3->xmax;
    }

    tmp=Mulmacs_S16_S16_AsrSat( MUL, poly3->coef[0],x,poly3->asr)+poly3->coef[1];
    tmp=Mulmacs_S16_S16_AsrSat( MUL, tmp, x, poly3->asr)+poly3->coef[2];
    result=Mulmacs_S16_S16_AsrSat( MUL, tmp, x, poly3->asr)+poly3->coef[3];


    return result;
}
