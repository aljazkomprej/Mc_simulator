// poly.h DD20160103

#ifndef POLY_H
#define POLY_H

#include <stdint.h>

typedef struct
{
    int16_t coef[3];
    int16_t xmin;
    int16_t xmax;
    int16_t asr;
} poly3_t;

void init_poly3(poly3_t *poly3, int16_t xmin, int16_t xmax, int16_t asr,
                int16_t coef0, int16_t coef1, int16_t coef2, int16_t coef3 );
int16_t poly3(poly3_t *poly3, int16_t x);

#endif /* POLY_H */
