/* test_poly.c */

#include <stdio.h>
#include <stdint.h>

#include "poly.h"
#include "test_poly.h"

poly3_t ply3;
static int16_t result;


void test_poly(void)
{
    uint16_t i;
    int16_t xmin, xmax;

    printf("\nPoly test \n");


    xmin=0;
    xmax=1000;
    init_poly3(&ply3, xmin, xmax, 11, 8590, 0, 0, 0 );


    for(i=xmin; i<=xmax; i++)
    {
        result=poly3(&ply3,i);
        printf("i=%3u   pires=%5d\n",i,result);
    }
}

