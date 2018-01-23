/** \file trigonometry.c
 * \brief Trigonometric functions (sin, cos)
 * \ingroup MCONTROL
 *
 * $URL: file:///C:/Users/drevensekd/Documents/TECES/Code/ELMOS/elmos_foc_repo/branches/SSCM/motor_control/trigonometry.c $
 * $Rev: 63 $
 * \author Dusan Drevensek, TECES (Initial version 201511)
 * $Author: drevensekd $
 * $Date: 2016-01-26 13:54:43 +0100 (tor, 26 jan 2016) $
*/

#include <stdint.h>
#include "structs.h"
#include "trigonometry.h"
//#include "h430_mul_mac_regs.h"
#include "h430_mul_mac.h"
#include <stdio.h>

/** \brief Size of the sine lookup table */
#define SINE_LOOKUP_SIZE 	128U

/** \brief Number of places for a logical shift in the interpolation for
           a given LUT */
#define SINE_LUT_SHIFT  	7

/** \brief Lookup table of sine function covering 0..90 degrees */
int16_t sinetable[129]={
	0,
	402,
	804,
	1206,
	1608,
	2009,
	2410,
	2811,
	3212,
	3612,
	4011,
	4410,
	4808,
	5205,
	5602,
	5998,
	6393,
	6786,
	7179,
	7571,
	7962,
	8351,
	8739,
	9126,
	9512,
	9896,
	10278,
	10659,
	11039,
	11417,
	11793,
	12167,
	12539,
	12910,
	13279,
	13645,
	14010,
	14372,
	14732,
	15090,
	15446,
	15800,
	16151,
	16499,
	16846,
	17189,
	17530,
	17869,
	18204,
	18537,
	18868,
	19195,
	19519,
	19841,
	20159,
	20475,
	20787,
	21096,
	21403,
	21705,
	22005,
	22301,
	22594,
	22884,
	23170,
	23452,
	23731,
	24007,
	24279,
	24547,
	24811,
	25072,
	25329,
	25582,
	25832,
	26077,
	26319,
	26556,
	26790,
	27019,
	27245,
	27466,
	27683,
	27896,
	28105,
	28310,
	28510,
	28706,
	28898,
	29085,
	29268,
	29447,
	29621,
	29791,
	29956,
	30117,
	30273,
	30424,
	30571,
	30714,
	30852,
	30985,
	31113,
	31237,
	31356,
	31470,
	31580,
	31685,
	31785,
	31880,
	31971,
	32057,
	32137,
	32213,
	32285,
	32351,
	32412,
	32469,
	32521,
	32567,
	32609,
	32646,
	32678,
	32705,
	32728,
	32745,
	32757,
	32765,
	32767
};

static int16_t sin_lookup(uint16_t angle);


/** \brief Interpolates sine function from th LUT in the range 0 .. 90 degrees
 * \author Dusan Drevensek
 * \date   2015-01
 *
 * \param angle The scaling is like following:  0 .. 2^15 corresponds to 0 .. &pi.
 * \return value of sinus function
 *
 */
static int16_t sin_lookup(uint16_t angle)
{
    uint16_t index;
    int16_t dx,dy;
    int16_t sine;          /* sinus value */

    if(angle>16384U)
    {
      angle=16384U;
    }

    index=angle>>SINE_LUT_SHIFT;
    dx=(int16_t)angle-(int16_t)index*128;
    sine=sinetable[index];
    dy=Mulmacs_S16_S16_AsrSat(MUL, (sinetable[index+1U]-sine), dx, SINE_LUT_SHIFT);
    sine+=dy;
    return sine;
}


/** \brief Calculation on sine function
 * \author Dušan Drevenšek
 * \date   2015-01
 *
 * \param angle
 * \return value of sinus function
 *
 */
int16_t sin_lt(uint16_t angle)
{
    uint8_t quadrant;
    int16_t s;          /* sinus value */

    quadrant=(uint8_t)(((uint16_t)angle)>>14);

    switch (quadrant)
    {
        case 0: /* 0 .. 90deg  */
                s=sin_lookup((uint16_t)angle);
                break;

        case 1: /* 90 .. 180deg  */
                s=sin_lookup(32768U-angle);
                break;

        case 2: /* 180 .. 270deg  */
                s=-sin_lookup(angle-32768U);
                break;

        case 3: /* 270 .. 360deg  */
                s=-sin_lookup(0U-angle);
                break;

        default:
                s=0;
                break;
    }
    return s;
}

/** \brief Calculation of cosine function
 * \author Dušan Drevenšek
 * \date   2015-01
 *
 * \param angle
 * \return value of cosinus function
 *
 */
int16_t cos_lt(uint16_t angle)
{
    int16_t c;          /* sinus value */

    c=sin_lt(angle+16384U);
    return c;
}


