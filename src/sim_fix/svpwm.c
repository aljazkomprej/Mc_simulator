/** \file svpwm.c
 * \brief Space vector modulator module.
 * \ingroup MCONTROL
 *
 * $URL: file:///C:/Users/drevensekd/Documents/TECES/Code/ELMOS/elmos_foc_repo/branches/SSCM/motor_control/svpwm.c $
 * $Rev: 82 $
 * \author Dusan Drevensek, TECES (Initial version 20151111)
 * $Author: drevensekd $
 * $Date: 2016-03-15 14:46:07 +0100 (tor, 15 mar 2016) $
*/

#include <stdlib.h>
#include "svpwm.h"
#include "constants.h"
#include "parameters.h"
#include "scaling.h"
#include "park.h"
#include "adc.h"
//#include "errors.h"


svpwm_t sv;
foc_ctrl_loop_data_t foc_data;

/* Scaling constant for FOC sinus table */
#define	MATH_CFG_FOC_TABLE_SCALE (1.0)

/* Function-like macro for scaling, round to nearest */
#define  Table_lScale(Value) \
((u16)((((double) (Value)) * MATH_CFG_FOC_TABLE_SCALE) + 0.5))



//static inline uint16_t scale_pwm(uint16_t period, uint16_t in);

/*==============================================================================
:* FUN:	void init_svpwm(struct _svpwm *pwm, unsigned int period)
:*
:* DES:	Initialization of space vector modulator
:*
:* PRM:	struct _svpwm *pwm : pointer to _swpwm structure
:*			unsigned int period : period equals to period register of epwm1
:*
:* RET:	none
:*
:* VER: 1.0 DD 20080228
:*============================================================================*/


/** \fn  init_svpwm
* \brief Initialization of the space vector modulator *
* \author Dusan Drevensek
* \date 20080228 (Initial version 1.0)
* \date 20151116 (Version 1.1)
*
* \param *pwm Pointer to SVPWM structure.
* \param period Period setting of PWM.
* \return Nothing.
*/
void init_svpwm(svpwm_t *pwm, uint16_t period)
{


  pwm->period=period;
  pwm->k1=(uint16_t)(SQRT3/4.*32768.);
  pwm->k2=(uint16_t)(3./4.*32768.);
  pwm->tmin=MIN_PULSE;
  pwm->tmax=pwm->period - pwm->tmin;


  /* minimal time=Tmin/32768*period */
  pwm->tmin_sscm=1000; // 3000=5.5us  , 1500 big distortion, 2000=not ok , 2500



  pwm->dtcomp.dead_time=0U;
  pwm->dtcomp.i_knee=(int16_t)(0.5/K16_CURRENT);
  pwm->dtcomp.k=(int16_t)((int32_t)pwm->dtcomp.dead_time*256/pwm->dtcomp.i_knee);
}

/*==============================================================================
:* FUN:	inline unsigned int scale_pwm(int period, unsigned int in)
:*
:* DES:	Since calculation of conduction times is independent of actual period set
:*			in EPWM1, those times must be recalculated. Since this needs to be done
:*       3 times and fast, and only in svpwm function, this function is declared
:*       as inline.
:*
:* PRM:	int period : period as set in EPWM1
:*			unsigned int in : conduction time as calculated in svpwm before scaling
:*
:* RET:	conduction time (value for conmpare register of EPWM1)
:*
:* VER: 1.0 DD 20080228
:*============================================================================*/
static inline uint16_t scale_pwm(uint16_t period, uint16_t in_par)
{
  uint16_t out;
  uint16_t in;
  int16_t tmp;

  in=in_par;

  if(in>49152U)
  {
    in=49152U;
  }
  else
  {
    if(in<16384U)
    {
      in=16384U;
    }
  }

  out=period>>1;
  tmp=-32768;
  tmp+=(int16_t)in;
  //out+=(uint16_t)Mulmacs_S16_U16_AsrSat(MUL, tmp, period, 15U);
  out+= ( (uint32_t)tmp * (uint32_t)period ) >> 15;
  return out;
}


#if 0
/*==============================================================================
:* FUN:	short deadtime_compensation(short i, short t, struct _dtcomp *dt)
:*
:* DES:	Function compensates deat-time for single phase.
:*			If i>i_knee -> add dead-time
:*			else if i<-i_knee -> substract dead_time
:*       else use compensation proportional to the current
:*
:* PRM:	short i : phase current
:*       short t : in svpwm calculated time
:*			struct _dtcomp *dt : pointer to deadtime structure (contains params.)
:*
:* RET:	modified conduction time for a phase
:*
:* VER: 1.0 DD 20080228
:*============================================================================*/
static int16_t deadtime_compensation(int16_t i, int16_t t, const dtcomp_t *dt)
{
    int16_t out,tmp;

    if(i > dt->i_knee)
    {
      out=t + (int16_t)dt->dead_time;
    }
    else if(i < -dt->i_knee)
    {
      out=t - (int16_t)dt->dead_time;
    }
    else
    {
        tmp=Mulmacs_S16_S16_AsrSat(MUL, dt->k, i, 8U);
        out=t+tmp;
    }
    return out;
}
#endif

static inline uint16_t scale_pwm_sscm(uint16_t period, uint16_t in)
{
  uint16_t out;

//  out=MulmacuAsrSat(MUL, in, period, 15U);
  out= ( (uint32_t)in * (uint32_t) period) >> 15;


  return out;
}


void svpwm_sscm(svpwm_t *pwm, const complex16_t *d)
{
  int16_t ubcmp;
  complex16_t dtr; /* transformed duty cycle */
  int16_t ubcmp_abs;
  int16_t re_abs;
  uint16_t d_abs;

  static const angle_t transf_ang[]=
  {
    { /* 0 degrees */
      .angle=0,
      .sin=0,
      .cos=32767
    },
    { /* 120 degrees */
      .angle=21845,
      .sin=28378,
      .cos=-16384
    },
    { /* 120 degrees */
      .angle=43691,
      .sin=-28378,
      .cos=-16384
    },
  };

  d_abs=Complex16_Abs(d);


//  ubcmp=Mulmacs_S16_S16_AsrSat(MUL, (int16_t)(32768./SQRT3), d->im, 15U);
  ubcmp=( (int32_t)(32768./SQRT3) * (int32_t)d->im)  >> 15U;
  ubcmp_abs=abs(ubcmp);
  re_abs=abs(d->re);

  if(d_abs < 6000)
  { /* unified small signal PWM calculation using active vectors V2, V3, V5, V6 - 0327650 */
    static int16_t time_vect_2;
    static int16_t time_vect_3;
    static uint16_t time_vzero;
    static int16_t time_v2;
    static int16_t time_v3;
    static int16_t time_v5;
    static int16_t time_v6;

    pwm->region=1;

    /* calculate linear combination of vectors v2 & v3 */
    time_vect_2= ( (int32_t)d->re * (int32_t)( 3./2./2.*32768.) + (int32_t)d->im * (int32_t)(SQRT3/2./2.*32768.) ) >> 14U;
    time_vect_3= ( (int32_t)d->re * (int32_t)(-3./2./2.*32768.) + (int32_t)d->im * (int32_t)(SQRT3/2./2.*32768.) ) >> 14U;

    if(time_vect_2 >= 0)
    {
        time_v2=time_vect_2 + pwm->tmin_sscm;
        time_v5=pwm->tmin_sscm;
    }
    else
    {
        time_v2=pwm->tmin_sscm;
        time_v5=-time_vect_2 + pwm->tmin_sscm;
    }

    if(time_vect_3 >= 0)
    {
        time_v3=time_vect_3 + pwm->tmin_sscm;
        time_v6=pwm->tmin_sscm;
    }
    else
    {
        time_v3=pwm->tmin_sscm;
        time_v6=-time_vect_3 + pwm->tmin_sscm;
    }

    time_vzero=32767 - time_v2 - time_v3 - time_v5 - time_v6;

    pwm->t2up=time_vzero*7/16;
    pwm->t1up=pwm->t2up + time_v3;
    pwm->t3up=pwm->t1up + time_v2;
    pwm->t2down=pwm->t3up + time_vzero/8;
    pwm->t1down=pwm->t2down + time_v6;
    pwm->t3down=pwm->t1down + time_v5;

    pwm->m1=pwm->t1up - time_v3/2;
    pwm->m2=pwm->t3down - time_v5/2;


  }
  else
  {


  // determine the sector
  if( ubcmp_abs >= re_abs )
  {	/* 2nd & 5th sextant */
    if(ubcmp>=0)
    {
        pwm->sector=2;
    }
    else
    {
        pwm->sector=5;
    }
  }
  else if( (re_abs >= ubcmp_abs ) && ( ( (uint16_t)(d->re)&0x8000U)==((uint16_t)(d->im)&0x8000U) ) )
  {	/* 1st & 4th sextant */
      if( d->re >= 0)
      {
          pwm->sector=1;
      }
      else
      {
          pwm->sector=4;
      }
  }
  else
  {	/* 3rd & 6th sextant */
      if( d->re >= 0 )
      {
          pwm->sector=6;
      }
      else
      {
          pwm->sector=3;
      }
  }

  if( (pwm->sector & 0x01U) == 0x01U ) /* for odd sectors: 1,3,5 */
  {
      int16_t time_vect_1, time_vect_2;
      uint16_t t11s, t12s, t13s;  /* starting times for pulses in 1st sector for phases */
      uint16_t t11e, t12e, t13e;  /* end times for pulses in 1st sector for phases */

      /* transform into 1st sector */
      transf_ab_dq_fix16(d,&dtr,&transf_ang[(pwm->sector-1U)/2U]);

      /* calculate 1st sector */
      time_vect_1= ( (int32_t)dtr.re * (int32_t)(3./2./2.*32768.) + (int32_t)dtr.im * (int32_t)(-SQRT3/2./2.*32768.) ) >> 14U;
      time_vect_2=( (int32_t)dtr.im * (int32_t)(SQRT3/2.*32768.) ) >>  14;

      if(time_vect_1 < 0 )
      {
        time_vect_1=0;
      }

      if(time_vect_2 < 0 )
      {
        time_vect_2=0;
      }

        if(time_vect_1 < (int16_t)pwm->tmin_sscm)
      { /* left boundary, sequence: 03210 */
        int16_t time_v1;
        int16_t time_v2;
        int16_t time_v3;
        uint16_t time_vzero;

        pwm->region=2;

        /* d1=dmin */
        time_v1 = pwm->tmin_sscm;
        time_v2 = time_vect_1 + time_vect_2 - pwm->tmin_sscm;
        time_v3 = pwm->tmin_sscm - time_vect_1;
        time_vzero=32767U - (uint16_t)time_v2 - pwm->tmin_sscm;

        /* calculation of PWM points */
        t12s = time_vzero/2U;
        t11s = t12s + (uint16_t)time_v3;
        t13s = 16384U;
        t13e = 16384U;
        t12e = t11s + (uint16_t)time_v2;
        t11e = t12e + (uint16_t)time_v1;


        /* calculation of current sampling points */
        pwm->m1 = 16384; //  sample in the middle of the period, old: t12e - 200U;
        pwm->m2 = t11e - 200U;

        /* udc sampling point */
        pwm->m_udc = t11s + 100U;

        pwm->enable_short_meas_sequence=1;
      }
      else if(time_vect_2 < (int16_t)pwm->tmin_sscm)
      { /* right boundary, sequence: 0127610 */
        int16_t time_v1;
        int16_t time_v2;
        int16_t time_v6;
        uint16_t time_vzero;

        pwm->region=3;

        time_v1=time_vect_1 + time_vect_2 - pwm->tmin_sscm;
        time_v2=pwm->tmin_sscm;
        time_v6=pwm->tmin_sscm - time_vect_2;
        time_vzero=32767U - (uint16_t)time_v1 - pwm->tmin_sscm;

        /* calculation of PWM points */
        t11s = time_vzero*7/16;
        t12s = t11s + (uint16_t)time_v1/2U;
        t13s = t12s + time_v2;
        t12e = t13s + time_vzero/8U;
        t13e = t12e + time_v6;
        t11e = t13e + ( (uint16_t)time_v1/2U );

        /* calculation of current sampling points */
        pwm->m1 = t13s - time_v2/2U; // - 200U ;
        pwm->m2 = t11e - (uint16_t)time_v1/4; // - 200U ;

        pwm->m_udc=t11e - 1000U;   /* udc sampling point */

        pwm->enable_short_meas_sequence=1;
      }
      else
      { /* no pwm pattern modification needed */
        int16_t time_v1;
        int16_t time_v2;
        uint16_t time_vzero;

        pwm->region=0;
        time_v1=time_vect_1;
        time_v2=time_vect_2;
        time_vzero=32767U - (uint16_t)time_v1 - (uint16_t)time_v2;

        /* calculation of PWM points */
        t11s=time_vzero/4U;
        t12s=t11s + ( ((uint16_t)time_v1)/2U );
        t13s=t12s + ( (uint16_t)time_v2/2U );
        t13e=16384U + ( time_vzero/4U );
        t12e=t13e + ( (uint16_t)time_v2/2U );
        t11e=t12e + ( (uint16_t)time_v1/2U );

        /* calculation of current sampling points */
        pwm->m1=t13s - time_v2/4U; // - 200U;
        pwm->m2=t11e - time_v1/4U; // - 200U;

        /* udc sampling point */
        pwm->m_udc=16384U;

         pwm->enable_short_meas_sequence=0;
      }

      /* transform times to particular sectors */
      switch ( pwm->sector )
      {
      case 1:
          pwm->t1up=t11s;
          pwm->t2up=t12s;
          pwm->t3up=t13s;
          pwm->t1down=t11e;
          pwm->t2down=t12e;
          pwm->t3down=t13e;
          pwm->udc_meas_channel=0;
          break;
      case 3:
          pwm->t1up=t13s;
          pwm->t2up=t11s;
          pwm->t3up=t12s;
          pwm->t1down=t13e;
          pwm->t2down=t11e;
          pwm->t3down=t12e;
          pwm->udc_meas_channel=1;
          break;
      case 5:
          pwm->t1up=t12s;
          pwm->t2up=t13s;
          pwm->t3up=t11s;
          pwm->t1down=t12e;
          pwm->t2down=t13e;
          pwm->t3down=t11e;
          pwm->udc_meas_channel=2;
          break;

      default:
        /* rise an error, because we never should reach this point */
        //set_error(error_software_system_error);
        break;
      }
  }
  else /* for even sectors: 2,4,6 */
  {
      int16_t time_vect_2, time_vect_3;
      uint16_t t21s, t22s, t23s;  /* starting times for pulses in 1st sector for phases */
      uint16_t t21e, t22e, t23e;  /* end times for pulses in 1st sector for phases */

      /* transform */
      transf_ab_dq_fix16(d,&dtr,&transf_ang[(pwm->sector - 2U)/2U]);

      /* calculate 2nd sector */
      time_vect_2= ( (int32_t)dtr.re * (int32_t)( 3./2./2.*32768.) + (int32_t)dtr.im * (int32_t)(SQRT3/2./2.*32768.) ) >> 14U;
      time_vect_3=( (int32_t)dtr.re * (int32_t)(-3./2./2.*32768.) + (int32_t)dtr.im * (int32_t)(SQRT3/2./2.*32768.) ) >> 14U;

      if(time_vect_3 < 0 )
      {
        time_vect_3=0;
      }

      if(time_vect_2 < 0 )
      {
        time_vect_2=0;
      }

      if(time_vect_2 < (int16_t)pwm->tmin_sscm)
      { /* left boundary 0347230 */
        int16_t time_v2;
        int16_t time_v3;
        int16_t time_v4;
        uint16_t time_vzero;

        pwm->region=2U;
        time_v2 = pwm->tmin_sscm;
        time_v3 = time_vect_2 + time_vect_3 - pwm->tmin_sscm;
        time_v4 = pwm->tmin_sscm  - time_vect_2;
        time_vzero=32767U - (uint16_t)time_v3 - pwm->tmin_sscm;

        /* calculation of PWM points */
        t22s = time_vzero*7/16;
        t23s = t22s + ( (uint16_t)time_v3/2U);
        t21s = t23s + (uint16_t)time_v4;
        t23e = t21s + time_vzero/8U;
        t21e = t23e + time_v2;
        t22e = t21e + ( (uint16_t)time_v3/2U );

        /* calculation of current sampling points */
        pwm->m1 = t23s - time_v3/4U ;// - 200U;
        pwm->m2 = t21e - time_v2/2U ; // - 200U;

        /* udc sampling point */
        pwm->m_udc=t22e -1000U;

        pwm->enable_short_meas_sequence=1;
      }
      else if(time_vect_3 < (int16_t)pwm->tmin_sscm)
      { /* right boundary 03210 */
        int16_t time_v1;
        int16_t time_v2;
        int16_t time_v3;
        uint16_t time_vzero;

        pwm->region=3U;

        /* d3=dmin */
        time_v3 = pwm->tmin_sscm;
        time_v1 = pwm->tmin_sscm - time_vect_3 ;
        time_v2 = time_vect_2 + time_vect_3 - pwm->tmin_sscm;
        time_vzero=32767U - pwm->tmin_sscm - (uint16_t)time_v2 ;

        /* 03210 */
         /* calculation of PWM points */
        t22s = time_vzero/2U;
        t21s = t22s + (uint16_t)time_v3;
        t23s = 16384U;
        t23e = 16384U;
        t22e = t21s + (uint16_t)time_v2;
        t21e = t22e + (uint16_t)time_v1;


        /* calculation of current sampling points */
        pwm->m1 = t21s - 200U;
        pwm->m2 = 16384; // sample in the middle of the period, old: t22e - 200U;

        /* udc sampling point */
        pwm->m_udc=t22s + 100;

        pwm->enable_short_meas_sequence=1;
      }
      else
      { /* no pwm pattern modification needed */
        int16_t time_v2;
        int16_t time_v3;
        uint16_t time_vzero;

        pwm->region=0U;
        time_v2=time_vect_2;
        time_v3=time_vect_3;
        time_vzero=32767U - (uint16_t)time_v2 - (uint16_t)time_v3;

        /* calculation of PWM points */
        t22s=time_vzero/4U;
        t21s=t22s + ( (uint16_t)time_v3/2U );
        t23s=t21s + ( (uint16_t)time_v2/2U );
        t23e=16384U + ( time_vzero/4U );
        t21e=t23e + ( (uint16_t)time_v2/2U );
        t22e=t21e + ( (uint16_t)time_v3/2U );

        /* calculation of current sampling points */
        pwm->m1=t21s - time_v3/4U; //200U;
        pwm->m2=t21e - time_v2/4U; //200U;

        /* udc sampling point */
        pwm->m_udc=16384U;

        pwm->enable_short_meas_sequence=0;
      }

      /* transform times to particular sectors */
      switch ( pwm->sector )
      {
      case 2:
          pwm->t1up=t21s;
          pwm->t2up=t22s;
          pwm->t3up=t23s;
          pwm->t1down=t21e;
          pwm->t2down=t22e;
          pwm->t3down=t23e;
          pwm->udc_meas_channel=1;
          break;
      case 4:
          pwm->t1up=t23s;
          pwm->t2up=t21s;
          pwm->t3up=t22s;
          pwm->t1down=t23e;
          pwm->t2down=t21e;
          pwm->t3down=t22e;
          pwm->udc_meas_channel=2;
          break;
      case 6:
          pwm->t1up=t22s;
          pwm->t2up=t23s;
          pwm->t3up=t21s;
          pwm->t1down=t22e;
          pwm->t2down=t23e;
          pwm->t3down=t21e;
          pwm->udc_meas_channel=0;
          break;

      default:
        /* rise error, because we never should reach this point */
        //set_error(error_software_system_error);
        break;
      }


  }
  }

  /* what is the range of signal here? -> 0 .. 32676, centered around 16384 */


  /* pwm.tx range: 16384 (32768-16384)  .. 49152 (32768+16384) */

  /* prescaling for ePWM */
  pwm->t1up=scale_pwm_sscm(pwm->period, pwm->t1up);
  pwm->t2up=scale_pwm_sscm(pwm->period, pwm->t2up);
  pwm->t3up=scale_pwm_sscm(pwm->period, pwm->t3up);

  pwm->t1down=scale_pwm_sscm(pwm->period, pwm->t1down);
  pwm->t2down=scale_pwm_sscm(pwm->period, pwm->t2down);
  pwm->t3down=scale_pwm_sscm(pwm->period, pwm->t3down);

  pwm->m1=scale_pwm_sscm(pwm->period, pwm->m1);
  pwm->m2=scale_pwm_sscm(pwm->period, pwm->m2);
  pwm->m_udc=scale_pwm_sscm(pwm->period, pwm->m_udc);

}

const u16 math_svm_sin60_LUT[] =
{
             0u,        Table_lScale(  134), Table_lScale(  268), Table_lScale(  402)
, Table_lScale(  536), Table_lScale(  670), Table_lScale(  804), Table_lScale(  938)
, Table_lScale( 1072), Table_lScale( 1206), Table_lScale( 1340), Table_lScale( 1473)
, Table_lScale( 1607), Table_lScale( 1741), Table_lScale( 1875), Table_lScale( 2009)
, Table_lScale( 2143), Table_lScale( 2276), Table_lScale( 2410), Table_lScale( 2544)
, Table_lScale( 2677), Table_lScale( 2811), Table_lScale( 2944), Table_lScale( 3078)
, Table_lScale( 3211), Table_lScale( 3345), Table_lScale( 3478), Table_lScale( 3611)
, Table_lScale( 3744), Table_lScale( 3878), Table_lScale( 4011), Table_lScale( 4144)
, Table_lScale( 4277), Table_lScale( 4409), Table_lScale( 4542), Table_lScale( 4675)
, Table_lScale( 4808), Table_lScale( 4940), Table_lScale( 5073), Table_lScale( 5205)
, Table_lScale( 5337), Table_lScale( 5469), Table_lScale( 5602), Table_lScale( 5734)
, Table_lScale( 5866), Table_lScale( 5997), Table_lScale( 6129), Table_lScale( 6261)
, Table_lScale( 6392), Table_lScale( 6524), Table_lScale( 6655), Table_lScale( 6786)
, Table_lScale( 6917), Table_lScale( 7048), Table_lScale( 7179), Table_lScale( 7310)
, Table_lScale( 7440), Table_lScale( 7571), Table_lScale( 7701), Table_lScale( 7831)
, Table_lScale( 7961), Table_lScale( 8091), Table_lScale( 8221), Table_lScale( 8351)
, Table_lScale( 8480), Table_lScale( 8610), Table_lScale( 8739), Table_lScale( 8868)
, Table_lScale( 8997), Table_lScale( 9126), Table_lScale( 9255), Table_lScale( 9383)
, Table_lScale( 9512), Table_lScale( 9640), Table_lScale( 9768), Table_lScale( 9896)
, Table_lScale(10023), Table_lScale(10151), Table_lScale(10278), Table_lScale(10405)
, Table_lScale(10532), Table_lScale(10659), Table_lScale(10786), Table_lScale(10912)
, Table_lScale(11039), Table_lScale(11165), Table_lScale(11291), Table_lScale(11416)
, Table_lScale(11542), Table_lScale(11667), Table_lScale(11793), Table_lScale(11918)
, Table_lScale(12042), Table_lScale(12167), Table_lScale(12291), Table_lScale(12415)
, Table_lScale(12539), Table_lScale(12663), Table_lScale(12787), Table_lScale(12910)
, Table_lScale(13033), Table_lScale(13156), Table_lScale(13278), Table_lScale(13401)
, Table_lScale(13523), Table_lScale(13645), Table_lScale(13767), Table_lScale(13888)
, Table_lScale(14010), Table_lScale(14131), Table_lScale(14251), Table_lScale(14372)
, Table_lScale(14492), Table_lScale(14613), Table_lScale(14732), Table_lScale(14852)
, Table_lScale(14971), Table_lScale(15090), Table_lScale(15209), Table_lScale(15328)
, Table_lScale(15446), Table_lScale(15564), Table_lScale(15682), Table_lScale(15800)
, Table_lScale(15917), Table_lScale(16034), Table_lScale(16151), Table_lScale(16267)
, Table_lScale(16384), Table_lScale(16499), Table_lScale(16615), Table_lScale(16731)
, Table_lScale(16846), Table_lScale(16960), Table_lScale(17075), Table_lScale(17189)
, Table_lScale(17303), Table_lScale(17417), Table_lScale(17530), Table_lScale(17643)
, Table_lScale(17756), Table_lScale(17869), Table_lScale(17981), Table_lScale(18093)
, Table_lScale(18204), Table_lScale(18316), Table_lScale(18427), Table_lScale(18537)
, Table_lScale(18648), Table_lScale(18758), Table_lScale(18868), Table_lScale(18977)
, Table_lScale(19086), Table_lScale(19195), Table_lScale(19303), Table_lScale(19412)
, Table_lScale(19519), Table_lScale(19627), Table_lScale(19734), Table_lScale(19841)
, Table_lScale(19947), Table_lScale(20054), Table_lScale(20159), Table_lScale(20265)
, Table_lScale(20370), Table_lScale(20475), Table_lScale(20579), Table_lScale(20684)
, Table_lScale(20787), Table_lScale(20891), Table_lScale(20994), Table_lScale(21097)
, Table_lScale(21199), Table_lScale(21301), Table_lScale(21403), Table_lScale(21504)
, Table_lScale(21605), Table_lScale(21706), Table_lScale(21806), Table_lScale(21906)
, Table_lScale(22005), Table_lScale(22104), Table_lScale(22203), Table_lScale(22301)
, Table_lScale(22399), Table_lScale(22497), Table_lScale(22594), Table_lScale(22691)
, Table_lScale(22788), Table_lScale(22884), Table_lScale(22980), Table_lScale(23075)
, Table_lScale(23170), Table_lScale(23265), Table_lScale(23359), Table_lScale(23453)
, Table_lScale(23546), Table_lScale(23639), Table_lScale(23732), Table_lScale(23824)
, Table_lScale(23916), Table_lScale(24007), Table_lScale(24098), Table_lScale(24189)
, Table_lScale(24279), Table_lScale(24369), Table_lScale(24458), Table_lScale(24547)
, Table_lScale(24636), Table_lScale(24724), Table_lScale(24812), Table_lScale(24899)
, Table_lScale(24986), Table_lScale(25073), Table_lScale(25159), Table_lScale(25244)
, Table_lScale(25330), Table_lScale(25414), Table_lScale(25499), Table_lScale(25583)
, Table_lScale(25666), Table_lScale(25749), Table_lScale(25832), Table_lScale(25914)
, Table_lScale(25996), Table_lScale(26077), Table_lScale(26158), Table_lScale(26239)
, Table_lScale(26319), Table_lScale(26399), Table_lScale(26478), Table_lScale(26557)
, Table_lScale(26635), Table_lScale(26713), Table_lScale(26790), Table_lScale(26867)
, Table_lScale(26944), Table_lScale(27020), Table_lScale(27095), Table_lScale(27170)
, Table_lScale(27245), Table_lScale(27319), Table_lScale(27393), Table_lScale(27466)
, Table_lScale(27539), Table_lScale(27612), Table_lScale(27684), Table_lScale(27755)
, Table_lScale(27826), Table_lScale(27897), Table_lScale(27967), Table_lScale(28036)
, Table_lScale(28106), Table_lScale(28174), Table_lScale(28242), Table_lScale(28310)
};

u16 math_get_svm_sin60 ( u16 index )
{
  return math_svm_sin60_LUT[ index ];
}


const uint16_t Table_Sin60[] =
    {
        0, Table_lScale(134), Table_lScale(268), Table_lScale(402), Table_lScale(536), Table_lScale(670), Table_lScale(804), Table_lScale(938), Table_lScale(1072), Table_lScale(1206), Table_lScale(1340), Table_lScale(1473), Table_lScale(1607), Table_lScale(1741), Table_lScale(1875), Table_lScale(2009), Table_lScale(2143), Table_lScale(2276), Table_lScale(2410), Table_lScale(2544), Table_lScale(2677), Table_lScale(2811), Table_lScale(2944), Table_lScale(3078), Table_lScale(3211), Table_lScale(3345), Table_lScale(3478), Table_lScale(3611), Table_lScale(3744), Table_lScale(3878), Table_lScale(4011), Table_lScale(4144), Table_lScale(4277), Table_lScale(4409), Table_lScale(4542), Table_lScale(4675), Table_lScale(4808), Table_lScale(4940), Table_lScale(5073), Table_lScale(5205), Table_lScale(5337), Table_lScale(5469), Table_lScale(5602), Table_lScale(5734), Table_lScale(5866), Table_lScale(5997), Table_lScale(6129), Table_lScale(6261), Table_lScale(6392), Table_lScale(6524), Table_lScale(6655), Table_lScale(6786), Table_lScale(6917), Table_lScale(7048), Table_lScale(7179), Table_lScale(7310), Table_lScale(7440), Table_lScale(7571), Table_lScale(7701), Table_lScale(7831), Table_lScale(7961), Table_lScale(8091), Table_lScale(8221), Table_lScale(8351), Table_lScale(8480), Table_lScale(8610), Table_lScale(8739), Table_lScale(8868), Table_lScale(8997), Table_lScale(9126), Table_lScale(9255), Table_lScale(9383), Table_lScale(9512), Table_lScale(9640), Table_lScale(9768), Table_lScale(9896), Table_lScale(10023), Table_lScale(10151), Table_lScale(10278), Table_lScale(10405), Table_lScale(10532), Table_lScale(10659), Table_lScale(10786), Table_lScale(10912), Table_lScale(11039), Table_lScale(11165), Table_lScale(11291), Table_lScale(11416), Table_lScale(11542), Table_lScale(11667), Table_lScale(11793), Table_lScale(11918), Table_lScale(12042), Table_lScale(12167), Table_lScale(12291), Table_lScale(12415), Table_lScale(12539), Table_lScale(12663), Table_lScale(12787), Table_lScale(12910), Table_lScale(13033), Table_lScale(13156), Table_lScale(13278), Table_lScale(13401), Table_lScale(13523), Table_lScale(13645), Table_lScale(13767), Table_lScale(13888), Table_lScale(14010), Table_lScale(14131), Table_lScale(14251), Table_lScale(14372), Table_lScale(14492), Table_lScale(14613), Table_lScale(14732), Table_lScale(14852), Table_lScale(14971), Table_lScale(15090), Table_lScale(15209), Table_lScale(15328), Table_lScale(15446), Table_lScale(15564), Table_lScale(15682), Table_lScale(15800), Table_lScale(15917), Table_lScale(16034), Table_lScale(16151), Table_lScale(16267), Table_lScale(16384), Table_lScale(16499), Table_lScale(16615), Table_lScale(16731), Table_lScale(16846), Table_lScale(16960), Table_lScale(17075), Table_lScale(17189), Table_lScale(17303), Table_lScale(17417), Table_lScale(17530), Table_lScale(17643), Table_lScale(17756), Table_lScale(17869), Table_lScale(17981), Table_lScale(18093), Table_lScale(18204), Table_lScale(18316), Table_lScale(18427), Table_lScale(18537), Table_lScale(18648), Table_lScale(18758), Table_lScale(18868), Table_lScale(18977), Table_lScale(19086), Table_lScale(19195), Table_lScale(19303), Table_lScale(19412), Table_lScale(19519), Table_lScale(19627), Table_lScale(19734), Table_lScale(19841), Table_lScale(19947), Table_lScale(20054), Table_lScale(20159), Table_lScale(20265), Table_lScale(20370), Table_lScale(20475), Table_lScale(20579), Table_lScale(20684), Table_lScale(20787), Table_lScale(20891), Table_lScale(20994), Table_lScale(21097), Table_lScale(21199), Table_lScale(21301), Table_lScale(21403), Table_lScale(21504), Table_lScale(21605), Table_lScale(21706), Table_lScale(21806), Table_lScale(21906), Table_lScale(22005), Table_lScale(22104), Table_lScale(22203), Table_lScale(22301), Table_lScale(22399), Table_lScale(22497), Table_lScale(22594), Table_lScale(22691), Table_lScale(22788), Table_lScale(22884), Table_lScale(22980), Table_lScale(23075), Table_lScale(23170), Table_lScale(23265), Table_lScale(23359), Table_lScale(23453), Table_lScale(23546), Table_lScale(23639), Table_lScale(23732), Table_lScale(23824), Table_lScale(23916), Table_lScale(24007), Table_lScale(24098), Table_lScale(24189), Table_lScale(24279), Table_lScale(24369), Table_lScale(24458), Table_lScale(24547), Table_lScale(24636), Table_lScale(24724), Table_lScale(24812), Table_lScale(24899), Table_lScale(24986), Table_lScale(25073), Table_lScale(25159), Table_lScale(25244), Table_lScale(25330), Table_lScale(25414), Table_lScale(25499), Table_lScale(25583), Table_lScale(25666), Table_lScale(25749), Table_lScale(25832), Table_lScale(25914), Table_lScale(25996), Table_lScale(26077), Table_lScale(26158), Table_lScale(26239), Table_lScale(26319), Table_lScale(26399), Table_lScale(26478), Table_lScale(26557), Table_lScale(26635), Table_lScale(26713), Table_lScale(26790), Table_lScale(26867), Table_lScale(26944), Table_lScale(27020), Table_lScale(27095), Table_lScale(27170), Table_lScale(27245), Table_lScale(27319), Table_lScale(27393), Table_lScale(27466), Table_lScale(27539), Table_lScale(27612), Table_lScale(27684), Table_lScale(27755), Table_lScale(27826), Table_lScale(27897), Table_lScale(27967), Table_lScale(28036), Table_lScale(28106), Table_lScale(28174), Table_lScale(28242), Table_lScale(28310)};

const u16 arctan_LUT[MATH_ARCTAN_LUT_LENGTH] =
    {
        0u, 40u, 81u, 122u, 162u, 203u, 244u, 285u, 325u, 366u, 407u, 447u, 488u,
        529u, 569u, 610u, 651u, 691u, 732u, 772u, 813u, 853u, 894u, 934u, 974u,
        1015u, 1055u, 1096u, 1136u, 1176u, 1216u, 1256u, 1297u, 1337u, 1377u, 1417u,
        1457u, 1497u, 1537u, 1576u, 1616u, 1656u, 1696u, 1735u, 1775u, 1814u, 1854u,
        1893u, 1933u, 1972u, 2011u, 2051u, 2090u, 2129u, 2168u, 2207u, 2246u, 2285u,
        2323u, 2362u, 2401u, 2439u, 2478u, 2516u, 2555u, 2593u, 2631u, 2669u, 2707u,
        2746u, 2783u, 2821u, 2859u, 2897u, 2934u, 2972u, 3010u, 3047u, 3084u, 3121u,
        3159u, 3196u, 3233u, 3270u, 3306u, 3343u, 3380u, 3416u, 3453u, 3489u, 3526u,
        3562u, 3598u, 3634u, 3670u, 3706u, 3742u, 3777u, 3813u, 3848u, 3884u, 3919u,
        3954u, 3989u, 4024u, 4059u, 4094u, 4129u, 4163u, 4198u, 4232u, 4267u, 4301u,
        4335u, 4369u, 4403u, 4437u, 4471u, 4504u, 4538u, 4571u, 4605u, 4638u, 4671u,
        4704u, 4737u, 4770u, 4803u, 4835u, 4868u, 4900u, 4933u, 4965u, 4997u, 5029u,
        5061u, 5093u, 5125u, 5156u, 5188u, 5219u, 5251u, 5282u, 5313u, 5344u, 5375u,
        5406u, 5436u, 5467u, 5497u, 5528u, 5558u, 5588u, 5618u, 5648u, 5678u, 5708u,
        5737u, 5767u, 5796u, 5826u, 5855u, 5884u, 5913u, 5942u, 5971u, 6000u, 6028u,
        6057u, 6085u, 6114u, 6142u, 6170u, 6198u, 6226u, 6254u, 6281u, 6309u, 6337u,
        6364u, 6391u, 6419u, 6446u, 6473u, 6500u, 6526u, 6553u, 6580u, 6606u, 6633u,
        6659u, 6685u, 6711u, 6737u, 6763u, 6789u, 6815u, 6841u, 6866u, 6892u, 6917u,
        6942u, 6967u, 6992u, 7017u, 7042u, 7067u, 7092u, 7116u, 7141u, 7165u, 7189u,
        7214u, 7238u, 7262u, 7286u, 7310u, 7333u, 7357u, 7381u, 7404u, 7427u, 7451u,
        7474u, 7497u, 7520u, 7543u, 7566u, 7589u, 7611u, 7634u, 7656u, 7679u, 7701u,
        7723u, 7746u, 7768u, 7790u, 7811u, 7833u, 7855u, 7877u, 7898u, 7920u, 7941u,
        7962u, 7984u, 8005u, 8026u, 8047u, 8068u, 8089u, 8109u, 8130u, 8150u, 8171u,
        8191u};

/**
   @var math_svm_value_LUT
   @brief lookup table for SVM waveform
 */
const s16 math_svm_value_LUT[MATH_SINE_LUT_LENGTH] =
    {
        0, 4817, 9588, 14267, 18809, 23169, 25450, 26558, 27410, 27998, 28316, 28361,
        28134, 27635, 26871, 25847, 24575, 25847, 26871, 27635, 28134, 28361, 28316,
        27998, 27410, 26558, 25450, 23169, 18809, 14267, 9588, 4817, 0, -4817, -9588,
        -14267, -18809, -23169, -25450, -26558, -27410, -27998, -28316, -28361, -28134,
        -27635, -26871, -25847, -24575, -25847, -26871, -27635, -28134, -28361, -28316,
        -27998, -27410, -26558, -25450, -23169, -18809, -14267, -9588, -4817};

/**
   @var math_svm_gradient_LUT
   @brief lookup table for SVM waveform gradient
 */
const s16 math_svm_gradient_LUT[MATH_SINE_LUT_LENGTH] =
    {
        1204, 1192, 1169, 1135, 1090, 570, 276, 212, 146, 79, 11, -56, -124, -191, -255,
        -318, 318, 255, 191, 124, 56, -11, -79, -146, -212, -276, -570, -1090, -1135,
        -1169, -1192, -1204, -1204, -1192, -1169, -1135, -1090, -570, -276, -212, -146,
        -79, -11, 56, 124, 191, 255, 318, -318, -255, -191, -124, -56, 11, 79, 146, 212,
        276, 570, 1090, 1135, 1169, 1192, 1204};

/**
   @var math_sine_value_LUT
   @brief lookup table for sine waveform
 */
const s16 math_sine_value_LUT[MATH_SINE_LUT_LENGTH] =
    {
        0, 3207, 6384, 9499, 12523, 15426, 18181, 20761, 23140, 25297, 27210, 28861,
        30234, 31316, 32097, 32568, 32726, 32568, 32097, 31316, 30234, 28861, 27210,
        25297, 23140, 20761, 18181, 15426, 12523, 9499, 6384, 3207, 0, -3207, -6384,
        -9499, -12523, -15426, -18181, -20761, -23140, -25297, -27210, -28861, -30234,
        -31316, -32097, -32568, -32726, -32568, -32097, -31316, -30234, -28861, -27210,
        -25297, -23140, -20761, -18181, -15426, -12523, -9499, -6384, -3207};

/**
   @var math_sine_gradient_LUT
   @brief lookup table for sine waveform gradient
 */
const s16 math_sine_gradient_LUT[MATH_SINE_LUT_LENGTH] =
    {
        801, 794, 778, 756, 725, 688, 645, 594, 539, 478, 412, 343, 270, 195, 117,
        39, -39, -118, -196, -271, -344, -413, -479, -540, -595, -645, -689, -726,
        -756, -779, -795, -802, -802, -795, -779, -756, -726, -689, -645, -595, -540,
        -479, -413, -344, -271, -196, -118, -39, 39, 117, 195, 270, 343, 412, 478,
        539, 594, 645, 688, 725, 756, 778, 794, 801};

u16 math_get_angle_unsafe(s16 y, s16 x)
{
  u16 result = 0u;

  if (x > 0)
  {
    if (y > 0)
    {
      if (x < y)
      {
        /* x > 0  &  y > 0  &  x < y */
        result = arctan_LUT[(((s32)x) << MATH_ARCTAN_LUT_LENGTH_BITS) / y];
      }
      else
      {
        /* x > 0  &  y > 0  &  x >= y */
        result = (u16)(65535UL / 4UL) - arctan_LUT[(((s32)y) << MATH_ARCTAN_LUT_LENGTH_BITS) / x];
      }
    }
    else
    {
      y = -y;
      if (x < y)
      {
        /* x > 0  &  y <= 0  & x < y */
        result = (u16)(65535UL / 2UL) - arctan_LUT[(((s32)x) << MATH_ARCTAN_LUT_LENGTH_BITS) / y];
      }
      else
      {
        /* x > 0  &  y <= 0  & x >= y */
        result = (u16)(65535UL / 4UL) + arctan_LUT[(((s32)y) << MATH_ARCTAN_LUT_LENGTH_BITS) / x];
      }
    }
  }
  else
  {
    x = -x;
    if (y > 0)
    {
      if (x < y)
      {
        /* x <= 0  &  y > 0  & x < y */
        result = (u16)(65535UL) - arctan_LUT[(((s32)x) << MATH_ARCTAN_LUT_LENGTH_BITS) / y];
      }
      else
      {
        /* x <= 0  &  y > 0  & x >= y */
        result = (u16)(65535UL * 3UL / 4UL) + arctan_LUT[(((s32)y) << MATH_ARCTAN_LUT_LENGTH_BITS) / x];
      }
    }
    else
    {
      y = -y;

      if (x < y)
      {
        /* x <= 0  &  y <= 0  &  x < y */
        result = (u16)(65535UL / 2UL) + arctan_LUT[(((s32)x) << MATH_ARCTAN_LUT_LENGTH_BITS) / y];
      }
      else
      {
        if (x == 0)
        {
          /* x == 0  &  y <= 0  &  x >= y */
          result = (u16)(65535UL * 3UL / 4UL) - arctan_LUT[MATH_ARCTAN_LUT_LENGTH - 1];
        }
        else
        {
          /* x < 0  &  y <= 0  &  x >= y */
          result = (u16)(65535UL * 3UL / 4UL) - arctan_LUT[(((s32)y) << MATH_ARCTAN_LUT_LENGTH_BITS) / x];
        }
      }
    }
  }

  return result;
}

s16 math_sine_unsafe(const u16 alpha)
{
  s16 loc_return_value = 0;
  s16 result;

  /* with interpolation and slope lookup table */
  u16 idx = alpha >> (16u - MATH_SINE_LUT_LENGTH_BITS);
  s16 sinL = math_sine_value_LUT[idx];

  result = ((s32)((s16)alpha & (s16)(0xFFFF >> MATH_SINE_LUT_LENGTH_BITS)) * math_sine_gradient_LUT[idx]) >> 8u;

  loc_return_value = (s16)result + sinL;

  return loc_return_value;
}

s16 math_cosine_unsafe(const u16 alpha)
{
  return math_sine_unsafe(alpha + (u16)(MATH_ANGLE_MAX / 4u));
}

#if 1
void foc_svm_improved ( foc_ctrl_loop_data_t * foc_data, s16 a, s16 b, u16 svm_switch )
{
  u16 V_TA;
  u16 V_TB;
  u16 T1;
  u16 T2;
  u16 Sector;
  u32 Angle;
  u16 Index;
  u16 Time;
  u16 Compare0up;
  u16 Compare1up;
  u16 Compare2up;
  u16 Compare0down;
  u16 Compare1down;
  u16 Compare2down;
  u16 T13ValueUp;
  u16 T13ValueDown;
  u16 i;
  u16 per;

  static s16 time_vect_2;
  static s16 time_vect_3;
  static u16 time_vzero;
  static s16 time_v2;
  static s16 time_v3;
  static s16 time_v5;
  static s16 time_v6;
  s16 uab_re;
  s16 uab_im;
  s16 temp;
  u16 mval;

  foc_data->svm_phi = math_get_angle_unsafe( b, a );
#if 0
  if( abs( a ) > abs( b ) )
  {
    /* length = alpha / cos(phi) */
    /*foc_data->svm_length = (s16) math_divS32_unsafe( ( (s32) a ) << 15, math_cosine_unsafe( foc_data->svm_phi ) );*/
    foc_data->svm_length = (s16)((((s32) a) << 15) / math_cosine_unsafe(foc_data->svm_phi));
  }
  else
  {
    /* length = beta / sin(phi) */
    /*foc_data->svm_length = (s16) math_divS32_unsafe( ( (s32) b ) << 15, math_sine_unsafe( foc_data->svm_phi ) );*/
    foc_data->svm_length = (s16)((((s32) b) << 15) / math_sine_unsafe(foc_data->svm_phi));
  }
  #endif // 0


  /* in case of sector borders this defines the min. Null Vector duration **
   ** 10 ticks => 250ns@40MHz                                              **/
  per = (u16) ( PWM_SINGLE_SHUNT_MAX_CMP_VAL - PWM_SVM_CNT_LIM_MARGIN );

  if( svm_switch == 0u )
  {/*modified modulation for low output amplitudes, due to current measurements*/
  #if 0

    /* prescale input signal so that output amplitudes of both algorithms is the same
     * TODO put value 16200 into #define parameter, value was calculated with simulation*/
    /*uab_re = ( (s32) a * 16200 ) >> 10u;*/
    math_mulS16_unsafe( a, 16200 );
    math_mul_shiftright_result( 10u );
    uab_re = (s16) math_mul_get_result();

    /*uab_im = ( (s32) b * 16200 ) >> 10u;*/
    math_mulS16_unsafe( b, 16200 );
    math_mul_shiftright_result( 10u );
    uab_im = (s16) math_mul_get_result();

    /*TODO optimiziraj, dolocena mnozenja lahko odstranis ker se ponavljajo*/
    /* calculate linear combination of vectors v2 & v3 */
    /*time_vect_2 = ( (s32) uab_re * (s32) ( 3. / 2. / 2. * 2400. ) + (s32) uab_im * (s32) ( SQRT3 / 2. / 2. * 2400. ) ) >> 14U;*/
    math_mulS16_unsafe( uab_re, (s16) ( 3. / 2. / 2. * (double) PWM_PERIOD_DURATION_TICKS ) );
    math_mulS16_acc_unsafe( uab_im, (s16) ( SQRT3 / 2. / 2. * (double) PWM_PERIOD_DURATION_TICKS ) );
    math_mul_shiftright_result( 14u );
    time_vect_2 = (s16) math_mul_get_result();

    /*time_vect_3 = ( (s32) uab_re * (s32) ( -3. / 2. / 2. * 2400. ) + (s32) uab_im * (s32) ( SQRT3 / 2. / 2. * 2400. ) ) >> 14U;*/
    math_mulS16_unsafe( uab_re, (s16) ( -3. / 2. / 2. * (double) PWM_PERIOD_DURATION_TICKS ) );
    math_mulS16_acc_unsafe( uab_im, (s16) ( SQRT3 / 2. / 2. * (double) PWM_PERIOD_DURATION_TICKS ) );
    math_mul_shiftright_result( 14u );
    time_vect_3 = (s16) math_mul_get_result();


    if( time_vect_2 >= 0 )
    {
      temp    = time_vect_2 + (s16) PWM_SVM_MIN_TIME;
      time_v2 = (s16) temp;
      time_v5 = (s16) PWM_SVM_MIN_TIME;
    }
    else
    {
      time_v2 = (s16) PWM_SVM_MIN_TIME;
      temp    = -time_vect_2 + (s16) PWM_SVM_MIN_TIME;
      time_v5 = (s16) temp;
    }

    if( time_vect_3 >= 0 )
    {
      temp    = time_vect_3 + (s16) PWM_SVM_MIN_TIME;
      time_v3 = (s16) temp;
      time_v6 = (s16) PWM_SVM_MIN_TIME;
    }
    else
    {
      time_v3 = (s16) PWM_SVM_MIN_TIME;
      temp    = -time_vect_3 + (s16) PWM_SVM_MIN_TIME;
      time_v6 = (s16) temp;
    }

    time_vzero = ( (u16) PWM_PERIOD_DURATION_TICKS - (u16) time_v2 - (u16) time_v3 - (u16) time_v5 - (u16) time_v6 );

    Compare1up   = (u16) ( ( (u32) time_vzero * 7u ) >> 4U );
    Compare0up   = Compare1up + (u16) time_v3;
    Compare2up   = Compare0up + (u16) time_v2;
    Compare1down = ( Compare2up + ( time_vzero >> 3U ) );
    Compare0down = ( Compare1down + (u16) time_v6 );
    Compare2down = ( Compare0down + (u16) time_v5 );
    #if 1
      foc_data->svm_i1_samp_time_up   = Compare0up - ( PWM_SVM_MIN_TIME / 2u ) + PWM_SVM_STIME_OFFSET_LOWA;
      foc_data->svm_i2_samp_time_down = Compare2down - ( PWM_SVM_MIN_TIME / 2u ) + PWM_SVM_STIME_OFFSET_LOWA;
    #else
      foc_data->svm_i1_samp_time_up   = Compare0up - (u16) math_divU32_unsafe( (u32) time_v3, 2u ) + PWM_SVM_STIME_OFFSET_LOWA;
      foc_data->svm_i2_samp_time_down = Compare0down + (u16) math_divU32_unsafe( (u32) time_v5, 2u ) + PWM_SVM_STIME_OFFSET_LOWA;
    #endif
  #endif
  }
  else
  {
#if 0
  Angle = (u32)foc_data->svm_phi*6u;
  mval = Angle>>16u;
  Sector = mval & 7u;
  foc_data->svm_sector = (u16) Sector;
  Index                = ( (u16) ( Angle >> 8u ) ) & 0xFFu;
#else
    math_mulU16_unsafe( foc_data->svm_phi, 6u );
    Angle = (u32) math_mul_get_result();

    math_mul_shiftright_result( 16u );
    Sector               = ( u16 ) math_mul_get_result() & 7u;
    foc_data->svm_sector = (u16) Sector;
    Index                = ( (u16) ( Angle >> 8u ) ) & 0xFFu;
#endif

    /* Calculate and limit times */
    /* RandVektor0 = Amp * sin(60 - gamma) */
    //T1 = ( ( (u32) foc_data->svm_length ) * math_get_svm_sin60( (u16) ( 255u - Index ) ) ) >> 15u;
    math_mulU16_unsafe( (u16) foc_data->svm_length, math_get_svm_sin60( (u16) ( 255u - Index ) ) );
    math_mul_shiftright_result( 15u );
    T1 = ( u16 ) math_mul_get_result();

    /* RandVektor1 = Amp * sin(gamma) */
    //T2 = ( ( (u32) foc_data->svm_length ) * math_get_svm_sin60( (u16) Index ) ) >> 15u;
    math_mulU16_unsafe( (u16) foc_data->svm_length, math_get_svm_sin60( (u16) Index ) );
    math_mul_shiftright_result( 15u );
    T2 = ( u16 ) math_mul_get_result();

    /*calc. mean value between T1 and T2*/
    Time = ( T1 + T2 ) >> 1u;

    /*calculate compare values for phases modulated by Amplitude */
    V_TA = (u16) ( PWM_SINGLE_SHUNT_MAX_CMP_VAL >> 2u ) + Time;
    V_TB = (u16) ( PWM_SINGLE_SHUNT_MAX_CMP_VAL >> 2u ) - Time;


    /* Set compare values according to sector number */
    switch( Sector )
    {
      case 0u:
      {
        Compare0up   = V_TB;
        Compare1up   = V_TB + T1;
        Compare2up   = V_TA;
        Compare0down = (u16) PWM_SINGLE_SHUNT_MAX_CMP_VAL - Compare0up;
        Compare1down = (u16) PWM_SINGLE_SHUNT_MAX_CMP_VAL - Compare1up;
        Compare2down = (u16) PWM_SINGLE_SHUNT_MAX_CMP_VAL - Compare2up;

        if( T1 < PWM_SVM_MIN_TIME )
        {
          /* pattern has to be applied asymetric because T1 too short */
          i = PWM_SVM_MIN_TIME - T1;
          if( ( Compare0down + i ) > per )
          {
            i = i + ( per - ( Compare0down + i ) );
          }
          Compare0up   = Compare0up + i;
          Compare0down = Compare0down + i;
        }

        /* at least T1 has enough room for ADC measurement **
        ** now check also T2 against min. SVM Time         */
        /*				T1Halbe = (uint16)T1/2;*/
        if( T2 < PWM_SVM_MIN_TIME )
        {
          /* pattern has to be applied asymetric because of T2 too short */
          i = PWM_SVM_MIN_TIME - T2;
          if( ( Compare2up + i ) > ( (u16) PWM_SINGLE_SHUNT_MAX_CMP_VAL >> 1u ) )
          {
            i = i + ( ( (u16) PWM_SINGLE_SHUNT_MAX_CMP_VAL >> 1u ) - ( Compare2up + i ) );
          }
          Compare2up   = Compare2up + i;
          Compare2down = Compare2down + i;
        }

        T13ValueUp   = ( Compare1up + Compare2up ) >> 1u;
        T13ValueDown = (u16) PWM_SINGLE_SHUNT_MAX_CMP_VAL - ( ( Compare1down + Compare0down ) >> 1u );
        break;
      }
      case 1u:
      {
        Compare0up   = V_TB + T2;
        Compare1up   = V_TB;
        Compare2up   = V_TA;
        Compare0down = (u16) PWM_SINGLE_SHUNT_MAX_CMP_VAL - Compare0up;
        Compare1down = (u16) PWM_SINGLE_SHUNT_MAX_CMP_VAL - Compare1up;
        Compare2down = (u16) PWM_SINGLE_SHUNT_MAX_CMP_VAL - Compare2up;

        if( T2 < PWM_SVM_MIN_TIME ){
          i = PWM_SVM_MIN_TIME - T2;
          if( ( Compare1down + i ) > per ){
            i = i + ( per - ( Compare1down + i ) );
          }
          Compare1up   = Compare1up + i;
          Compare1down = Compare1down + i;
        }

        if( T1 < PWM_SVM_MIN_TIME ){
          i = PWM_SVM_MIN_TIME - T1;
          if( ( Compare2up + i ) > ( (u16) PWM_SINGLE_SHUNT_MAX_CMP_VAL >> 1u ) )
          {
            i = i + ( ( (u16) PWM_SINGLE_SHUNT_MAX_CMP_VAL >> 1u ) - ( Compare2up + i ) );
          }
          Compare2up   = Compare2up + i;
          Compare2down = Compare2down + i;
        }

        T13ValueUp   = ( Compare0up + Compare2up ) >> 1u;
        T13ValueDown = (u16) PWM_SINGLE_SHUNT_MAX_CMP_VAL - ( ( Compare0down + Compare1down ) >> 1u );
        break;
      }
      case 2u:
      {
        Compare0up   = V_TA;
        Compare1up   = V_TB;
        Compare2up   = V_TB + T1;
        Compare0down = (u16) PWM_SINGLE_SHUNT_MAX_CMP_VAL - Compare0up;
        Compare1down = (u16) PWM_SINGLE_SHUNT_MAX_CMP_VAL - Compare1up;
        Compare2down = (u16) PWM_SINGLE_SHUNT_MAX_CMP_VAL - Compare2up;
        if( T1 < PWM_SVM_MIN_TIME ){
          i = PWM_SVM_MIN_TIME - T1;
          if( ( Compare1down + i ) > per )
          {
            i = i + ( per - ( Compare1down + i ) );
          }
          Compare1up   = Compare1up + i;
          Compare1down = Compare1down + i;
        }

        if( T2 < PWM_SVM_MIN_TIME ){
          i = PWM_SVM_MIN_TIME - T2;
          if( ( Compare0up + i ) > ( (u16) PWM_SINGLE_SHUNT_MAX_CMP_VAL >> 1u ) )
          {
            i = i + ( ( (u16) PWM_SINGLE_SHUNT_MAX_CMP_VAL >> 1u ) - ( Compare0up + i ) );
          }
          Compare0up   = Compare0up + i;
          Compare0down = Compare0down + i;
        }

        T13ValueUp   = ( Compare2up + Compare0up ) >> 1u;
        T13ValueDown = (u16) PWM_SINGLE_SHUNT_MAX_CMP_VAL - ( ( Compare2down + Compare1down ) >> 1u );
        break;
      }
      case 3u:
      {
        Compare0up   = V_TA;
        Compare1up   = V_TB + T2;
        Compare2up   = V_TB;
        Compare0down = (u16) PWM_SINGLE_SHUNT_MAX_CMP_VAL - Compare0up;
        Compare1down = (u16) PWM_SINGLE_SHUNT_MAX_CMP_VAL - Compare1up;
        Compare2down = (u16) PWM_SINGLE_SHUNT_MAX_CMP_VAL - Compare2up;

        if( T2 < PWM_SVM_MIN_TIME ){
          i = PWM_SVM_MIN_TIME - T2;
          if( ( Compare2down + i ) > per )
          {
            i = i + ( per - ( Compare2down + i ) );
          }
          Compare2up   = Compare2up + i;
          Compare2down = Compare2down + i;
        }

        if( T1 < PWM_SVM_MIN_TIME ){
          i = PWM_SVM_MIN_TIME - T1;
          if( ( Compare0up + i ) > ( (u16) PWM_SINGLE_SHUNT_MAX_CMP_VAL >> 1u ) )
          {
            i = i + ( ( (u16) PWM_SINGLE_SHUNT_MAX_CMP_VAL >> 1u ) - ( Compare0up + i ) );
          }
          Compare0up   = Compare0up + i;
          Compare0down = Compare0down + i;
        }

        T13ValueUp   = ( Compare1up + Compare0up ) >> 1u;
        T13ValueDown = (u16) PWM_SINGLE_SHUNT_MAX_CMP_VAL - ( ( Compare1down + Compare2down ) >> 1u );
        break;
      }
      case 4u:
      {
        Compare0up   = V_TB + T1;
        Compare1up   = V_TA;
        Compare2up   = V_TB;
        Compare0down = (u16) PWM_SINGLE_SHUNT_MAX_CMP_VAL - Compare0up;
        Compare1down = (u16) PWM_SINGLE_SHUNT_MAX_CMP_VAL - Compare1up;
        Compare2down = (u16) PWM_SINGLE_SHUNT_MAX_CMP_VAL - Compare2up;

        if( T1 < PWM_SVM_MIN_TIME ){
          i = PWM_SVM_MIN_TIME - T1;
          if( ( Compare2down + i ) > per )
          {
            i = i + ( per - ( Compare2down + i ) );
          }
          Compare2up   = Compare2up + i;
          Compare2down = Compare2down + i;
        }

        if( T2 < PWM_SVM_MIN_TIME ){
          i = PWM_SVM_MIN_TIME - T2;
          if( ( Compare1up + i ) > ( (u16) PWM_SINGLE_SHUNT_MAX_CMP_VAL >> 1u ) ){
            i = i + ( ( (u16) PWM_SINGLE_SHUNT_MAX_CMP_VAL >> 1u ) - ( Compare1up + i ) );
          }
          Compare1up   = Compare1up + i;
          Compare1down = Compare1down + i;
        }

        T13ValueUp   = ( Compare0up + Compare1up ) >> 1u;
        T13ValueDown = (u16) PWM_SINGLE_SHUNT_MAX_CMP_VAL - ( ( Compare0down + Compare2down ) >> 1u );
        break;
      }
      default:                                                       /* case 5u: */
      {
        Compare0up   = V_TB;
        Compare1up   = V_TA;
        Compare2up   = V_TB + T2;
        Compare0down = (u16) PWM_SINGLE_SHUNT_MAX_CMP_VAL - Compare0up;
        Compare1down = (u16) PWM_SINGLE_SHUNT_MAX_CMP_VAL - Compare1up;
        Compare2down = (u16) PWM_SINGLE_SHUNT_MAX_CMP_VAL - Compare2up;

        if( T2 < PWM_SVM_MIN_TIME ){
          i = PWM_SVM_MIN_TIME - T2;
          if( ( Compare0down + i ) > per )
          {
            i = i + ( per - ( Compare0down + i ) );
          }
          Compare0up   = Compare0up + i;
          Compare0down = Compare0down + i;
        }

        if( T1 < PWM_SVM_MIN_TIME ){
          i = PWM_SVM_MIN_TIME - T1;
          if( ( Compare1up + i ) > ( ( (u16) PWM_SINGLE_SHUNT_MAX_CMP_VAL >> 1u ) - 10u ) )
          {
            i = i + ( ( (u16) PWM_SINGLE_SHUNT_MAX_CMP_VAL >> 1u ) - ( Compare1up + i ) );
          }
          Compare1up   = Compare1up + i;
          Compare1down = Compare1down + i;
        }

        T13ValueUp   = ( Compare2up + Compare1up ) >> 1u;
        T13ValueDown = (u16) PWM_SINGLE_SHUNT_MAX_CMP_VAL - ( ( Compare2down + Compare0down ) >> 1u );
        break;
      }
    }

    /* Set pwm compare values for sampling phase currents */
    foc_data->svm_i1_samp_time_up = (u16) T13ValueUp + ( PWM_SVM_MIN_TIME / 4u ); /*+ dbg.svm_stime_offset;*//*PWM_SVM_STIME_OFFSET_HIGHA;*/
    foc_data->svm_i2_samp_time_down = (u16) PWM_SINGLE_SHUNT_MAX_CMP_VAL - (u16) T13ValueDown + ( PWM_SVM_MIN_TIME / 4u ); /*+ dbg.svm_stime_offset;*//*PWM_SVM_STIME_OFFSET_HIGHA;*/
  }

  if( Compare0up < PWM_SVM_CNT_LIM_MARGIN ){Compare0up = PWM_SVM_CNT_LIM_MARGIN;}
  if( Compare0down > per ){Compare0down = per;}
  if( Compare1up < PWM_SVM_CNT_LIM_MARGIN ){Compare1up = PWM_SVM_CNT_LIM_MARGIN;}
  if( Compare1down > per ){Compare1down = per;}
  if( Compare2up < PWM_SVM_CNT_LIM_MARGIN ){Compare2up = PWM_SVM_CNT_LIM_MARGIN;}
  if( Compare2down > per ){Compare2down = per;}

  if( foc_data->svm_i1_samp_time_up >= (u16) ( PWM_SINGLE_SHUNT_MAX_CMP_VAL / 2u ) )
  {
    foc_data->svm_i1_samp_time_up = (u16) ( PWM_SINGLE_SHUNT_MAX_CMP_VAL / 2u );
  }
  if( foc_data->svm_i2_samp_time_down >= (u16) ( PWM_SINGLE_SHUNT_MAX_CMP_VAL ) )
  {
    foc_data->svm_i2_samp_time_down = (u16) ( PWM_SINGLE_SHUNT_MAX_CMP_VAL );
  }

  foc_data->svm_t0_up   = (u16) Compare0up;
  foc_data->svm_t0_down = (u16) Compare0down;
  foc_data->svm_t1_up   = (u16) Compare1up;
  foc_data->svm_t1_down = (u16) Compare1down;
  foc_data->svm_t2_up   = (u16) Compare2up;
  foc_data->svm_t2_down = (u16) Compare2down;

#if 0
  /*voltages v_u, v_v, v_w are needed for speed measurement*/
  math_mulS16_unsafe( math_svm_waveform_unsafe( foc_data->svm_phi + (u16) ( MATH_ANGLE_MAX / 4u ) ), foc_data->svm_length );
  math_mul_shiftright_result( 15u );
  foc_data->v_u = ( s16 ) math_mul_get_result();

  math_mulS16_unsafe( math_svm_waveform_unsafe( foc_data->svm_phi + (u16) ( MATH_ANGLE_MAX / 4u ) - (u16) ( MATH_ANGLE_MAX / 3u ) ), foc_data->svm_length );
  math_mul_shiftright_result( 15u );
  foc_data->v_v = ( s16 ) math_mul_get_result();

  math_mulS16_unsafe( math_svm_waveform_unsafe( foc_data->svm_phi + (u16) ( MATH_ANGLE_MAX / 4u ) + (u16) ( MATH_ANGLE_MAX / 3u ) ), foc_data->svm_length );
  math_mul_shiftright_result( 15u );
  foc_data->v_w = ( s16 ) math_mul_get_result();
#endif

  /*za test*/
  foc_data->ua   = (u16) PWM_SINGLE_SHUNT_MAX_CMP_VAL - ( Compare0down - Compare0up );
  foc_data->va   = (u16) PWM_SINGLE_SHUNT_MAX_CMP_VAL - ( Compare1down - Compare1up );
  foc_data->wa   = (u16) PWM_SINGLE_SHUNT_MAX_CMP_VAL - ( Compare2down - Compare2up );
  foc_data->u_vu = (s16) foc_data->va - (s16) foc_data->ua;
  foc_data->u_wv = (s16) foc_data->wa - (s16) foc_data->va;
  foc_data->u_uw = (s16) foc_data->ua - (s16) foc_data->wa;

}
#endif

void mc_svm_extract_phase_currents( void )
{
  u16 sector;
  s16 im1;
  s16 im2;

  /* im1 is first measurement from adc list, sampled in first half of pwm period, value im1 is positive
   * im2 is second measurement from adc list, sampled in second half of pwm period, value im2 is negative */
#define IOFFSET 2048
  im1 = ( (s16) adc.i1_digits - IOFFSET  );
  im2 = ( IOFFSET  - (s16) adc.i2_digits );

/* Use sector calculated in previous calculation period, otherwise wrong current will be calculated!
 * Sector and duty cycles are calculated at the end of the period, but phase currents are calculate at
 * beginning of the period where ad results are valid for sector calculated in previous period.
 */

  sector = foc_data.svm_sector;

  switch( sector )
  {
    case 0u:
    {   /*measurement im1, im2 -> iw, -iu*/
      adc.iu.value= im2;
      adc.iw.value= im1;
      adc.iv.value= -( adc.iw.value + adc.iu.value ) ;

    } break;

    case 1u:
    {   /*measurement im1, im2 -> iw, -iv*/
      adc.iv.value= im2;
      adc.iw.value= im1;
      adc.iu.value=( -( adc.iw.value + adc.iv.value ) );
    } break;

    case 2u:
    {   /*measurement im1, im2 -> iu, -iv*/
      adc.iu.value= im1;
      adc.iv.value= im2;
      adc.iw.value= -( adc.iu.value + adc.iv.value );

    } break;

    case 3u:
    {   /*measurement im1, im2 -> iu, -iw*/
      adc.iu.value= im1;
      adc.iw.value= im2;
      adc.iv.value=( -( adc.iw.value + adc.iu.value ) );
    } break;

    case 4u:
    {   /*measurement im1, im2 -> iv, -iw*/
      adc.iv.value= im1;
      adc.iw.value= im2;
      adc.iu.value=( -( adc.iw.value + adc.iv.value ) );

    } break;

    case 5u:
    {   /*measurement im1, im2 -> iv, -iu*/
      adc.iu.value= im2;
      adc.iv.value= im1;
      adc.iw.value=( -( adc.iv.value + adc.iu.value ) );
    } break;

    default:
    {
      /* unexpected error => halt motor, reset device */

    } break;

  }
   foc_data.svm_stored_sector = foc_data.svm_sector;

}
