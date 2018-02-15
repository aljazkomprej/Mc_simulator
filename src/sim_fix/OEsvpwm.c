#include "OEsvpwm.h"


#define	VECTOR1	    0x0000U     /*!< 0 degrees */
#define	VECTOR2	    0x2AAAU	/*!< 60 degrees */
#define	VECTOR3	    0x5555U	/*!< 120 degrees */
#define	VECTOR4	    0x8000U	/*!< 180 degrees */
#define	VECTOR5	    0xAAAAU	/*!< 240 degrees */
#define	VECTOR6     0xD555U	/*!< 300 degrees */
#define	SIXTY_DEG   0x2AAAU	/*!< 60 degrees */

#define VECTOR_LIMIT             30583                /*!< 95% Spacevector */
#define VECTOR_LIMIT_SQUARE      28543               /*!< VECTOR_LIMIT * VECTOR_LIMIT >> 15 */
#define FW_ACTIVE_VECTOR         (VECTOR_LIMIT-250) /*!< Voltagevector value limit to set fw current */
#define FW_DEACTIVE_VECTOR         (VECTOR_LIMIT-1000) /*!< Voltagevector value limit to reset fw current */

#define MIN_EDGE_DIFF           80U                 /*!< Minimum difference of dutycycle between two phase voltage edges */
#define CURRENT_TRIGGER_DELAY   (24U)               /*!< delay from edge to measure point */

static uint16_t PWMN_Max_Counter = (uint16_t)((FSYS / PWM_FREQUENCY)>>1); /* Period time (uC_Frequency/ PWM Frequency)/2 */
// PWM_Max_Cunter=2400/2=1200;

/* Variables used by FOC_SVM() */
static uint8_t  u8_sector;                              /*!< Sector of SpaceVector */

/* Variables used by FOC_CarthesianToPolar() */
uint16_t u16_SpaceVectorAngle;                   /*!< Space vector angle */
uint16_t u16_SpaceVectorABS;
DutyCycle_t DutyCycle;
                   /*!< Space vector length */

/*!< Sinus Table 60 degrees*/
static const uint16_t u16c_sinetable[172] = {
0U,     201U,   401U,   602U,   803U,   1003U,  1204U,  1404U,  1605U,  1805U,  2005U,  2206U,  2406U,
2606U,  2806U,  3006U,  3205U,  3405U,  3605U,  3804U,  4003U,  4202U,  4401U,  4600U,  4799U,  4997U,
5195U,  5393U,  5591U,  5789U,  5986U,  6183U,  6380U,  6577U,  6773U,  6970U,  7166U,  7361U,  7557U,
7752U,  7947U,  8141U,  8335U,  8529U,  8723U,  8916U,  9109U,  9302U,  9494U,  9686U,  9877U,  10068U,
10259U, 10449U, 10639U, 10829U, 11018U, 11207U, 11395U, 11583U, 11771U, 11958U, 12144U, 12331U, 12516U,
12701U, 12886U, 13070U, 13254U, 13437U, 13620U, 13802U, 13984U, 14165U, 14346U, 14526U, 14706U, 14885U,
15063U, 15241U, 15419U, 15595U, 15772U, 15947U, 16122U, 16297U, 16470U, 16643U, 16816U, 16988U, 17159U,
17330U, 17500U, 17669U, 17838U, 18006U, 18173U, 18340U, 18506U, 18671U, 18835U, 18999U, 19162U, 19325U,
19487U, 19647U, 19808U, 19967U, 20126U, 20284U, 20441U, 20598U, 20753U, 20908U, 21062U, 21216U, 21368U,
21520U, 21671U, 21821U, 21970U, 22119U, 22266U, 22413U, 22559U, 22704U, 22848U, 22992U, 23134U, 23276U,
23417U, 23557U, 23696U, 23834U, 23971U, 24107U, 24243U, 24377U, 24511U, 24644U, 24776U, 24906U, 25036U,
25165U, 25293U, 25420U, 25547U, 25672U, 25796U, 25919U, 26042U, 26163U, 26283U, 26403U, 26521U, 26638U,
26755U, 26870U, 26984U, 27098U, 27210U, 27321U, 27431U, 27541U, 27649U, 27756U, 27862U, 27967U, 28071U,
28174U, 28276U, 28377U
};

/*******************************************************************************
Functions
*******************************************************************************/
void PWMN_CalcDutyCycle(void);
void FOC_SVM(void);

/******************************************************************************/
/*!

 \brief     space vector modulation

 \details   calculated duty cycles for each phase to generate space vector

 \param[in] u16_volts_par    space vector length
 \param[in] u16_angle_par    space vector angle

 OK - claculates duty cycle of each phase from vector length and angle
 u16_SpaceVectorABS, u16_SpaceVectorAngle
 */
/******************************************************************************/
void FOC_SVM(void) {

  uint16_t u16_temp;

  /* These variables hold the normalized sector angles used to find t1, t2. */
  uint16_t u16_angle1 = 0U;
  uint16_t u16_angle2 = 0U;

  /* These variables hold the space vector times. */
  uint16_t u16_half_t0 = 0U;

  /* Calculate the total PWM count period, which is the value in the PWMN_Max_Counter register. */
  static const uint16_t u16c_tpwm = (uint16_t)((FSYS / PWM_FREQUENCY)>>1); /* Period time (uC_Frequency/ PWM Frequency)/2 */

  uint16_t u16_t1 = 0U;
  uint16_t u16_t2 = 0U;

  /* Limit u16_SpaceVectorABS to avoid overmodulation. */
  if (u16_SpaceVectorABS > (uint16_t)VECTOR_LIMIT) {

          u16_SpaceVectorABS = (uint16_t)VECTOR_LIMIT;
  }

   u16_temp = (uint16_t) (((uint32_t)u16c_tpwm *  (uint32_t)u16_SpaceVectorABS) >> 15U);



  if (u16_SpaceVectorAngle < VECTOR2) { /* Sector 1 */

          u8_sector = 1U;
          u16_angle2 = u16_SpaceVectorAngle - VECTOR1; /* Reference SVM angle to the current sector */
          u16_angle1 = SIXTY_DEG - u16_angle2; /* Calculate second angle referenced to sector */

          u16_t1 = u16c_sinetable[(uint8_t) (u16_angle1 >> 6U)]; /* Look up values from table. */
          u16_t2 = u16c_sinetable[(uint8_t) (u16_angle2 >> 6U)];

//          u16_t1 = (uint16_t) ((mul_uu(u16_t1, u16_temp)) >> 15U);
//          u16_t2 = (uint16_t) ((mul_uu(u16_t2, u16_temp)) >> 15U);

          u16_t1 = (uint16_t) (((uint32_t)u16_t1 *  (uint32_t)u16_temp) >> 15U);
          u16_t2 = (uint16_t) (((uint32_t)u16_t2 * (uint32_t)u16_temp) >> 15U);
          /* Calculate half_t0 null time from period and t1,t2 */
          u16_half_t0 = ((u16c_tpwm - u16_t1) - u16_t2) >> 1U;

          /* Calculate duty cycles for Sector 1  (0 - 59 degrees) */
          DutyCycle.u16_PhA = u16_t1 + u16_t2 + u16_half_t0;
          DutyCycle.u16_PhB = u16_t2 + u16_half_t0;
          DutyCycle.u16_PhC = u16_half_t0;

  }
  else if (u16_SpaceVectorAngle < VECTOR3) { /* Sector 2 */

          u8_sector = 2U;
          u16_angle2 = u16_SpaceVectorAngle - VECTOR2; /* Reference SVM angle to the current sector */
          u16_angle1 = SIXTY_DEG - u16_angle2; /* Calculate second angle referenced to sector */

          u16_t1 = u16c_sinetable[(uint8_t) (u16_angle1 >> 6U)]; /* Look up values from table. */
          u16_t2 = u16c_sinetable[(uint8_t) (u16_angle2 >> 6U)];

//          u16_t1 = (uint16_t) ((mul_uu(u16_t1, u16_temp)) >> 15U);
//          u16_t2 = (uint16_t) ((mul_uu(u16_t2, u16_temp)) >> 15U);

          u16_t1 = (uint16_t) (((uint32_t)u16_t1 *  (uint32_t)u16_temp) >> 15U);
          u16_t2 = (uint16_t) (((uint32_t)u16_t2 * (uint32_t)u16_temp) >> 15U);
          /* Calculate half_t0 null time from period and t1,t2 */
          u16_half_t0 = ((u16c_tpwm - u16_t1) - u16_t2) >> 1U;

          /* Calculate duty cycles for Sector 2  (60 - 119 degrees) */
          DutyCycle.u16_PhA = u16_t1 + u16_half_t0;
          DutyCycle.u16_PhB = u16_t1 + u16_t2 + u16_half_t0;
          DutyCycle.u16_PhC = u16_half_t0;

  }
  else if (u16_SpaceVectorAngle < VECTOR4) { /* Sector 3 */

          u8_sector = 3U;
          u16_angle2 = u16_SpaceVectorAngle - VECTOR3; /* Reference SVM angle to the current sector */
          u16_angle1 = SIXTY_DEG - u16_angle2; /* Calculate second angle referenced to sector */

          u16_t1 = u16c_sinetable[(uint8_t) (u16_angle1 >> 6U)]; /* Look up values from table. */
          u16_t2 = u16c_sinetable[(uint8_t) (u16_angle2 >> 6U)];

//          u16_t1 = (uint16_t) ((mul_uu(u16_t1, u16_temp)) >> 15U);
//          u16_t2 = (uint16_t) ((mul_uu(u16_t2, u16_temp)) >> 15U);

          u16_t1 = (uint16_t) (((uint32_t)u16_t1 *  (uint32_t)u16_temp) >> 15U);
          u16_t2 = (uint16_t) (((uint32_t)u16_t2 * (uint32_t)u16_temp) >> 15U);

          /* Calculate half_t0 null time from period and t1,t2 */
          u16_half_t0 = ((u16c_tpwm - u16_t1) - u16_t2) >> 1U;

          /* Calculate duty cycles for Sector 3  (120 - 179 degrees) */
          DutyCycle.u16_PhA = u16_half_t0;
          DutyCycle.u16_PhB = u16_t1 + u16_t2 + u16_half_t0;
          DutyCycle.u16_PhC = u16_t2 + u16_half_t0;

  }
  else if (u16_SpaceVectorAngle < VECTOR5) { /* Sector 4 */

          u8_sector = 4U;
          u16_angle2 = u16_SpaceVectorAngle - VECTOR4; /* Reference SVM angle to the current sector */
          u16_angle1 = SIXTY_DEG - u16_angle2; /* Calculate second angle referenced to sector */

          u16_t1 = u16c_sinetable[(uint8_t) (u16_angle1 >> 6U)]; /* Look up values from table. */
          u16_t2 = u16c_sinetable[(uint8_t) (u16_angle2 >> 6U)];

//          u16_t1 = (uint16_t) ((mul_uu(u16_t1, u16_temp)) >> 15U);
//          u16_t2 = (uint16_t) ((mul_uu(u16_t2, u16_temp)) >> 15U);

          u16_t1 = (uint16_t) (((uint32_t)u16_t1 *  (uint32_t)u16_temp) >> 15U);
          u16_t2 = (uint16_t) (((uint32_t)u16_t2 * (uint32_t)u16_temp) >> 15U);

          /* Calculate half_t0 null time from period and t1,t2 */
          u16_half_t0 = ((u16c_tpwm - u16_t1) - u16_t2) >> 1U;

          /* Calculate duty cycles for Sector 4  (180 - 239 degrees) */
          DutyCycle.u16_PhA = u16_half_t0;
          DutyCycle.u16_PhB = u16_t1 + u16_half_t0;
          DutyCycle.u16_PhC = u16_t1 + u16_t2 + u16_half_t0;

  }
  else if (u16_SpaceVectorAngle < VECTOR6) { /* Sector 5 */

          u8_sector = 5U;
          u16_angle2 = u16_SpaceVectorAngle - VECTOR5; /* Reference SVM angle to the current sector */
          u16_angle1 = SIXTY_DEG - u16_angle2; /* Calculate second angle referenced to sector */

          u16_t1 = u16c_sinetable[(uint8_t) (u16_angle1 >> 6U)]; /* Look up values from table. */
          u16_t2 = u16c_sinetable[(uint8_t) (u16_angle2 >> 6U)];

//          u16_t1 = (uint16_t) ((mul_uu(u16_t1, u16_temp)) >> 15U);
//          u16_t2 = (uint16_t) ((mul_uu(u16_t2, u16_temp)) >> 15U);

          u16_t1 = (uint16_t) (((uint32_t)u16_t1 *  (uint32_t)u16_temp) >> 15U);
          u16_t2 = (uint16_t) (((uint32_t)u16_t2 * (uint32_t)u16_temp) >> 15U);

          /* Calculate half_t0 null time from period and t1,t2 */
          u16_half_t0 = ((u16c_tpwm - u16_t1) - u16_t2) >> 1U;

          /* Calculate duty cycles for Sector 5  (240 - 299 degrees) */
          DutyCycle.u16_PhA = u16_t2 + u16_half_t0;
          DutyCycle.u16_PhB = u16_half_t0;
          DutyCycle.u16_PhC = u16_t1 + u16_t2 + u16_half_t0;

  }
  else { /* Sector 6 */

          u8_sector = 6U;
          u16_angle2 = u16_SpaceVectorAngle - VECTOR6; /* Reference SVM angle to the current sector */
          u16_angle1 = SIXTY_DEG - u16_angle2; /* Calculate second angle referenced to sector */

          u16_t1 = u16c_sinetable[(uint8_t) (u16_angle1 >> 6U)]; /* Look up values from table */
          u16_t2 = u16c_sinetable[(uint8_t) (u16_angle2 >> 6U)];

//          u16_t1 = (uint16_t) ((mul_uu(u16_t1, u16_temp)) >> 15U);
//          u16_t2 = (uint16_t) ((mul_uu(u16_t2, u16_temp)) >> 15U);

          u16_t1 = (uint16_t) (((uint32_t)u16_t1 *  (uint32_t)u16_temp) >> 15U);
          u16_t2 = (uint16_t) (((uint32_t)u16_t2 * (uint32_t)u16_temp) >> 15U);

          /* Calculate half_t0 null time from period and t1,t2 */
          u16_half_t0 = ((u16c_tpwm - u16_t1) - u16_t2) >> 1U;

          /* Calculate duty cycles for Sector 6  ( 300 - 359 degrees ) */
          DutyCycle.u16_PhA = u16_t1 + u16_t2 + u16_half_t0;
          DutyCycle.u16_PhB = u16_half_t0;
          DutyCycle.u16_PhC = u16_t1 + u16_half_t0;
  }

}
/******************************************************************************/
/*!

\brief	   Calculation of dutycycle

\details   manipulation of pwm pattern for single shunt measuring

?? declaration of PWMN_Max_Counter
*/
/******************************************************************************/
void PWMN_CalcDutyCycle(void){

  uint16_t u16_modify_temp = MIN_EDGE_DIFF;

  switch(u8_sector){

    case(1U):
      /* A>B>C */
      DutyCycle.u16_PhAmodify = DutyCycle.u16_PhA;
      DutyCycle.u16_PhAnext   = DutyCycle.u16_PhA;

      if(DutyCycle.u16_PhA>MIN_EDGE_DIFF){

       u16_modify_temp = DutyCycle.u16_PhA - MIN_EDGE_DIFF;
      }

      if(DutyCycle.u16_PhB>u16_modify_temp){

        DutyCycle.u16_PhBmodify = u16_modify_temp;
        DutyCycle.u16_PhBnext = (DutyCycle.u16_PhB<<1) - u16_modify_temp;

        if(DutyCycle.u16_PhBnext > PWMN_Max_Counter){

          DutyCycle.u16_PhBnext = PWMN_Max_Counter;
        }

      }else{

        DutyCycle.u16_PhBmodify = DutyCycle.u16_PhB;
        DutyCycle.u16_PhBnext = DutyCycle.u16_PhB;
      }

      if(DutyCycle.u16_PhBmodify>MIN_EDGE_DIFF){

       u16_modify_temp = DutyCycle.u16_PhBmodify - MIN_EDGE_DIFF;
      }

      if(DutyCycle.u16_PhC>u16_modify_temp){

        DutyCycle.u16_PhCmodify = u16_modify_temp;
        DutyCycle.u16_PhCnext = (DutyCycle.u16_PhC<<1) - u16_modify_temp;

        if(DutyCycle.u16_PhCnext > PWMN_Max_Counter){

          DutyCycle.u16_PhCnext = PWMN_Max_Counter;
        }

      }else{

        DutyCycle.u16_PhCmodify = DutyCycle.u16_PhC;
        DutyCycle.u16_PhCnext = DutyCycle.u16_PhC;
      }

      //ADC_set_PhaseCurrentTrigger((DutyCycle.u16_PhBmodify - CURRENT_TRIGGER_DELAY),(DutyCycle.u16_PhCmodify - CURRENT_TRIGGER_DELAY),u8_sector);
        DutyCycle.u16_CurrentTrigger1=DutyCycle.u16_PhBmodify - CURRENT_TRIGGER_DELAY;
        DutyCycle.u16_CurrentTrigger2=DutyCycle.u16_PhCmodify - CURRENT_TRIGGER_DELAY;
      break;

    case(2U):
      /* B>A>C */
      DutyCycle.u16_PhBmodify = DutyCycle.u16_PhB;
      DutyCycle.u16_PhBnext   = DutyCycle.u16_PhB;

      if(DutyCycle.u16_PhB>MIN_EDGE_DIFF){

       u16_modify_temp = DutyCycle.u16_PhB - MIN_EDGE_DIFF;
      }

      if(DutyCycle.u16_PhA>u16_modify_temp){

        DutyCycle.u16_PhAmodify = u16_modify_temp;
        DutyCycle.u16_PhAnext = (DutyCycle.u16_PhA<<1) - u16_modify_temp;

        if(DutyCycle.u16_PhAnext > PWMN_Max_Counter){

          DutyCycle.u16_PhAnext = PWMN_Max_Counter;
        }

      }else{

        DutyCycle.u16_PhAmodify = DutyCycle.u16_PhA;
        DutyCycle.u16_PhAnext = DutyCycle.u16_PhA;
      }

      if(DutyCycle.u16_PhAmodify>MIN_EDGE_DIFF){

       u16_modify_temp = DutyCycle.u16_PhAmodify - MIN_EDGE_DIFF;
      }

      if(DutyCycle.u16_PhC>u16_modify_temp){

        DutyCycle.u16_PhCmodify = u16_modify_temp;
        DutyCycle.u16_PhCnext = (DutyCycle.u16_PhC<<1) - u16_modify_temp;

        if(DutyCycle.u16_PhCnext > PWMN_Max_Counter){

          DutyCycle.u16_PhCnext = PWMN_Max_Counter;
        }

      }else{

        DutyCycle.u16_PhCmodify = DutyCycle.u16_PhC;
        DutyCycle.u16_PhCnext = DutyCycle.u16_PhC;
      }

//      ADC_set_PhaseCurrentTrigger((DutyCycle.u16_PhAmodify - CURRENT_TRIGGER_DELAY),(DutyCycle.u16_PhCmodify - CURRENT_TRIGGER_DELAY),u8_sector);
        DutyCycle.u16_CurrentTrigger1=DutyCycle.u16_PhAmodify - CURRENT_TRIGGER_DELAY;
        DutyCycle.u16_CurrentTrigger2=DutyCycle.u16_PhCmodify - CURRENT_TRIGGER_DELAY;
      break;

    case(3U):
      /* B>C>A */
      DutyCycle.u16_PhBmodify = DutyCycle.u16_PhB;
      DutyCycle.u16_PhBnext   = DutyCycle.u16_PhB;

      if(DutyCycle.u16_PhB>MIN_EDGE_DIFF){

       u16_modify_temp = DutyCycle.u16_PhB - MIN_EDGE_DIFF;
      }

      if(DutyCycle.u16_PhC>u16_modify_temp){

        DutyCycle.u16_PhCmodify = u16_modify_temp;
        DutyCycle.u16_PhCnext = (DutyCycle.u16_PhC<<1) - u16_modify_temp;

        if(DutyCycle.u16_PhCnext > PWMN_Max_Counter){

          DutyCycle.u16_PhCnext = PWMN_Max_Counter;
        }

      }else{

        DutyCycle.u16_PhCmodify = DutyCycle.u16_PhC;
        DutyCycle.u16_PhCnext = DutyCycle.u16_PhC;
      }

      if(DutyCycle.u16_PhCmodify>MIN_EDGE_DIFF){

       u16_modify_temp = DutyCycle.u16_PhCmodify - MIN_EDGE_DIFF;
      }

      if(DutyCycle.u16_PhA>u16_modify_temp){

        DutyCycle.u16_PhAmodify = u16_modify_temp;
        DutyCycle.u16_PhAnext = (DutyCycle.u16_PhA<<1) - u16_modify_temp;

        if(DutyCycle.u16_PhAnext > PWMN_Max_Counter){

          DutyCycle.u16_PhAnext = PWMN_Max_Counter;
        }

      }else{

        DutyCycle.u16_PhAmodify = DutyCycle.u16_PhA;
        DutyCycle.u16_PhAnext = DutyCycle.u16_PhA;
      }

      //ADC_set_PhaseCurrentTrigger((DutyCycle.u16_PhCmodify - CURRENT_TRIGGER_DELAY),(DutyCycle.u16_PhAmodify - CURRENT_TRIGGER_DELAY),u8_sector);
        DutyCycle.u16_CurrentTrigger1=DutyCycle.u16_PhCmodify - CURRENT_TRIGGER_DELAY;
        DutyCycle.u16_CurrentTrigger2=DutyCycle.u16_PhAmodify - CURRENT_TRIGGER_DELAY;
      break;

    case(4U):
      /* C>B>A */
      DutyCycle.u16_PhCmodify = DutyCycle.u16_PhC;
      DutyCycle.u16_PhCnext   = DutyCycle.u16_PhC;

      if(DutyCycle.u16_PhC>MIN_EDGE_DIFF){

       u16_modify_temp = DutyCycle.u16_PhC - MIN_EDGE_DIFF;
      }

      if(DutyCycle.u16_PhB>u16_modify_temp){

        DutyCycle.u16_PhBmodify = u16_modify_temp;
        DutyCycle.u16_PhBnext = (DutyCycle.u16_PhB<<1) - u16_modify_temp;

        if(DutyCycle.u16_PhBnext > PWMN_Max_Counter){

          DutyCycle.u16_PhBnext = PWMN_Max_Counter;
        }

      }else{

        DutyCycle.u16_PhBmodify = DutyCycle.u16_PhB;
        DutyCycle.u16_PhBnext = DutyCycle.u16_PhB;
      }

      if(DutyCycle.u16_PhBmodify>MIN_EDGE_DIFF){

       u16_modify_temp = DutyCycle.u16_PhBmodify - MIN_EDGE_DIFF;
      }

      if(DutyCycle.u16_PhA>u16_modify_temp){

        DutyCycle.u16_PhAmodify = u16_modify_temp;
        DutyCycle.u16_PhAnext = (DutyCycle.u16_PhA<<1) - u16_modify_temp;

        if(DutyCycle.u16_PhAnext > PWMN_Max_Counter){

          DutyCycle.u16_PhAnext = PWMN_Max_Counter;
        }

      }else{

        DutyCycle.u16_PhAmodify = DutyCycle.u16_PhA;
        DutyCycle.u16_PhAnext = DutyCycle.u16_PhA;
      }

      //ADC_set_PhaseCurrentTrigger((DutyCycle.u16_PhBmodify - CURRENT_TRIGGER_DELAY),(DutyCycle.u16_PhAmodify - CURRENT_TRIGGER_DELAY),u8_sector);
        DutyCycle.u16_CurrentTrigger1=DutyCycle.u16_PhBmodify - CURRENT_TRIGGER_DELAY;
        DutyCycle.u16_CurrentTrigger2=DutyCycle.u16_PhAmodify - CURRENT_TRIGGER_DELAY;
      break;

    case(5U):
      /* C>A>B */
      DutyCycle.u16_PhCmodify = DutyCycle.u16_PhC;
      DutyCycle.u16_PhCnext   = DutyCycle.u16_PhC;

      if(DutyCycle.u16_PhC>MIN_EDGE_DIFF){

       u16_modify_temp = DutyCycle.u16_PhC - MIN_EDGE_DIFF;
      }

      if(DutyCycle.u16_PhA>u16_modify_temp){

        DutyCycle.u16_PhAmodify = u16_modify_temp;
        DutyCycle.u16_PhAnext = (DutyCycle.u16_PhA<<1) - u16_modify_temp;

        if(DutyCycle.u16_PhAnext > PWMN_Max_Counter){

          DutyCycle.u16_PhAnext = PWMN_Max_Counter;
        }

      }else{

        DutyCycle.u16_PhAmodify = DutyCycle.u16_PhA;
        DutyCycle.u16_PhAnext = DutyCycle.u16_PhA;
      }

      if(DutyCycle.u16_PhAmodify>MIN_EDGE_DIFF){

       u16_modify_temp = DutyCycle.u16_PhAmodify - MIN_EDGE_DIFF;
      }

      if(DutyCycle.u16_PhB>u16_modify_temp){

        DutyCycle.u16_PhBmodify = u16_modify_temp;
        DutyCycle.u16_PhBnext = (DutyCycle.u16_PhB<<1) - u16_modify_temp;

        if(DutyCycle.u16_PhBnext > PWMN_Max_Counter){

          DutyCycle.u16_PhBnext = PWMN_Max_Counter;
        }

      }else{

        DutyCycle.u16_PhBmodify = DutyCycle.u16_PhB;
        DutyCycle.u16_PhBnext = DutyCycle.u16_PhB;
      }

      //ADC_set_PhaseCurrentTrigger((DutyCycle.u16_PhAmodify - CURRENT_TRIGGER_DELAY),(DutyCycle.u16_PhBmodify - CURRENT_TRIGGER_DELAY),u8_sector);
        DutyCycle.u16_CurrentTrigger1=DutyCycle.u16_PhAmodify - CURRENT_TRIGGER_DELAY;
        DutyCycle.u16_CurrentTrigger2=DutyCycle.u16_PhBmodify - CURRENT_TRIGGER_DELAY;
      break;

    case(6U):
      /* A>C>B */
      DutyCycle.u16_PhAmodify = DutyCycle.u16_PhA;
      DutyCycle.u16_PhAnext   = DutyCycle.u16_PhA;

      if(DutyCycle.u16_PhA>MIN_EDGE_DIFF){

       u16_modify_temp = DutyCycle.u16_PhA - MIN_EDGE_DIFF;
      }

      if(DutyCycle.u16_PhC>u16_modify_temp){

        DutyCycle.u16_PhCmodify = u16_modify_temp;
        DutyCycle.u16_PhCnext = (DutyCycle.u16_PhC<<1) - u16_modify_temp;

        if(DutyCycle.u16_PhCnext > PWMN_Max_Counter){

          DutyCycle.u16_PhCnext = PWMN_Max_Counter;
        }

      }else{

        DutyCycle.u16_PhCmodify = DutyCycle.u16_PhC;
        DutyCycle.u16_PhCnext = DutyCycle.u16_PhC;
      }

      if(DutyCycle.u16_PhCmodify>MIN_EDGE_DIFF){

       u16_modify_temp = DutyCycle.u16_PhCmodify - MIN_EDGE_DIFF;
      }

      if(DutyCycle.u16_PhB>u16_modify_temp){

        DutyCycle.u16_PhBmodify = u16_modify_temp;
        DutyCycle.u16_PhBnext = (DutyCycle.u16_PhB<<1) - u16_modify_temp;

        if(DutyCycle.u16_PhBnext > PWMN_Max_Counter){

          DutyCycle.u16_PhBnext = PWMN_Max_Counter;
        }

      }else{

        DutyCycle.u16_PhBmodify = DutyCycle.u16_PhB;
        DutyCycle.u16_PhBnext = DutyCycle.u16_PhB;
      }

      //ADC_set_PhaseCurrentTrigger((DutyCycle.u16_PhCmodify - CURRENT_TRIGGER_DELAY),(DutyCycle.u16_PhBmodify - CURRENT_TRIGGER_DELAY),u8_sector);
        DutyCycle.u16_CurrentTrigger1=DutyCycle.u16_PhCmodify - CURRENT_TRIGGER_DELAY;
        DutyCycle.u16_CurrentTrigger2=DutyCycle.u16_PhBmodify - CURRENT_TRIGGER_DELAY;
      break;

    default:
      break;
  }

}


