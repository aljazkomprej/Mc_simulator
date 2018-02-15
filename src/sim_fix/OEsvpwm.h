/******************************************************************************/
/*!

\file               motor.h

\brief              including motor functions

\details	    motor calculations and control


\author             T.Frye, S.Grees

\par Company:
	   	    Online Engineering GmbH
\date
\par Target Device:
		    ELMOS E523.05
\par IDE:
 		    IAR Embedded Workbench IDE
\par Compiler:
 		    IAR C/C++ Compiler for MSP430 v5.40.1

*/
/******************************************************************************/

#ifndef  __OEsvpwm_H__
#define __OEsvpwm_H__

/*******************************************************************************
Includes
*******************************************************************************/
#include <stdint.h>

/*******************************************************************************
Defines
*******************************************************************************/
#define FSYS            (48000000L)    /*!< uC clock frequency */ /* do not change */
#define PWM_FREQUENCY   (20000L)       /*!< PWM Frequency */
typedef struct {
    uint16_t    u16_PhA;
    uint16_t	u16_PhAnext;
    uint16_t    u16_PhAmodify;
    uint16_t    u16_PhB;
    uint16_t	u16_PhBnext;
    uint16_t    u16_PhBmodify;
    uint16_t    u16_PhC;
    uint16_t	u16_PhCnext;
    uint16_t    u16_PhCmodify;
    uint16_t	u16_CurrentTrigger1;
    uint16_t    u16_CurrentTrigger2;
}DutyCycle_t;    /*!< SVM specific variables */
/*******************************************************************************
Variables
*******************************************************************************/

/*******************************************************************************
Function Prototypes
*******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif
void FOC_SVM(void);
void PWMN_CalcDutyCycle(void);
#ifdef __cplusplus
}
#endif
/*void CCT0_init(void);
void Motor_Stop(void);
void Motor_Start(void);
void PWMN_ON(void);
void PWMN_OFF(void);
void PWMN_init(void);
void PWMN_Motor_break(void);
void Motor_Speed_Ratelimiter(int16_t s16_SpeedRef_par);
void Motor_FilterSpeed(void);
void Motor_TemperatureDerating(void);
int16_t Motor_blank_speed_setpoint(int16_t s16_speed_sp_par, int16_t s16_speed_par);
uint8_t Motor_ObserveLoad(void);

void Motor_set_SpeedRef(int16_t s16_par);
void Motor_set_PowerMode(uint8_t u8_par);
void Motor_set_DeblockWoblle(uint8_t u8_par);

int16_t Motor_get_SpeedRef(void);
int16_t Motor_get_SpeedFiltered(void);
int16_t Motor_get_IqRef(void);
int16_t Motor_get_IdRef(void);
int16_t Motor_get_Iq(void);
int16_t Motor_get_Id(void);

uint8_t Motor_get_PowerMode(void);
uint8_t Motor_get_PowerLimit(void);
uint8_t Motor_get_MotorOn(void);
uint8_t Motor_get_ClosedLoop(void);
uint8_t Motor_get_DeblockWobble(void);
uint8_t Motor_get_WmEn(void);
uint8_t Motor_get_VppDetected(void);

uint16_t Motor_get_SpaceVectorLength(void);
*/



#endif  /*  #ifndef  __MOTOR_H__  */
