#ifndef H430_MUL_MAC_H
#define H430_MUL_MAC_H

#include <stdint.h>

/* ========================================================================== */
/* MULTIPLY constants */
/* ========================================================================== */

typedef enum {
	MUL,  /* multiply */
	MAC   /* multiply with accumulate */
} mult_type;


typedef enum
{
  mul_shift_out_of_range=1,
} mul_error_t;

/* variables */
//extern uint16_t mul_mac_error;

#ifdef __cplusplus
extern "C" {
#endif


void SetAccumulator(int32_t value);
int32_t GetAccumulator(void);
int32_t GetAccumulatorAsr(uint16_t asr);
int16_t GetReshi_Q1_15(void);
int16_t GetReshi(void);
int16_t GetAccumulatorAsrSat(uint16_t asr);
void MultiplierSetShiftRight(uint8_t asr);
int32_t GetAccumulatorSat(void);
int16_t GetAccumulatorLow(void);



void Multiply_S16_S16(int16_t fac1_s16, int16_t fac2_s16);
void MultiplyAccumulate_S16_S16(int16_t fac1_s16, int16_t fac2_s16);
void multiply_accumulate_s16_u16(int16_t fac1_s16, uint16_t fac2_u16);
//uint16_t mulu_u16_u16_asr15(uint16_t fac_a, uint16_t fac_b);
int16_t Mulmacs_S16_S16_AsrSat(mult_type mtype, int16_t fac1_s16, int16_t fac2_s16, uint16_t asr);
int16_t Mulmacs_S16_U16_AsrSat(mult_type mtype, int16_t fac1_s16, uint16_t fac2_u16, uint16_t asr);
uint16_t MulmacuAsrSat(mult_type mtype, uint16_t fac1_u16, uint16_t fac2_u16, uint16_t asr);
int16_t MulsMacsAsr15_Sat( int16_t f1a, int16_t f1b, int16_t f2a, int16_t f2b);
int16_t MulsMacsAsrSat( int16_t f1a, int16_t f1b, int16_t f2a, int16_t f2b, uint16_t asr );

void math_mulU16_unsafe ( uint16_t m1, uint16_t m2 );
void math_mul_shiftright_result ( uint16_t cnt );

/* test functions */
void test_mul(void);


#ifdef __cplusplus
}
#endif


#endif /* H430_MUL_MAC_H */
