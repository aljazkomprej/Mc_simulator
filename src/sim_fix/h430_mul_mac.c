#include "h430_mul_mac.h"

static int64_t mpyacc;

/* ========================================================================== */
/* MULTIPLY serve functions */
/* ========================================================================== */

/* inline functions */
void SetAccumulator(int32_t value)
{
  mpyacc=value;
}


/* xxxxxx changed from get_accumulator DD20151123 */
int32_t GetAccumulatorSat(void)
{
  int32_t result;

  if(mpyacc>INT32_MAX)
  {
    result=INT32_MAX;
  }
  else if(mpyacc<INT32_MIN)
  {
    result=INT32_MIN;
  }
  else
  {
    result=mpyacc;
  }


  return result;
}



int32_t GetAccumulator(void)
{
  int32_t result;

  result=mpyacc;

  return result;
}

int16_t GetAccumulatorLow(void)
{
    int16_t result;

    result=mpyacc;
    return result;
}



int16_t GetAccumulatorAsrSat(uint16_t asr)
{
  int16_t result;

    mpyacc >>= asr;

  if(mpyacc>INT16_MAX)
  {
    result=INT16_MAX;
  }
  else if(mpyacc<INT16_MIN)
  {
    result=INT16_MIN;
  }
  else
  {
    result=mpyacc;
  }

  return result;
}


/* get accumulator without saturation! DD20151123 */
int32_t GetAccumulatorAsr(uint16_t asr)
{
  int16_t result;

    mpyacc >>= asr;

    result=mpyacc;

  return result;
}


int16_t GetReshi_Q1_15(void)
{
    int32_t res32;
    int16_t result;

    res32=mpyacc>>16;
    if(res32 > 32767L)
    {
        result=32767;
    }
    else if(res32 < -32768L)
    {
        result=-32768;
    }
    else
    {
        result=res32;
    }
    return result;
}

int16_t GetReshi(void)
{
    int32_t res32;
    int16_t result;

    res32=mpyacc>>16;
    result=res32&0xffff;
    return result;
}


void MultiplierSetShiftRight(uint8_t asr)
{
    mpyacc >>= asr;
}




/*** Multiply and accumulate inline functions which do not return value ***/



void Multiply_S16_S16(int16_t fac1_s16, int16_t fac2_s16)
{
    mpyacc=(int64_t)fac1_s16*fac2_s16;
  return;
}


void MultiplyAccumulate_S16_S16(int16_t fac1_s16, int16_t fac2_s16)
{
    mpyacc+=(int64_t)fac1_s16*fac2_s16;
  return;
}


void multiply_accumulate_s16_u16(int16_t fac1_s16, uint16_t fac2_u16)
{
    mpyacc+=(int64_t)fac1_s16*fac2_u16;
  return;
}


void math_mulU16_unsafe ( uint16_t m1, uint16_t m2 )
{
    mpyacc=(int64_t)m1*m2;
  return;
}

void math_mul_shiftright_result ( uint16_t cnt )
{

    mpyacc >>= cnt;
}

int32_t math_mul_get_result ( void )
{
    int16_t result;

    result=mpyacc;
    return result;
}




/*** Multiply inline functions with saturation and shifting ***/

/* with variable shift right shift: 8 .. 15
\brief
Multiply or multiply and accumulate function can be selected.

Function uses saturation, which is only possible when reading RESHI.
Therefore for small shift values, the value of accumulator would shift outside
of 40-bit accumulator. To prevent this, only shift right for more or
equal to 8 places is possible.
When somebody tries to call the function using shift value lower than 8,
the function would return 0 and set
mul_mac_error=mul_shift_out_of_range .
*/
/* the whole range of shift in one function */

int16_t Mulmacs_S16_S16_AsrSat(mult_type mtype, int16_t fac1_s16, int16_t fac2_s16, uint16_t asr)
{
  int16_t result;

  if(mtype == MUL)
  {
    mpyacc=(int64_t)fac1_s16*fac2_s16;
  }
  else
  {
    mpyacc+=(int64_t)fac1_s16*fac2_s16;
  }

  result=GetAccumulatorAsrSat(asr);

  return result;
}



int16_t Mulmacs_S16_U16_AsrSat(mult_type mtype, int16_t fac1_s16, uint16_t fac2_u16, uint16_t asr)
{
  int16_t result;

  if(mtype == MUL)
  {
    mpyacc=(int64_t)fac1_s16*fac2_u16;
  }
  else
  {
    mpyacc+=(int64_t)fac1_s16*fac2_u16;
  }

  result=GetAccumulatorAsrSat(asr);

  return result;
}



/* unsigned variant */
uint16_t MulmacuAsrSat(mult_type mtype, uint16_t fac1_u16, uint16_t fac2_u16, uint16_t asr)
{
  uint16_t result;

  if(mtype == MUL)
  {
    mpyacc=(int64_t)fac1_u16*fac2_u16;  // DD xxxxx ali cast operator deluje pricakovano??
  }
  else
  {
    mpyacc+=(int64_t)fac1_u16*fac2_u16;
  }

  mpyacc >>= asr;

  if(mpyacc > 65535)
  {
     result=65535;
  }
  else
  {
    result=mpyacc;
  }

  return result;
}



int16_t MulsMacsAsr15_Sat( int16_t f1a, int16_t f1b, int16_t f2a, int16_t f2b)
{
  int16_t result;
  //int64_t res64;

  mpyacc=(int64_t)f1a*f1b + (int64_t)f2a*f2b;

  result=GetAccumulatorAsrSat(15);

  return result;
}

int16_t MulsMacsAsrSat( int16_t f1a, int16_t f1b, int16_t f2a, int16_t f2b, uint16_t asr)
{
  int16_t result;
  //int64_t res64;

  mpyacc=(int64_t)f1a*f1b + (int64_t)f2a*f2b;

  result=GetAccumulatorAsrSat(asr);

  return result;
}


/* novo za test DD20151123 */
int16_t muls_macs_asr_sat( int16_t f1a, int16_t f1b, int16_t f2a, int16_t f2b, uint16_t asr)
{
  int16_t result;
  //int64_t res64;

  mpyacc=(int64_t)f1a*f1b + (int64_t)f2a*f2b;

  result=GetAccumulatorAsrSat(asr);

  return result;
}


#if 0
static inline uint16_t mulu_u16_u16_asr15(uint16_t fac_a, uint16_t fac_b)
{
  MULTIPLIER16->MPY = fac_a;
  MULTIPLIER16->OP2U = fac_b;
  MULTIPLIER16->SHIFT_LEFT = 1U - 1U;
  return MULTIPLIER16->RESHI;
}
#endif
