
#include <stdio.h>
#include "h430_mul_mac.h"

#include "test_mulmac.h"


/********* h430_mul_mac      *************/
void test_mulmacs_ss(void)
{
  static int16_t res;

  //__disable_interrupt();

  static int16_t fa=256;
  static int16_t fb=128;

  printf("\nmulmacs_s16_s16_asr_sat:\n");

  res=Mulmacs_S16_S16_AsrSat(MUL, fa, fb, 15);
  printf("asr=%u res=%d\n",15,res);

  res=Mulmacs_S16_S16_AsrSat(MUL, fa, fb,  14);
  printf("asr=%u res=%d\n",14,res);

  res=Mulmacs_S16_S16_AsrSat(MUL, fa, fb, 13);
  printf("asr=%u res=%d\n",13,res);

  res=Mulmacs_S16_S16_AsrSat(MUL, fa, fb,  12);
  printf("asr=%u res=%d\n",12,res);

  res=Mulmacs_S16_S16_AsrSat(MUL, fa, fb,  11);
  printf("asr=%u res=%d\n",11,res);

  res=Mulmacs_S16_S16_AsrSat(MUL, fa, fb,  10);
  printf("asr=%u res=%d\n",10,res);

  res=Mulmacs_S16_S16_AsrSat(MUL, fa, fb, 9);
  printf("asr=%u res=%d\n", 9, res);

  res=Mulmacs_S16_S16_AsrSat(MUL, fa, fb, 8);
  printf("asr=%u res=%d\n", 8, res);

  res=Mulmacs_S16_S16_AsrSat(MUL, fa, fb, 7);
  printf("asr=%u res=%d\n", 7, res);

  res=Mulmacs_S16_S16_AsrSat(MUL, fa, fb, 6);
  printf("asr=%u res=%d\n", 6, res);

  res=Mulmacs_S16_S16_AsrSat(MUL, fa, fb, 5);
  printf("asr=%u res=%d\n", 5, res);

  res=Mulmacs_S16_S16_AsrSat(MUL, fa, fb, 4);
  printf("asr=%u res=%d\n", 4, res);

  res=Mulmacs_S16_S16_AsrSat(MUL, fa, fb, 3);
  printf("asr=%u res=%d\n", 3, res);

  res=Mulmacs_S16_S16_AsrSat(MUL, fa, fb, 2);
  printf("asr=%u res=%d\n", 2, res);

  res=Mulmacs_S16_S16_AsrSat(MUL, fa, fb, 1);
  printf("asr=%u res=%d\n", 1, res);

  res=Mulmacs_S16_S16_AsrSat(MUL, fa, fb, 0);
  printf("asr=%u res=%d\n", 0, res);


  return;
}



void test_mulmacs_su(void)
{
  static int16_t res;

//  __disable_interrupt();

  static  int16_t fa=256;
  static uint16_t fb=128;

  printf("\nmulmacs_s16_u16_asr_sat:\n");

  res=Mulmacs_S16_U16_AsrSat(MUL, fa, fb, 15);
  printf("asr=%u res=%d\n",15,res);

  res=Mulmacs_S16_U16_AsrSat(MUL, fa, fb,  14);
  printf("asr=%u res=%d\n",14,res);

  res=Mulmacs_S16_U16_AsrSat(MUL, fa, fb, 13);
  printf("asr=%u res=%d\n",13,res);

  res=Mulmacs_S16_U16_AsrSat(MUL, fa, fb,  12);
  printf("asr=%u res=%d\n",12,res);

  res=Mulmacs_S16_U16_AsrSat(MUL, fa, fb,  11);
  printf("asr=%u res=%d\n",11,res);

  res=Mulmacs_S16_U16_AsrSat(MUL, fa, fb,  10);
  printf("asr=%u res=%d\n",10,res);

  res=Mulmacs_S16_U16_AsrSat(MUL, fa, fb, 9);
  printf("asr=%u res=%d\n", 9, res);

  res=Mulmacs_S16_U16_AsrSat(MUL, fa, fb, 8);
  printf("asr=%u res=%d\n", 8, res);

  res=Mulmacs_S16_U16_AsrSat(MUL, fa, fb, 7);
  printf("asr=%u res=%d\n", 7, res);

  res=Mulmacs_S16_U16_AsrSat(MUL, fa, fb, 6);
  printf("asr=%u res=%d\n", 6, res);

  res=Mulmacs_S16_U16_AsrSat(MUL, fa, fb, 5);
  printf("asr=%u res=%d\n", 5, res);

  res=Mulmacs_S16_U16_AsrSat(MUL, fa, fb, 4);
  printf("asr=%u res=%d\n", 4, res);

  res=Mulmacs_S16_U16_AsrSat(MUL, fa, fb, 3);
  printf("asr=%u res=%d\n", 3, res);

  res=Mulmacs_S16_U16_AsrSat(MUL, fa, fb, 2);
  printf("asr=%u res=%d\n", 2, res);

  res=Mulmacs_S16_U16_AsrSat(MUL, fa, fb, 1);
  printf("asr=%u res=%d\n", 1, res);

  res=Mulmacs_S16_U16_AsrSat(MUL, fa, fb, 0);
  printf("asr=%u res=%d\n", 0, res);

  return;
}



void test_mulmacu(void)
{
  static uint16_t res;

//  __disable_interrupt();

  static uint16_t fa=256*32;
  static uint16_t fb=128;

  printf("\nmulmacu_asr_sat:\n");

  res=MulmacuAsrSat(MUL, fa, fb, 15);
  printf("asr=%u res=%u\n",15,res);

  res=MulmacuAsrSat(MUL, fa, fb,  14);
  printf("asr=%u res=%u\n",14,res);

  res=MulmacuAsrSat(MUL, fa, fb, 13);
  printf("asr=%u res=%u\n",13,res);

  res=MulmacuAsrSat(MUL, fa, fb,  12);
  printf("asr=%u res=%u\n",12,res);

  res=MulmacuAsrSat(MUL, fa, fb,  11);
  printf("asr=%u res=%u\n",11,res);

  res=MulmacuAsrSat(MUL, fa, fb,  10);
  printf("asr=%u res=%u\n",10,res);

  res=MulmacuAsrSat(MUL, fa, fb, 9);
  printf("asr=%u res=%u\n", 9, res);

  res=MulmacuAsrSat(MUL, fa, fb, 8);
  printf("asr=%u res=%u\n", 8, res);

  res=MulmacuAsrSat(MUL, fa, fb, 7);
  printf("asr=%u res=%u\n", 7, res);

  res=MulmacuAsrSat(MUL, fa, fb, 6);
  printf("asr=%u res=%u\n", 6, res);

  res=MulmacuAsrSat(MUL, fa, fb, 5);
  printf("asr=%u res=%u\n", 5, res);

  res=MulmacuAsrSat(MUL, fa, fb, 4);
  printf("asr=%u res=%u\n", 4, res);

  res=MulmacuAsrSat(MUL, fa, fb, 3);
  printf("asr=%u res=%u\n", 3, res);

  res=MulmacuAsrSat(MUL, fa, fb, 2);
  printf("asr=%u res=%u\n", 2, res);

  res=MulmacuAsrSat(MUL, fa, fb, 1);
  printf("asr=%u res=%u\n", 1, res);

  res=MulmacuAsrSat(MUL, fa, fb, 0);
  printf("asr=%u res=%u\n", 0, res);

  return;
}

void test_mulmac(void)
{

    test_mulmacu();
}
