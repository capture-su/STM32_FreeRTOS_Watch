#ifndef _ENCODER_H_
#define _ENCODER_H_
#include "stm32f10x.h"                  // Device header

void Encoder_Init(void);
int8_t Encoder_GetValue(void);
void Encoder_Clear(void);
#endif
