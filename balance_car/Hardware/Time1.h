#ifndef __TIME1_H
#define __TIME1_H
#include "stm32f10x.h"  
extern uint16_t Time1;
extern uint16_t Time2;
void Time1_Init(void);
void TIM1_UP_IRQHandler(void);

#endif