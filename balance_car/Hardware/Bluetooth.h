#ifndef __BLUETOOCH_H
#define __BLUETOOCH_H
#include "stm32f10x.h"
void Bluetooth_Init(void);
void USART3_IRQHandler(void);
void BT_SendByte(uint8_t Byte);
uint8_t BT_GetRxFlag(void);
uint8_t BT_GetRxData(void);
#endif