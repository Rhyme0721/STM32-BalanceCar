#include "stm32f10x.h"                  // Device header
#include "delay.h"

void Key_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//上拉输入默认高电平按键按下接地拉低
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 |GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

uint8_t Key_GetNum(void)
{
	uint8_t KeyNum = 0;
	if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_15) == 0)//left
	{
		KeyNum = 3;
	}
	if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3) == 0)//right
	{
		KeyNum = 4;
	}
	if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_14) == 0)//down
	{
		KeyNum = 2;
	}
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2) == 0)//up
	{
		KeyNum = 1;
	}
//    if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13) == 0)
//	{
//		delay_ms(20);
//		while (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13) == 0);
//		delay_ms(20);
//		KeyNum = 5;
//	}
	return KeyNum;
}
