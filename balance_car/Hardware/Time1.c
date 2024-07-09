#include "stm32f10x.h"                  // Device header
#include "Motor.h"
#include "OLED.h"
#include "LED.h"
uint16_t Time1=0;
uint16_t Time2=0;
void Time1_Init(void)//72MHz/3600/20=1kHz 1ms
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);  //打开TIM1的中断时钟
    
    TIM_InternalClockConfig(TIM1);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 3599;
	TIM_TimeBaseInitStructure.TIM_Prescaler = 19;
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);             //初始化
	
    TIM_ClearFlag(TIM1, TIM_FLAG_Update);
    TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE);                    //打开定时器中断
	
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_Cmd(TIM1, ENABLE);
    
}


void TIM1_UP_IRQHandler(void)
{
    Time1++;
    Time2++;
    MPU6050_Data_read();	//获取陀螺仪数据	
//    OLED_ShowSignedNum(3,1,pitch,5);
//    OLED_ShowSignedNum(4,1,gyroy,5);
	if(Time1>=5)//5ms读取一次编码器
	{
		Time1=0;        
	    Moto_Speed_Read(); //读取速度
        
//        OLED_ShowSignedNum(3,1,speed_num1,5);
//        OLED_ShowSignedNum(3,7,speed_num2,5);
	}
//    if(Time2>=2)
//    {
//        Time2=0;
//        Smart_Car_Task();
//    }
Smart_Car_Task();
   
    TIM_ClearITPendingBit(TIM1,TIM_IT_Update);
}