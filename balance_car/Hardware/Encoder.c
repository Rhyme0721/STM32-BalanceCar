#include "stm32f10x.h"                  // Device header

void Encoder_Init(void) //左编码器引脚ac1 PA1 ac2 PA0-TIM2 ch1 ch2  有编码器bc1 bc2 PB7 PB6-TIM4 ch1 ch2
{
    //时钟设置TIM2 GPIOA GPIOB
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    
    //PA0 PA1 初始化
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //上拉输入
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    //PB6 PB7
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //上拉输入
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//定时器时基单元配置
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;//时钟分频
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;//编码器模式下该参数没作用
	TIM_TimeBaseInitStructure.TIM_Period = 65536 - 1;		//ARR 65535最大计数值
	TIM_TimeBaseInitStructure.TIM_Prescaler = 1 - 1;		//PSC 编码器信号不分频
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);
    
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;//时钟分频
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;//编码器模式下该参数没作用
	TIM_TimeBaseInitStructure.TIM_Period = 65536 - 1;		//ARR 65535最大计数值
	TIM_TimeBaseInitStructure.TIM_Prescaler = 1 - 1;		//PSC 编码器信号不分频
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);
	
    //定时器2输入捕获通道设置
	TIM_ICInitTypeDef TIM_ICInitStructure;
    TIM_ICStructInit(&TIM_ICInitStructure);//定时器输入捕获结构体初始化 编码器模式结构体中部分参数没作用，调用初始化函数给默认值
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;//通道1
	TIM_ICInitStructure.TIM_ICFilter = 0xF;//滤波器
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;//此处上升沿表示编码器信号不反向 高低不反转
	TIM_ICInit(TIM2, &TIM_ICInitStructure);
    
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;//通道2
	TIM_ICInitStructure.TIM_ICFilter = 0xF;//滤波器
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;//此处上升沿表示编码器信号不反向 高低不反转
	TIM_ICInit(TIM2, &TIM_ICInitStructure);
    
    //定时器4输入捕获通道设置
    TIM_ICStructInit(&TIM_ICInitStructure);//定时器输入捕获结构体初始化 编码器模式结构体中部分参数没作用，调用初始化函数给默认值
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;//通道1
	TIM_ICInitStructure.TIM_ICFilter = 0xF;//滤波器
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;//此处上升沿表示编码器信号不反向 高低不反转
	TIM_ICInit(TIM4, &TIM_ICInitStructure);
    
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;//通道2
	TIM_ICInitStructure.TIM_ICFilter = 0xF;//滤波器
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;//此处上升沿表示编码器信号不反向 高低不反转
	TIM_ICInit(TIM4, &TIM_ICInitStructure);
    
    //编码器模式配置 
    TIM_EncoderInterfaceConfig(TIM2,TIM_EncoderMode_TI12,TIM_ICPolarity_Falling,TIM_ICPolarity_Rising);//后两个参数同为编码器信号反向设置此次配置会覆盖上面，所以上面不设置也行
    
    TIM_EncoderInterfaceConfig(TIM4,TIM_EncoderMode_TI12,TIM_ICPolarity_Falling,TIM_ICPolarity_Rising);
    
    TIM_Cmd(TIM2,ENABLE);//定时器使能
    
    TIM_Cmd(TIM4,ENABLE);
}


int16_t EncoderLeft_Get(void)//int16_t 计数器从0自减为65535时可自动补码为-1
{
    int16_t Temp;
    Temp=TIM_GetCounter(TIM2);
    TIM_SetCounter(TIM2,0);
    return Temp;
}

int16_t EncoderRight_Get(void)//int16_t 计数器从0自减为65535时可自动补码为-1
{
    int16_t Temp;
    Temp=TIM_GetCounter(TIM4);
    TIM_SetCounter(TIM4,0);
    return Temp;
}