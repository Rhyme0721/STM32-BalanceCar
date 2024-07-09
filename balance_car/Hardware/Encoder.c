#include "stm32f10x.h"                  // Device header

void Encoder_Init(void) //�����������ac1 PA1 ac2 PA0-TIM2 ch1 ch2  �б�����bc1 bc2 PB7 PB6-TIM4 ch1 ch2
{
    //ʱ������TIM2 GPIOA GPIOB
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    
    //PA0 PA1 ��ʼ��
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //��������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    //PB6 PB7
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //��������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//��ʱ��ʱ����Ԫ����
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;//ʱ�ӷ�Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;//������ģʽ�¸ò���û����
	TIM_TimeBaseInitStructure.TIM_Period = 65536 - 1;		//ARR 65535������ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler = 1 - 1;		//PSC �������źŲ���Ƶ
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);
    
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;//ʱ�ӷ�Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;//������ģʽ�¸ò���û����
	TIM_TimeBaseInitStructure.TIM_Period = 65536 - 1;		//ARR 65535������ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler = 1 - 1;		//PSC �������źŲ���Ƶ
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);
	
    //��ʱ��2���벶��ͨ������
	TIM_ICInitTypeDef TIM_ICInitStructure;
    TIM_ICStructInit(&TIM_ICInitStructure);//��ʱ�����벶��ṹ���ʼ�� ������ģʽ�ṹ���в��ֲ���û���ã����ó�ʼ��������Ĭ��ֵ
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;//ͨ��1
	TIM_ICInitStructure.TIM_ICFilter = 0xF;//�˲���
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;//�˴������ر�ʾ�������źŲ����� �ߵͲ���ת
	TIM_ICInit(TIM2, &TIM_ICInitStructure);
    
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;//ͨ��2
	TIM_ICInitStructure.TIM_ICFilter = 0xF;//�˲���
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;//�˴������ر�ʾ�������źŲ����� �ߵͲ���ת
	TIM_ICInit(TIM2, &TIM_ICInitStructure);
    
    //��ʱ��4���벶��ͨ������
    TIM_ICStructInit(&TIM_ICInitStructure);//��ʱ�����벶��ṹ���ʼ�� ������ģʽ�ṹ���в��ֲ���û���ã����ó�ʼ��������Ĭ��ֵ
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;//ͨ��1
	TIM_ICInitStructure.TIM_ICFilter = 0xF;//�˲���
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;//�˴������ر�ʾ�������źŲ����� �ߵͲ���ת
	TIM_ICInit(TIM4, &TIM_ICInitStructure);
    
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;//ͨ��2
	TIM_ICInitStructure.TIM_ICFilter = 0xF;//�˲���
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;//�˴������ر�ʾ�������źŲ����� �ߵͲ���ת
	TIM_ICInit(TIM4, &TIM_ICInitStructure);
    
    //������ģʽ���� 
    TIM_EncoderInterfaceConfig(TIM2,TIM_EncoderMode_TI12,TIM_ICPolarity_Falling,TIM_ICPolarity_Rising);//����������ͬΪ�������źŷ������ô˴����ûḲ�����棬�������治����Ҳ��
    
    TIM_EncoderInterfaceConfig(TIM4,TIM_EncoderMode_TI12,TIM_ICPolarity_Falling,TIM_ICPolarity_Rising);
    
    TIM_Cmd(TIM2,ENABLE);//��ʱ��ʹ��
    
    TIM_Cmd(TIM4,ENABLE);
}


int16_t EncoderLeft_Get(void)//int16_t ��������0�Լ�Ϊ65535ʱ���Զ�����Ϊ-1
{
    int16_t Temp;
    Temp=TIM_GetCounter(TIM2);
    TIM_SetCounter(TIM2,0);
    return Temp;
}

int16_t EncoderRight_Get(void)//int16_t ��������0�Լ�Ϊ65535ʱ���Զ�����Ϊ-1
{
    int16_t Temp;
    Temp=TIM_GetCounter(TIM4);
    TIM_SetCounter(TIM4,0);
    return Temp;
}