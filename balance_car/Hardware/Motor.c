#include "stm32f10x.h"                  // Device header
#include "mpu6050.h"  
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "Encoder.h"
u8 mpu6050_data_flag;  //�Ƿ�������ȡ����

int16_t speed_num1=0;      //1�ŵ���ٶ� 
int16_t speed_num2=0;      //2�ŵ���ٶ�
float pitch,roll,yaw;  //ŷ����
short aacx,aacy,aacz;	 //���ٶȴ�����ԭʼ����
short gyrox,gyroy,gyroz;//������ԭʼ����
int sudu_out=0.0f;     //�ٶȻ����
int zhili_out=0;       //ֱ�������
int zhuan_out=0;        //ת�����
int PWM_OUT1=0;        //�������������
int PWM_OUT2=0;        //����������ҵ��
//Ŀ���ƶ��ٶ�
int Movement=0; 
//Ŀ��ת��Ƕ�
int turnment=0;   
//��е���
float Car_zero=-1.8f;//0.3
//ֱ����
float zhili_Kp=30.0f;  //����+  65*0.6 39                25.0
float zhili_Kd=-0.12f;       //����-   -0.32*0.6  -0.192    -0.15
//�ٶȻ�
float sudu_Kp=8.0f;     //����+   43  60              5.0
float sudu_Ki=0.040f;    //����+ kp/200  0.3              0.025
//ת��
float zhuan_Kp=-10.0f;   //���Ը� ����С��ת��������
float zhuan_Kd=0.05f;    //����������С��ת�򣬸�����


void Motor_Init(void) //����ain1 PA7-TIM3-ch2  ain2 PB1-TIM3-ch4 �ҵ�� bin1 PA6-TIM3-ch1 bin2 PB0-TIM3-ch3
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    
    //PA6 PA7 PWM���ų�ʼ�� TIM3 ch1 ch2
    GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7|GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//�����������
	GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    //PA4 PA5 ���ɲ�����ų�ʼ�� 0ɲ�� 1�ͷ�
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//�������
	GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    //PB0 PB1 ����������ų�ʼ�� 0��ת 1��ת(ʵ�ʷ�����ݵ����װλ�ý��е���)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    
    TIM_InternalClockConfig(TIM3);
    
    //ʱ����Ԫ��ʼ�� ��ʱ��Ƶ��=72MHz/arr/psc=10KHz
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;   
    TIM_TimeBaseInitStructure.TIM_Period=3600-1;                     //arr�Զ���װֵ����7200,PWMƵ��10KHz  
    TIM_TimeBaseInitStructure.TIM_Prescaler=0;                       //psc����Ƶ
    TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;        //���˲�
    TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;    //���ϼ���ģʽ
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);               //��ʼ��
    

    //TIM3 ch1 ch2������Ƚ�ͨ������
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;            //PWMģʽ1
    TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;  //��PWMʹ��
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     //�������
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // ����Ƚ�ʹ��
	TIM_OCInitStructure.TIM_Pulse = 0;		//CCRռ�ձ�
	TIM_OC1Init(TIM3, & TIM_OCInitStructure); 	                 //��ʼ��  ʹ��ͨ��1
	TIM_OC2Init(TIM3, & TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM3,TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM3,TIM_OCPreload_Enable);
    
    
    //TIM3ʹ��

    TIM_Cmd(TIM3,ENABLE);
    
    //�����ռ�ձȾ�ֹ ռ�ձ�Ϊ0����
    TIM_SetCompare1(TIM3,3600);
    TIM_SetCompare2(TIM3,3600);
    //��ʼ�����ɲ��
    GPIO_ResetBits(GPIOA,GPIO_Pin_4|GPIO_Pin_5);
    GPIO_ResetBits(GPIOB,GPIO_Pin_1|GPIO_Pin_0);
}

int Motor_xianfu(int pwm)
{
    if(pwm>1500) pwm=1500;
    if(pwm<-1500) pwm=-1500;
    return pwm;
}
void Motor1_SetSpeed(int pwm)//pwm 0-3200 ��ֹ-����
{
    Motor_xianfu(pwm);
    if(pwm>0)
    {
        TIM_SetCompare1(TIM3,3600-pwm);
        GPIO_SetBits(GPIOA,GPIO_Pin_4);//�ͷ�
        GPIO_ResetBits(GPIOB,GPIO_Pin_0);//��ת
    }
    if(pwm<0)
    {
        TIM_SetCompare1(TIM3,3600+pwm);
        GPIO_SetBits(GPIOA,GPIO_Pin_4);//�ͷ�
        GPIO_SetBits(GPIOB,GPIO_Pin_0);//��ת       
    }
    if(pwm==0)
    {
        TIM_SetCompare1(TIM3,3600);
        GPIO_ResetBits(GPIOA,GPIO_Pin_4);//ɲ��
        GPIO_ResetBits(GPIOB,GPIO_Pin_0);
    }
    
}

void Motor2_SetSpeed(int pwm)// -2000<=Speed<=2000
{
    Motor_xianfu(pwm);
    if(pwm>=0)
    {
        TIM_SetCompare2(TIM3,3600-pwm);
        GPIO_SetBits(GPIOA,GPIO_Pin_5);//�ͷ�
        GPIO_SetBits(GPIOB,GPIO_Pin_1);//��ת
    }
    if(pwm<0)
    {
        TIM_SetCompare2(TIM3,3600+pwm);
        GPIO_SetBits(GPIOA,GPIO_Pin_5);//�ͷ�
        GPIO_ResetBits(GPIOB,GPIO_Pin_1);//��ת       
    }
    if(pwm==0)
    {
        TIM_SetCompare2(TIM3,3600);
        GPIO_ResetBits(GPIOA,GPIO_Pin_5);//�ͷ�
        GPIO_ResetBits(GPIOB,GPIO_Pin_1);
    }
}

//�Ƕȼ��ٶȻ�ȡ
void MPU6050_Data_read(void)
{
   //mpu6050_data_flag=mpu_dmp_get_data(&pitch,&roll,&yaw);      //�õ��Ƕ�����
    while(mpu_dmp_get_data(&pitch,&roll,&yaw));
	//MPU_Get_Accelerometer(&aacx,&aacy,&aacz); //�õ����ٶȴ���������
	 //MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	 //�õ�����������	
    while(MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz));
}


void Moto_Speed_Read(void)
{
    speed_num1=EncoderLeft_Get();
    speed_num2=EncoderRight_Get();
}

//ֱ����
int zhili(float Angle,float Gyro)
{  
   float err;
	 int pwm_zhili;
	 err=Car_zero-Angle;    //����ֵ-ʵ��ֵ����������С��ƽ�⣬�������ֵ���ǻ�е��ֵ       
	 pwm_zhili=zhili_Kp*err+Gyro*zhili_Kd;//����ƽ����Ƶĵ��PWM
	 return pwm_zhili;
}

//�ٶȻ�
int sudu(int encoder_left,int encoder_right)
{  
	  static int pwm_sudu,Encoder_Least,Encoder;
	  static int Encoder_Integral;	
		Encoder_Least =(encoder_left+encoder_right)-Movement;  //��ȡ�����ٶ�ƫ��==�����ٶȣ����ұ�����֮�ͣ�-Ŀ���ٶȣ��˴�Ϊ�㣩 
		Encoder *= 0.8;		                             //һ�׵�ͨ�˲���       
		Encoder += Encoder_Least*0.2;	                 //һ�׵�ͨ�˲���    
  	Encoder_Integral +=Encoder;                     //���ֳ�λ�� ����ʱ�䣺5ms
		if(Encoder_Integral>2000)  Encoder_Integral=2000;  //�����޷�
		if(Encoder_Integral<-2000)	Encoder_Integral=-2000; //�����޷�	
		pwm_sudu=sudu_Kp*Encoder+sudu_Ki*Encoder_Integral;     //�ٶ�PI������	
		if((pitch>=80)||(pitch<=-80))  //С������������
		{
		  Encoder_Integral=0;    
		}			
	  return pwm_sudu;
}


//ת��
int zhuan(float Set_turn,float Gyro_Z)
{
  int PWM_Out=0; 
	if(Set_turn==0)
	{
	 PWM_Out=zhuan_Kd*Gyro_Z; //û��ת������KdԼ��С��ת��
	}
	if(Set_turn!=0)
	{
	 PWM_Out=zhuan_Kp*Set_turn; //��ת������KpΪ����С��ת�� 
	}
	return PWM_Out;
}


void Smart_Car_Task(void)
{
	zhili_out=zhili(pitch,gyroy);															//ֱ����
	sudu_out=sudu(speed_num1,speed_num2);                     //�ٶȻ�
    zhuan_out=zhuan(turnment,gyroz);                          //ת�򻷣����й�ϵ
	PWM_OUT1=zhili_out+sudu_out+zhuan_out;                    //�ٶȲ����ת��Ч��
	PWM_OUT2=zhili_out+sudu_out-zhuan_out;                    //�ٶȲ����ת��Ч��
    
    //����
    if(PWM_OUT1>=0)
    {
            PWM_OUT1+=140;
    }
    else
    {
        PWM_OUT1-=140;
    }

    if(PWM_OUT2>=0)
    {
            PWM_OUT2+=140;
    }
    else
    {
        PWM_OUT2-=140;
    }

	if((pitch>=80)||(pitch<=-80))
	{
	    PWM_OUT1=0;
		PWM_OUT2=0;       
	}

	Motor1_SetSpeed(PWM_OUT1);
    Motor2_SetSpeed(PWM_OUT2);
	
}