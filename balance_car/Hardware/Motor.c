#include "stm32f10x.h"                  // Device header
#include "mpu6050.h"  
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "Encoder.h"
u8 mpu6050_data_flag;  //是否正常读取数据

int16_t speed_num1=0;      //1号电机速度 
int16_t speed_num2=0;      //2号电机速度
float pitch,roll,yaw;  //欧拉角
short aacx,aacy,aacz;	 //加速度传感器原始数据
short gyrox,gyroy,gyroz;//陀螺仪原始数据
int sudu_out=0.0f;     //速度环输出
int zhili_out=0;       //直立环输出
int zhuan_out=0;        //转向换输出
int PWM_OUT1=0;        //最终输出到左电机
int PWM_OUT2=0;        //最终输出到右电机
//目标移动速度
int Movement=0; 
//目标转向角度
int turnment=0;   
//机械零点
float Car_zero=-1.8f;//0.3
//直立环
float zhili_Kp=30.0f;  //极性+  65*0.6 39                25.0
float zhili_Kd=-0.12f;       //极性-   -0.32*0.6  -0.192    -0.15
//速度环
float sudu_Kp=8.0f;     //极性+   43  60              5.0
float sudu_Ki=0.040f;    //极性+ kp/200  0.3              0.025
//转向环
float zhuan_Kp=-10.0f;   //极性负 期望小车转向，正反馈
float zhuan_Kd=0.05f;    //极性正抑制小车转向，负反馈


void Motor_Init(void) //左电机ain1 PA7-TIM3-ch2  ain2 PB1-TIM3-ch4 右电机 bin1 PA6-TIM3-ch1 bin2 PB0-TIM3-ch3
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    
    //PA6 PA7 PWM引脚初始化 TIM3 ch1 ch2
    GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7|GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//复用推挽输出
	GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    //PA4 PA5 电机刹车引脚初始化 0刹车 1释放
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//推挽输出
	GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    //PB0 PB1 电机方向引脚初始化 0反转 1正转(实际方向根据电机安装位置进行调整)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    
    TIM_InternalClockConfig(TIM3);
    
    //时基单元初始化 定时器频率=72MHz/arr/psc=10KHz
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;   
    TIM_TimeBaseInitStructure.TIM_Period=3600-1;                     //arr自动重装值计数7200,PWM频率10KHz  
    TIM_TimeBaseInitStructure.TIM_Prescaler=0;                       //psc不分频
    TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;        //不滤波
    TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;    //向上计数模式
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);               //初始化
    

    //TIM3 ch1 ch2出输出比较通道配置
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;            //PWM模式1
    TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;  //打开PWM使能
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     //输出极性
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // 输出比较使能
	TIM_OCInitStructure.TIM_Pulse = 0;		//CCR占空比
	TIM_OC1Init(TIM3, & TIM_OCInitStructure); 	                 //初始化  使用通道1
	TIM_OC2Init(TIM3, & TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM3,TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM3,TIM_OCPreload_Enable);
    
    
    //TIM3使能

    TIM_Cmd(TIM3,ENABLE);
    
    //电机满占空比静止 占空比为0满速
    TIM_SetCompare1(TIM3,3600);
    TIM_SetCompare2(TIM3,3600);
    //初始化电机刹车
    GPIO_ResetBits(GPIOA,GPIO_Pin_4|GPIO_Pin_5);
    GPIO_ResetBits(GPIOB,GPIO_Pin_1|GPIO_Pin_0);
}

int Motor_xianfu(int pwm)
{
    if(pwm>1500) pwm=1500;
    if(pwm<-1500) pwm=-1500;
    return pwm;
}
void Motor1_SetSpeed(int pwm)//pwm 0-3200 静止-满速
{
    Motor_xianfu(pwm);
    if(pwm>0)
    {
        TIM_SetCompare1(TIM3,3600-pwm);
        GPIO_SetBits(GPIOA,GPIO_Pin_4);//释放
        GPIO_ResetBits(GPIOB,GPIO_Pin_0);//正转
    }
    if(pwm<0)
    {
        TIM_SetCompare1(TIM3,3600+pwm);
        GPIO_SetBits(GPIOA,GPIO_Pin_4);//释放
        GPIO_SetBits(GPIOB,GPIO_Pin_0);//反转       
    }
    if(pwm==0)
    {
        TIM_SetCompare1(TIM3,3600);
        GPIO_ResetBits(GPIOA,GPIO_Pin_4);//刹车
        GPIO_ResetBits(GPIOB,GPIO_Pin_0);
    }
    
}

void Motor2_SetSpeed(int pwm)// -2000<=Speed<=2000
{
    Motor_xianfu(pwm);
    if(pwm>=0)
    {
        TIM_SetCompare2(TIM3,3600-pwm);
        GPIO_SetBits(GPIOA,GPIO_Pin_5);//释放
        GPIO_SetBits(GPIOB,GPIO_Pin_1);//正转
    }
    if(pwm<0)
    {
        TIM_SetCompare2(TIM3,3600+pwm);
        GPIO_SetBits(GPIOA,GPIO_Pin_5);//释放
        GPIO_ResetBits(GPIOB,GPIO_Pin_1);//反转       
    }
    if(pwm==0)
    {
        TIM_SetCompare2(TIM3,3600);
        GPIO_ResetBits(GPIOA,GPIO_Pin_5);//释放
        GPIO_ResetBits(GPIOB,GPIO_Pin_1);
    }
}

//角度加速度获取
void MPU6050_Data_read(void)
{
   //mpu6050_data_flag=mpu_dmp_get_data(&pitch,&roll,&yaw);      //得到角度数据
    while(mpu_dmp_get_data(&pitch,&roll,&yaw));
	//MPU_Get_Accelerometer(&aacx,&aacy,&aacz); //得到加速度传感器数据
	 //MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	 //得到陀螺仪数据	
    while(MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz));
}


void Moto_Speed_Read(void)
{
    speed_num1=EncoderLeft_Get();
    speed_num2=EncoderRight_Get();
}

//直立环
int zhili(float Angle,float Gyro)
{  
   float err;
	 int pwm_zhili;
	 err=Car_zero-Angle;    //期望值-实际值，这里期望小车平衡，因此期望值就是机械中值       
	 pwm_zhili=zhili_Kp*err+Gyro*zhili_Kd;//计算平衡控制的电机PWM
	 return pwm_zhili;
}

//速度环
int sudu(int encoder_left,int encoder_right)
{  
	  static int pwm_sudu,Encoder_Least,Encoder;
	  static int Encoder_Integral;	
		Encoder_Least =(encoder_left+encoder_right)-Movement;  //获取最新速度偏差==测量速度（左右编码器之和）-目标速度（此次为零） 
		Encoder *= 0.8;		                             //一阶低通滤波器       
		Encoder += Encoder_Least*0.2;	                 //一阶低通滤波器    
  	Encoder_Integral +=Encoder;                     //积分出位移 积分时间：5ms
		if(Encoder_Integral>2000)  Encoder_Integral=2000;  //积分限幅
		if(Encoder_Integral<-2000)	Encoder_Integral=-2000; //积分限幅	
		pwm_sudu=sudu_Kp*Encoder+sudu_Ki*Encoder_Integral;     //速度PI控制器	
		if((pitch>=80)||(pitch<=-80))  //小车跌倒后清零
		{
		  Encoder_Integral=0;    
		}			
	  return pwm_sudu;
}


//转向环
int zhuan(float Set_turn,float Gyro_Z)
{
  int PWM_Out=0; 
	if(Set_turn==0)
	{
	 PWM_Out=zhuan_Kd*Gyro_Z; //没有转向需求，Kd约束小车转向
	}
	if(Set_turn!=0)
	{
	 PWM_Out=zhuan_Kp*Set_turn; //有转向需求，Kp为期望小车转向 
	}
	return PWM_Out;
}


void Smart_Car_Task(void)
{
	zhili_out=zhili(pitch,gyroy);															//直立环
	sudu_out=sudu(speed_num1,speed_num2);                     //速度环
    zhuan_out=zhuan(turnment,gyroz);                          //转向环，并行关系
	PWM_OUT1=zhili_out+sudu_out+zhuan_out;                    //速度差产生转向效果
	PWM_OUT2=zhili_out+sudu_out-zhuan_out;                    //速度差产生转向效果
    
    //死区
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