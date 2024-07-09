#ifndef __MOTOR_H
#define __MOTOR_H
#include "stm32f10x.h"
extern u8 mpu6050_data_flag;  //是否正常读取数据
extern int16_t speed_num1;      //1号电机速度 
extern int16_t speed_num2;      //2号电机速度
extern float pitch,roll,yaw;  //欧拉角
extern short aacx,aacy,aacz;	 //加速度传感器原始数据
extern short gyrox,gyroy,gyroz;//陀螺仪原始数据
extern int sudu_out;     //速度环输出
extern int zhili_out;       //直立环输出
extern int zhuan_out;        //转向换输出
extern int PWM_OUT1;        //最终输出到左电机
extern int PWM_OUT2;        //最终输出到右电机
//目标移动速度
extern int Movement;   
extern int turnment;   
//机械零点
extern float Car_zero;
//直立环
extern float zhili_Kp;  //出现大幅度低频振荡 -500  
extern float zhili_Kd;    //出现小幅度高频振荡 1.8   
//速度环
extern float sudu_Kp;  
extern float sudu_Ki;    //除以200
//转向环
extern float zhuan_Kp;   //期望小车转向，正反馈
extern float zhuan_Kd;    //抑制小车转向，负反馈


void Motor_Init(void);
int Motor_xianfu(int pwm);
void Motor1_SetSpeed(int Speed);
void Motor2_SetSpeed(int Speed);
void MPU6050_Data_read(void);
void Moto_Speed_Read(void);
int zhili(float Angle,float Gyro);
int sudu(int encoder_left,int encoder_right);
int zhuan(float Set_turn,float Gyro_Z);
void Smart_Car_Task(void);

#endif