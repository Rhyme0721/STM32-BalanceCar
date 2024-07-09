#ifndef __MOTOR_H
#define __MOTOR_H
#include "stm32f10x.h"
extern u8 mpu6050_data_flag;  //�Ƿ�������ȡ����
extern int16_t speed_num1;      //1�ŵ���ٶ� 
extern int16_t speed_num2;      //2�ŵ���ٶ�
extern float pitch,roll,yaw;  //ŷ����
extern short aacx,aacy,aacz;	 //���ٶȴ�����ԭʼ����
extern short gyrox,gyroy,gyroz;//������ԭʼ����
extern int sudu_out;     //�ٶȻ����
extern int zhili_out;       //ֱ�������
extern int zhuan_out;        //ת�����
extern int PWM_OUT1;        //�������������
extern int PWM_OUT2;        //����������ҵ��
//Ŀ���ƶ��ٶ�
extern int Movement;   
extern int turnment;   
//��е���
extern float Car_zero;
//ֱ����
extern float zhili_Kp;  //���ִ���ȵ�Ƶ�� -500  
extern float zhili_Kd;    //����С���ȸ�Ƶ�� 1.8   
//�ٶȻ�
extern float sudu_Kp;  
extern float sudu_Ki;    //����200
//ת��
extern float zhuan_Kp;   //����С��ת��������
extern float zhuan_Kd;    //����С��ת�򣬸�����


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