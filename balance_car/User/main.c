#include "stm32f10x.h"                  // Device header
#include "OLED.h"
#include "Key.h"
#include "delay.h"
#include "sys.h"
#include "mpu6050.h"  
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "Encoder.h"
#include "Motor.h"
#include "Time1.h"
#include "Bluetooth.h"
int i=0;
uint8_t RxData;
int main(void)
{ 
    Key_Init();
    OLED_Init();     
    Encoder_Init();
    Bluetooth_Init();
    Motor_Init();
    OLED_ShowString(1,1,"INIT");
    //ÍÓÂÝÒÇ³õÊ¼»¯
    i=MPU_Init();
    OLED_ShowNum(1,6,i,1);
    while(mpu_dmp_init()){}
        OLED_ShowString(1,8,"ok");
    OLED_ShowSignedNum(3,1,zhili_Kp,4);
    OLED_ShowSignedNum(3,6,zhili_Kd,4);
    OLED_ShowSignedNum(4,1,sudu_Kp,4);
    OLED_ShowSignedNum(4,6,sudu_Ki,4);
    Time1_Init();
        
    while(1)
    {
     
//        i=Key_GetNum();
//      switch(i)
//      {
//          case 1:
//              OLED_ShowNum(3,1,1,1);break;
//          case 2:
//              OLED_ShowNum(3,1,2,1);break;
//          case 3:
//              OLED_ShowNum(3,1,3,1);break;
//          case 4:                   
//              OLED_ShowNum(3,1,4,1);break;
//          default:
//              OLED_ShowNum(3,1,0,1);break;
//            
//        
//      }
          
//        Motor1_SetSpeed(-120);//ËÀÇø
//        Motor2_SetSpeed(-120);
//        Motor1_SetSpeed(1500);
//        Motor2_SetSpeed(1500);
        
//        MPU6050_Data_read();
//        
//        OLED_ShowSignedNum(3,1,pitch,5);
//        OLED_ShowSignedNum(4,1,gyroy,5);
        


        if (BT_GetRxFlag() == 1)
		{
			RxData = BT_GetRxData();
            
            if(RxData==0x00)
            {
                Movement=0;
                turnment=0;
            }
            if(RxData==0x01)
            {
                Movement=80;
            }
            
            if(RxData==0x02)
            {
                Movement=-80;
            }
            
            if(RxData==0x03)
            {
                turnment=10;
            }
             
            if(RxData==0x04)
            {
                turnment=-10;
            }
            
            
//			BT_SendByte(RxData);
//			OLED_ShowHexNum(2, 8, RxData, 2);
		}
//        
//        
   
        
        
//        OLED_ShowSignedNum(2,1,(int32_t)pitch,3);
//        OLED_ShowNum(2,6,((int32_t)(pitch*100))%100,2);
////        Motor1_SetSpeed(-3500);
////        Motor2_SetSpeed(-3500);
////        Motor1_SetSpeed(3600);
////        Motor2_SetSpeed(3500);
//         OLED_ShowSignedNum(3,1,speed_num1,5);
//         OLED_ShowSignedNum(3,7,speed_num2,5);
//         OLED_ShowSignedNum(4,1,PWM_OUT1,5);
//         OLED_ShowSignedNum(4,7,PWM_OUT2,5);
//        
        
    }
}
