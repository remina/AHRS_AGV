/******************** 版权所有 武汉汉迪机器人科技有限公司 杜骁释 **************
* 文件名             : main.c
* 作者               : 杜骁释
* 版本               : V1.0
* 日期               : 9/13/2012
* 描述               : 全向轮移动平台控制系统
* 功能               ：CAN通讯，16个红外传感器读取，串口4蓝牙通讯，内存管理					   				   	
*******************************************************************************/
	
#include "USART1.h"
#include "USART2.h"
#include "USART3.h"
#include "UART4.h"
#include "CAN.h"
#include "Time.h"
#include "LED.h"
#include "MotorControl.h"
#include "magnav.h"
#include "sensors.h"
#include "task.h"
#include "odometry.h"
#include "FLASH.h"
#include "AHRS.h"


/*const unsigned char Msg0[]="/********************piooigiuhjb**************\n\r";
const unsigned char Msg1[]="* 文件名             版权所有 武汉汉迪机器人科技有限公司 杜骁释 : main.c\n\r";
const unsigned char Msg2[]="* 作者               : 杜骁释\n\r";
const unsigned char Msg3[]="* 版本               : V1.0\n\r";
const unsigned char Msg4[]="* 日期               : 9/13/2012\n\r";
const unsigned char Msg5[]="* 描述               : 全向轮移动平台控制系统\n\r";
const unsigned char Msg6[]="* 功能               ：CAN通讯，16个红外传感器读取，串口4蓝牙通讯，内存管理\n\r";
const unsigned char Msg7[]="*******************************************************************************//*\n\r";*/

/*void Welcome(void)
{																																		  
	UART4DispFun((unsigned char *)Msg0); 
	UART4DispFun((unsigned char *)Msg1); 
	UART4DispFun((unsigned char *)Msg2);
	UART4DispFun((unsigned char *)Msg3);
	UART4DispFun((unsigned char *)Msg4);
	UART4DispFun((unsigned char *)Msg5);
	UART4DispFun((unsigned char *)Msg6);
	UART4DispFun((unsigned char *)Msg7); 
}*/

/*******************************************************************************			 
* 文件名	  	 : main.c
* 描述	         : 采用PLL作为系统的时钟源（72MHZ）
* 输入           : 无
* 输出           : 无
* 返回           : 无
*******************************************************************************/
u8 t = 0;
float time_taken = 0.0;
unsigned long t3 = 0;
unsigned long t4 = 0;
u8 op = -1;

extern u8 UART4RecvPtrW, UART4RecvPtrR;
int main(void)
{
//	u8 a[] = {0xAA, 0xAA, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88,0x99,0xa0,0x04,0x10,0x08,0x01,0x08,0x99};
    u8 a[] = {0x11, 0x22, 0x33,0x44, 0x55, 0x66, 0x77};
	RobotRate rate;
	WheelSpeed wheelspeed;

  	SystemInit();
	//?????
	USART1_Init(115200);
	USART2_Init(115200);
	USART3_Init(38400);
	UART4_Init(115200);

	CAN1_Init();
	LED_Init();
//	TIM2_Init();
	TIM3_Init();
	SysTick_Init();
	Motor_init();	  
	amp_init();
	mag_sensor_init();
	flash_init();

    DelayMs(1000);	   //Time for Motor Driver Board to init

	set_all_speedctl();
	t3 = micros();
	//*************************initial sensor***************************************************************//
	while(t < 0x15)
	{
		if(UART4RecvPtrR != UART4RecvPtrW) 
		{
			op = AHRSCheckDataFrame();
			if(op == ACC_METER || op == GYRO || op == ANGLE_OUTPUT || op == MAG_METER ) 
			{
				SensorInitial(op);
				t++;
			} 
		}
		
		t4 = micros();
		time_taken = t4 - t3;
		if(time_taken > 3000000)
		{
			//break;	
		}
	
	}
	sch_init();
	sch_add_task(sensors, 6, 20);
	sch_add_task(AHRS_compute, 1, 50);
//	sch_add_task(led_task, 4, 100);
	sch_add_task(UART2Proc, 10, 20);
//  sch_add_task(UART3Proc, 3, 20);
//	sch_add_task(UART3Proc, 4, 20);
//	sch_add_task(FRIDCheck, 2, 20);
	sch_start();

		while (1)
		{
			sch_dispatch_tasks();	
			//Welcome();
		}

}
/******************* (C) ???? ????????????? *****END OF FILE****/
