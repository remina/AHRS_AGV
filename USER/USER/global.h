#ifndef	_GLOBAL_H	 
#define _GLOBAL_H

#include "stm32f10x.h"
#include "MotorControl.h"
/******************************************************************************
�����ж��������ȼ�
����1���ж����ȼ���Ϊ0����Ӧ����Ϊ3
����2���ж����ȼ���Ϊ0����Ӧ����Ϊ6
����4���ж����ȼ���Ϊ0����Ӧ����Ϊ4
����1DMA TX: �ж����ȼ���Ϊ0����Ӧ����Ϊ0	
CAN	  TX:�ж����ȼ���Ϊ0����Ӧ����Ϊ2
CAN	  RX:�ж����ȼ���Ϊ0����Ӧ����Ϊ1
TIME2���ж����ȼ���Ϊ0����Ӧ����Ϊ5
			   				   	
*******************************************************************************/
typedef signed long long s64;  

#define	GPID 0x0121
#define LF_MOTOR_ID 30
#define RF_MOTOR_ID 20
#define LB_MOTOR_ID 23
#define RB_MOTOR_ID 26
#define LL_MOTOR_ID 0x0125
#define RL_MOTOR_ID 0x0126

#define WHEEL_LF  0
#define WHEEL_LB  1
#define WHEEL_RF  2
#define WHEEL_RB  3

#define PID_FRONT 0
#define PID_BACK  1
#define PID_LEFT  2
#define PID_RIGHT 3

#define FRONT  1
#define BACK   2
#define LEFT   3
#define RIGHT  4
#define LEFT_ROTATE 5
#define RIGHT_ROTATE 6
#define ALLDIR 5

#define OPENSENSOR		1	
#define CLOSESENSOR		0

#define TRUE  1
#define FALSE 0 

#define  PI			   3.1415926

//extern u8 Manul_Control_Flag;
//extern u8 Auto_Nav_Flag;
//extern u8 Time_50ms_Flag;



#endif

