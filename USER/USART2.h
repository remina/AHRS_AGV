#ifndef __USART2_H
#define	__USART2_H

#include "USART2Config.h"
#include "stm32f10x.h"
#include "OSQMem.h"
#include "MotorControl.h"
#include "stdio.h"
#include "math.h"




#define OS_MEM_USART2_MAX 		1024			//���ͻ��������ڴ��С
#define OS_MEM_USART2_BLK 		32				//ÿһ����ĳ���
#define USART2_SEND_MAX_Q	  	(OS_MEM_USART2_BLK-4)	//�����ڴ���ڵ����ռ�
#define USART2_SEND_MAX_BOX		20	   					//�����ڴ����������

#define USART2_RECV_MAX_Q	  	256					//���ջ��������ռ�

void USART2_Init(unsigned long baud);
void USART2SendUpdate(void);
u8 USART2WriteDataToBuffer(u8 *buffer,u8 count);
u8 USART2DispFun(u8 *buffer);
void USART2RecvUpdate(void);
void UART2Proc(void);
extern u8 USART2RecvBuffer[USART2_RECV_MAX_Q];
extern float g_bt_manual_botrate;

#endif /* __USART2_H */
