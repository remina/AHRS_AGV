/******************** ��Ȩ���� �人���ϻ����˿Ƽ����޹�˾ ������ **************
* �ļ���             : UART4.h
* ����               : ������
* �汾               : V1.0
* ����               : 9/13/2012
* ����               : ����4�м��ͷ�ļ�
* ����               ���ײ㴮��4�����ķ�װ��Ϊ����ͨѶ�ṩ�ӿ�				   				   	
*******************************************************************************/
#ifndef  _UART4_H
#define  _UART4_H

#include "stm32f10x.h"
#include "OSQMem.h"
#include "UART4Config.h"


#define OS_MEM_UART4_MAX 		1024			//���ͻ��������ڴ��С
#define OS_MEM_UART4_BLK 		32				//ÿһ����ĳ���
#define UART4_SEND_MAX_Q	  	(OS_MEM_UART4_BLK-4)	//�����ڴ���ڵ����ռ�
#define UART4_SEND_MAX_BOX		20	   					//�����ڴ����������

#define UART4_RECV_MAX_Q	  	256					//���ջ��������ռ�

void UART4_Init(unsigned long baud);
void UART4SendUpdate(void);
void UART4RecvUpdate(void);
u8 UART4WriteDataToBuffer(u8 *buffer,u8 count);
u8 UART4DispFun(u8 *buffer);
extern void FRIDCheck(void);
extern void FRIDCpyData(void);

extern u8 UART4RecvBuffer[UART4_RECV_MAX_Q];

extern u8 g_rfid_num;
extern u8 g_rfid_num_last;
extern u8 g_rfid_data_flag;

//ע���ڴ��ڽ��յ�ʱ���Ǵ��˶�ʱ����
//���У�USART1-------TIM2
//���У�USART2-------TIM3
//���У�USART3-------TIM4
//���У�USART4-------TIM5
//���У�USART5-------TIM6

#endif

