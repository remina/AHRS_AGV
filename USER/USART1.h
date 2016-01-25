/******************** ��Ȩ���� �人���ϻ����˿Ƽ����޹�˾ ������ **************
* �ļ���             : USART1.h
* ����               : ������
* �汾               : V1.0
* ����               : 9/13/2012
* ����               : ����1�м��ͷ�ļ�
* ����               ���ײ㴮��1�����ķ�װ				   				   	
*******************************************************************************/
#ifndef  _USART1_H
#define  _USART1_H

#include "stm32f10x.h"
#include "OSQMem.h"
#include "USART1Config.h"

#define		DMA_MODE 		0 		//�����ǲ���DMAģʽ��������ͨ���ж�ģʽ

#define OS_MEM_USART1_MAX 		1024			//���ͻ��������ڴ��С
#define OS_MEM_USART1_BLK 		32				//ÿһ����ĳ���
//#define OS_MEM_USART1_BLK 		64				//ÿһ����ĳ���
#define USART1_SEND_MAX_Q	  	(OS_MEM_USART1_BLK-4)	//�����ڴ���ڵ����ռ�
#define USART1_SEND_MAX_BOX		20	   					//�����ڴ����������

#define USART1_RECV_MAX_Q	  	256					//���ջ��������ռ�


void USART1_Init(unsigned long baud);
void USART1SendUpdate(void);
u8 USART1WriteDataToBuffer(u8 *buffer,u8 count);
u8 USART1DispFun(u8 *buffer);
void USART1DMAUpdate(void);
void USART1RecvUpdate(void);
void USART1GetRadarData(void);

extern u8 USART1RecvBuffer[USART1_RECV_MAX_Q];

//ע���ڴ��ڽ��յ�ʱ���Ǵ��˶�ʱ����
//���У�USART1-------TIM2
//���У�USART2-------TIM3
//���У�USART3-------TIM4
//���У�USART4-------TIM5
//���У�USART5-------TIM6

#endif



