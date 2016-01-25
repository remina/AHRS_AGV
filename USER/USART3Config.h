/******************** ��Ȩ���� �人���ϻ����˿Ƽ����޹�˾ ������ **************
* �ļ���             : USART2Config.h
* ����               : ������
* �汾               : V1.0
* ����               : 9/13/2012
* ����               : USART2������ͷ�ļ�
* ����               ��USART2������ͷ�ļ�				   				   	
*******************************************************************************/
#ifndef	_USART3CONFIG_H	 
#define _USART3CONFIG_H

#include "stm32f10x.h"
#include "USART3.h"

void USART3_Configuration(unsigned long baud);

#define USART3SendByte(temp)	USART_SendData(USART3, temp) //USART1���͵�����
#define USART3RecvByte()		USART_ReceiveData(USART3)    //USART1���յ�����
#define USART3StopSendISR()		USART_ITConfig(USART3,USART_IT_TC,DISABLE) //ֹͣ�����ж�
#define USART3StartSendISR()	USART_ITConfig(USART3,USART_IT_TC,ENABLE); //���������ж�
#define USART3StopRecvISR()		USART_ITConfig(USART3,USART_IT_RXNE,DISABLE); //ֹͣ�����ж�
#define USART3StartRecvISR() 	USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);//���������ж�


#endif
