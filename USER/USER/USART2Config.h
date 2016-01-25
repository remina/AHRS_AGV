/******************** ��Ȩ���� �人���ϻ����˿Ƽ����޹�˾ ������ **************
* �ļ���             : USART2Config.h
* ����               : ������
* �汾               : V1.0
* ����               : 9/13/2012
* ����               : USART2������ͷ�ļ�
* ����               ��USART2������ͷ�ļ�				   				   	
*******************************************************************************/
#ifndef	_USART2CONFIG_H	 
#define _USART2CONFIG_H

#include "stm32f10x.h"
#include "USART2.h"

void USART2_Configuration(unsigned long baud);

#define USART2SendByte(temp)	USART_SendData(USART2, temp) //USART1���͵�����
#define USART2RecvByte()		USART_ReceiveData(USART2)    //USART1���յ�����
#define USART2StopSendISR()		USART_ITConfig(USART2,USART_IT_TC,DISABLE) //ֹͣ�����ж�
#define USART2StartSendISR()	USART_ITConfig(USART2,USART_IT_TC,ENABLE); //���������ж�
#define USART2StopRecvISR()		USART_ITConfig(USART2,USART_IT_RXNE,DISABLE); //ֹͣ�����ж�
#define USART2StartRecvISR() 	USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);//���������ж�


#endif

