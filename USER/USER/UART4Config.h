/******************** ��Ȩ���� �人���ϻ����˿Ƽ����޹�˾ ������ **************
* �ļ���             : UART4Config.h
* ����               : ������
* �汾               : V1.0
* ����               : 9/13/2012
* ����               : UART4������ͷ�ļ�
* ����               ��UART4������ͷ�ļ�,���ⲿ����ģ��ͨѶ				   				   	
*******************************************************************************/

#ifndef	_UART4CONFIG_H	 
#define _UART4CONFIG_H

#include "stm32f10x.h"
#include "UART4.h"

void UART4_Configuration(unsigned long baud);

#define UART4SendByte(temp)	USART_SendData(UART4, temp) //UART4���͵�����
#define UART4RecvByte()		USART_ReceiveData(UART4)    //UART4���յ�����
#define UART4StopSendISR()		USART_ITConfig(UART4,USART_IT_TC,DISABLE) //ֹͣ�����ж�
#define UART4StartSendISR()	USART_ITConfig(UART4,USART_IT_TC,ENABLE); //���������ж�
#define UART4StopRecvISR()		USART_ITConfig(UART4,USART_IT_RXNE,DISABLE); //ֹͣ�����ж�
#define UART4StartRecvISR() 	USART_ITConfig(UART4,USART_IT_RXNE,ENABLE);//���������ж�

#endif
