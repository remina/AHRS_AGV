/******************** ��Ȩ���� �人���ϻ����˿Ƽ����޹�˾ ������ **************
* �ļ���             : USART1Config.h
* ����               : ������
* �汾               : V1.0
* ����               : 9/13/2012
* ����               : USART������ͷ�ļ�
* ����               ��USART������ͷ�ļ�				   				   	
*******************************************************************************/
#ifndef	_USART1CONFIG_H	 
#define _USART1CONFIG_H

#include "stm32f10x.h"
#include "USART1.h"

void USART1DMAConfiguration(u32 TxBuffer1,u16 num);
void USART1_Configuration(unsigned long baud);

#define USART1SendByte(temp)	USART_SendData(USART1, temp) //USART1���͵�����
#define USART1RecvByte()		USART_ReceiveData(USART1)    //USART1���յ�����
#define USART1StopSendISR()		USART_ITConfig(USART1,USART_IT_TC,DISABLE) //ֹͣ�����ж�
#define USART1StartSendISR()	USART_ITConfig(USART1,USART_IT_TC,ENABLE); //���������ж�
#define USART1StopRecvISR()		USART_ITConfig(USART1,USART_IT_RXNE,DISABLE); //ֹͣ�����ж�
#define USART1StartRecvISR() 	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);//���������ж�


#endif

