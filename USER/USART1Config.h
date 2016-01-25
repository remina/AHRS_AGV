/******************** 版权所有 武汉汉迪机器人科技有限公司 杜骁释 **************
* 文件名             : USART1Config.h
* 作者               : 杜骁释
* 版本               : V1.0
* 日期               : 9/13/2012
* 描述               : USART驱动层头文件
* 功能               ：USART驱动层头文件				   				   	
*******************************************************************************/
#ifndef	_USART1CONFIG_H	 
#define _USART1CONFIG_H

#include "stm32f10x.h"
#include "USART1.h"

void USART1DMAConfiguration(u32 TxBuffer1,u16 num);
void USART1_Configuration(unsigned long baud);

#define USART1SendByte(temp)	USART_SendData(USART1, temp) //USART1发送的数据
#define USART1RecvByte()		USART_ReceiveData(USART1)    //USART1接收的数据
#define USART1StopSendISR()		USART_ITConfig(USART1,USART_IT_TC,DISABLE) //停止发送中断
#define USART1StartSendISR()	USART_ITConfig(USART1,USART_IT_TC,ENABLE); //开启发送中断
#define USART1StopRecvISR()		USART_ITConfig(USART1,USART_IT_RXNE,DISABLE); //停止接收中断
#define USART1StartRecvISR() 	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);//开启接收中断


#endif

