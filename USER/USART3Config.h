/******************** 版权所有 武汉汉迪机器人科技有限公司 杜骁释 **************
* 文件名             : USART2Config.h
* 作者               : 杜骁释
* 版本               : V1.0
* 日期               : 9/13/2012
* 描述               : USART2驱动层头文件
* 功能               ：USART2驱动层头文件				   				   	
*******************************************************************************/
#ifndef	_USART3CONFIG_H	 
#define _USART3CONFIG_H

#include "stm32f10x.h"
#include "USART3.h"

void USART3_Configuration(unsigned long baud);

#define USART3SendByte(temp)	USART_SendData(USART3, temp) //USART1发送的数据
#define USART3RecvByte()		USART_ReceiveData(USART3)    //USART1接收的数据
#define USART3StopSendISR()		USART_ITConfig(USART3,USART_IT_TC,DISABLE) //停止发送中断
#define USART3StartSendISR()	USART_ITConfig(USART3,USART_IT_TC,ENABLE); //开启发送中断
#define USART3StopRecvISR()		USART_ITConfig(USART3,USART_IT_RXNE,DISABLE); //停止接收中断
#define USART3StartRecvISR() 	USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);//开启接收中断


#endif
