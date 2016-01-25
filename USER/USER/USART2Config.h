/******************** 版权所有 武汉汉迪机器人科技有限公司 杜骁释 **************
* 文件名             : USART2Config.h
* 作者               : 杜骁释
* 版本               : V1.0
* 日期               : 9/13/2012
* 描述               : USART2驱动层头文件
* 功能               ：USART2驱动层头文件				   				   	
*******************************************************************************/
#ifndef	_USART2CONFIG_H	 
#define _USART2CONFIG_H

#include "stm32f10x.h"
#include "USART2.h"

void USART2_Configuration(unsigned long baud);

#define USART2SendByte(temp)	USART_SendData(USART2, temp) //USART1发送的数据
#define USART2RecvByte()		USART_ReceiveData(USART2)    //USART1接收的数据
#define USART2StopSendISR()		USART_ITConfig(USART2,USART_IT_TC,DISABLE) //停止发送中断
#define USART2StartSendISR()	USART_ITConfig(USART2,USART_IT_TC,ENABLE); //开启发送中断
#define USART2StopRecvISR()		USART_ITConfig(USART2,USART_IT_RXNE,DISABLE); //停止接收中断
#define USART2StartRecvISR() 	USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);//开启接收中断


#endif

