/******************** 版权所有 武汉汉迪机器人科技有限公司 杜骁释 **************
* 文件名             : UART4Config.h
* 作者               : 杜骁释
* 版本               : V1.0
* 日期               : 9/13/2012
* 描述               : UART4驱动层头文件
* 功能               ：UART4驱动层头文件,与外部蓝牙模块通讯				   				   	
*******************************************************************************/

#ifndef	_UART4CONFIG_H	 
#define _UART4CONFIG_H

#include "stm32f10x.h"
#include "UART4.h"

void UART4_Configuration(unsigned long baud);

#define UART4SendByte(temp)	USART_SendData(UART4, temp) //UART4发送的数据
#define UART4RecvByte()		USART_ReceiveData(UART4)    //UART4接收的数据
#define UART4StopSendISR()		USART_ITConfig(UART4,USART_IT_TC,DISABLE) //停止发送中断
#define UART4StartSendISR()	USART_ITConfig(UART4,USART_IT_TC,ENABLE); //开启发送中断
#define UART4StopRecvISR()		USART_ITConfig(UART4,USART_IT_RXNE,DISABLE); //停止接收中断
#define UART4StartRecvISR() 	USART_ITConfig(UART4,USART_IT_RXNE,ENABLE);//开启接收中断

#endif
