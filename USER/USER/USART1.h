/******************** 版权所有 武汉汉迪机器人科技有限公司 杜骁释 **************
* 文件名             : USART1.h
* 作者               : 杜骁释
* 版本               : V1.0
* 日期               : 9/13/2012
* 描述               : 串口1中间层头文件
* 功能               ：底层串口1驱动的封装				   				   	
*******************************************************************************/
#ifndef  _USART1_H
#define  _USART1_H

#include "stm32f10x.h"
#include "OSQMem.h"
#include "USART1Config.h"

#define		DMA_MODE 		0 		//定义是采用DMA模式，还是普通的中断模式

#define OS_MEM_USART1_MAX 		1024			//发送缓冲区的内存大小
#define OS_MEM_USART1_BLK 		32				//每一个块的长度
//#define OS_MEM_USART1_BLK 		64				//每一个块的长度
#define USART1_SEND_MAX_Q	  	(OS_MEM_USART1_BLK-4)	//发送内存块内的最大空间
#define USART1_SEND_MAX_BOX		20	   					//发送内存块的最大数量

#define USART1_RECV_MAX_Q	  	256					//接收缓冲区最大空间


void USART1_Init(unsigned long baud);
void USART1SendUpdate(void);
u8 USART1WriteDataToBuffer(u8 *buffer,u8 count);
u8 USART1DispFun(u8 *buffer);
void USART1DMAUpdate(void);
void USART1RecvUpdate(void);
void USART1GetRadarData(void);

extern u8 USART1RecvBuffer[USART1_RECV_MAX_Q];

//注意在串口接收的时候是打开了定时器，
//其中：USART1-------TIM2
//其中：USART2-------TIM3
//其中：USART3-------TIM4
//其中：USART4-------TIM5
//其中：USART5-------TIM6

#endif



