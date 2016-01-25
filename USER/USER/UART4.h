/******************** 版权所有 武汉汉迪机器人科技有限公司 杜骁释 **************
* 文件名             : UART4.h
* 作者               : 杜骁释
* 版本               : V1.0
* 日期               : 9/13/2012
* 描述               : 串口4中间层头文件
* 功能               ：底层串口4驱动的封装，为蓝牙通讯提供接口				   				   	
*******************************************************************************/
#ifndef  _UART4_H
#define  _UART4_H

#include "stm32f10x.h"
#include "OSQMem.h"
#include "UART4Config.h"


#define OS_MEM_UART4_MAX 		1024			//发送缓冲区的内存大小
#define OS_MEM_UART4_BLK 		32				//每一个块的长度
#define UART4_SEND_MAX_Q	  	(OS_MEM_UART4_BLK-4)	//发送内存块内的最大空间
#define UART4_SEND_MAX_BOX		20	   					//发送内存块的最大数量

#define UART4_RECV_MAX_Q	  	256					//接收缓冲区最大空间

void UART4_Init(unsigned long baud);
void UART4SendUpdate(void);
void UART4RecvUpdate(void);
u8 UART4WriteDataToBuffer(u8 *buffer,u8 count);
u8 UART4DispFun(u8 *buffer);
extern void FRIDCheck(void);
extern void FRIDCpyData(void);

extern u8 UART4RecvBuffer[UART4_RECV_MAX_Q];

extern u8 g_rfid_num;
extern u8 g_rfid_num_last;
extern u8 g_rfid_data_flag;

//注意在串口接收的时候是打开了定时器，
//其中：USART1-------TIM2
//其中：USART2-------TIM3
//其中：USART3-------TIM4
//其中：USART4-------TIM5
//其中：USART5-------TIM6

#endif

