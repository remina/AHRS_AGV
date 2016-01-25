/******************** 版权所有 武汉汉迪机器人科技有限公司 杜骁释 **************
* 文件名             : CAN1Config.h
* 作者               : 杜骁释
* 版本               : V1.0
* 日期               : 9/13/2012
* 描述               : CAN底层驱动头文件
* 功能               ：				   				   	
*******************************************************************************/

#ifndef	_CAN1CONFIG_H	 
#define _CAN1CONFIG_H
#include "stm32f10x.h"
#include "CAN.h"
#include "global.h"
#include "LED.h"
#define CAN1StopSendISR()	CAN_ITConfig(CAN1, CAN_IT_RQCP0, DISABLE);
#define CAN1StartSendISR()	CAN_ITConfig(CAN1, CAN_IT_RQCP0, ENABLE);
#define CAN1StopRecvISR()   CAN_ITConfig(CAN1, CAN_IT_FMP0, DISABLE);
#define CAN1StartRecvISR()  CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);

void CAN1_Configuration(void);
//void CAN1SendByte(CAN1SendTcb tcb);
void CAN1_Init(void);
//void CAN1SendByte(CAN1SendTcb tcb, u8 num, u8 index);

extern u8 canrecv[24];
extern int g_RcvWheelSpeed[4];

typedef struct {
	u32 id;
	u8 data[8];
	u8 len;
	u8 ch;
	u8 format;     /*0 - 标准帧， 1 - 扩展帧*/
	u8 type;       /*0 - 数据帧， 1 - 远程帧*/
}CAN_msg;

#endif

