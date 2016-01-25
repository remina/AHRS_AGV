/******************** ��Ȩ���� �人���ϻ����˿Ƽ����޹�˾ ������ **************
* �ļ���             : CAN1Config.h
* ����               : ������
* �汾               : V1.0
* ����               : 9/13/2012
* ����               : CAN�ײ�����ͷ�ļ�
* ����               ��				   				   	
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
	u8 format;     /*0 - ��׼֡�� 1 - ��չ֡*/
	u8 type;       /*0 - ����֡�� 1 - Զ��֡*/
}CAN_msg;

#endif

