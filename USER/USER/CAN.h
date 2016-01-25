/******************** ��Ȩ���� �人���ϻ����˿Ƽ����޹�˾ ������ **************
* �ļ���             : CAN.h
* ����               : ������
* �汾               : V1.0
* ����               : 9/13/2012
* ����               : CAN�м��ͷ�ļ�
* ����               ����CAN�ײ����������ķ�װ��Ϊ�ϲ�Ӧ���ṩ�ӿ�				   				   	
*******************************************************************************/
#ifndef  _CAN_H
#define  _CAN_H
#include "CAN1Config.h"
#include "OSQMem.h"


typedef struct CAN1SENDTCB{
unsigned char Num;
unsigned long ID;
unsigned char frame_type;
unsigned char *Index;
}CAN1SendTcb;



#define CAN1_SEND_MAX_Q          (OS_MEM_CAN1_BLK-4)    //�����ڴ���ڵ����ռ�
#define CAN1_SEND_MAX_BOX         OS_MEM_CAN1_MAX/OS_MEM_CAN1_BLK
#define CAN1_RECV_MAX_Q          256                        //�ڴ���ڵ����ռ�

#define     OS_MEM_CAN1_MAX         1024             //���ͻ��������ڴ��С
#define     OS_MEM_CAN1_BLK         32               //ÿһ����ĳ���   

#define CAN_GetReadLen()	(CAN1RecvPtrW-CAN1RecvPtrR)

u8 CAN1WriteDataToBuffer(u8 *buffer, u8 count, u32 ID, u8 type);
void CAN1SendUpdate(void);
void CAN1RecvUpdate(CanRxMsg RxMessage);

extern u8 CAN1QRecvBuffer[CAN1_RECV_MAX_Q];
extern u8 CAN1RecvPtrW, CAN1RecvPtrR;

#endif


