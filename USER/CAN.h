/******************** 版权所有 武汉汉迪机器人科技有限公司 杜骁释 **************
* 文件名             : CAN.h
* 作者               : 杜骁释
* 版本               : V1.0
* 日期               : 9/13/2012
* 描述               : CAN中间层头文件
* 功能               ：对CAN底层驱动函数的封装，为上层应用提供接口				   				   	
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



#define CAN1_SEND_MAX_Q          (OS_MEM_CAN1_BLK-4)    //发送内存块内的最大空间
#define CAN1_SEND_MAX_BOX         OS_MEM_CAN1_MAX/OS_MEM_CAN1_BLK
#define CAN1_RECV_MAX_Q          256                        //内存块内的最大空间

#define     OS_MEM_CAN1_MAX         1024             //发送缓冲区的内存大小
#define     OS_MEM_CAN1_BLK         32               //每一个块的长度   

#define CAN_GetReadLen()	(CAN1RecvPtrW-CAN1RecvPtrR)

u8 CAN1WriteDataToBuffer(u8 *buffer, u8 count, u32 ID, u8 type);
void CAN1SendUpdate(void);
void CAN1RecvUpdate(CanRxMsg RxMessage);

extern u8 CAN1QRecvBuffer[CAN1_RECV_MAX_Q];
extern u8 CAN1RecvPtrW, CAN1RecvPtrR;

#endif


