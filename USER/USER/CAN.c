/******************** 版权所有 武汉汉迪机器人科技有限公司 杜骁释 **************
* 文件名             : CAN.c
* 作者               : 杜骁释
* 版本               : V1.0
* 日期               : 9/13/2012
* 描述               : CAN中间层头文件
* 功能               ：底层CAN驱动的封装
*******************************************************************************/
#include "CAN.h"
                                                    
unsigned char CAN1SendQBoxHost=0;                         //内存块头指针                            
unsigned char CAN1SendQBoxTail=0;                         //内存块尾指针
unsigned int  CAN1SendQFree=CAN1_SEND_MAX_BOX;   
unsigned char CAN1SendOVF=0;                              //CAN1发送任务块溢出标志
unsigned char CAN1RunningFlag=0;

CAN1SendTcb CAN1SendTCB[CAN1_SEND_MAX_BOX];

u8 CAN1QRecvBuffer[CAN1_RECV_MAX_Q];          //接收内存块    
u8 CAN1RecvOVF=0;                               //CAN2接收任务块溢出标志  
u32  CAN1Recv1Index=0x00;
u32  CAN1Recv1Count=0x00;
u8 CAN1RecvFlag=0;
u8 CAN1RecvPtrW, CAN1RecvPtrR;

u8 CAN1SendQBoxPre = 0x00;

u8 OSCAN1MemQ[OS_MEM_CAN1_MAX];        //空白缓冲区
OSMEMTcb* OSQCAN1Index;                             //内存块管理区指针

//错误定义
#define ERR_NO_SPACE    0xff

void CAN1_Init(void)
{
	u8 MemTestErr;

	CAN1RecvPtrW = 0;
	CAN1RecvPtrR = 0;

	CAN1_Configuration();
	OSQCAN1Index=(OSMEMTcb *)OSMemCreate(OSCAN1MemQ,OS_MEM_CAN1_BLK,OS_MEM_CAN1_MAX/OS_MEM_CAN1_BLK,&MemTestErr);
}

/*******************************************************************************
* 文件名         : CAN1SendUpdate
* 描述           : 检查结构体里面有没有数据还未发送完毕，若没有发送，则继续发送，
                   若发送完毕，退出
* 输入           : 无
* 输出           : 无
* 返回           : 无
*******************************************************************************/
void CAN1SendUpdate(void)
{
	
    static unsigned int count=0; //CAN发送数据指针

	if(CAN1SendQFree==CAN1_SEND_MAX_BOX)
	{
		CAN1RunningFlag=0;
		OSMemDelete(OSQCAN1Index,CAN1SendTCB[CAN1SendQBoxTail].Index);
		return;
	}

	CAN1StopSendISR();
    //如果现在的内存块的数据还没有发送完毕，启动发送，Num减8
	if ((CAN1SendTCB[CAN1SendQBoxTail].Num) && (CAN1SendQBoxTail != CAN1SendQBoxHost))
	{
		if (CAN1SendTCB[CAN1SendQBoxTail].Num >= 8)
		{
			CAN1SendTCB[CAN1SendQBoxTail].Num -= 8;
			CAN1SendByte(CAN1SendTCB[CAN1SendQBoxTail], 8, count);
			count += 8;
		}
		else
		{			
			CAN1SendByte(CAN1SendTCB[CAN1SendQBoxTail], CAN1SendTCB[CAN1SendQBoxTail].Num, count);
			CAN1SendTCB[CAN1SendQBoxTail].Num = 0; //数据发送完毕清零
		}
 	}
	else if (CAN1SendQBoxTail != CAN1SendQBoxHost)
	{//一个发送块已经发送完毕了， CAN1SendQFree++, 尾指针加一，指向下一个发送块
		OSMemDelete(OSQCAN1Index,CAN1SendTCB[CAN1SendQBoxTail].Index);
		if (++CAN1SendQBoxTail >= CAN1_SEND_MAX_BOX) CAN1SendQBoxTail = 0;
		if (++CAN1SendQFree >= CAN1_SEND_MAX_BOX) CAN1SendQFree = CAN1_SEND_MAX_BOX;
		count = 0;
		//CAN1SendQBoxTail等于CAN1SendQBoxHost的时候就标志这块发送结束了，可以直接退出
		if (CAN1SendQBoxTail != CAN1SendQBoxHost)
		{
			if (CAN1SendTCB[CAN1SendQBoxTail].Num >= 8)
			{
				CAN1SendTCB[CAN1SendQBoxTail].Num -= 8;
				CAN1SendByte(CAN1SendTCB[CAN1SendQBoxTail], 8, count);
				count += 8;
			}
			else
			{
				CAN1SendByte(CAN1SendTCB[CAN1SendQBoxTail], CAN1SendTCB[CAN1SendQBoxTail].Num, count);
			}			
		}
		else
		{
			CAN1RunningFlag = 0;
			CAN1SendQFree = CAN1_SEND_MAX_BOX;
			count = 0;
		}
	}
	else
	{
		CAN1RunningFlag = 0;
		CAN1SendQFree = CAN1_SEND_MAX_BOX;
		count = 0;
	}	
	CAN1StartSendISR();
}

/*******************************************************************************
* 文件名         : CAN1WriteDataToBuffer
* 描述           : 检查发送缓冲区的大小，若空间足够，将待发送的数据放入到发送缓冲
                   区中去,并且启动发送
* 输入           : buffer待发送的数据的指针，count待发送的数据的数量
* 输出           : 无
* 返回           : 若正确放入到发送缓冲区中去了，就返回0x00     ，否则返回0x01
*******************************************************************************/
u8 CAN1WriteDataToBuffer(u8 *buffer, u8 count, u32 ID, u8 type)
{
    u8 i = count;
    u8 err;
    /*此处可以加入信号灯或者关闭中断*/
	if (count == 0) return 0x01;

	CAN1StopSendISR();
	//计算放入count个数据需要多少个内存块
	if (count % CAN1_SEND_MAX_Q) 
		count = count / CAN1_SEND_MAX_Q + 1;
	else
		count = count / CAN1_SEND_MAX_Q;
	//需要count个数据块,如果内存不足，直接返回
	if (CAN1SendQFree < count)
	{
		CAN1StartSendISR();
		return 0x01;
	}
	//首先申请内存块, CAN1SendQBoxHost在下一个内存申请后才加一

	CAN1SendTCB[CAN1SendQBoxHost].Index = (u8 *)OSMemGet(OSQCAN1Index, &err);
	if (CAN1SendQBoxHost >= CAN1_SEND_MAX_BOX) CAN1SendQBoxHost = 0;
	count = 0;
	while (i != '\0')
	{
		*(CAN1SendTCB[CAN1SendQBoxHost].Index+count) = *buffer;
		count++;
		if (count >= CAN1_SEND_MAX_Q)
		{
			CAN1SendTCB[CAN1SendQBoxHost].ID=ID; 
			CAN1SendTCB[CAN1SendQBoxHost].Num = CAN1_SEND_MAX_Q;
			CAN1SendTCB[CAN1SendQBoxHost].frame_type = type;
			//需要一个新的内存块存放接下来的数据，所以更新CAN1SendQBoxHost
			if (++CAN1SendQBoxHost >= CAN1_SEND_MAX_BOX) CAN1SendQBoxHost = 0;
			CAN1SendTCB[CAN1SendQBoxHost].Index = (u8 *)OSMemGet(OSQCAN1Index, &err);
			CAN1SendQFree--;
			count = 0;
		}
		buffer++;
		i--;
	}

	//此处是尚未整块存玩的数据，它们也要存放在一个新的内存块里
	if (count != 0)
	{
		CAN1SendTCB[CAN1SendQBoxHost].ID=ID;
		CAN1SendTCB[CAN1SendQBoxHost].Num = count;
		CAN1SendTCB[CAN1SendQBoxHost].frame_type = type;
		CAN1SendQFree--;
		if (++CAN1SendQBoxHost >= CAN1_SEND_MAX_BOX) CAN1SendQBoxHost = 0;
	}
    
    //如果是第一次，就启动发送，如果是已经启动就没有这个必要了
    if(CAN1RunningFlag==0)
    {
	    CAN1RunningFlag=1; // 2012年9月17日：将这个赋值需要放在CAN1SendUpdate之前。
						   // 				 不然放在CAN1SendUpdate后面会一直到
						   // 				 CAN1SendUpdate执行完了才会执行	CAN1SendUpdate = 1

        CAN1SendUpdate();
//		CAN1RunningFlag=1;       
    }
    /*此处可以开启信号灯或者打开中断*/
    CAN1StartSendISR();
	//LED1(ON);
    return 0x00;
}


/*******************************************************************************
* 文件名         : CAN1DispFun
* 描述           : 检查发送缓冲区的大小，若空间足够，将待发送的数据放入到发送缓冲
                   区中去,并且启动发送,与CAN1WriteDataToBuffer不同的是，启动发送
                   函数世不需要指定文件大小的，这就给调用提供了方便.
* 输入           : buffer待发送的数据的指针
* 输出           : 无
* 返回           : 若正确放入到发送缓冲区中去了，就返回0x00     ，否则返回0x01
*******************************************************************************/


/*******************************************************************************
* 文件名           : CAN1RecvResetBufferIndex
* 描述             : 当发生超时中断的时候，将接收的指针归零，并且关闭检查超时的时钟
* 输入             : 无
* 输出             : 无
* 返回             : 无
*******************************************************************************/
void CAN1RecvResetBufferIndex(void)
{    
    int i=0; 
//    CAN1WriteDataToBuffer(CAN1QRecvBuffer,CAN1Recv1Index);
//    CAN1StopCounter();
    CAN1Recv1Index=0;    
    for(i=0;i<CAN1_RECV_MAX_Q;i++)CAN1QRecvBuffer[i]=0;   
}

//unsigned char test1;    
/*******************************************************************************
* 文件名         : CAN1RecvFun
* 描述           : 当接收到完整的一帧数据以后的处理函数
* 输入           : ptr接收到的数据帧的头指针，接收到的数据帧的数据个数
* 输出           : 无
* 返回           : 无
*******************************************************************************/
void CAN1RecvFun(unsigned char *ptr,unsigned int count)
{
    int i=0;  
    for(i=0;i<CAN1_RECV_MAX_Q;i++)CAN1QRecvBuffer[i]=0;
    
}    

/*******************************************************************************
* 文件名         : CAN1RecvUpdate
* 描述           : 处理接收到一个数据
* 输入           : 无
* 输出           : 无
* 返回           : 无
*******************************************************************************/    
void CAN1RecvUpdate(CanRxMsg RxMessage)
{
	u8 i = 0;
	u8 num = RxMessage.DLC;
	for (i = 0; i < num; i++)
	{
		CAN1QRecvBuffer[CAN1RecvPtrW++] =  RxMessage.Data[i];
		if (CAN1RecvPtrW == CAN1RecvPtrR) //读缓冲已满
		{
			CAN1RecvPtrR++;//抛弃掉最老的数据	
		}		
	}
}
/*******************************************************************************
* 文件名         : CAN1RecvData
* 描述           : 当接收到完整的一帧数据以后的处理函数
* 输入           : count：要接收到的一帧数据数据的个数，flag：1开启超时中断
                   0关闭超时中断
* 输出           : 无
* 返回           : 无
*******************************************************************************/    
unsigned char CAN1RecvData(unsigned int count,unsigned char flag)
{
    if(count!=0)
    {
        CAN1Recv1Count=count;
        CAN1RecvFlag=flag;
        if(flag==1)
        {
//           TIM3_Configuration();
//           CAN1StartRecvISR();
        }
        #if     DMA_MODE
        if(flag==2)CAN1DMAConfig_RX(CAN1QRecvBuffer,count);    
        #endif
    }
    else if(count>CAN1_RECV_MAX_Q) return ERR_NO_SPACE;
    return 0x00;
}  


