/******************** 版权所有 武汉汉迪机器人科技有限公司 杜骁释 **************
* 文件名             : USART1.c
* 作者               : 杜骁释
* 版本               : V1.0
* 日期               : 9/13/2012
* 描述               : 串口1中间层
* 功能               ：底层串口1驱动的封装				   				   	
*******************************************************************************/
#include "USART1.h"
#include "string.h"
#include "global.h"

u8 USART1SendQBoxHost;						//发送内存块头指针							
u8 USART1SendQBoxTail;						//发送内存块尾指针
u8 USART1SendQFree;					    	//发送内存块空闲区
u8 USART1RunningFlag=0;
u8 USART1RecvPtrW, USART1RecvPtrR;
u8 UART1RecvBufStart, UART1RecvBufEnd;

typedef struct{
u8 Num;
u8 *Index;
}USART1SendTcb;                          //task control block
USART1SendTcb USART1SendTCB[USART1_SEND_MAX_BOX];


u8 USART1RecvBuffer[USART1_RECV_MAX_Q]; //接收缓冲区	
u8 USART1tempBuffer[28]={0};
u8 USART1RecvOVF=0; 				        //USART1接收任务块溢出标志  
u32 Recv1Index=0x00;
u32 Recv1Count=0x08;
u8 UART1RecvFlag=0;


u8 OSUSART1MemQ[OS_MEM_USART1_MAX];  			//空白内存块

OSMEMTcb* OSQUSART1Index;

void USART1DMAUpdate(void);
//错误定义
#define ERR_NO_SPACE	0xff

void USART1_Init(unsigned long baud)
{	
	u8 MemTestErr;
	USART1RecvPtrW = 0;
	USART1RecvPtrR = 0;
	USART1SendQBoxHost = 0;
	USART1SendQBoxTail = 0;
	USART1SendQFree = USART1_SEND_MAX_BOX;

	USART1_Configuration(baud);
	OSQUSART1Index=(OSMEMTcb *)OSMemCreate(OSUSART1MemQ,OS_MEM_USART1_BLK,OS_MEM_USART1_MAX/OS_MEM_USART1_BLK, &MemTestErr);
}

/*******************************************************************************
* 文件名	  	 : USART1SendUpdate
* 描述	         : 检查结构体里面有没有数据还未发送完毕，若没有发送，则继续发送，
				   若发送完毕，退出
* 输入           : 无
* 输出           : 无
* 返回           : 无
*******************************************************************************/
void USART1SendUpdate(void)
{
	static unsigned char count=0;
	
	if(USART1SendQFree==USART1_SEND_MAX_BOX){return;}
	USART1StopSendISR();
	//如果现在的内存块的数据还没有发送完毕，启动发送，Num减一
	if((USART1SendTCB[USART1SendQBoxTail].Num)&&(USART1SendQBoxTail!=USART1SendQBoxHost))
	{
		USART1SendTCB[USART1SendQBoxTail].Num--;
		USART1SendByte(*(USART1SendTCB[USART1SendQBoxTail].Index+count));
		count++;
	}
	//一个发送块已经发送完毕了 ，USART1SendQFree++,尾指针加一。指向下一个发送块
	else if(USART1SendQBoxTail!=USART1SendQBoxHost)
	{		
		OSMemDelete(OSQUSART1Index,USART1SendTCB[USART1SendQBoxTail].Index);
		if(++USART1SendQBoxTail>=USART1_SEND_MAX_BOX)USART1SendQBoxTail=0;
		if(++USART1SendQFree>=USART1_SEND_MAX_BOX)USART1SendQFree=USART1_SEND_MAX_BOX;
		count=0;
		//USART1SendQBoxTail等于USART1SendQBoxTail的时候就标志这发送结束了，可以直接退出
		if((USART1SendQBoxTail!=USART1SendQBoxHost))
//		if((USART1SendTCB[USART1SendQBoxTail].Num)&&(USART1SendQBoxTail!=USART1SendQBoxHost))
		{
			USART1SendTCB[USART1SendQBoxTail].Num--;
			USART1SendByte(*(USART1SendTCB[USART1SendQBoxTail].Index+count));
			count++;
		}
		else
		{	
		//USART1SendQBoxTail等于USART1SendQBoxTail的时候就标志这发送结束了，可以直接退出
			USART1RunningFlag=0;
			USART1SendQFree=USART1_SEND_MAX_BOX;
			count=0;
		}	
	}
	//由于头指针一直是指向空的发送块的，所以USART1SendQBoxTail等于USART1SendQBoxTail
	//的时候就标志这发送结束了，可以直接退出
	else
	{
		USART1RunningFlag=0;
		USART1SendQFree=USART1_SEND_MAX_BOX;
		count=0;
	}
	USART1StartSendISR();	
}

/*******************************************************************************
* 文件名	  	 : USART1RecvUpdate
* 描述	         : 
* 输入           : 无
* 输出           : 无
* 返回           : 无
*******************************************************************************/
void USART1RecvUpdate(void)
{
	USART1RecvBuffer[USART1RecvPtrW++] = USART1RecvByte();
	if (USART1RecvPtrW == USART1RecvPtrR) //读缓冲已满
	{
		USART1RecvPtrR++;//抛弃掉最老的数据	
	}	
}



/*******************************************************************************
* 文件名	  	 : USART1WriteDataToBuffer
* 描述	         : 检查发送缓冲区的大小，若空间足够，将待发送的数据放入到发送缓冲
				   区中去,并且启动发送
* 输入           : buffer待发送的数据的指针，count待发送的数据的数量
* 输出           : 无
* 返回           : 若正确放入到发送缓冲区中去了，就返回0x00	 ，否则返回0x01
*******************************************************************************/
u8 USART1WriteDataToBuffer(u8 *buffer,u8 count)
{
	u8 i=count;
	u8 err;
	/*此处可以加入信号灯或者关闭中断*/
	if(count==0)return 0x01;
	USART1StopSendISR();
	/*计算放入count个数据需要多少个内存块*/
	if(count%USART1_SEND_MAX_Q)count=count/USART1_SEND_MAX_Q+1;
	else count=count/USART1_SEND_MAX_Q;
	/*需要count个数据块*/
	/*如果内存不足，直接返回*/		 
	if(USART1SendQFree<count){USART1StartSendISR();return ERR_NO_SPACE;}
	//首先申请内存块，USART1SendQBoxHost在下一个内存申请后才加一
	USART1SendTCB[USART1SendQBoxHost].Index=(u8 *)OSMemGet(OSQUSART1Index,&err);
	if(USART1SendQBoxHost>=USART1_SEND_MAX_BOX)USART1SendQBoxHost=0;	
	count=0;
	while(i!='\0')
	//while(i>0)										 
	{
		*(USART1SendTCB[USART1SendQBoxHost].Index+count)=*buffer;
		count++;
		if(count>=USART1_SEND_MAX_Q)
		{
			USART1SendTCB[USART1SendQBoxHost].Num=USART1_SEND_MAX_Q;
			//需要一个新的内存块存放接下来的数据，所以更新USART1SendQBoxHost
			if(++USART1SendQBoxHost>=USART1_SEND_MAX_BOX)USART1SendQBoxHost=0;
			//需要一个新的内存块存放接下来的数据	
			USART1SendTCB[USART1SendQBoxHost].Index=(u8 *)OSMemGet(OSQUSART1Index,&err);
			//空的发送任务块减一 			
			USART1SendQFree--;
			count=0;
		}
		buffer++;
		i--;
	}
	//此处是尚未整块存完的数据，它们也要存放在一个新的内存块里
	if(count!=0)
	{
		USART1SendTCB[USART1SendQBoxHost].Num=count; 
		USART1SendQFree--;
		if(++USART1SendQBoxHost>=USART1_SEND_MAX_BOX)USART1SendQBoxHost=0;
	}
	//如果是第一次，就启动发送，如果是已经启动就没有这个必要了
	if(USART1RunningFlag==0)
	{
#if	  	DMA_MODE
		USART1DMAConfig(USART1SendTCB[USART1SendQBoxTail].Index,USART1SendTCB[USART1SendQBoxTail].Num);
#else	
		USART1SendUpdate();
#endif		
		USART1RunningFlag=1;
	}
	/*此处可以开启信号灯或者打开中断*/
	USART1StartSendISR();
	return 0x00;
}
/*******************************************************************************
* 文件名	  	 : USART1DispFun
* 描述	         : 检查发送缓冲区的大小，若空间足够，将待发送的数据放入到发送缓冲
				   区中去,并且启动发送,与USART1WriteDataToBuffer不同的是，启动发送
				   函数世不需要指定文件大小的，这就给调用提供了方便.
* 输入           : buffer待发送的数据的指针
* 输出           : 无
* 返回           : 若正确放入到发送缓冲区中去了，就返回0x00	 ，否则返回0x01
*******************************************************************************/
u8 USART1DispFun(u8 *buffer)
{
	u32 count=0;
	while(buffer[count]!='\0') count++;//count how many data shoud be put into buffer ,then transmitting
	//count=36;
	return(USART1WriteDataToBuffer(buffer,count));
}

/*******************************************************************************
* 文件名	  	 : USART1DMAUpdate.c
* 描述	         : USART_DMA的驱动函数
* 移植步骤		 : 中间层函数
* 输入           : 无
* 输出           : 无
* 返回           : 无
*******************************************************************************/
void USART1DMAUpdate(void)
{
	if(USART1SendQBoxTail!=USART1SendQBoxHost)
	{
		OSMemDelete(OSQUSART1Index,USART1SendTCB[USART1SendQBoxTail].Index);
		if(++USART1SendQBoxTail>=USART1_SEND_MAX_BOX)USART1SendQBoxTail=0;
		if(++USART1SendQFree>=USART1_SEND_MAX_BOX)USART1SendQFree=USART1_SEND_MAX_BOX;
		if(USART1SendQBoxTail!=USART1SendQBoxHost)
		{
			USART1DMAConfig((u32)USART1SendTCB[USART1SendQBoxTail].Index,USART1SendTCB[USART1SendQBoxTail].Num);	
		}
		else USART1RunningFlag=0;	
	}
	else 
	{		
		OSMemDelete(OSQUSART1Index,USART1SendTCB[USART1SendQBoxTail].Index);
		if(++USART1SendQBoxTail>=USART1_SEND_MAX_BOX)USART1SendQBoxTail=0;
		if(++USART1SendQFree>=USART1_SEND_MAX_BOX)USART1SendQFree=USART1_SEND_MAX_BOX;
		USART1RunningFlag=0;
	}	
}

/*******************************************************************************
* 文件名	  	 : USART1GetRadarData.c
* 描述	         : 接收获取倒车雷达的数据  存入原始数组中
* 移植步骤		 : 中间层函数
* 输入           : 无
* 输出           : 无
* 返回           : 无
*******************************************************************************/
void USART1GetRadarData(void)
{
	unsigned char i, k;
    while (USART1RecvPtrR != USART1RecvPtrW) 
    {
	    if (UART1RecvFlag == 0) 
		{
	      k = 0;
	      i = (USART1RecvPtrR - 2);
	      if (USART1RecvBuffer[i] == 0x55) 
		  {
	        k++;
	      }
	      
	      i = (USART1RecvPtrR - 1);
	      if (USART1RecvBuffer[i] == 0xAA) 
		  {
	        k++;
	      }

		  i = (USART1RecvPtrR + 2*Recv1Count);
	      if (USART1RecvBuffer[i] == 0xff) 
		  {
	        k++;
	      }
	
		  if (k == 3) 
		  {
	        //获取有效数据的起始地址和末尾地址（末尾地址指向整个数据帧的帧尾，校验和）
	        UART1RecvFlag = 1;
	        UART1RecvBufStart = USART1RecvPtrR;
	        UART1RecvBufEnd = (USART1RecvPtrR + 2*Recv1Count);
	      } 
		}  
		else 
		{
      		//开始接收数据处理
      		/*if(USART1RecvPtrR == UART1RecvBufEnd) 
	  		{
				UART1RecvFlag = 0;
		        for(i = 0; i < 2*Recv1Count; i=i+2) 
				{
		          Pre_Distance[i/2][0]= (USART1RecvBuffer[UART1RecvBufStart+i]<<8)|USART1RecvBuffer[UART1RecvBufStart+i+1];
		        } 
			}*/
			/*for(i = 0; i < 4; i++) 
			{
	          Pre_Distance[i][0]= USART1RecvBuffer[UART1RecvBufStart+i];
	        }
			for(i = 4; i < 8; i++) 
			{
	          Pre_Distance[4+i][0]= USART1RecvBuffer[UART1RecvBufStart+i];
	        } */
		}
		USART1RecvPtrR = (USART1RecvPtrR + 1);
	}

}

