/******************** 版权所有 武汉汉迪机器人科技有限公司 杜骁释 **************
* 文件名             : USART2.c
* 作者               : 杜骁释
* 版本               : V1.0
* 日期               : 9/13/2012
* 描述               : USART2驱动层
* 移植步骤		 	 :（1）配置函数（管脚，时钟等）
				      （2）控制的参数（极性，波特率，位数，校验位等）
				      （3）中断函数
* 功能               ：配置串口管脚时钟，控制参数以及中断				   				   	
*******************************************************************************/
#include "USART2.h"
#include "global.h"
#include "odometry.h"
#include "AHRS.h"
#include "sensors.h"
#include "magnav.h"

u8 USART2SendQBoxHost;						//发送内存块头指针							
u8 USART2SendQBoxTail;						//发送内存块尾指针
u32  USART2SendQFree;						//发送内存块空闲区
u8 USART2RunningFlag=0;
typedef struct{
u8 Num;
u8 *Index;
}USART2SendTcb;
USART2SendTcb USART2SendTCB[USART2_SEND_MAX_BOX];


u8 USART2RecvBuffer[USART2_RECV_MAX_Q]; //接收缓冲区	
u8 USART2RecvOVF=0; 				        //USART2接收任务块溢出标志  
u32 Recv2Index=0x00;
u32 Recv2Count=0x00;
u8 USART2RecvFlag=0;
u8 USART2RecvPtrW, USART2RecvPtrR;
u8 USART2RecvBufStart, USART2RecvBufEnd;
u8 USART2RecvState;
u8 USART2RecvFrameLen;
u8 OSUSART2MemQ[OS_MEM_USART2_MAX];  			//空白内存块

OSMEMTcb* OSQUSART2Index;

//宏定义
#define NO_START_RCV         0
#define START_RCV            1
#define FIRST_FRAMEHEADER	0x55
#define SENCOND_FRAMEHEADER	0xAA
#define	BUTCONMODE			0xA1
#define	DISCONMODE			0xA2
#define	COORDCONMODE		0xA3
#define	DEMOMODE			0xA4
#define	MY_ADDR				0x1E
#define	MYYAW				0x1A
#define	TEST				0x1B
#define STOP_TEST			0x1C
#define MOVE_ALL			0x1D
#define STOP_MOVE_ALL		0x1E
#define MOVE_TARGET			0x20
#define STOP_MOVE_TARGET	0x21
#define PID_ADJUST			0x30
#define AHRSPID				0x31

int ucTranslateRate[3], ucAngle[3], ucRotateRate[3];
u8  rateIndex = 0;

float g_bt_manual_botrate = 0.0;
u8 g_bt_manual_flag = 0;

extern float value_to_speed(u8 value);
extern float hold_yaw;

//错误定义
#define ERR_NO_SPACE	0xff

void USART2_Init(unsigned long baud)
{	
	u8 MemTestErr;
	USART2RecvPtrW = 0;
	USART2RecvPtrR = 0;
	USART2SendQBoxHost = 0;
	USART2SendQBoxTail = 0;
	USART2SendQFree = USART2_SEND_MAX_BOX;

	USART2_Configuration(baud);
	OSQUSART2Index=(OSMEMTcb *)OSMemCreate(OSUSART2MemQ,OS_MEM_USART2_BLK,OS_MEM_USART2_MAX/OS_MEM_USART2_BLK, &MemTestErr);
}

/*******************************************************************************
* 文件名	  	 : USART2SendUpdate
* 描述	         : 检查结构体里面有没有数据还未发送完毕，若没有发送，则继续发送，
				   若发送完毕，退出
* 输入           : 无
* 输出           : 无
* 返回           : 无
*******************************************************************************/
void USART2SendUpdate(void)
{
	static unsigned char count=0;
	
	if(USART2SendQFree==USART2_SEND_MAX_BOX){return;}
	USART2StopSendISR();
	//如果现在的内存块的数据还没有发送完毕，启动发送，Num减一
	if((USART2SendTCB[USART2SendQBoxTail].Num)&&(USART2SendQBoxTail!=USART2SendQBoxHost))
	{
		USART2SendTCB[USART2SendQBoxTail].Num--;
		USART2SendByte(*(USART2SendTCB[USART2SendQBoxTail].Index+count));
		count++;
	}
	//一个发送块已经发送完毕了 ，USART2SendQFree++,尾指针加一。指向下一个发送块
	else if(USART2SendQBoxTail!=USART2SendQBoxHost)
	{		
		OSMemDelete(OSQUSART2Index,USART2SendTCB[USART2SendQBoxTail].Index);
		if(++USART2SendQBoxTail>=USART2_SEND_MAX_BOX)USART2SendQBoxTail=0;
		if(++USART2SendQFree>=USART2_SEND_MAX_BOX)USART2SendQFree=USART2_SEND_MAX_BOX;
		count=0;
		//USART2SendQBoxTail等于USART2SendQBoxTail的时候就标志这发送结束了，可以直接退出
		if((USART2SendQBoxTail!=USART2SendQBoxHost))
//		if((USART2SendTCB[USART2SendQBoxTail].Num)&&(USART2SendQBoxTail!=USART2SendQBoxHost))
		{
			USART2SendTCB[USART2SendQBoxTail].Num--;
			USART2SendByte(*(USART2SendTCB[USART2SendQBoxTail].Index+count));
			count++;
		}
		else
		{	
		//USART2SendQBoxTail等于USART2SendQBoxTail的时候就标志这发送结束了，可以直接退出
			USART2RunningFlag=0;
			USART2SendQFree=USART2_SEND_MAX_BOX;
			count=0;
		}	
	}
	//由于头指针一直是指向空的发送块的，所以USART2SendQBoxTail等于USART2SendQBoxTail
	//的时候就标志这发送结束了，可以直接退出
	else
	{
		USART2RunningFlag=0;
		USART2SendQFree=USART2_SEND_MAX_BOX;
		count=0;
	}
	USART2StartSendISR();	
}

/*******************************************************************************
* 文件名	  	 : USART2RecvUpdate
* 描述	         : 
* 输入           : 无
* 输出           : 无
* 返回           : 无
*******************************************************************************/
void USART2RecvUpdate(void)
{
	USART2RecvBuffer[USART2RecvPtrW++] = USART2RecvByte();
	if (USART2RecvPtrW == USART2RecvPtrR) //读缓冲已满
	{
		USART2RecvPtrR++;//抛弃掉最老的数据	
	}	
}

/*******************************************************************************
* 文件名	  	 : USART2WriteDataToBuffer
* 描述	         : 检查发送缓冲区的大小，若空间足够，将待发送的数据放入到发送缓冲
				   区中去,并且启动发送
* 输入           : buffer待发送的数据的指针，count待发送的数据的数量
* 输出           : 无
* 返回           : 若正确放入到发送缓冲区中去了，就返回0x00	 ，否则返回0x01
*******************************************************************************/
u8 USART2WriteDataToBuffer(u8 *buffer,u8 count)
{
	u8 i=count;
	u8 err;
	/*此处可以加入信号灯或者关闭中断*/
	if(count==0)return 0x01;
	USART2StopSendISR();
	/*计算放入count个数据需要多少个内存块*/
	if(count%USART2_SEND_MAX_Q)count=count/USART2_SEND_MAX_Q+1;
	else count=count/USART2_SEND_MAX_Q;
	/*需要count个数据块*/
	/*如果内存不足，直接返回*/		 
	if(USART2SendQFree<count){USART2StartSendISR();return ERR_NO_SPACE;}
	//首先申请内存块，USART2SendQBoxHost在下一个内存申请后才加一
	USART2SendTCB[USART2SendQBoxHost].Index=(u8 *)OSMemGet(OSQUSART2Index,&err);
	if(USART2SendQBoxHost>=USART2_SEND_MAX_BOX)USART2SendQBoxHost=0;	
	count=0;
	while(i!='\0')										 
	{
		*(USART2SendTCB[USART2SendQBoxHost].Index+count)=*buffer;
		count++;
		if(count>=USART2_SEND_MAX_Q)
		{
			USART2SendTCB[USART2SendQBoxHost].Num=USART2_SEND_MAX_Q;
			//需要一个新的内存块存放接下来的数据，所以更新USART2SendQBoxHost
			if(++USART2SendQBoxHost>=USART2_SEND_MAX_BOX)USART2SendQBoxHost=0;
			//需要一个新的内存块存放接下来的数据	
			USART2SendTCB[USART2SendQBoxHost].Index=(u8 *)OSMemGet(OSQUSART2Index,&err);
			//空的发送任务块减一 			
			USART2SendQFree--;
			count=0;
		}
		buffer++;
		i--;
	}
	//此处是尚未整块存完的数据，它们也要存放在一个新的内存块里
	if(count!=0)
	{
		USART2SendTCB[USART2SendQBoxHost].Num=count; 
		USART2SendQFree--;
		if(++USART2SendQBoxHost>=USART2_SEND_MAX_BOX)USART2SendQBoxHost=0;
	}
	//如果是第一次，就启动发送，如果是已经启动就没有这个必要了
	if(USART2RunningFlag==0)
	{
#if	  	DMA_MODE
		USART2DMAConfig(USART2SendTCB[USART2SendQBoxTail].Index,USART2SendTCB[USART2SendQBoxTail].Num);
#else	
		USART2SendUpdate();
#endif		
		USART2RunningFlag=1;
	}
	/*此处可以开启信号灯或者打开中断*/
	USART2StartSendISR();
	return 0x00;
}
/*******************************************************************************
* 文件名	  	 : USART2DispFun
* 描述	         : 检查发送缓冲区的大小，若空间足够，将待发送的数据放入到发送缓冲
				   区中去,并且启动发送,与USART2WriteDataToBuffer不同的是，启动发送
				   函数世不需要指定文件大小的，这就给调用提供了方便.
* 输入           : buffer待发送的数据的指针
* 输出           : 无
* 返回           : 若正确放入到发送缓冲区中去了，就返回0x00	 ，否则返回0x01
*******************************************************************************/
u8 USART2DispFun(u8 *buffer)
{
	u32 count=0;
	while(buffer[count]!='\0')count++;
	return(USART2WriteDataToBuffer(buffer,count));
}

int GetIntData(unsigned char ucStartPtr)
{
  u8 i, LowByte;
  u16 WordData = 0x0000;
  int val;
  
  i = ucStartPtr;
  LowByte = USART2RecvBuffer[i];
  i = (i + 1);
  WordData = (USART2RecvBuffer[i]<<8) | LowByte;
  val = (int)WordData;
  return(val);
}

/********************************************
 *名称: CommandProcess                      *
 *用途: 根据收到的数据帧命令做相应处理      *
 ********************************************/
//RobotRate robotrate_last;

static void CommandProcess(void)
{
//	int ucTranslateRate, ucAngle, ucRotateRate;
//	static u8 sensorflag = OPENSENSOR; 
	u8 ucCommand, j, value;
	s16 long_value;
	RobotRate	robotrate;
	extern u8 g_dir;
	extern u8 test_flag;
	extern u8 move_all_flag;
	extern u8 move_target_flag;
	extern K_PD yaw_pid;
	extern float twoKp_z;
	extern float twoKi_z;
	extern float twoKd_z;
	extern float twoKp_x;
	extern float twoKi_x;
	extern float twoKd_x;
	extern float twoKp_y;
	extern float twoKi_y;
	extern float twoKd_y;
	u8 ack_frame[8];

//	WheelSpeed  realspeed;

	ucCommand = USART2RecvBuffer[USART2RecvBufStart];

	switch (ucCommand) 		//方向控制模式
	{
	case BUTCONMODE:
		/*j = (USART2RecvBufStart + 1);
		ucTranslateRate[rateIndex] = GetIntData(j);	 //速度大小
		j = (USART2RecvBufStart + 3);
		ucAngle[rateIndex] = GetIntData(j);		     //方向
		j = (USART2RecvBufStart + 5);
		ucRotateRate[rateIndex++] = (int)USART2RecvBuffer[j]; // 旋转角速度  must use int to do type convert,or ucRotateRate will get random value
		if (rateIndex >= 3) rateIndex = 0;
		j = (USART2RecvBufStart + 6);

		//滤除不正确的数据
		if ((ucTranslateRate[0] != ucTranslateRate[1]) || (ucTranslateRate[1] != ucTranslateRate[2])) return;
		if ((ucAngle[0] != ucAngle[1])||(ucAngle[1] != ucAngle[2])) return;
		if ((ucRotateRate[0] != ucRotateRate[1])||( ucRotateRate[1] != ucRotateRate[2])) return;
		if (USART2RecvBuffer[j] == 1) ucRotateRate[2] = (-1) * ucRotateRate[2];  
	
		robotrate.fBotTranslationRateX = (float)(ucTranslateRate[2] * cos(ucAngle[2] * PI / 180.0));
		robotrate.fBotTranslationRateY = (float)(ucTranslateRate[2] * sin(ucAngle[2] * PI / 180.0));
		robotrate.fBotAngularRate = (float)(PI * ucRotateRate[2] / 180.0); 		  //旋转角速度	   //0x20

		if (robotrate.fBotTranslationRateX != 0 ||
			robotrate.fBotTranslationRateY != 0 ||
			robotrate.fBotAngularRate != 0)
			g_bt_manual_flag = TRUE;
		else
			g_bt_manual_flag = FALSE;
				
//		realspeed = ComputeEachWheelSpeed(robotrate);
//		SendCmdToMotorDriver(realspeed);
		g_bt_manual_botrate = robotrate;*/
		
		value = USART2RecvBuffer[USART2RecvBufStart + 1];
		g_dir = value;
		value = USART2RecvBuffer[USART2RecvBufStart+2];
		g_bt_manual_botrate = value_to_speed(value);
//				strafe_block(speed_temp, from, to);
		if (g_bt_manual_botrate >= 400)
			break;
		hold_yaw = yaw;
		g_bt_manual_flag = TRUE;
		g_distance = 0;
		break;
	case DISCONMODE:   
		//autoflag =1;		
		stop_base();
		g_bt_manual_flag = FALSE;
		g_distance = 0;
		g_bt_manual_botrate = 0;
		break;
	case COORDCONMODE:	 //	目标点控制模式  行走距离控制模式
//		sensorflag = OPENSENSOR;
		//autoflag=1;	
//		RobotAutonomously(); 
		break;
	case DEMOMODE:	     //演示模式
//		sensorflag = CLOSESENSOR;
		break;
	case MYYAW:
		/*ack_frame[0] = (((s16)(yaw *10)) >> 8) & 0xFF;
		ack_frame[1] = ((s16)(yaw *10)) & 0xFF;*/
		sprintf(ack_frame,"%s","  yaw = ");
		USART2WriteDataToBuffer(ack_frame, 8);
		sprintf(ack_frame,"%f",yaw);
		USART2WriteDataToBuffer(ack_frame, 7);
	
		sprintf(ack_frame,"%s","  pitch = ");
		USART2WriteDataToBuffer(ack_frame, 10);
		sprintf(ack_frame,"%f",pitch);
		USART2WriteDataToBuffer(ack_frame, 7);
	
		sprintf(ack_frame,"%s","  roll = ");
		USART2WriteDataToBuffer(ack_frame, 9);
		sprintf(ack_frame,"%f",roll);
		USART2WriteDataToBuffer(ack_frame, 7);
	
		/*sprintf(ack_frame,"%s","  kp = ");
		USART2WriteDataToBuffer(ack_frame, 7);
		sprintf(ack_frame,"%f",twoKp_z);
		USART2WriteDataToBuffer(ack_frame, 7);
		
		sprintf(ack_frame,"%s","  ki = ");
		USART2WriteDataToBuffer(ack_frame, 7);
		sprintf(ack_frame,"%f",twoKi_z);
		USART2WriteDataToBuffer(ack_frame, 7);
		
		sprintf(ack_frame,"%s","  kd = ");
		USART2WriteDataToBuffer(ack_frame, 7);
		sprintf(ack_frame,"%f",twoKd_z);
		USART2WriteDataToBuffer(ack_frame, 7);

		sprintf(ack_frame,"%s","  time = ");
		USART2WriteDataToBuffer(ack_frame, 15);
		sprintf(ack_frame,"%f",interval);
		USART2WriteDataToBuffer(ack_frame, 7);*/
		
		sprintf(ack_frame,"%s","\r\n");
		USART2WriteDataToBuffer(ack_frame, 2);
		break;
	case TEST:
		test_flag = 1;
		value = USART2RecvBuffer[USART2RecvBufStart + 1];
		g_bt_manual_botrate = value_to_speed(value);
		break;
	case STOP_TEST:
		stop_base();
		test_flag = 0;
		g_bt_manual_botrate = 0;
		break;
	case MOVE_ALL:
		value = USART2RecvBuffer[USART2RecvBufStart + 1];
		g_bt_manual_botrate = (float)(PI * value / 180.0);
		if(g_bt_manual_botrate >= 0.6) 
		g_bt_manual_botrate = 0.6;	 // limit
		else if(g_bt_manual_botrate <= -0.6) 
		g_bt_manual_botrate = -0.6;
		move_all_flag = 1;
		break;
	case STOP_MOVE_ALL:
		stop_base();
		move_all_flag = 0;
		g_bt_manual_botrate = 0;
		break;
	case MOVE_TARGET:
		move_target_flag = 1;
		g_bt_manual_flag = TRUE;
		long_value = ((USART2RecvBuffer[USART2RecvBufStart + 1]) << 8) | (USART2RecvBuffer[USART2RecvBufStart + 2]);
		hold_yaw = (float)(long_value / 10.0);
		g_bt_manual_botrate = 0.0;
		break;
	case STOP_MOVE_TARGET:
		move_target_flag = 0;
		stop_base();
		g_bt_manual_botrate = 0.0;
		g_bt_manual_flag = FALSE;
		break;
	case PID_ADJUST:
		value = USART2RecvBuffer[USART2RecvBufStart + 1];
		yaw_pid.kp = (float)(value / 100.0);
		value = USART2RecvBuffer[USART2RecvBufStart + 2];
		yaw_pid.ki = (float)(value / 100.0);
		value = USART2RecvBuffer[USART2RecvBufStart + 3];
		yaw_pid.kd = (float)(value / 100.0);
		break;
case AHRSPID:
		value = USART2RecvBuffer[USART2RecvBufStart + 1];
		twoKp_z = (float) value;
		twoKp_x = twoKp_z;
		twoKp_y = twoKp_z;
		value = USART2RecvBuffer[USART2RecvBufStart + 2];
		twoKi_z = (float)(value / 10.0);
		twoKi_x = twoKi_z; 
		twoKi_y = twoKi_z;
		value = USART2RecvBuffer[USART2RecvBufStart + 3];
		twoKd_x = (float)(value / 10.0);
		twoKd_y = twoKd_x; 
		twoKd_z = twoKd_x;
    break;
	}
}

int USART2CheckDataFrame(void)
{
  unsigned char i, j, k;
  int flag = -1;
  while (USART2RecvPtrR != USART2RecvPtrW) 
  {
    if (USART2RecvState == NO_START_RCV) 
		{
      k = 0;
      i = (USART2RecvPtrR - 5);
      if (USART2RecvBuffer[i] == FIRST_FRAMEHEADER) 
			{
        k++;
      }
      
      i = (USART2RecvPtrR - 4);
      if (USART2RecvBuffer[i] == SENCOND_FRAMEHEADER) 
			{
        k++;
      }
      
			i = (USART2RecvPtrR - 3);
			if (USART2RecvBuffer[i] == MY_ADDR)
			{
				k++;
			}

      if (k == 3) 
			{
        //获取有效数据的起始地址和末尾地址（末尾地址指向整个数据帧的帧尾，校验和）
        USART2RecvState = START_RCV;
        i = (USART2RecvPtrR - 1);
        USART2RecvFrameLen = USART2RecvBuffer[i];
        USART2RecvBufStart = USART2RecvPtrR;
        USART2RecvBufEnd = (USART2RecvPtrR + USART2RecvFrameLen);
      }      
    } 
		else 
		{
      //开始接收数据处理
      if(USART2RecvPtrR == USART2RecvBufEnd) 
			{
        //数据帧接收完
        USART2RecvState = NO_START_RCV;
        
        j = USART2RecvBufStart;
        k = 0;
        for(i = 0; i < USART2RecvFrameLen; i++) 
				{
          k += USART2RecvBuffer[j];
         
          j = (j + 1);
        }
//			USART1WriteDataToBuffer(USART2RecvBuffer + USART2RecvBufStart, USART2RecvFrameLen);
        k = ~k;
        if(k == USART2RecvBuffer[j]) 
				{
          flag = USART2RecvBuffer[USART2RecvBufStart];
        }
      }
    }
    USART2RecvPtrR = (USART2RecvPtrR + 1);
  }
  return (flag);
}

void UART2Proc(void) 
{
  if(USART2RecvPtrR != USART2RecvPtrW) 
  {
    if(USART2CheckDataFrame() != -1) 
	{
      CommandProcess();
    } 		    
  }
}

