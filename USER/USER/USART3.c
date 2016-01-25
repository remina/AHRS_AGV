/******************** 版权所有 武汉汉迪机器人科技有限公司 杜骁释 **************
* 文件名             : USART3.c
* 描述               : USART3驱动层
* 移植步骤		 	 :（1）配置函数（管脚，时钟等）
				      （2）控制的参数（极性，波特率，位数，校验位等）
				      （3）中断函数
* 功能               ：配置串口管脚时钟，控制参数以及中断				   				   	
*******************************************************************************/
#include "USART3.h"
//#include "USART1.h"
#include "global.h"
#include "CAN.h"
#include "magnav.h"
#include "UART4.h"
#include "odometry.h"
#include "AHRS.h"
#include "sensors.h"
#include "FLASH.h"


u8 USART3SendQBoxHost;						//发送内存块头指针							
u8 USART3SendQBoxTail;						//发送内存块尾指针
u32  USART3SendQFree;						//发送内存块空闲区
u8 USART3RunningFlag=0;
typedef struct{
u8 Num;
u8 *Index;
}USART3SendTcb;

USART3SendTcb USART3SendTCB[USART3_SEND_MAX_BOX];

#define FIRST_FRAMEHEADER    0x55
#define SENCOND_FRAMEHEADER  0xAA
#define BASE_ADDRESS         0x15




#define WRITE_CMD_FRAME		 0x01
#define READ_CMD_FRAME		 0x02
#define ACK_WRITE_CMD_FRAME  0x03
#define ACK_READ_CMD_FRAME   0x04

#define ACTION_CMD_FRAME     0x05
#define BROARDCARST_FRAME	 0xFF

#define TURN_AND_GO_CMD  0x01
#define STRAFE_BLOCK_CMD 0x02
#define BACK_AND_TURN_CMD  0x03
#define STOP_CMD   0x04
#define BACK_AND_STOP_CMD 0x05


//#define STOP_CMD  0x01
//#define HEAD_CMD  0x02
//#define TAIL_CMD  0x03
//#define LEFT_CMD  0x04
//#define RIGHT_CMD 0x05
//#define TURN_CMD  0x06
#define MAG_NAV_MODE_ENABLE_CMD     0x07
#define MAG_NAV_MODE_DISABLE_CMD    0x08
#define MANUAL_CTL_MODE_ENABLE_CMD  0x09
#define MANUAL_CTL_MODE_DISABLE_CMD 0x10
#define BACK_TO_ORIGIN_CMD 0x11
#define READY_TO_GO_CMD 0x12
#define NAV_PD_CMD      0x13
#define DISTANCE_CMD      0x14

#define SPEED_CMD 0x01
#define POS_CMD   0x02
#define VOL_CMD   0x03
#define RFID_CMD  0x04
#define MAG_SENSOR_CMD 0x05
#define STATE_CMD 0x06
#define DISTANCE_INFO_CMD 0x07

#define VERIFY    0x01
#define CANCEL    0x02

#define NO_START_RCV         0
#define START_RCV            1
#define ERR_NO_SPACE	0xff	 //错误定义
u8 USART3RecvBuffer[USART3_RECV_MAX_Q]; //接收缓冲区	
u8 USART3RecvOVF=0; 				        //USART2接收任务块溢出标志  
u8 Recv3Index=0x00;
u8 Recv3Count=0x00;
u8 USART3RecvFlag=0;
u8 USART3RecvPtrW, USART3RecvPtrR;
u8 USART3RecvBufStart, USART3RecvBufEnd;
u8 USART3RecvState;
u8 USART3RecvFrameLen;
u8 OSUSART3MemQ[OS_MEM_USART3_MAX];  			//空白内存块

OSMEMTcb* OSQUSART3Index;

void CommandProcess(void);
u8 make_frame(u8 frame_type, u8 cmd_type, u8 data[], u8 data_len, u8 frame_data[]);
u8 g_zigbee_manual_flag = FALSE;
u8 g_magnav_auto_flag = FALSE;
u8 g_done_flag = FALSE;
u8 g_magnav_action_flag = FALSE;
u8 g_dir;
RobotRate g_zigbee_manual_botrate = {0.0, 0.0, 0.0};
float g_magnav_auto_botrate = 0.0;
float hold_yaw;

u8  back_and_turn(float speed, u8 from, u8 to);
u8  strafe_block(float speed, u8 from, u8 to);
u8  turn_and_go(float speed, u8 from, u8 to);

u8 error_code2[] = {0x55, 0xaa, 0x01, 0x42, 0xff};


void USART3_Init(unsigned long baud)
{	
	u8 MemTestErr;
	USART3RecvPtrW = 0;
	USART3RecvPtrR = 0;
	USART3SendQBoxHost = 0;
	USART3SendQBoxTail = 0;
	USART3SendQFree = USART3_SEND_MAX_BOX;

	USART3_Configuration(baud);
	OSQUSART3Index=(OSMEMTcb *)OSMemCreate(OSUSART3MemQ,OS_MEM_USART3_BLK,OS_MEM_USART3_MAX/OS_MEM_USART3_BLK, &MemTestErr);
}

/*******************************************************************************
* 文件名	  	 : USART2SendUpdate
* 描述	         : 检查结构体里面有没有数据还未发送完毕，若没有发送，则继续发送，
				   若发送完毕，退出
* 输入           : 无
* 输出           : 无
* 返回           : 无
*******************************************************************************/
void USART3SendUpdate(void)
{
	static unsigned char count=0;
	
	if(USART3SendQFree==USART3_SEND_MAX_BOX){return;}
	USART3StopSendISR();
	//如果现在的内存块的数据还没有发送完毕，启动发送，Num减一
	if((USART3SendTCB[USART3SendQBoxTail].Num)&&(USART3SendQBoxTail!=USART3SendQBoxHost))
	{
		USART3SendTCB[USART3SendQBoxTail].Num--;
		USART3SendByte(*(USART3SendTCB[USART3SendQBoxTail].Index+count));
		count++;
	}
	//一个发送块已经发送完毕了 ，USART2SendQFree++,尾指针加一。指向下一个发送块
	else if(USART3SendQBoxTail!=USART3SendQBoxHost)
	{		
		OSMemDelete(OSQUSART3Index,USART3SendTCB[USART3SendQBoxTail].Index);
		if(++USART3SendQBoxTail>=USART3_SEND_MAX_BOX)USART3SendQBoxTail=0;
		if(++USART3SendQFree>=USART3_SEND_MAX_BOX)USART3SendQFree=USART3_SEND_MAX_BOX;
		count=0;
		//USART2SendQBoxTail等于USART2SendQBoxTail的时候就标志这发送结束了，可以直接退出
		if((USART3SendQBoxTail!=USART3SendQBoxHost))
//		if((USART2SendTCB[USART2SendQBoxTail].Num)&&(USART2SendQBoxTail!=USART2SendQBoxHost))
		{
			USART3SendTCB[USART3SendQBoxTail].Num--;
			USART3SendByte(*(USART3SendTCB[USART3SendQBoxTail].Index+count));
			count++;
		}
		else
		{	
		//USART2SendQBoxTail等于USART2SendQBoxTail的时候就标志这发送结束了，可以直接退出
			USART3RunningFlag=0;
			USART3SendQFree=USART3_SEND_MAX_BOX;
			count=0;
		}	
	}
	//由于头指针一直是指向空的发送块的，所以USART2SendQBoxTail等于USART2SendQBoxTail
	//的时候就标志这发送结束了，可以直接退出
	else
	{
		USART3RunningFlag=0;
		USART3SendQFree=USART3_SEND_MAX_BOX;
		count=0;
	}
	USART3StartSendISR();	
}

/*******************************************************************************
* 文件名	  	 : USART2RecvUpdate
* 描述	         : 
* 输入           : 无
* 输出           : 无
* 返回           : 无
*******************************************************************************/
void USART3RecvUpdate(void)
{
	USART3RecvBuffer[USART3RecvPtrW++] = USART3RecvByte();

//    /* test */
//	USART1WriteDataToBuffer(USART3RecvBuffer + USART3RecvPtrW - 1, 1);
//	/* test */

	if (USART3RecvPtrW == USART3RecvPtrR) //读缓冲已满.trR和trW均未U8型，故当trW和trR均等于0（256）时，则溢出
	{
		USART3RecvPtrR++;//抛弃掉最老的数据	
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
u8 USART3WriteDataToBuffer(u8 *buffer,u8 count)
{
	u8 i=count;
	u8 err;
	/*此处可以加入信号灯或者关闭中断*/
	if(count==0)return 0x01;
	USART3StopSendISR();
	/*计算放入count个数据需要多少个内存块*/
	if(count%USART3_SEND_MAX_Q)count=count/USART3_SEND_MAX_Q+1;
	else count=count/USART3_SEND_MAX_Q;
	/*需要count个数据块*/
	/*如果内存不足，直接返回*/		 
	if(USART3SendQFree<count){USART3StartSendISR();return ERR_NO_SPACE;}
	//首先申请内存块，USART2SendQBoxHost在下一个内存申请后才加一
	USART3SendTCB[USART3SendQBoxHost].Index=(u8 *)OSMemGet(OSQUSART3Index,&err);
	if(USART3SendQBoxHost>=USART3_SEND_MAX_BOX)USART3SendQBoxHost=0;	
	count=0;
	while(i!='\0')										 
	{
		*(USART3SendTCB[USART3SendQBoxHost].Index+count)=*buffer;
		count++;
		if(count>=USART3_SEND_MAX_Q)
		{
			USART3SendTCB[USART3SendQBoxHost].Num=USART3_SEND_MAX_Q;
			//需要一个新的内存块存放接下来的数据，所以更新USART2SendQBoxHost
			if(++USART3SendQBoxHost>=USART3_SEND_MAX_BOX)USART3SendQBoxHost=0;
			//需要一个新的内存块存放接下来的数据	
			USART3SendTCB[USART3SendQBoxHost].Index=(u8 *)OSMemGet(OSQUSART3Index,&err);
			//空的发送任务块减一 			
			USART3SendQFree--;
			count=0;
		}
		buffer++;
		i--;
	}
	//此处是尚未整块存完的数据，它们也要存放在一个新的内存块里
	if(count!=0)
	{
		USART3SendTCB[USART3SendQBoxHost].Num=count; 
		USART3SendQFree--;
		if(++USART3SendQBoxHost>=USART3_SEND_MAX_BOX)USART3SendQBoxHost=0;
	}
	//如果是第一次，就启动发送，如果是已经启动就没有这个必要了
	if(USART3RunningFlag==0)
	{
#if	  	DMA_MODE
		USART3DMAConfig(USART3SendTCB[USART3SendQBoxTail].Index,USART3SendTCB[USART3SendQBoxTail].Num);
#else	
		USART3SendUpdate();
#endif		
		USART3RunningFlag=1;
	}
	/*此处可以开启信号灯或者打开中断*/
	USART3StartSendISR();
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
u8 USART3DispFun(u8 *buffer)
{
	u32 count=0;
	while(buffer[count]!='\0')count++;
	return(USART3WriteDataToBuffer(buffer,count));
}


int USART3CheckDataFrame(void)
{
  u8 i, j, k;
  int flag = -1;
  while (USART3RecvPtrR != USART3RecvPtrW) 
  {
    if (USART3RecvState == NO_START_RCV) 
	{
      k = 0;
      i = (USART3RecvPtrR - 4);
      if (USART3RecvBuffer[i] == FIRST_FRAMEHEADER) 
	  {
        k++;
      }
      
      i = (USART3RecvPtrR - 3);
      if (USART3RecvBuffer[i] == SENCOND_FRAMEHEADER) 
	  {
        k++;
      }

	  i = (USART3RecvPtrR - 2);
	  if (USART3RecvBuffer[i] == BASE_ADDRESS)
	  {
	  	k++;
	  }
      
      if (k == 3) 
	  {
        //获取有效数据的起始地址和末尾地址（末尾地址指向整个数据帧的帧尾，校验和）
        USART3RecvState = START_RCV;
        i = (USART3RecvPtrR - 1);
        USART3RecvFrameLen = USART3RecvBuffer[i];
        USART3RecvBufStart = USART3RecvPtrR;
        USART3RecvBufEnd = (USART3RecvPtrR + USART3RecvFrameLen);
      }      
    } 
	else 
	{
      //开始接收数据处理
      if(USART3RecvPtrR == USART3RecvBufEnd) 
	  {
        //数据帧接收完
        USART3RecvState = NO_START_RCV;
        
        j = USART3RecvBufStart;
        k = 0;
        for(i = 0; i < USART3RecvFrameLen; i++) 
		{
          k += USART3RecvBuffer[j];
         
          j = (j + 1);
        }
//		USART1WriteDataToBuffer(USART3RecvBuffer + USART3RecvBufStart, USART3RecvFrameLen);
        if(k == USART3RecvBuffer[j]) 
		{
          flag = USART3RecvBuffer[USART3RecvBufStart];
        }
      }
    }
    USART3RecvPtrR ++;
  }
  return (flag);	
}

void UART3Proc(void) 
{
//  u8 status = FALSE;
  if(USART3RecvPtrR != USART3RecvPtrW) 
  {
    if(USART3CheckDataFrame() != -1) 
	{
//      status = CommandProcess(rate);
	  CommandProcess();
    } 		    
  }

//  return status;	 
}

float value_to_speed(u8 value)
{
	float speed;
	speed = (float)value / 255.0 * 800.0; 
	return speed;		
}

float value_to_angular(u8 value)
{
	float angular_speed;
	if (value >= 1 && value < 100)
		angular_speed = 0.314;
	else if(value >= 100 && value <= 255)
		angular_speed = -0.314;
	else
		angular_speed = 0.0;
	return angular_speed;
}

u8 mag_status_to_data(u8 data[], u8 len)
{
	u8 i, mag_data;
	mag_data = 0;

	for (i = 0; i < len; i++)
	{
		mag_data |= (data[i] << i);
	}
	
	return mag_data;		
}

u16 GetWordData(unsigned char ucStartPtr)
{
  u8 i, LowByte;
  u16 WordData = 0x0000;
  int val;
  
  i = ucStartPtr;
  LowByte = USART3RecvBuffer[i];
  i = (i + 1);
  WordData = (USART3RecvBuffer[i]<<8) | LowByte;
  
  return WordData;
}



/********************************************
 *名称: CommandProcess                      *
 *用途: 根据收到的数据帧命令做相应处理      *
 ********************************************/
u8 g_from, g_to;
u8 last_cmd_type;
u8 lose_flag=0;
static void CommandProcess(void)
{
//	int ucTranslateRate, ucAngle, ucRotateRate;
	u8 frame_type, cmd_type;
	u8 value;
	u8 ack_frame[255] = {0};
	u8 ack_frame_data_len = 0;
	RobotRate robotrate = {0.0, 0.0, 0.0};
	//RobotRate turn_rate = {0.0, 0.0, 0.0};
	float speed_temp;
//	u8 front_status, back_status;
	u8 mag_sensor_status[8];
	u8 mag_sensor_data[4];
	u8 dir;
//	u8 from, to;
	u8 i;
	u8 test[4];
	

	WheelSpeed realspeed;

	g_done_flag = FALSE;   //每次接收到帧时首先处理为FALSE，当收到确认帧时才置为TRUE

	frame_type = USART3RecvBuffer[USART3RecvBufStart];

	switch (frame_type) 		
	{
	case WRITE_CMD_FRAME:
	    /*返回应答帧，写入成功或者写入失败*/
		/*写入停止小车，写入小车导航速度方向，写入小车旋转方向*/
		cmd_type = USART3RecvBuffer[USART3RecvBufStart+1];
		switch (cmd_type)
		{
//		case STOP_CMD:
//			stop_base();
//			break;
/*		case HEAD_CMD:
			value = USART3RecvBuffer[USART3RecvBufStart+2];
			speed_temp = value_to_speed(value);
			if (g_magnav_auto_flag)
			{
	  			g_magnav_auto_botrate = speed_temp;
				g_dir = FRONT;										   
			}
			if (g_zigbee_manual_flag)
			{

				robotrate.fBotTranslationRateX = speed_temp;
				robotrate.fBotTranslationRateY = 0;
				robotrate.fBotAngularRate = 0;
				g_zigbee_manual_botrate = robotrate;
			}
			break;
		case TAIL_CMD:
			value = USART3RecvBuffer[USART3RecvBufStart+2];
			speed_temp = value_to_speed(value);

			if (g_magnav_auto_flag)
			{
	  			g_magnav_auto_botrate = speed_temp;
				g_dir = BACK;
			}
			if (g_zigbee_manual_flag)
			{
				robotrate.fBotTranslationRateX = -speed_temp;
				robotrate.fBotTranslationRateY = 0;
				robotrate.fBotAngularRate = 0;
				g_zigbee_manual_botrate = robotrate;
			}
			break;
		case LEFT_CMD:
			value = USART3RecvBuffer[USART3RecvBufStart+2];
			speed_temp = value_to_speed(value);

			if (g_magnav_auto_flag)
			{
	  			g_magnav_auto_botrate = speed_temp;
				g_dir = LEFT;
			}
			if (g_zigbee_manual_flag)
			{

				robotrate.fBotTranslationRateX = 0;
				robotrate.fBotTranslationRateY = speed_temp;
				robotrate.fBotAngularRate = 0;
				g_zigbee_manual_botrate = robotrate;
			}
			break;
		case RIGHT_CMD:
			value = USART3RecvBuffer[USART3RecvBufStart+2];
			speed_temp = value_to_speed(value);

			if (g_magnav_auto_flag)
			{
	  			g_magnav_auto_botrate = speed_temp;
				g_dir = RIGHT;
			}
			if (g_zigbee_manual_flag)
			{
				robotrate.fBotTranslationRateX = 0;
				robotrate.fBotTranslationRateY = -speed_temp;
				robotrate.fBotAngularRate = 0;
				g_zigbee_manual_botrate = robotrate;
			}
			break;
		case TURN_CMD:
			value = USART3RecvBuffer[USART3RecvBufStart+2];
			robotrate.fBotTranslationRateX = 0;
			robotrate.fBotTranslationRateY = 0;
			robotrate.fBotAngularRate = value_to_angular(value);

			realspeed = ComputeEachWheelSpeed(robotrate);
 			SendCmdToMotorDriver(realspeed);
			break;
		case BACK_TO_ORIGIN_CMD:
			value = USART3RecvBuffer[USART3RecvBufStart+2];
			robotrate.fBotTranslationRateX = 0;
			robotrate.fBotTranslationRateY = 0;
			robotrate.fBotAngularRate = value_to_angular(value);
			realspeed = ComputeEachWheelSpeed(robotrate);
 			SendCmdToMotorDriver(realspeed);			
			break;
		case READY_TO_GO_CMD:
			value = USART3RecvBuffer[USART3RecvBufStart+2];
			robotrate.fBotTranslationRateX = 0;
			robotrate.fBotTranslationRateY = 0;
			robotrate.fBotAngularRate = value_to_angular(value);
			realspeed = ComputeEachWheelSpeed(robotrate);
 			SendCmdToMotorDriver(realspeed);
			break;
		case MAG_NAV_MODE_ENABLE_CMD:
			g_magnav_auto_flag = TRUE;
			g_zigbee_manual_flag = FALSE;
			g_zigbee_manual_botrate.fBotTranslationRateX = 0.0;
			g_zigbee_manual_botrate.fBotTranslationRateY = 0.0;
			g_zigbee_manual_botrate.fBotAngularRate = 0.0;
		case MAG_NAV_MODE_DISABLE_CMD:
			g_magnav_auto_flag = FALSE;
			g_magnav_auto_botrate = 0.0;
			break;
		case MANUAL_CTL_MODE_ENABLE_CMD:
			g_zigbee_manual_flag = TRUE;
			g_magnav_auto_flag = FALSE;
			g_magnav_auto_botrate = 0.0;
			break;
		case MANUAL_CTL_MODE_DISABLE_CMD:
			g_zigbee_manual_flag = FALSE;
			g_zigbee_manual_botrate.fBotTranslationRateX = 0.0;
			g_zigbee_manual_botrate.fBotTranslationRateY = 0.0;
			g_zigbee_manual_botrate.fBotAngularRate = 0.0;
			break; */
		case NAV_PD_CMD:
			{
			//u8 test[4] = {0x11, 0x22,0x33,0x44};
			value = USART3RecvBuffer[USART3RecvBufStart+2];
			g_pd[PID_FRONT].kp = value / 1000.0;
			value = USART3RecvBuffer[USART3RecvBufStart+3];
			g_pd[PID_FRONT].kd = value / 1000.0;
			value = USART3RecvBuffer[USART3RecvBufStart+4];
			g_pd[PID_BACK].kp = value / 1000.0;
			value = USART3RecvBuffer[USART3RecvBufStart+5];
			g_pd[PID_BACK].kd = value / 1000.0;
			value = USART3RecvBuffer[USART3RecvBufStart+6];
			g_pd[PID_LEFT].kp = -value / 1000.0;
			value = USART3RecvBuffer[USART3RecvBufStart+7];
			g_pd[PID_LEFT].kd = -value / 1000.0;
			value = USART3RecvBuffer[USART3RecvBufStart+8];
			g_pd[PID_RIGHT].kp = value / 1000.0;
			value = USART3RecvBuffer[USART3RecvBufStart+9];
			g_pd[PID_RIGHT].kd = value / 1000.0;
			
			for(i=0;i<8;i++)			
			{
				writeflashdata[8+i]=USART3RecvBuffer[USART3RecvBufStart+2+i];
			}
			//FlashWriteStr(FLASH_ADR+12, 8, USART3RecvBuffer+USART3RecvBufStart+2 );
			FlashWriteStr(FLASH_ADR, 16, writeflashdata);
			//FlashWriteStr(StartAddr, 4,test );

			ack_frame_data_len = make_frame(frame_type, cmd_type, USART3RecvBuffer+USART3RecvBufStart+2, 8, ack_frame);

			USART3WriteDataToBuffer(ack_frame, ack_frame_data_len);

			g_pd_flag = TRUE;
			}
			break;
		case DISTANCE_CMD:
			{
//			u8 test[4] = {0x11, 0x22,0x33,0x44};
			g_front_distance = GetWordData(USART3RecvBufStart+2);
			//FlashWriteWord(FLASH_ADR, g_front_distance);
			g_mid_distance = GetWordData(USART3RecvBufStart+4);
			//FlashWriteWord(FLASH_ADR+4, g_mid_distance);
     		g_back_distance = GetWordData(USART3RecvBufStart+6);
			g_mid_distance_1 = GetWordData(USART3RecvBufStart+8);
			//FlashWriteWord(FLASH_ADR+8, g_back_distance);

			for(i=0;i<8;i++)			
			{
				writeflashdata[i]=USART3RecvBuffer[USART3RecvBufStart+2+i];
			}
			//FlashWriteStr(FLASH_ADR+12, 8, USART3RecvBuffer+USART3RecvBufStart+2 );
			FlashWriteStr(FLASH_ADR, 16, writeflashdata);

//			u8 test[6] = {0x11, 0x22, 0x33,0x44, 0x55, 0x66};
//			g_front_distance = GetWordData(USART3RecvBufStart+2);
//			test[0]=g_front_distance%256;
//			test[1]=g_front_distance/256;
//
//			//FlashWriteWord(FLASH_ADR, g_front_distance);
//			g_mid_distance = GetWordData(USART3RecvBufStart+4);
//			test[2]=g_mid_distance%256;
//			test[3]=g_mid_distance/256;
//
//			//FlashWriteWord(FLASH_ADR+4, g_mid_distance);
//     		g_back_distance = GetWordData(USART3RecvBufStart+6);
//			//FlashWriteWord(FLASH_ADR+8, g_back_distance);
//			test[4]=g_back_distance%256;
//			test[5]=g_back_distance/256;
//
//			//FlashWriteStr(FLASH_ADR, 6, test);


			last_cmd_type = cmd_type; 
				
			ack_frame_data_len = make_frame(frame_type, cmd_type, USART3RecvBuffer+USART3RecvBufStart+2, 8, ack_frame);

			USART3WriteDataToBuffer(ack_frame, ack_frame_data_len);
			}
			break;
		default:
			break;
		}
		

		
		/* test */
//	    USART1WriteDataToBuffer(ack_frame, ack_frame_data_len);
	    /* test */ 
		
		break;

	case READ_CMD_FRAME:
		/* 返回读取结果 */
		/* 读取电量，读取速度，读取位置RFID*/
		cmd_type = USART3RecvBuffer[USART3RecvBufStart+1];
		switch (cmd_type)
		{
		case SPEED_CMD:
			break;
		case POS_CMD:
			break;
		case VOL_CMD:
			break;
		case RFID_CMD:
		    break;
			break;
		case STATE_CMD:
			ack_frame_data_len = make_frame(frame_type, cmd_type, 0, 0, ack_frame);
			USART3WriteDataToBuffer(ack_frame, ack_frame_data_len);				
			break; 
		case DISTANCE_INFO_CMD:
		    test[0]=g_distance/20;
			ack_frame_data_len = make_frame(frame_type, cmd_type, test, 1 ,ack_frame);
			USART3WriteDataToBuffer(ack_frame, ack_frame_data_len);
			break;
		default:
			break;
		}
		break;
	case ACTION_CMD_FRAME:
		cmd_type = USART3RecvBuffer[USART3RecvBufStart+1];

		switch (cmd_type)
		{
			/*case TURN_AND_GO_CMD:
//				ack_frame_data_len = make_frame(frame_type, cmd_type, USART3RecvBuffer+USART3RecvBufStart+2, 4, ack_frame);
//				USART3WriteDataToBuffer(ack_frame, ack_frame_data_len);

				value = USART3RecvBuffer[USART3RecvBufStart+2];
				g_dir = FRONT;
				value = USART3RecvBuffer[USART3RecvBufStart+3];
				g_magnav_auto_botrate = value_to_speed(value);
				value = USART3RecvBuffer[USART3RecvBufStart+4];
				g_from = value;
				value = USART3RecvBuffer[USART3RecvBufStart+5];
				g_to = value;
				if (g_magnav_auto_botrate >= 400)
					break;
				if (g_from < 1 || g_from > 16)
					break;
				if (g_to < 1 || g_to > 16)
					break;
				if (g_to ==(g_from+8))
				{
					if(g_magnav_action_flag == FALSE)  g_distance = 0;
					g_magnav_action_flag = TRUE;					
				   	ack_frame_data_len = make_frame(frame_type, cmd_type, USART3RecvBuffer+USART3RecvBufStart+2, 4, ack_frame);
					USART3WriteDataToBuffer(ack_frame, ack_frame_data_len);					
				}
				else
				{
					//error
				}
				break;*/
			case STRAFE_BLOCK_CMD:
//				ack_frame_data_len = make_frame(frame_type, cmd_type, USART3RecvBuffer+USART3RecvBufStart+2, 4, ack_frame);
//				USART3WriteDataToBuffer(ack_frame, ack_frame_data_len);
				value = USART3RecvBuffer[USART3RecvBufStart+2];
				g_dir = value;
				value = USART3RecvBuffer[USART3RecvBufStart+3];
				g_magnav_auto_botrate = value_to_speed(value);
//				strafe_block(speed_temp, from, to);
				if (g_magnav_auto_botrate >= 400)
					break;
				hold_yaw = yaw;
				g_magnav_action_flag = TRUE;
				g_distance = 0;
				ack_frame_data_len = make_frame(frame_type, cmd_type, USART3RecvBuffer+USART3RecvBufStart+2, 4, ack_frame);
				USART3WriteDataToBuffer(ack_frame, ack_frame_data_len);				
				break;
/*			case BACK_AND_TURN_CMD:
//				ack_frame_data_len = make_frame(frame_type, cmd_type, USART3RecvBuffer+USART3RecvBufStart+2, 4, ack_frame);
//				USART3WriteDataToBuffer(ack_frame, ack_frame_data_len);
				value = USART3RecvBuffer[USART3RecvBufStart+2];
				g_dir = BACK;
				value = USART3RecvBuffer[USART3RecvBufStart+3];
				g_magnav_auto_botrate = value_to_speed(value);
				value = USART3RecvBuffer[USART3RecvBufStart+4];
				g_from = value;
				value = USART3RecvBuffer[USART3RecvBufStart+5];
				g_to = value;

//				back_and_turn(speed_temp, from, to);
				if (g_magnav_auto_botrate >= 400)
					break;
				if (g_from < 1 || g_from > 16)
					break;
				if (g_to < 1 || g_to > 16)
					break;
				if (g_to ==(g_from-8))
				{
					if(g_magnav_action_flag == FALSE)  g_distance = 0;
					g_magnav_action_flag = TRUE;
					lose_flag=0;					
					ack_frame_data_len = make_frame(frame_type, cmd_type, USART3RecvBuffer+USART3RecvBufStart+2, 4, ack_frame);
					USART3WriteDataToBuffer(ack_frame, ack_frame_data_len);					
				}
				else
				{
//					g_magnav_action_flag = FALSE;
//					g_magnav_auto_botrate = 0.0;
				}
				break;*/
/*			case BACK_AND_STOP_CMD:
//				ack_frame_data_len = make_frame(frame_type, cmd_type, USART3RecvBuffer+USART3RecvBufStart+2, 4, ack_frame);
//				USART3WriteDataToBuffer(ack_frame, ack_frame_data_len);
				value = USART3RecvBuffer[USART3RecvBufStart+2];
				g_dir = BACK;
				value = USART3RecvBuffer[USART3RecvBufStart+3];
				g_magnav_auto_botrate = value_to_speed(value);
				value = USART3RecvBuffer[USART3RecvBufStart+4];
				g_from = value;
				value = USART3RecvBuffer[USART3RecvBufStart+5];
				g_to = value;
				value = USART3RecvBuffer[USART3RecvBufStart+6];
				lose_flag = value;

//				back_and_turn(speed_temp, from, to);
				if (g_magnav_auto_botrate >= 400)
					break;
				if (g_from < 1 || g_from > 16)
					break;
				if (g_to < 1 || g_to > 16)
					break;
				if (g_to ==(g_from-8))
				{
					if(g_magnav_action_flag == FALSE)  g_distance = 0;
					g_magnav_action_flag = TRUE;					
					ack_frame_data_len = make_frame(frame_type, cmd_type, USART3RecvBuffer+USART3RecvBufStart+2, 4, ack_frame);
					USART3WriteDataToBuffer(ack_frame, ack_frame_data_len);					
				}
				else
				{
//					g_magnav_action_flag = FALSE;
//					g_magnav_auto_botrate = 0.0;
				}
				break;*/
			case STOP_CMD:
				stop_base();

				g_magnav_action_flag = FALSE;
				g_distance = 0;
				g_magnav_auto_botrate = 0;
				ack_frame_data_len = make_frame(frame_type, cmd_type, 0, 0, ack_frame);
				USART3WriteDataToBuffer(ack_frame, ack_frame_data_len);
				break;
			default:							 
				
				break;
		}
//		if (cmd_type == TURN_AND_GO_CMD || cmd_type == STRAFE_BLOCK_CMD || cmd_type == BACK_AND_TURN_CMD)
//		{
//			ack_frame_data_len = make_frame(frame_type, cmd_type, 0, 0, ack_frame);
//			USART3WriteDataToBuffer(ack_frame, ack_frame_data_len);
//		}
		break;
	case ACK_WRITE_CMD_FRAME:
		cmd_type = USART3RecvBuffer[USART3RecvBufStart+1]; 
	    switch (cmd_type)
		{
		case VERIFY:   
			/*执行写入命令帧*/
			if (last_cmd_type <= 0x06)
				g_done_flag = TRUE;
			break;
		case CANCEL:	
			/*取消写入命令帧*/
			g_done_flag = FALSE;

			robotrate.fBotTranslationRateX = 0;	   /*需要清零吗, 应该是保持现在的状态*/
			robotrate.fBotTranslationRateY = 0;
			robotrate.fBotAngularRate = 0;

			break;
		default:
			break;
		}
		break;
	case ACK_READ_CMD_FRAME:

		break;
	case BROARDCARST_FRAME:
		/* 无返回 */
		switch (cmd_type)
		{
		case VERIFY:
			/*执行写入命令帧*/
			g_done_flag	= TRUE;
			break;
		case CANCEL:
			/*取消写入命令帧*/
			g_done_flag = FALSE;
			robotrate.fBotTranslationRateX = 0;	   /*需要清零吗*/
			robotrate.fBotTranslationRateY = 0;
			robotrate.fBotAngularRate = 0;

			break;
		default:
			break;
		}
	    break;
	default:
		break;
	}

}



/*
 * @frame_type: input
 * @cmd_type:
 * @data[]: input
 * @data_len: input
 * @frame_data[]: output
 * @return: frame_len
 */
u8 make_frame(u8 frame_type, u8 cmd_type, u8 data[], u8 data_len, u8 frame_data[])
{
	u8 i, j, checksum;
	checksum = 0;
	i = 0;
	frame_data[i++] = FIRST_FRAMEHEADER;
	frame_data[i++] = SENCOND_FRAMEHEADER;
	frame_data[i++] = BASE_ADDRESS;
	frame_data[i++] = data_len + 2;
	frame_data[i++] = frame_type;
	frame_data[i++] = cmd_type;
	if(data_len == 0) 
	{
		checksum = frame_type + cmd_type;
		frame_data[i++] = checksum;
	}
	else
	{
		checksum = frame_type + cmd_type;
		for (j = 0; j < data_len; j++)
		{
			frame_data[i++] = data[j];
			checksum += data[j];
		}
		frame_data[i++] = checksum;
	}
	
	return i;			
}


