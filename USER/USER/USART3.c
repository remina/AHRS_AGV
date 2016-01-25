/******************** ��Ȩ���� �人���ϻ����˿Ƽ����޹�˾ ������ **************
* �ļ���             : USART3.c
* ����               : USART3������
* ��ֲ����		 	 :��1�����ú������ܽţ�ʱ�ӵȣ�
				      ��2�����ƵĲ��������ԣ������ʣ�λ����У��λ�ȣ�
				      ��3���жϺ���
* ����               �����ô��ڹܽ�ʱ�ӣ����Ʋ����Լ��ж�				   				   	
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


u8 USART3SendQBoxHost;						//�����ڴ��ͷָ��							
u8 USART3SendQBoxTail;						//�����ڴ��βָ��
u32  USART3SendQFree;						//�����ڴ�������
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
#define ERR_NO_SPACE	0xff	 //������
u8 USART3RecvBuffer[USART3_RECV_MAX_Q]; //���ջ�����	
u8 USART3RecvOVF=0; 				        //USART2��������������־  
u8 Recv3Index=0x00;
u8 Recv3Count=0x00;
u8 USART3RecvFlag=0;
u8 USART3RecvPtrW, USART3RecvPtrR;
u8 USART3RecvBufStart, USART3RecvBufEnd;
u8 USART3RecvState;
u8 USART3RecvFrameLen;
u8 OSUSART3MemQ[OS_MEM_USART3_MAX];  			//�հ��ڴ��

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
* �ļ���	  	 : USART2SendUpdate
* ����	         : ���ṹ��������û�����ݻ�δ������ϣ���û�з��ͣ���������ͣ�
				   ��������ϣ��˳�
* ����           : ��
* ���           : ��
* ����           : ��
*******************************************************************************/
void USART3SendUpdate(void)
{
	static unsigned char count=0;
	
	if(USART3SendQFree==USART3_SEND_MAX_BOX){return;}
	USART3StopSendISR();
	//������ڵ��ڴ������ݻ�û�з�����ϣ��������ͣ�Num��һ
	if((USART3SendTCB[USART3SendQBoxTail].Num)&&(USART3SendQBoxTail!=USART3SendQBoxHost))
	{
		USART3SendTCB[USART3SendQBoxTail].Num--;
		USART3SendByte(*(USART3SendTCB[USART3SendQBoxTail].Index+count));
		count++;
	}
	//һ�����Ϳ��Ѿ���������� ��USART2SendQFree++,βָ���һ��ָ����һ�����Ϳ�
	else if(USART3SendQBoxTail!=USART3SendQBoxHost)
	{		
		OSMemDelete(OSQUSART3Index,USART3SendTCB[USART3SendQBoxTail].Index);
		if(++USART3SendQBoxTail>=USART3_SEND_MAX_BOX)USART3SendQBoxTail=0;
		if(++USART3SendQFree>=USART3_SEND_MAX_BOX)USART3SendQFree=USART3_SEND_MAX_BOX;
		count=0;
		//USART2SendQBoxTail����USART2SendQBoxTail��ʱ��ͱ�־�ⷢ�ͽ����ˣ�����ֱ���˳�
		if((USART3SendQBoxTail!=USART3SendQBoxHost))
//		if((USART2SendTCB[USART2SendQBoxTail].Num)&&(USART2SendQBoxTail!=USART2SendQBoxHost))
		{
			USART3SendTCB[USART3SendQBoxTail].Num--;
			USART3SendByte(*(USART3SendTCB[USART3SendQBoxTail].Index+count));
			count++;
		}
		else
		{	
		//USART2SendQBoxTail����USART2SendQBoxTail��ʱ��ͱ�־�ⷢ�ͽ����ˣ�����ֱ���˳�
			USART3RunningFlag=0;
			USART3SendQFree=USART3_SEND_MAX_BOX;
			count=0;
		}	
	}
	//����ͷָ��һֱ��ָ��յķ��Ϳ�ģ�����USART2SendQBoxTail����USART2SendQBoxTail
	//��ʱ��ͱ�־�ⷢ�ͽ����ˣ�����ֱ���˳�
	else
	{
		USART3RunningFlag=0;
		USART3SendQFree=USART3_SEND_MAX_BOX;
		count=0;
	}
	USART3StartSendISR();	
}

/*******************************************************************************
* �ļ���	  	 : USART2RecvUpdate
* ����	         : 
* ����           : ��
* ���           : ��
* ����           : ��
*******************************************************************************/
void USART3RecvUpdate(void)
{
	USART3RecvBuffer[USART3RecvPtrW++] = USART3RecvByte();

//    /* test */
//	USART1WriteDataToBuffer(USART3RecvBuffer + USART3RecvPtrW - 1, 1);
//	/* test */

	if (USART3RecvPtrW == USART3RecvPtrR) //����������.trR��trW��δU8�ͣ��ʵ�trW��trR������0��256��ʱ�������
	{
		USART3RecvPtrR++;//���������ϵ�����	
	}	
}

/*******************************************************************************
* �ļ���	  	 : USART2WriteDataToBuffer
* ����	         : ��鷢�ͻ������Ĵ�С�����ռ��㹻���������͵����ݷ��뵽���ͻ���
				   ����ȥ,������������
* ����           : buffer�����͵����ݵ�ָ�룬count�����͵����ݵ�����
* ���           : ��
* ����           : ����ȷ���뵽���ͻ�������ȥ�ˣ��ͷ���0x00	 �����򷵻�0x01
*******************************************************************************/
u8 USART3WriteDataToBuffer(u8 *buffer,u8 count)
{
	u8 i=count;
	u8 err;
	/*�˴����Լ����źŵƻ��߹ر��ж�*/
	if(count==0)return 0x01;
	USART3StopSendISR();
	/*�������count��������Ҫ���ٸ��ڴ��*/
	if(count%USART3_SEND_MAX_Q)count=count/USART3_SEND_MAX_Q+1;
	else count=count/USART3_SEND_MAX_Q;
	/*��Ҫcount�����ݿ�*/
	/*����ڴ治�㣬ֱ�ӷ���*/		 
	if(USART3SendQFree<count){USART3StartSendISR();return ERR_NO_SPACE;}
	//���������ڴ�飬USART2SendQBoxHost����һ���ڴ������ż�һ
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
			//��Ҫһ���µ��ڴ���Ž����������ݣ����Ը���USART2SendQBoxHost
			if(++USART3SendQBoxHost>=USART3_SEND_MAX_BOX)USART3SendQBoxHost=0;
			//��Ҫһ���µ��ڴ���Ž�����������	
			USART3SendTCB[USART3SendQBoxHost].Index=(u8 *)OSMemGet(OSQUSART3Index,&err);
			//�յķ���������һ 			
			USART3SendQFree--;
			count=0;
		}
		buffer++;
		i--;
	}
	//�˴�����δ�����������ݣ�����ҲҪ�����һ���µ��ڴ����
	if(count!=0)
	{
		USART3SendTCB[USART3SendQBoxHost].Num=count; 
		USART3SendQFree--;
		if(++USART3SendQBoxHost>=USART3_SEND_MAX_BOX)USART3SendQBoxHost=0;
	}
	//����ǵ�һ�Σ����������ͣ�������Ѿ�������û�������Ҫ��
	if(USART3RunningFlag==0)
	{
#if	  	DMA_MODE
		USART3DMAConfig(USART3SendTCB[USART3SendQBoxTail].Index,USART3SendTCB[USART3SendQBoxTail].Num);
#else	
		USART3SendUpdate();
#endif		
		USART3RunningFlag=1;
	}
	/*�˴����Կ����źŵƻ��ߴ��ж�*/
	USART3StartSendISR();
	return 0x00;
}

/*******************************************************************************
* �ļ���	  	 : USART2DispFun
* ����	         : ��鷢�ͻ������Ĵ�С�����ռ��㹻���������͵����ݷ��뵽���ͻ���
				   ����ȥ,������������,��USART2WriteDataToBuffer��ͬ���ǣ���������
				   ����������Ҫָ���ļ���С�ģ���͸������ṩ�˷���.
* ����           : buffer�����͵����ݵ�ָ��
* ���           : ��
* ����           : ����ȷ���뵽���ͻ�������ȥ�ˣ��ͷ���0x00	 �����򷵻�0x01
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
        //��ȡ��Ч���ݵ���ʼ��ַ��ĩβ��ַ��ĩβ��ַָ����������֡��֡β��У��ͣ�
        USART3RecvState = START_RCV;
        i = (USART3RecvPtrR - 1);
        USART3RecvFrameLen = USART3RecvBuffer[i];
        USART3RecvBufStart = USART3RecvPtrR;
        USART3RecvBufEnd = (USART3RecvPtrR + USART3RecvFrameLen);
      }      
    } 
	else 
	{
      //��ʼ�������ݴ���
      if(USART3RecvPtrR == USART3RecvBufEnd) 
	  {
        //����֡������
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
 *����: CommandProcess                      *
 *��;: �����յ�������֡��������Ӧ����      *
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

	g_done_flag = FALSE;   //ÿ�ν��յ�֡ʱ���ȴ���ΪFALSE�����յ�ȷ��֡ʱ����ΪTRUE

	frame_type = USART3RecvBuffer[USART3RecvBufStart];

	switch (frame_type) 		
	{
	case WRITE_CMD_FRAME:
	    /*����Ӧ��֡��д��ɹ�����д��ʧ��*/
		/*д��ֹͣС����д��С�������ٶȷ���д��С����ת����*/
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
		/* ���ض�ȡ��� */
		/* ��ȡ��������ȡ�ٶȣ���ȡλ��RFID*/
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
			/*ִ��д������֡*/
			if (last_cmd_type <= 0x06)
				g_done_flag = TRUE;
			break;
		case CANCEL:	
			/*ȡ��д������֡*/
			g_done_flag = FALSE;

			robotrate.fBotTranslationRateX = 0;	   /*��Ҫ������, Ӧ���Ǳ������ڵ�״̬*/
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
		/* �޷��� */
		switch (cmd_type)
		{
		case VERIFY:
			/*ִ��д������֡*/
			g_done_flag	= TRUE;
			break;
		case CANCEL:
			/*ȡ��д������֡*/
			g_done_flag = FALSE;
			robotrate.fBotTranslationRateX = 0;	   /*��Ҫ������*/
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


