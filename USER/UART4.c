/******************** ��Ȩ���� �人���ϻ����˿Ƽ����޹�˾ ������ **************
* �ļ���             : UART4.h
* ����               : ������
* �汾               : V1.0
* ����               : 9/13/2012
* ����               : ����4�м��ͷ�ļ�
* ����               ���ײ㴮��4�����ķ�װ,Ϊ����ͨѶ�ṩ�ӿ�				   				   	
*******************************************************************************/
#include "UART4.h"
#include "UART4Config.h"
#include "USART1.h"
#include "global.h"
#include "MotorControl.h"

u8 UART4SendQBoxHost;						//�����ڴ��ͷָ��							
u8 UART4SendQBoxTail;						//�����ڴ��βָ��
u32 UART4SendQFree;					     	//�����ڴ�������
u8 UART4RunningFlag=0;
typedef struct{
u8 Num;
u8 *Index;
}UART4SendTcb;
UART4SendTcb UART4SendTCB[UART4_SEND_MAX_BOX];

//u8 g_rfid_data_flag = FALSE;

//typedef struct{
//u8 Num;
//u8 RxData[12];
//}RFIDDATA;
//RFIDDATA RFIDData;
u8 RFIDData[5] = {0};
u8 g_rfid_num=0;
u8 g_rfid_num_last = 0;
u8 g_rfid_data_flag = FALSE;
u8 last_data[5] ={0};

#define RFIDNUM 20


u8 UART4RecvBuffer[UART4_RECV_MAX_Q]; //���ջ�����	
u8 UART4RecvOVF=0; 				            //USART1��������������־  
u32 Recv4Index=0x00;
u32 Recv4Count=0x06;
u8 UART4RecvFlag=0;
u8 UART4RecvPtrW, UART4RecvPtrR;

/////////////////////////////////////////////////////
u8 UART4RecvBufStart, UART4RecvBufEnd;
u8 UART4RecvState;
u8 UART4RecvFrameLen = 10;
#define NO_START_RCV         0
#define START_RCV            1
#define FIRST_FRAMEHEADER    0x55
#define SENCOND_FRAMEHEADER  0xAA
#define  MY_ADDR       0x1E
//int ucTranslateRate[3], ucAngle[3], ucRotateRate[3];
//u8 rateIndex = 0;				  
/////////////////////////////////////////////////////

u8 OSUART4MemQ[OS_MEM_UART4_MAX];  			//�հ��ڴ��

OSMEMTcb* OSQUART4Index;

//������
#define ERR_NO_SPACE	0xff

void UART4_Init(unsigned long baud)
{	
	u8 MemTestErr;
	UART4RecvPtrW = 0;
	UART4RecvPtrR = 0;
	UART4SendQBoxHost = 0;
	UART4SendQBoxTail = 0;
	UART4SendQFree = UART4_SEND_MAX_BOX;
	UART4_Configuration(baud);

	OSQUART4Index=(OSMEMTcb *)OSMemCreate(OSUART4MemQ,OS_MEM_UART4_BLK,OS_MEM_UART4_MAX/OS_MEM_UART4_BLK, &MemTestErr);
}

/*******************************************************************************
* �ļ���	  	 : USART1SendUpdate
* ����	         : ���ṹ��������û�����ݻ�δ������ϣ���û�з��ͣ���������ͣ�
				   ��������ϣ��˳�
* ����           : ��
* ���           : ��
* ����           : ��
*******************************************************************************/
void UART4SendUpdate(void)
{
	static unsigned char count=0;
	u8 *template;
	
	if(UART4SendQFree==UART4_SEND_MAX_BOX){return;}
	UART4StopSendISR();
	//������ڵ��ڴ������ݻ�û�з�����ϣ��������ͣ�Num��һ
	if((UART4SendTCB[UART4SendQBoxTail].Num)&&(UART4SendQBoxTail!=UART4SendQBoxHost))
	{
		UART4SendTCB[UART4SendQBoxTail].Num--;
		template = UART4SendTCB[UART4SendQBoxTail].Index+count;
		UART4SendByte(*template);
		count++;
	}
	//һ�����Ϳ��Ѿ���������� ��USART1SendQFree++,βָ���һ��ָ����һ�����Ϳ�
	else if(UART4SendQBoxTail!=UART4SendQBoxHost)
	{		
		OSMemDelete(OSQUART4Index,UART4SendTCB[UART4SendQBoxTail].Index);
		if(++UART4SendQBoxTail>=UART4_SEND_MAX_BOX)UART4SendQBoxTail=0;
		if(++UART4SendQFree>=UART4_SEND_MAX_BOX)UART4SendQFree=UART4_SEND_MAX_BOX;
		count=0;
		//UART4SendQBoxTail����UART4SendQBoxTail��ʱ��ͱ�־�ⷢ�ͽ����ˣ�����ֱ���˳�
		if((UART4SendQBoxTail!=UART4SendQBoxHost))
//		if((UART4SendTCB[UART4SendQBoxTail].Num)&&(UART4SendQBoxTail!=UART4SendQBoxHost))
		{
			UART4SendTCB[UART4SendQBoxTail].Num--;
			UART4SendByte(*(UART4SendTCB[UART4SendQBoxTail].Index+count));
			count++;
		}
		else
		{	
		//UART4SendQBoxTail����UART4SendQBoxTail��ʱ��ͱ�־�ⷢ�ͽ����ˣ�����ֱ���˳�
			UART4RunningFlag=0;
			UART4SendQFree=UART4_SEND_MAX_BOX;
			count=0;
		}	
	}
	//����ͷָ��һֱ��ָ��յķ��Ϳ�ģ�����UART4SendQBoxTail����UART4SendQBoxTail
	//��ʱ��ͱ�־�ⷢ�ͽ����ˣ�����ֱ���˳�
	else
	{
		UART4RunningFlag=0;
		UART4SendQFree=UART4_SEND_MAX_BOX;
		count=0;
	}
	UART4StartSendISR();	
}

/*******************************************************************************
* �ļ���	  	 : UART4RecvUpdate
* ����	         : 
* ����           : ��
* ���           : ��
* ����           : ��
*******************************************************************************/
void UART4RecvUpdate(void)
{
	UART4RecvBuffer[UART4RecvPtrW++] = UART4RecvByte();
//	USART1WriteDataToBuffer(UART4RecvBuffer + UART4RecvPtrW - 1, 1);
	if (UART4RecvPtrW == UART4RecvPtrR) //����������
	{
		UART4RecvPtrR++;//���������ϵ�����	
	}	
}

/*******************************************************************************
* �ļ���	  	 : USART1WriteDataToBuffer
* ����	         : ��鷢�ͻ������Ĵ�С�����ռ��㹻���������͵����ݷ��뵽���ͻ���
				   ����ȥ,������������
* ����           : buffer�����͵����ݵ�ָ�룬count�����͵����ݵ�����
* ���           : ��
* ����           : ����ȷ���뵽���ͻ�������ȥ�ˣ��ͷ���0x00	 �����򷵻�0x01
*******************************************************************************/
u8 UART4WriteDataToBuffer(u8 *buffer,u8 count)
{
	u8 i=count;
	u8 err;
	/*�˴����Լ����źŵƻ��߹ر��ж�*/
	if(count==0)return 0x01;
	UART4StopSendISR();
	/*�������count��������Ҫ���ٸ��ڴ��*/
	if(count%UART4_SEND_MAX_Q)count=count/UART4_SEND_MAX_Q+1;
	else count=count/UART4_SEND_MAX_Q;
	/*��Ҫcount�����ݿ�*/
	/*����ڴ治�㣬ֱ�ӷ���*/		 
	if(UART4SendQFree<count){UART4StartSendISR();return ERR_NO_SPACE;}
	//���������ڴ�飬USART1SendQBoxHost����һ���ڴ������ż�һ
	UART4SendTCB[UART4SendQBoxHost].Index=(u8 *)OSMemGet(OSQUART4Index,&err);
	if(UART4SendQBoxHost>=UART4_SEND_MAX_BOX)UART4SendQBoxHost=0;	
	count=0;
	while(i!='\0')										 
	{
		*(UART4SendTCB[UART4SendQBoxHost].Index+count)=*buffer;
		count++;
		if(count>=UART4_SEND_MAX_Q)
		{
			UART4SendTCB[UART4SendQBoxHost].Num=UART4_SEND_MAX_Q;
			//��Ҫһ���µ��ڴ���Ž����������ݣ����Ը���USART1SendQBoxHost
			if(++UART4SendQBoxHost>=UART4_SEND_MAX_BOX)UART4SendQBoxHost=0;
			//��Ҫһ���µ��ڴ���Ž�����������	
			UART4SendTCB[UART4SendQBoxHost].Index=(u8 *)OSMemGet(OSQUART4Index,&err);
			//�յķ���������һ 			
			UART4SendQFree--;
			count=0;
		}
		buffer++;
		i--;
	}
	//�˴�����δ�����������ݣ�����ҲҪ�����һ���µ��ڴ����
	if(count!=0)
	{
		UART4SendTCB[UART4SendQBoxHost].Num=count; 
		UART4SendQFree--;
		if(++UART4SendQBoxHost>=UART4_SEND_MAX_BOX)UART4SendQBoxHost=0;
	}
	//����ǵ�һ�Σ����������ͣ�������Ѿ�������û�������Ҫ��
	if(UART4RunningFlag==0)
	{

		UART4SendUpdate();
		UART4RunningFlag=1;
	}
	/*�˴����Կ����źŵƻ��ߴ��ж�*/
	UART4StartSendISR();
	return 0x00;
}

/*******************************************************************************
* �ļ���	  	 : USART1DispFun
* ����	         : ��鷢�ͻ������Ĵ�С�����ռ��㹻���������͵����ݷ��뵽���ͻ���
				   ����ȥ,������������,��USART1WriteDataToBuffer��ͬ���ǣ���������
				   ����������Ҫָ���ļ���С�ģ���͸������ṩ�˷���.
* ����           : buffer�����͵����ݵ�ָ��
* ���           : ��
* ����           : ����ȷ���뵽���ͻ�������ȥ�ˣ��ͷ���0x00	 �����򷵻�0x01
*******************************************************************************/
u8 UART4DispFun(u8 *buffer)
{
	u32 count=0;
	while(buffer[count]!='\0')count++;
	return(UART4WriteDataToBuffer(buffer,count));
}

u8 USART4CheckDataFrame(void)
{
	unsigned char i, j, k;
	u8 flag=0;
	u8 UART4RecvTempBuffer[10];
    while (UART4RecvPtrR != UART4RecvPtrW) 
    {
	    if (UART4RecvFlag == 0) 
		{
	      k = 0;
	      i = (UART4RecvPtrR - 1);
	      if (UART4RecvBuffer[i] == 0x02) 
		  {
	        k++;
	      }

		  i = (UART4RecvPtrR + 2*Recv4Count);
	      if (UART4RecvBuffer[i] == 0x0D) 
		  {
	        k++;
	      }
          i = (UART4RecvPtrR + 2*Recv4Count+1);
	      if (UART4RecvBuffer[i] == 0x0A) 
		  {
	        k++;
	      }
		  i = (UART4RecvPtrR + 2*Recv4Count+2);
	      if (UART4RecvBuffer[i] == 0x03) 
		  {
	        k++;
	      }
	
		  if (k == 4) 
		  {
	        //��ȡ��Ч���ݵ���ʼ��ַ��ĩβ��ַ��ĩβ��ַָ����������֡��֡β��У��ͣ�
	        UART4RecvFlag = 1;
			flag=1;
	        UART4RecvBufStart = UART4RecvPtrR;
	        UART4RecvBufEnd = (UART4RecvPtrR + 2*Recv4Count);
	      } 
		}  
		else 
		{
      		//��ʼ�������ݴ���
      		if(UART4RecvPtrR == UART4RecvBufEnd) 
	  		{
				UART4RecvFlag = 0;
		        for(i = 0, j = 0; i < 2*(Recv4Count-1); i=i+2) 
				{
//		          if(UART4RecvBuffer[UART4RecvBufStart+i]>=0x30&&UART4RecvBuffer[UART4RecvBufStart+i]<=0x39)  {UART4RecvBuffer[UART4RecvBufStart+i]-=0x30;}
//				  else if(UART4RecvBuffer[UART4RecvBufStart+i]>=0x41&&UART4RecvBuffer[UART4RecvBufStart+i]<=0x46)	 {UART4RecvBuffer[UART4RecvBufStart+i]=UART4RecvBuffer[UART4RecvBufStart+i]-0x41+10;}
//				  if(UART4RecvBuffer[UART4RecvBufStart+i+1]>=0x30&&UART4RecvBuffer[UART4RecvBufStart+i+1]<=0x39)  {UART4RecvBuffer[UART4RecvBufStart+i+1]-=0x30;}
//				  else if(UART4RecvBuffer[UART4RecvBufStart+i+1]>=0x41&&UART4RecvBuffer[UART4RecvBufStart+i+1]<=0x46)	 {UART4RecvBuffer[UART4RecvBufStart+i+1]=UART4RecvBuffer[UART4RecvBufStart+i+1]-0x41+10;}
//				  RFIDData[j++]=UART4RecvBuffer[UART4RecvBufStart+i]*16+UART4RecvBuffer[UART4RecvBufStart+i+1];

				  if(UART4RecvBuffer[UART4RecvBufStart+i]>=0x30&&UART4RecvBuffer[UART4RecvBufStart+i]<=0x39)  {UART4RecvTempBuffer[i]=UART4RecvBuffer[UART4RecvBufStart+i]-0x30;}
				  else if(UART4RecvBuffer[UART4RecvBufStart+i]>=0x41&&UART4RecvBuffer[UART4RecvBufStart+i]<=0x46)	 {UART4RecvTempBuffer[i]=UART4RecvBuffer[UART4RecvBufStart+i]-0x41+10;}
				  if(UART4RecvBuffer[UART4RecvBufStart+i+1]>=0x30&&UART4RecvBuffer[UART4RecvBufStart+i+1]<=0x39)  {UART4RecvTempBuffer[i+1]=UART4RecvBuffer[UART4RecvBufStart+i+1]-0x30;}
				  else if(UART4RecvBuffer[UART4RecvBufStart+i+1]>=0x41&&UART4RecvBuffer[UART4RecvBufStart+i+1]<=0x46)	 {UART4RecvTempBuffer[i+1]=UART4RecvBuffer[UART4RecvBufStart+i+1]-0x41+10;}
				  RFIDData[j++]=UART4RecvTempBuffer[i]*16+UART4RecvTempBuffer[i+1];

		        } 
				flag = 1;

			}
		}
		UART4RecvPtrR++;
	}
	return flag;
}


/*02 30 31 	30 42 	44 38 	36 35 	44 35 	31 45 0D 0A 03 
02 30 31 	30 42 	44 37 	39 38 	33 31 	41 43 0D 0A 03
02 30 31 	30 42 	44 37 	46 35 	32 30 	46 38 0D 0A 03 
02 30 31 	30 42 	44 37 	39 45 	32 30 	41 31 0D 0A 03

02 30 31 	30 42 	44 37 	41 31 	43 37 	34 42 0D 0A 03
02 30 31 	30 42 	44 37 	42 33 	37 45 	31 34 0D 0A 03
02 30 31 	30 42 	44 37 	43 44 	37 42 	32 42 0D 0A 03
02 30 31 	30 42 	44 37 	45 46 	39 36 	36 38 0D 0A 03

02 30 31 	30 42 	44 37 	41 41 	32 34 	42 31 0D 0A 03
02 30 31 	30 42 	44 37 	45 44 	45 38 	42 38 0D 0A 03
02 30 31 	30 42 	44 38 	33 43 	30 38 	32 38 0D 0A 03
02 30 31 	30 42 	44 37 	43 43 	43 32 	37 31 0D 0A 03

02 30 31 	30 42 	44 37 	42 30 	31 44 	42 30 0D 0A 03
02 30 31 	30 42 	44 37 	45 44 	46 31 	43 31 0D 0A 03
02 30 31 	30 43 	31 31 	34 35 	34 39 	41 43 0D 0A 03	*/


/*u8 RFIDCardData[RFIDNUM][5]={{0,0,0,0,0},{0x51,0x00,0x33,0x90,0xD5},{0x51,0x00,0x33,0x43,0x50},{0x51,0x00,0x33,0x61,0xB9},{0x51,0x00,0x33,0x7F,0xB1},
   {0x51,0x00,0x33,0x43,0xFD},{0x51,0x00,0x32,0x1F,0xDE},{0x51,0x00,0x32,0x44,0xC9},{0x51,0x00,0x33,0x2F,0xC9},
   {0x4F,0x00,0x61,0x91,0xCC},{0x4F,0x00,0x61,0xBD,0xE2}}; */

/*����*/
//u8 RFIDCardData[RFIDNUM][5]={{0,0,0,0,0},{0x01,0x0B,0xD8,0x4B,0xFE},{0x01,0x0B,0xD7,0xC3,0x89},{0x01,0x0B,0xD7,0xF4,0x5B},{0x01,0x0B,0xD7,0xB7,0xDF},
//   {0x01,0x0B,0xD7,0xCE,0x5B},{0x01,0x0B,0xD8,0x5F,0x9A},{0x01,0x0B,0xD8,0x5A,0xF7},{0x01,0x0B,0xD8,0x41,0x45},
//   {0x01,0x0B,0xD7,0xEB,0xBE},{0x01,0x0B,0xD7,0xF6,0x38},{0x01,0x0B,0xD7,0xFB,0xF3},{0x01,0x0B,0xD8,0x21,0xB5},
//   {0x01,0x0B,0xD7,0xD5,0xC2},{0x01,0x0B,0xD7,0x8A,0x69},{0x01,0x0B,0xD8,0x03,0x29},{0x01,0x0B,0xD7,0xAA,0x24},{0x01,0x0B,0xD8,0x09,0x1E}};
//   /*{0x01,0x0B,0xD8,0x09,0x1E},{0x01,0x0B,0xD7,0x85,0x08},{0x01,0x0B,0xD7,0xAD,0xAB},{0x01,0x0B,0xD7,0xF6,0x38},
//   {0x01,0x0B,0xD8,0x4B,0xFE},{0x01,0x0B,0xD7,0xAB,0xEB},{0x01,0x0B,0xD7,0xD5,0xE6},{0x01,0x0B,0xD7,0xFC,0xDF},  
//   {0x01,0x0B,0xD7,0xAF,0xBD},{0x01,0x0B,0xD7,0xCC,0xC2},{0x01,0x0B,0xD7,0xB4,0x12},{0x01,0x0B,0xD7,0xC0,0xD8},
//   {0x01,0x0B,0xD7,0xA5,0x11},{0x01,0x0B,0xD7,0xB0,0x1D},{0x01,0x0B,0xD8,0x3C,0x08},{0x01,0x0B,0xD7,0xBD,0xC9}
//   }; */

/*�人*/
/*u8 RFIDCardData[RFIDNUM][5]={{0,0,0,0,0},
   {0x3d,0 ,0xa6, 0xdc, 0x30},	 //1
   {0x3d,0 ,0xa6 ,0xf1 ,0x5c},	 //2
   {0x3d,0 ,0xa6 ,0xec ,0x8e},	 //3
   {0x3d,0 ,0xa6 ,0xe6 ,0xb8},	 //4
   {0x3d,0 ,0xa6 ,0xfc ,0xc4},	 //5
   {0x3d,0 ,0xa6 ,0xf4 ,0x72},	 //6
   {0x3d,0 ,0xa6 ,0xdc ,0xa0},	 //7
   {0x3d,0 ,0xa6 ,0xdc ,0xf8},	 //8
   {0x3d,0 ,0xa6 ,0xe7 ,0xcf},	 //9
   {0x3d,0 ,0xa6 ,0xdb ,0x7d},	 //10
   {0x3d,0 ,0xa7 ,0x01 ,0x2f},	 //11
   {0x3d,0 ,0xa6 ,0xdd ,0x86},	 //12
   {0x3d,0 ,0xa6 ,0xdf ,0x73},	 //13
   {0x3d,0 ,0xa6 ,0xdf ,0x76},	 //14
   {0x3d,0 ,0xa6 ,0xf7 ,0x94},	 //15
   {0x3d,0 ,0xa7 ,0x01 ,0x70},	 //16
   {0,0,0,0,0}}; */
   

///*����  new*/
u8 RFIDCardData[RFIDNUM][5]={{0,0,0,0,0},
   {0x3d,0 ,0xa7, 0x01, 0x65},
   {0x3d,0 ,0xa6 ,0xd9 ,0xd9},
   {0x3d,0 ,0xa6 ,0xf5 ,0xf3},
   {0x3d,0 ,0xa6 ,0xf9 ,0xf2},
   {0x3d,0 ,0xa6 ,0xec ,0x9c},
   {0x3d,0 ,0xa6 ,0xf0 ,0xd5},
   {0x3d,0 ,0xa6 ,0xd9 ,0x3f},
   {0x3d,0 ,0xa6 ,0xf3 ,0xda},
   {0x3d,0 ,0xa7 ,0x06 ,0x0c},
   {0x3d,0 ,0xa6 ,0xe3 ,0x23},
   {0x3d,0 ,0xa6 ,0xdb ,0xe9},
   {0x3d,0 ,0xa6 ,0xe1 ,0xca},
   {0x3d,0 ,0xa6 ,0xff ,0xb9},
   {0x3d,0 ,0xa6 ,0xf0 ,0x72},
   {0x3d,0 ,0xa6 ,0xe7 ,0x47},
   {0x3d,0 ,0xa6 ,0xdd ,0xcb},
   {0,0,0,0,0}};	
 

void GetRFIDNum(u8 RFIDData[])
{
   int i;
   for(i=1;i<20;i++)
   {
	  if(RFIDData[0]==RFIDCardData[i][0]&&RFIDData[1]==RFIDCardData[i][1]&&RFIDData[2]==RFIDCardData[i][2]&&RFIDData[3]==RFIDCardData[i][3]&&RFIDData[4]==RFIDCardData[i][4])
	  {
	    break;
	  }
   }
   g_rfid_num_last = g_rfid_num;

   if (i < 1 || i >16)
   	  return;
   else
      g_rfid_num=i;
}

void FRIDCheck()
{  
   u8 frame_data_len = 0;
   u8 frame_data[255] = {0};
   u8 flag = 0;
   static u8 i = 0;
//   static u8 last_data[5];
  if(UART4RecvPtrR != UART4RecvPtrW) 
  {
     if(USART4CheckDataFrame() != 0) 
	{
		g_rfid_data_flag = TRUE;

//		frame_data_len = make_frame(0x02, 0x04, RFIDData, 5, frame_data);
//		USART3WriteDataToBuffer(frame_data, frame_data_len);
 
		if (memcmp(last_data, RFIDData, 5) == 0)
		{
//			USART3WriteDataToBuffer(frame_data, frame_data_len);
//			i++;			
		}
		else 
		{
			GetRFIDNum(RFIDData);

			/*����RFID����*/
			frame_data_len = make_frame(0x02, 0x04, &g_rfid_num, 1 ,frame_data);
			USART3WriteDataToBuffer(frame_data, frame_data_len);

			/** test **/
//			USART1WriteDataToBuffer(frame_data, frame_data_len);
			/** test **/

			memcpy(last_data, RFIDData, 5);
//			stop_base();
//			DelayMs(2000);
		}
						
    } 		    
  }
}
void FRIDCpyData()
{
		//memcpy(RFIDData, RFIDCardData, 5);
		u8 i;
		for(i=0;i<5;i++)
		{
			RFIDData[i]=RFIDCardData[g_rfid_num][i];
		}
		memcpy(last_data, RFIDData, 5);
}
