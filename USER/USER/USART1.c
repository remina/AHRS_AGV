/******************** ��Ȩ���� �人���ϻ����˿Ƽ����޹�˾ ������ **************
* �ļ���             : USART1.c
* ����               : ������
* �汾               : V1.0
* ����               : 9/13/2012
* ����               : ����1�м��
* ����               ���ײ㴮��1�����ķ�װ				   				   	
*******************************************************************************/
#include "USART1.h"
#include "string.h"
#include "global.h"

u8 USART1SendQBoxHost;						//�����ڴ��ͷָ��							
u8 USART1SendQBoxTail;						//�����ڴ��βָ��
u8 USART1SendQFree;					    	//�����ڴ�������
u8 USART1RunningFlag=0;
u8 USART1RecvPtrW, USART1RecvPtrR;
u8 UART1RecvBufStart, UART1RecvBufEnd;

typedef struct{
u8 Num;
u8 *Index;
}USART1SendTcb;                          //task control block
USART1SendTcb USART1SendTCB[USART1_SEND_MAX_BOX];


u8 USART1RecvBuffer[USART1_RECV_MAX_Q]; //���ջ�����	
u8 USART1tempBuffer[28]={0};
u8 USART1RecvOVF=0; 				        //USART1��������������־  
u32 Recv1Index=0x00;
u32 Recv1Count=0x08;
u8 UART1RecvFlag=0;


u8 OSUSART1MemQ[OS_MEM_USART1_MAX];  			//�հ��ڴ��

OSMEMTcb* OSQUSART1Index;

void USART1DMAUpdate(void);
//������
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
* �ļ���	  	 : USART1SendUpdate
* ����	         : ���ṹ��������û�����ݻ�δ������ϣ���û�з��ͣ���������ͣ�
				   ��������ϣ��˳�
* ����           : ��
* ���           : ��
* ����           : ��
*******************************************************************************/
void USART1SendUpdate(void)
{
	static unsigned char count=0;
	
	if(USART1SendQFree==USART1_SEND_MAX_BOX){return;}
	USART1StopSendISR();
	//������ڵ��ڴ������ݻ�û�з�����ϣ��������ͣ�Num��һ
	if((USART1SendTCB[USART1SendQBoxTail].Num)&&(USART1SendQBoxTail!=USART1SendQBoxHost))
	{
		USART1SendTCB[USART1SendQBoxTail].Num--;
		USART1SendByte(*(USART1SendTCB[USART1SendQBoxTail].Index+count));
		count++;
	}
	//һ�����Ϳ��Ѿ���������� ��USART1SendQFree++,βָ���һ��ָ����һ�����Ϳ�
	else if(USART1SendQBoxTail!=USART1SendQBoxHost)
	{		
		OSMemDelete(OSQUSART1Index,USART1SendTCB[USART1SendQBoxTail].Index);
		if(++USART1SendQBoxTail>=USART1_SEND_MAX_BOX)USART1SendQBoxTail=0;
		if(++USART1SendQFree>=USART1_SEND_MAX_BOX)USART1SendQFree=USART1_SEND_MAX_BOX;
		count=0;
		//USART1SendQBoxTail����USART1SendQBoxTail��ʱ��ͱ�־�ⷢ�ͽ����ˣ�����ֱ���˳�
		if((USART1SendQBoxTail!=USART1SendQBoxHost))
//		if((USART1SendTCB[USART1SendQBoxTail].Num)&&(USART1SendQBoxTail!=USART1SendQBoxHost))
		{
			USART1SendTCB[USART1SendQBoxTail].Num--;
			USART1SendByte(*(USART1SendTCB[USART1SendQBoxTail].Index+count));
			count++;
		}
		else
		{	
		//USART1SendQBoxTail����USART1SendQBoxTail��ʱ��ͱ�־�ⷢ�ͽ����ˣ�����ֱ���˳�
			USART1RunningFlag=0;
			USART1SendQFree=USART1_SEND_MAX_BOX;
			count=0;
		}	
	}
	//����ͷָ��һֱ��ָ��յķ��Ϳ�ģ�����USART1SendQBoxTail����USART1SendQBoxTail
	//��ʱ��ͱ�־�ⷢ�ͽ����ˣ�����ֱ���˳�
	else
	{
		USART1RunningFlag=0;
		USART1SendQFree=USART1_SEND_MAX_BOX;
		count=0;
	}
	USART1StartSendISR();	
}

/*******************************************************************************
* �ļ���	  	 : USART1RecvUpdate
* ����	         : 
* ����           : ��
* ���           : ��
* ����           : ��
*******************************************************************************/
void USART1RecvUpdate(void)
{
	USART1RecvBuffer[USART1RecvPtrW++] = USART1RecvByte();
	if (USART1RecvPtrW == USART1RecvPtrR) //����������
	{
		USART1RecvPtrR++;//���������ϵ�����	
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
u8 USART1WriteDataToBuffer(u8 *buffer,u8 count)
{
	u8 i=count;
	u8 err;
	/*�˴����Լ����źŵƻ��߹ر��ж�*/
	if(count==0)return 0x01;
	USART1StopSendISR();
	/*�������count��������Ҫ���ٸ��ڴ��*/
	if(count%USART1_SEND_MAX_Q)count=count/USART1_SEND_MAX_Q+1;
	else count=count/USART1_SEND_MAX_Q;
	/*��Ҫcount�����ݿ�*/
	/*����ڴ治�㣬ֱ�ӷ���*/		 
	if(USART1SendQFree<count){USART1StartSendISR();return ERR_NO_SPACE;}
	//���������ڴ�飬USART1SendQBoxHost����һ���ڴ������ż�һ
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
			//��Ҫһ���µ��ڴ���Ž����������ݣ����Ը���USART1SendQBoxHost
			if(++USART1SendQBoxHost>=USART1_SEND_MAX_BOX)USART1SendQBoxHost=0;
			//��Ҫһ���µ��ڴ���Ž�����������	
			USART1SendTCB[USART1SendQBoxHost].Index=(u8 *)OSMemGet(OSQUSART1Index,&err);
			//�յķ���������һ 			
			USART1SendQFree--;
			count=0;
		}
		buffer++;
		i--;
	}
	//�˴�����δ�����������ݣ�����ҲҪ�����һ���µ��ڴ����
	if(count!=0)
	{
		USART1SendTCB[USART1SendQBoxHost].Num=count; 
		USART1SendQFree--;
		if(++USART1SendQBoxHost>=USART1_SEND_MAX_BOX)USART1SendQBoxHost=0;
	}
	//����ǵ�һ�Σ����������ͣ�������Ѿ�������û�������Ҫ��
	if(USART1RunningFlag==0)
	{
#if	  	DMA_MODE
		USART1DMAConfig(USART1SendTCB[USART1SendQBoxTail].Index,USART1SendTCB[USART1SendQBoxTail].Num);
#else	
		USART1SendUpdate();
#endif		
		USART1RunningFlag=1;
	}
	/*�˴����Կ����źŵƻ��ߴ��ж�*/
	USART1StartSendISR();
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
u8 USART1DispFun(u8 *buffer)
{
	u32 count=0;
	while(buffer[count]!='\0') count++;//count how many data shoud be put into buffer ,then transmitting
	//count=36;
	return(USART1WriteDataToBuffer(buffer,count));
}

/*******************************************************************************
* �ļ���	  	 : USART1DMAUpdate.c
* ����	         : USART_DMA����������
* ��ֲ����		 : �м�㺯��
* ����           : ��
* ���           : ��
* ����           : ��
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
* �ļ���	  	 : USART1GetRadarData.c
* ����	         : ���ջ�ȡ�����״������  ����ԭʼ������
* ��ֲ����		 : �м�㺯��
* ����           : ��
* ���           : ��
* ����           : ��
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
	        //��ȡ��Ч���ݵ���ʼ��ַ��ĩβ��ַ��ĩβ��ַָ����������֡��֡β��У��ͣ�
	        UART1RecvFlag = 1;
	        UART1RecvBufStart = USART1RecvPtrR;
	        UART1RecvBufEnd = (USART1RecvPtrR + 2*Recv1Count);
	      } 
		}  
		else 
		{
      		//��ʼ�������ݴ���
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

