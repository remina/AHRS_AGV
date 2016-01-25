/******************** ��Ȩ���� �人���ϻ����˿Ƽ����޹�˾ ������ **************
* �ļ���             : CAN.c
* ����               : ������
* �汾               : V1.0
* ����               : 9/13/2012
* ����               : CAN�м��ͷ�ļ�
* ����               ���ײ�CAN�����ķ�װ
*******************************************************************************/
#include "CAN.h"
                                                    
unsigned char CAN1SendQBoxHost=0;                         //�ڴ��ͷָ��                            
unsigned char CAN1SendQBoxTail=0;                         //�ڴ��βָ��
unsigned int  CAN1SendQFree=CAN1_SEND_MAX_BOX;   
unsigned char CAN1SendOVF=0;                              //CAN1��������������־
unsigned char CAN1RunningFlag=0;

CAN1SendTcb CAN1SendTCB[CAN1_SEND_MAX_BOX];

u8 CAN1QRecvBuffer[CAN1_RECV_MAX_Q];          //�����ڴ��    
u8 CAN1RecvOVF=0;                               //CAN2��������������־  
u32  CAN1Recv1Index=0x00;
u32  CAN1Recv1Count=0x00;
u8 CAN1RecvFlag=0;
u8 CAN1RecvPtrW, CAN1RecvPtrR;

u8 CAN1SendQBoxPre = 0x00;

u8 OSCAN1MemQ[OS_MEM_CAN1_MAX];        //�հ׻�����
OSMEMTcb* OSQCAN1Index;                             //�ڴ�������ָ��

//������
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
* �ļ���         : CAN1SendUpdate
* ����           : ���ṹ��������û�����ݻ�δ������ϣ���û�з��ͣ���������ͣ�
                   ��������ϣ��˳�
* ����           : ��
* ���           : ��
* ����           : ��
*******************************************************************************/
void CAN1SendUpdate(void)
{
	
    static unsigned int count=0; //CAN��������ָ��

	if(CAN1SendQFree==CAN1_SEND_MAX_BOX)
	{
		CAN1RunningFlag=0;
		OSMemDelete(OSQCAN1Index,CAN1SendTCB[CAN1SendQBoxTail].Index);
		return;
	}

	CAN1StopSendISR();
    //������ڵ��ڴ������ݻ�û�з�����ϣ��������ͣ�Num��8
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
			CAN1SendTCB[CAN1SendQBoxTail].Num = 0; //���ݷ����������
		}
 	}
	else if (CAN1SendQBoxTail != CAN1SendQBoxHost)
	{//һ�����Ϳ��Ѿ���������ˣ� CAN1SendQFree++, βָ���һ��ָ����һ�����Ϳ�
		OSMemDelete(OSQCAN1Index,CAN1SendTCB[CAN1SendQBoxTail].Index);
		if (++CAN1SendQBoxTail >= CAN1_SEND_MAX_BOX) CAN1SendQBoxTail = 0;
		if (++CAN1SendQFree >= CAN1_SEND_MAX_BOX) CAN1SendQFree = CAN1_SEND_MAX_BOX;
		count = 0;
		//CAN1SendQBoxTail����CAN1SendQBoxHost��ʱ��ͱ�־��鷢�ͽ����ˣ�����ֱ���˳�
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
* �ļ���         : CAN1WriteDataToBuffer
* ����           : ��鷢�ͻ������Ĵ�С�����ռ��㹻���������͵����ݷ��뵽���ͻ���
                   ����ȥ,������������
* ����           : buffer�����͵����ݵ�ָ�룬count�����͵����ݵ�����
* ���           : ��
* ����           : ����ȷ���뵽���ͻ�������ȥ�ˣ��ͷ���0x00     �����򷵻�0x01
*******************************************************************************/
u8 CAN1WriteDataToBuffer(u8 *buffer, u8 count, u32 ID, u8 type)
{
    u8 i = count;
    u8 err;
    /*�˴����Լ����źŵƻ��߹ر��ж�*/
	if (count == 0) return 0x01;

	CAN1StopSendISR();
	//�������count��������Ҫ���ٸ��ڴ��
	if (count % CAN1_SEND_MAX_Q) 
		count = count / CAN1_SEND_MAX_Q + 1;
	else
		count = count / CAN1_SEND_MAX_Q;
	//��Ҫcount�����ݿ�,����ڴ治�㣬ֱ�ӷ���
	if (CAN1SendQFree < count)
	{
		CAN1StartSendISR();
		return 0x01;
	}
	//���������ڴ��, CAN1SendQBoxHost����һ���ڴ������ż�һ

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
			//��Ҫһ���µ��ڴ���Ž����������ݣ����Ը���CAN1SendQBoxHost
			if (++CAN1SendQBoxHost >= CAN1_SEND_MAX_BOX) CAN1SendQBoxHost = 0;
			CAN1SendTCB[CAN1SendQBoxHost].Index = (u8 *)OSMemGet(OSQCAN1Index, &err);
			CAN1SendQFree--;
			count = 0;
		}
		buffer++;
		i--;
	}

	//�˴�����δ�����������ݣ�����ҲҪ�����һ���µ��ڴ����
	if (count != 0)
	{
		CAN1SendTCB[CAN1SendQBoxHost].ID=ID;
		CAN1SendTCB[CAN1SendQBoxHost].Num = count;
		CAN1SendTCB[CAN1SendQBoxHost].frame_type = type;
		CAN1SendQFree--;
		if (++CAN1SendQBoxHost >= CAN1_SEND_MAX_BOX) CAN1SendQBoxHost = 0;
	}
    
    //����ǵ�һ�Σ����������ͣ�������Ѿ�������û�������Ҫ��
    if(CAN1RunningFlag==0)
    {
	    CAN1RunningFlag=1; // 2012��9��17�գ��������ֵ��Ҫ����CAN1SendUpdate֮ǰ��
						   // 				 ��Ȼ����CAN1SendUpdate�����һֱ��
						   // 				 CAN1SendUpdateִ�����˲Ż�ִ��	CAN1SendUpdate = 1

        CAN1SendUpdate();
//		CAN1RunningFlag=1;       
    }
    /*�˴����Կ����źŵƻ��ߴ��ж�*/
    CAN1StartSendISR();
	//LED1(ON);
    return 0x00;
}


/*******************************************************************************
* �ļ���         : CAN1DispFun
* ����           : ��鷢�ͻ������Ĵ�С�����ռ��㹻���������͵����ݷ��뵽���ͻ���
                   ����ȥ,������������,��CAN1WriteDataToBuffer��ͬ���ǣ���������
                   ����������Ҫָ���ļ���С�ģ���͸������ṩ�˷���.
* ����           : buffer�����͵����ݵ�ָ��
* ���           : ��
* ����           : ����ȷ���뵽���ͻ�������ȥ�ˣ��ͷ���0x00     �����򷵻�0x01
*******************************************************************************/


/*******************************************************************************
* �ļ���           : CAN1RecvResetBufferIndex
* ����             : ��������ʱ�жϵ�ʱ�򣬽����յ�ָ����㣬���ҹرռ�鳬ʱ��ʱ��
* ����             : ��
* ���             : ��
* ����             : ��
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
* �ļ���         : CAN1RecvFun
* ����           : �����յ�������һ֡�����Ժ�Ĵ�����
* ����           : ptr���յ�������֡��ͷָ�룬���յ�������֡�����ݸ���
* ���           : ��
* ����           : ��
*******************************************************************************/
void CAN1RecvFun(unsigned char *ptr,unsigned int count)
{
    int i=0;  
    for(i=0;i<CAN1_RECV_MAX_Q;i++)CAN1QRecvBuffer[i]=0;
    
}    

/*******************************************************************************
* �ļ���         : CAN1RecvUpdate
* ����           : ������յ�һ������
* ����           : ��
* ���           : ��
* ����           : ��
*******************************************************************************/    
void CAN1RecvUpdate(CanRxMsg RxMessage)
{
	u8 i = 0;
	u8 num = RxMessage.DLC;
	for (i = 0; i < num; i++)
	{
		CAN1QRecvBuffer[CAN1RecvPtrW++] =  RxMessage.Data[i];
		if (CAN1RecvPtrW == CAN1RecvPtrR) //����������
		{
			CAN1RecvPtrR++;//���������ϵ�����	
		}		
	}
}
/*******************************************************************************
* �ļ���         : CAN1RecvData
* ����           : �����յ�������һ֡�����Ժ�Ĵ�����
* ����           : count��Ҫ���յ���һ֡�������ݵĸ�����flag��1������ʱ�ж�
                   0�رճ�ʱ�ж�
* ���           : ��
* ����           : ��
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


