/******************** 版权所有 武汉汉迪机器人科技有限公司 杜骁释 **************
* 文件名             : CAN1Config.c
* 作者               : 杜骁释
* 版本               : V1.0
* 日期               : 9/13/2012
* 描述               : CAN底层驱动
* 功能               ：				   				   	
*******************************************************************************/
#include "CAN1Config.h"
#include "MotorControl.h"
#include "odometry.h"


/*******************************************************************************
* 文件名         : CAN1_Configuration
* 描述           : 配置CAN1，baud:CAN1波特率
* 输入           : 无
* 输出           : 无
* 返回           : 无
*******************************************************************************/
void CAN1_Configuration(void)
{
	CAN_InitTypeDef   CAN_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

    //#ifdef VECT_TAB_RAM
	/* Set the Vector Table base location at 0x20000000 */
	//NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
	//#else /* VECT_TAB_FLASH */
	/* Set the Vector Table base location at 0x08000000 */
	//NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
	//#endif
	/* CAN-RX*/
	NVIC_InitStructure.NVIC_IRQChannel=USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* CAN-TX*/
	NVIC_InitStructure.NVIC_IRQChannel=USB_HP_CAN1_TX_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* CAN2
	NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_Init(&NVIC_InitStructure);*/
	
	
	/* CAN register init */
  	CAN_DeInit(CAN1);
	//CAN_DeInit(CAN2);

	/* 使能外设CAN,AFIO */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	/* PB8->CANRX */
	GPIO_InitStructure.GPIO_Pin   =  GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed =  GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  =  GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* PB9->CANTX */
	GPIO_InitStructure.GPIO_Pin	  =  GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed =  GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  =  GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* 端口重映射到PB8,PB9 */
 	GPIO_PinRemapConfig(GPIO_Remap1_CAN1, ENABLE); 

  	CAN_StructInit(&CAN_InitStructure);
	/* CAN cell init */
  	CAN_InitStructure.CAN_TTCM=DISABLE;
  	CAN_InitStructure.CAN_ABOM=DISABLE;
  	CAN_InitStructure.CAN_AWUM=DISABLE;
  	CAN_InitStructure.CAN_NART=ENABLE; //DISABLE;
  	CAN_InitStructure.CAN_RFLM=DISABLE;
  	CAN_InitStructure.CAN_TXFP=DISABLE;
  	CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;//CAN_Mode_LoopBack
  	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;
  	CAN_InitStructure.CAN_BS1=CAN_BS1_2tq;//
  	CAN_InitStructure.CAN_BS2=CAN_BS2_3tq;//
  	CAN_InitStructure.CAN_Prescaler=12;//设置36/4/(1+3+5)=1Mhz波特率  36/(1+2+3)/12=0.5Mbps
  	CAN_Init(CAN1, &CAN_InitStructure);
	//CAN_Init(CAN2, &CAN_InitStructure);

	/* CAN filter init */
	//CAN_FilterMaskId寄存器的排列顺序是STDID[10:0]、EXTID[17:0]、IDE和RTR位
  	CAN_FilterInitStructure.CAN_FilterNumber=0;	                       //激活过滤器组0
  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;      //
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;	   //设置32位宽

	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;			//设置屏蔽位为0，普通标识符模式
  	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;				//偏移值，左移五位，可以快捷设置相应位
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000; 		//设置过滤器模式，低三位的数据不用过滤器
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;

  	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=0;
  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
  	CAN_FilterInit(&CAN_FilterInitStructure);

	/* CAN2 filter init 
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 1; //FIFO1
    CAN_FilterInitStructure.CAN_FilterNumber = 14;   //过滤器
    CAN_FilterInit(&CAN_FilterInitStructure); */
 
	CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
	CAN_ClearITPendingBit(CAN1, CAN_IT_RQCP0);
	CAN_ITConfig(CAN1, CAN_IT_RQCP0|CAN_IT_FMP0, ENABLE);
	//CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
	//CAN_ITConfig(CAN1, CAN_IT_TME, ENABLE);
	//CAN_ITConfig(CAN1, CAN_IT_B0F, ENABLE);
	/* IT Configuration for CAN1  
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE); 
    CAN_ITConfig(CAN1, CAN_IT_FMP1, ENABLE);
    // IT Configuration for CAN2 
    CAN_ITConfig(CAN2, CAN_IT_FMP0, ENABLE);
    CAN_ITConfig(CAN2, CAN_IT_FMP1, ENABLE); */
}

/*******************************************************************************
* 文件名         : CAN1发送函数
* 描述           : temp:CAN1发送的数据
* 输入           : 无
* 输出           : 无
* 返回           : 无
*******************************************************************************/
void CAN1SendByte(CAN1SendTcb tcb, u8 num, u8 index)
{
	CanTxMsg  TxMessage;	
	u8 i;

//	TxMessage.StdId	  =  (u16)tcb.ID;
	TxMessage.ExtId   =  tcb.ID; 
//	TxMessage.IDE     =  CAN_ID_STD;	 //消息标识符类型  
	TxMessage.IDE     =  CAN_ID_EXT;
//	TxMessage.RTR  =  CAN_RTR_DATA;	 //消息的帧类型
	TxMessage.RTR = tcb.frame_type; // 帧类型

	TxMessage.DLC     =  num;            //0~0x08    消息的帧长度
	for(i=0;i<num;i++) TxMessage.Data[i] =  *((tcb).Index+index+i);	
	CAN_Transmit(CAN1, &TxMessage);  
}



void CANSendMessage(CAN_msg *msg)
{
	CanTxMsg tx_msg;
	u8 i;

	if (msg->format == 0)
	{
		tx_msg.StdId = msg->id;
		tx_msg.IDE = CAN_ID_STD;
	}						
	else
		tx_msg.ExtId = msg->id;
		tx_msg.IDE = CAN_ID_EXT;

	if (msg->type == 0)
		tx_msg.RTR = CAN_RTR_DATA;
	else
		tx_msg.RTR = CAN_RTR_REMOTE;

	tx_msg.DLC = msg->len;
	
	for (i = 0; i < msg->len; i++)
		tx_msg.Data[i] = msg->data[i];
	CAN_Transmit(CAN1, &tx_msg);
}

/*******************************************************************************
* 文件名         : CAN1_IRQHandler
* 描述           : CAN1_IRQHandler（CAN1发送）中断函数通道
* 输入           : 无
* 输出           : 无
* 返回           : 无
*******************************************************************************/
void USB_HP_CAN1_TX_IRQHandler(void)
{
	
//	static u8 flag=0;
//	if(flag==0){flag=1;return;}
	if(CAN_GetITStatus(CAN1, CAN_IT_RQCP0)==SET)
	{
		CAN_ClearITPendingBit(CAN1, CAN_IT_RQCP0);
		CAN1SendUpdate();
	}
}

//u8 canrecv[24]={0};
int g_RcvWheelSpeed[4]={0,0,0,0};
int g_PreRcvWheelSpeed[4]={0,0,0,0};   //上一次的值
#define CON_COEFFICIENT  4.2 
/*******************************************************************************
* 文件名         : CAN1_IRQHandler
* 描述           : CAN1_IRQHandler（CAN1接收）中断函数通道
* 输入           : 无
* 输出           : 无
* 返回           : 无
*******************************************************************************/
typedef struct {
	u32	param1;
	u32 param2;
}U32Params;
void USB_LP_CAN1_RX0_IRQHandler(void)
{
	CanRxMsg rx_msg;

	U32Params *ack;
   	s32 speed;
//	s64 pos;
	CAN_Receive(CAN1, CAN_FIFO0, &rx_msg);	 
	
//	USART1WriteDataToBuffer(&rx_msg.RTR, 1);
	if (rx_msg.RTR == CAN_RTR_DATA)
	{
		switch (GET_PROP(rx_msg.ExtId))
		{
		case COMMON_COMMON_ACK:
//			ack = (U32Params*)rx_msg.Data;
////			USART1WriteDataToBuffer(&(ack->param1), 4);
//			if (ack->param1 == BUILD_ID(0, 1, 771))
////				USART1WriteDataToBuffer(&(ack->param1), 4);
//			else if (ack->param1 == BUILD_ID(0, 1, 772))
////				USART1WriteDataToBuffer(&(ack->param1), 4);
//			else if (ack->param1 == BUILD_ID(0, 1, 768))
////				USART1WriteDataToBuffer(&(ack->param1), 4);
			break;
		case SERVO_STATUS_NOWPOSITION:
		    switch (GET_DEV_ID(rx_msg.ExtId))
			{
			case 1:
				encoder_data[WHEEL_LF] = *((s64*)rx_msg.Data);
				break;
			case 2:
				encoder_data[WHEEL_RF] = *((s64*)rx_msg.Data);
				break;
			case 3:
				encoder_data[WHEEL_LB] = *((s64*)rx_msg.Data);
				break;
			case 4:
				encoder_data[WHEEL_RB] = *((s64*)rx_msg.Data);
				break;
			}
			break;
		case SERVO_STATUS_NOWSPEED:
			switch (GET_DEV_ID(rx_msg.ExtId))
			{
			case 1:
				speed = *((s32*)rx_msg.Data);
				feedback_wheelspeed.fWheelSpeed_LF = speed;	
				break;
			case 2:
				speed = *((s32*)rx_msg.Data);
				feedback_wheelspeed.fWheelSpeed_RF = -speed;
				break;
			case 3:
				speed = *((s32*)rx_msg.Data);
				feedback_wheelspeed.fWheelSpeed_LB = speed;
				break;
			case 4:
				speed = *((s32*)rx_msg.Data);
				feedback_wheelspeed.fWheelSpeed_RB = -speed;
				break;
			default:
				break;
			}
			break;
		default:
			break;
		}		
	}

	CAN_FIFORelease(CAN1,CAN_FIFO0);	   //释放 FIFO0
}


