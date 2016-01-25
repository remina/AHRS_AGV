/******************** 版权所有 武汉汉迪机器人科技有限公司 杜骁释 **************
* 文件名             : USART1Config.c
* 作者               : 杜骁释
* 版本               : V1.0
* 日期               : 9/13/2012
* 描述               : USART1驱动层
* 移植步骤		 	 :（1）配置函数（管脚，时钟等）
				      （2）控制的参数（极性，波特率，位数，校验位等）
				      （3）中断函数
* 功能               ：配置串口管脚时钟，控制参数以及中断				   				   	
*******************************************************************************/
#include "USART1Config.h"

void USART1DMAConfiguration(u32 TxBuffer1,u16 num);

/*******************************************************************************
* 文件名	  	 : USART1NVIC_Configuration
* 描述	         : USART1DMA中断通道4配置
* 输入           : 无
* 输出           : 无
* 返回           : 无
*******************************************************************************/
void USART1NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);   
	/* 配置串口1的中断，中断优先级别为0，响应级别为3 */
	NVIC_InitStructure.NVIC_IRQChannel=USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=3;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
/*******************************************************************************
* 文件名	  	 : USART1配置
* 描述	         : baud:USART1波特率
* 输入           : 无
* 输出           : 无
* 返回           : 无
*******************************************************************************/
void USART1_Configuration(unsigned long baud)
{	
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_ClockInitTypeDef  USART_ClockInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA| RCC_APB2Periph_AFIO, ENABLE);	
	//RCC_APB2PeriphClockCmd(USART1_GPIO_CLK |  RCC_APB2Periph_AFIO, ENABLE);
	/* 配置 USART1 参数：115200波特率，一位停止位，八位数据位，无硬件控制 */
	//RCC_APB1PeriphClockCmd(USART1,ENABLE);
 	//GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);
	
	/* 配置 USART1 Tx (PA.09) as alternate function push-pull */
  	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;
  	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
  	GPIO_Init(GPIOA,&GPIO_InitStructure);

	/* 配置 USART1 Rx (PA.10) as input floating */
  	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;
  	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;
  	GPIO_Init(GPIOA,&GPIO_InitStructure);

	USART_DeInit(USART1);
	USART_StructInit(&USART_InitStructure);
	USART_InitStructure.USART_BaudRate=baud;
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;
	USART_InitStructure.USART_StopBits=USART_StopBits_1;
	USART_InitStructure.USART_Parity=USART_Parity_No;
	USART_InitStructure.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None; 
	USART_Init(USART1,&USART_InitStructure);

#if	DMA_MODE
#else
	USART_ITConfig(USART1,USART_IT_TC,ENABLE);	 
	USART1NVIC_Configuration();
#endif
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
	USART_Cmd(USART1,ENABLE);
}

/*******************************************************************************
* 文件名	  	 : USART1_IRQHandler
* 描述	         : USART1_IRQHandler（USART1发送）中断函数通道
* 输入           : 无
* 输出           : 无
* 返回           : 无
*******************************************************************************/
void USART1_IRQHandler(void)
{
	static u8 Flag=1;
	if(USART_GetFlagStatus(USART1,USART_FLAG_RXNE)==SET)	  //查询接收
	//if(USART_GetFlagStatus(USART1,USART_IT_RXNE)!=SET)		  //中断方式
	{   
		USART_ClearFlag(USART1,USART_FLAG_RXNE);//TCIE,TE,RE
		if(Flag)
		{			
			USART1RecvUpdate();
		}
	}
	if(USART_GetFlagStatus(USART1,USART_FLAG_TC)==SET)
	{
	    USART_ClearFlag(USART1,USART_FLAG_TC);//TCIE,TE,RE
		if(Flag)
		{
			USART1SendUpdate();
		}
	}
	Flag=1;		 
}

/*******************************************************************************
* 文件名	  	 : USART1DMAConfiguration
* 描述	         : 开启接收中断
* 输入           : 无
* 输出           : 无
* 返回           : 无
*******************************************************************************/
#define USART1_DR_Base  0x40013804

void USART1DMAConfig(u32 TxBuffer1,u16 num)
{
    DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
    /* DMA1 Channel4 (triggered by USART1 Tx event) Config */

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    DMA_DeInit(DMA1_Channel4);
    DMA_InitStructure.DMA_PeripheralBaseAddr = USART1_DR_Base; //USART地址
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)TxBuffer1;	   //
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = num;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;    
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel4, &DMA_InitStructure);
	
	DMA_ITConfig(DMA1_Channel4,DMA_IT_TC,ENABLE);
	
	//配置DMA发送中断，优先级别为0，响应级别也为0
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	DMA_Cmd(DMA1_Channel4, ENABLE);
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
	USART_Cmd(USART1,ENABLE);
}

/*******************************************************************************
* 文件名	  	 : DMA1_Channel4_IRQHandler
* 描述	         : DMA1_Channel4_IRQHandler（USART1发送）DMA函数通道
* 输入           : 无
* 输出           : 无
* 返回           : 无
*******************************************************************************/
void DMA1_Channel4_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_IT_TC4)==SET)
	{
	    DMA_ClearFlag(DMA1_IT_TC4);//TCIE,TE,RE			
		USART1DMAUpdate();
	}
}




	 


