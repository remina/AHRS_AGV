/******************** 版权所有 武汉汉迪机器人科技有限公司 杜骁释 **************
* 文件名             : USART2.c
* 作者               : 杜骁释
* 版本               : V1.0
* 日期               : 9/13/2012
* 描述               : USART1驱动层
* 移植步骤		 	 :（1）配置函数（管脚，时钟等）
				      （2）控制的参数（极性，波特率，位数，校验位等）
				      （3）中断函数
* 功能               ：配置串口管脚时钟，控制参数以及中断				   				   	
*******************************************************************************/
#include "USART2Config.h"



void USART2NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);   
	/* 配置串口2的中断，中断优先级别为0，响应级别为6 */
	NVIC_InitStructure.NVIC_IRQChannel=USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=6;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
/*******************************************************************************
* 文件名	  	 : USART2配置
* 描述	         : baud:USART1波特率
* 输入           : 无
* 输出           : 无
* 返回           : 无
*******************************************************************************/
void USART2_Configuration(unsigned long baud)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

   	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	////RCC_APB2PeriphClockCmd(RCC_APB1Periph_USART2|RCC_APB2Periph_GPIOA| RCC_APB2Periph_AFIO, ENABLE);


	//NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);   
	// 配置串口2的中断，中断优先级别为0，响应级别为6 
	NVIC_InitStructure.NVIC_IRQChannel=USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=6;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* USART2 GPIO config */
    /* Configure USART2 Tx (PA.02) as alternate function push-pull */
	//GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	    
	/* Configure USART2 Rx (PA.03) as input floating */
	//GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);

	//USART_DeInit(USART2);  
	/* USART2 mode config */
	//USART_StructInit(&USART_InitStructure);
	USART_InitStructure.USART_BaudRate = baud;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);
	
	USART_ITConfig(USART2, USART_IT_RXNE,ENABLE);
	USART_ITConfig(USART2, USART_IT_TC, ENABLE);
	 
    USART_Cmd(USART2, ENABLE);		

	/*#if	DMA_MODE
#else
	USART_ITConfig(USART1,USART_IT_TC,ENABLE);	 
	USART1NVIC_Configuration();
#endif
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
	USART_Cmd(USART1,ENABLE); */

//
//	GPIO_InitTypeDef GPIO_InitStructure;
//	USART_InitTypeDef USART_InitStructure;
//
//	/* config USART2 clock */
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
//
//	/* USART2 GPIO config */
//   /* Configure USART2 Tx (PA.02) as alternate function push-pull */
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_Init(GPIOA, &GPIO_InitStructure);
//	    
//  /* Configure USART2 Rx (PA.03) as input floating */
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//  GPIO_Init(GPIOA, &GPIO_InitStructure);
//	  
//	/* USART2 mode config */
//	USART_InitStructure.USART_BaudRate = 115200;
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
//	USART_InitStructure.USART_StopBits = USART_StopBits_1;
//	USART_InitStructure.USART_Parity = USART_Parity_No ;
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
//
//	USART_Init(USART2, &USART_InitStructure); 
//  USART_Cmd(USART2, ENABLE);   

}

/*******************************************************************************
* 文件名	  	 : USART2_IRQHandler
* 描述	         : USART2_IRQHandler中断函数通道
* 输入           : 无
* 输出           : 无
* 返回           : 无
*******************************************************************************/
void USART2_IRQHandler(void)
{
	static u8 Flag=1;

	if(USART_GetFlagStatus(USART2,USART_FLAG_RXNE)==SET)
    //if(USART_GetFlagStatus(USART2,USART_IT_RXNE)==SET)
	{
	    USART_ClearFlag(USART2,USART_FLAG_RXNE);//TCIE,TE,RE
		//USART_ClearITPendingBit(USART2,USART_IT_RXNE);
		if(Flag)
		{			
			USART2RecvUpdate(); 
		}
	}
	if(USART_GetFlagStatus(USART2,USART_FLAG_TC)==SET)
	{
	    USART_ClearFlag(USART2,USART_FLAG_TC);//TCIE,TE,RE
		if(Flag)
		{
			USART2SendUpdate();
		}
	}

	Flag=1;
}


