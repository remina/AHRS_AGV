/******************** 版权所有 武汉汉迪机器人科技有限公司 杜骁释 **************
* 文件名             : UART4Config.c
* 作者               : 杜骁释
* 版本               : V1.0
* 日期               : 9/13/2012
* 描述               : UART4驱动层
* 移植步骤		 	 :（1）配置函数（管脚，时钟等）
				      （2）控制的参数（极性，波特率，位数，校验位等）
				      （3）中断函数
* 功能               ：配置串口管脚时钟，控制参数以及中断,与上层蓝牙接口通讯				   				   	
*******************************************************************************/
#include "UART4Config.h"

void UART4_Configuration(unsigned long baud)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;


	/* 配置串口1的中断，中断优先级别为0，响应级别为3 */
	NVIC_InitStructure.NVIC_IRQChannel=UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=4;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* config USART4 clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	/* USART4 GPIO config */
    /* Configure USART4 Tx (PC.10) as alternate function push-pull */
	
	//GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIOC->CRH &= 0xFFFFFBFF;
	GPIOC->CRH |= 0xFFFFFBFF;
	    
	/* Configure USART4 Rx (PC.11) as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	//GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);
	  
	/* USART4 mode config */
	USART_InitStructure.USART_BaudRate = baud;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(UART4, &USART_InitStructure);
	
	USART_ITConfig(UART4, USART_IT_RXNE,ENABLE);
	USART_ITConfig(UART4, USART_IT_TC, ENABLE); 

	USART_Cmd(UART4, ENABLE);
}

/*******************************************************************************
* 文件名	  	 : UART4_IRQHandler
* 描述	         : UART4_IRQHandler（USART1发送）中断函数通道
* 输入           : 无
* 输出           : 无
* 返回           : 无
*******************************************************************************/
void UART4_IRQHandler(void)
{
	static u8 Flag=0;
	
	if(USART_GetFlagStatus(UART4,USART_FLAG_RXNE)==SET)
	{
	    USART_ClearFlag(UART4,USART_FLAG_RXNE);//TCIE,TE,RE
		if(Flag)
		{			
			UART4RecvUpdate(); 
		}
	}
	if(USART_GetFlagStatus(UART4,USART_FLAG_TC)==SET)
	{
	    USART_ClearFlag(UART4,USART_FLAG_TC);//TCIE,TE,RE
		if(Flag)
		{
			UART4SendUpdate();
		}
	}
	Flag=1;
}


