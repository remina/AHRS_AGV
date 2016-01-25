
#include "USART3Config.h"

void USART3NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);   
	/* 配置串口3的中断，中断优先级别为0，响应级别为6 */
	NVIC_InitStructure.NVIC_IRQChannel=USART3_IRQn;
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
void USART3_Configuration(unsigned long baud)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

   	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	////RCC_APB2PeriphClockCmd(RCC_APB1Periph_USART2|RCC_APB2Periph_GPIOA| RCC_APB2Periph_AFIO, ENABLE);


	//NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);   
	// 配置串口3的中断，中断优先级别为0，响应级别为6 
	NVIC_InitStructure.NVIC_IRQChannel=USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=6;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* USART3 GPIO config */
    /* Configure USART3 Tx (PB.10) as alternate function push-pull */
	//GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	    
	/* Configure USART3 Rx (PB.11) as input floating */
	//GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* USART3 mode config */
	//USART_StructInit(&USART_InitStructure);
	USART_InitStructure.USART_BaudRate = baud;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);
	
	USART_ITConfig(USART3, USART_IT_RXNE,ENABLE);
	USART_ITConfig(USART3, USART_IT_TC, ENABLE);
	 
    USART_Cmd(USART3, ENABLE);		
}

/*******************************************************************************
* 文件名	  	 : USART2_IRQHandler
* 描述	         : USART2_IRQHandler中断函数通道
* 输入           : 无
* 输出           : 无
* 返回           : 无
*******************************************************************************/
void USART3_IRQHandler(void)
{
	if(USART_GetFlagStatus(USART3,USART_FLAG_RXNE)==SET)
	{
	    USART_ClearFlag(USART3,USART_FLAG_RXNE);//TCIE,TE,RE
		USART3RecvUpdate(); 
	}
	if(USART_GetFlagStatus(USART3,USART_FLAG_TC)==SET)
	{
	    USART_ClearFlag(USART3,USART_FLAG_TC);//TCIE,TE,RE
		USART3SendUpdate();
	}
	if(USART_GetFlagStatus(USART3,USART_FLAG_ORE)==SET) 
	{ 
	    USART_ClearFlag(USART3,USART_FLAG_ORE);  //读 SR 
	    USART_ReceiveData(USART3);       //读 DR 
	} 
}
