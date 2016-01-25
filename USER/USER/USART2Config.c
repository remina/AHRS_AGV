/******************** ��Ȩ���� �人���ϻ����˿Ƽ����޹�˾ ������ **************
* �ļ���             : USART2.c
* ����               : ������
* �汾               : V1.0
* ����               : 9/13/2012
* ����               : USART1������
* ��ֲ����		 	 :��1�����ú������ܽţ�ʱ�ӵȣ�
				      ��2�����ƵĲ��������ԣ������ʣ�λ����У��λ�ȣ�
				      ��3���жϺ���
* ����               �����ô��ڹܽ�ʱ�ӣ����Ʋ����Լ��ж�				   				   	
*******************************************************************************/
#include "USART2Config.h"



void USART2NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);   
	/* ���ô���2���жϣ��ж����ȼ���Ϊ0����Ӧ����Ϊ6 */
	NVIC_InitStructure.NVIC_IRQChannel=USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=6;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
/*******************************************************************************
* �ļ���	  	 : USART2����
* ����	         : baud:USART1������
* ����           : ��
* ���           : ��
* ����           : ��
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
	// ���ô���2���жϣ��ж����ȼ���Ϊ0����Ӧ����Ϊ6 
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
* �ļ���	  	 : USART2_IRQHandler
* ����	         : USART2_IRQHandler�жϺ���ͨ��
* ����           : ��
* ���           : ��
* ����           : ��
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


