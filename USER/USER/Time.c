/******************** ��Ȩ���� �人���ϻ����˿Ƽ����޹�˾ ������ **************
* �ļ���             : Time.c
* ����               : ������
* �汾               : V1.0
* ����               : 9/13/2012
* ����               : ʱ�����
* ����               ���ṩ��ȷ��ʱ����				   				   	
*******************************************************************************/
#include "Time.h"

volatile u32 time=0;

void DelayMs(unsigned long N)
{
	long i,j;
    for(i=0;i<N;i++)
      for(j=0;j<14200;j++); 
}

void DelayUs(unsigned long N)
{
	long i,j;
    for(i=0;i<N;i++)
      for(j=0;j<14;j++); 
}

/*
 * SystemCoreClock / 1000 //1ms �ж�һ��
 * SystemCoreClock / 100000 // 10us�ж�һ��
 * SystemCoreClock / 1000000 // 1us �ж�һ��
*/

void SysTick_Init(void)
{
	if (SysTick_Config(SystemCoreClock / 1000))
	{
		while (1);
	}	

	//�رյδ�ʱ��
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}

void SysTick_Start(void)
{
	//ʹ�ܵδ�ʱ��
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;	
}

/*
 * ��������TIM2_NVIC_Configuration
 * ����  ��TIM2�ж����ȼ�����
 * ����  ����
 * ���  ����	
 */
void TIM2_NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure; 
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);  													
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;	  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/*TIM_Period--5000   TIM_Prescaler--71 -->������ʱ��Ƶ��Ϊ 1MHZ �ж�����Ϊ5ms*/
void TIM2_Init(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);
	TIM2_NVIC_Configuration();
    TIM_DeInit(TIM2);
    TIM_TimeBaseStructure.TIM_Period=5000;		 			/* �Զ���װ�ؼĴ������ڵ�ֵ(����ֵ) */
    /* �ۼ� TIM_Period��Ƶ�ʺ����һ�����»����ж� */
    TIM_TimeBaseStructure.TIM_Prescaler= (72 - 1);		    /* ʱ��Ԥ��Ƶ�� 72M/72 */
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 		/* ������Ƶ */
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; /* ���ϼ���ģʽ */
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);				    /* �������жϱ�־ */
    TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
    TIM_Cmd(TIM2, ENABLE);									/* ����ʱ�� */
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , DISABLE);	/*�ȹرյȴ�ʹ��*/    
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void Initial_Timer3(void)
*��������:	  ��ʼ��Tim3  Tim4 ��������ʱ���������Բ���һ��32λ�Ķ�ʱ�����ṩϵͳus ���ļ�ʱ	
�����������
���������û��	
*******************************************************************************/
void TIM3_Init(void)
{
  	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE); 
	/* TIM2 configuration*/ 
  	/* Time Base configuration �������� ���ö�ʱ����ʱ����Ԫ*/
  	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure); 
  	TIM_TimeBaseStructure.TIM_Period = 0xffff; //�Զ���װֵ         
  	TIM_TimeBaseStructure.TIM_Prescaler = 0x0;       
  	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;    
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
  	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); 
  
  	TIM_PrescalerConfig(TIM3, 0, TIM_PSCReloadMode_Update);
  	/* Disable the TIM2 Update event */
  	TIM_UpdateDisableConfig(TIM3, ENABLE);
  	/* ----------------------TIM2 Configuration as slave for the TIM3 ----------*/
  	/* Select the TIM2 Input Trigger: TIM3 TRGO used as Input Trigger for TIM2*/
  	TIM_SelectInputTrigger(TIM3, TIM_TS_ITR3);
  	/* Use the External Clock as TIM2 Slave Mode */
  	TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_External1);
  	/* Enable the TIM2 Master Slave Mode */
  	TIM_SelectMasterSlaveMode(TIM3, TIM_MasterSlaveMode_Enable);
  	TIM_ARRPreloadConfig(TIM3, ENABLE);	
	/* ��ʱ������:
	1.���ö�ʱ��������ֵ 100000
	2.����ʱ�ӷ�Ƶϵ����TIM_CKD_DIV1
	3. ����Ԥ��Ƶ��  1Mhz/100000= 1hz 
	4.��ʱ������ģʽ  ���ϼ���ģʽ
	*/		 
  	TIM_TimeBaseStructure.TIM_Period = 0xffff;     
  	TIM_TimeBaseStructure.TIM_Prescaler = 72-1;	 //1M ��ʱ��  
  	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	//Ӧ�����õ�TIM4 
  	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	// ʹ��TIM3���ؼĴ���ARR
  	TIM_ARRPreloadConfig(TIM4, ENABLE);	

	TIM_SelectSlaveMode(TIM4, TIM_SlaveMode_Reset);
	TIM_UpdateRequestConfig(TIM4, TIM_UpdateSource_Regular);
	/* ----------------------TIM4 Configuration as Master for the TIM3 -----------*/
  	/* Use the TIM4 Update event  as TIM4 Trigger Output(TRGO) */
  	TIM_SelectOutputTrigger(TIM4, TIM_TRGOSource_Update);
  	/* Enable the TIM4 Master Slave Mode */
  	TIM_SelectMasterSlaveMode(TIM4, TIM_MasterSlaveMode_Enable);

  	//������ʱ��4
	TIM_Cmd(TIM4, ENABLE); 
  	TIM_Cmd(TIM3, ENABLE);                  
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		uint32_t micros(void)
*��������:	  ��ȡϵͳ���е�ʱ�� ��ϵͳ����ʱ�����ص�λΪus ��ʱ������	
�����������
�����������������ǰʱ�䣬���ϵ翪ʼ��ʱ  ��λ us
*******************************************************************************/
uint32_t micros(void)
{
 	uint32_t temp=0 ;
 	temp = TIM3->CNT; //����16λʱ��
 	temp = temp<<16;
 	temp += TIM4->CNT; //����16λʱ��
 	return temp;
}

void TIM2_IRQHandler(void)
{
	if ( TIM_GetITStatus(TIM2 , TIM_IT_Update) != RESET ) 
	{	
		TIM_ClearITPendingBit(TIM2 , TIM_FLAG_Update);    
  		 time++;
	}

	/*if (time%2==0)			//10ms  һ��
	{
        ImuSensorProcess();
	}
	if (time%4==3)			//20ms  һ��
	{
        NRFProcess();
	}
		
	if(time==1)			    //50  ms һ��
	{ 		
		UpdateObastacle();	
		UART4Proc();			
	}*/	
	
	if(time>=10)			    //50  ms һ��
	{
		time=0;	 		
	}		
}