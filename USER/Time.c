/******************** 版权所有 武汉汉迪机器人科技有限公司 杜骁释 **************
* 文件名             : Time.c
* 作者               : 杜骁释
* 版本               : V1.0
* 日期               : 9/13/2012
* 描述               : 时间管理
* 功能               ：提供精确延时功能				   				   	
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
 * SystemCoreClock / 1000 //1ms 中断一次
 * SystemCoreClock / 100000 // 10us中断一次
 * SystemCoreClock / 1000000 // 1us 中断一次
*/

void SysTick_Init(void)
{
	if (SysTick_Config(SystemCoreClock / 1000))
	{
		while (1);
	}	

	//关闭滴答定时器
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}

void SysTick_Start(void)
{
	//使能滴答定时器
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;	
}

/*
 * 函数名：TIM2_NVIC_Configuration
 * 描述  ：TIM2中断优先级配置
 * 输入  ：无
 * 输出  ：无	
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

/*TIM_Period--5000   TIM_Prescaler--71 -->计数器时钟频率为 1MHZ 中断周期为5ms*/
void TIM2_Init(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);
	TIM2_NVIC_Configuration();
    TIM_DeInit(TIM2);
    TIM_TimeBaseStructure.TIM_Period=5000;		 			/* 自动重装载寄存器周期的值(计数值) */
    /* 累计 TIM_Period个频率后产生一个更新或者中断 */
    TIM_TimeBaseStructure.TIM_Prescaler= (72 - 1);		    /* 时钟预分频数 72M/72 */
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 		/* 采样分频 */
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; /* 向上计数模式 */
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);				    /* 清除溢出中断标志 */
    TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
    TIM_Cmd(TIM2, ENABLE);									/* 开启时钟 */
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , DISABLE);	/*先关闭等待使用*/    
}

/**************************实现函数********************************************
*函数原型:		void Initial_Timer3(void)
*功　　能:	  初始化Tim3  Tim4 将两个定时器级联，以产生一个32位的定时器来提供系统us 级的计时	
输入参数：无
输出参数：没有	
*******************************************************************************/
void TIM3_Init(void)
{
  	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE); 
	/* TIM2 configuration*/ 
  	/* Time Base configuration 基本配置 配置定时器的时基单元*/
  	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure); 
  	TIM_TimeBaseStructure.TIM_Period = 0xffff; //自动重装值         
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
	/* 定时器配置:
	1.设置定时器最大计数值 100000
	2.设置时钟分频系数：TIM_CKD_DIV1
	3. 设置预分频：  1Mhz/100000= 1hz 
	4.定时器计数模式  向上计数模式
	*/		 
  	TIM_TimeBaseStructure.TIM_Period = 0xffff;     
  	TIM_TimeBaseStructure.TIM_Prescaler = 72-1;	 //1M 的时钟  
  	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	//应用配置到TIM4 
  	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	// 使能TIM3重载寄存器ARR
  	TIM_ARRPreloadConfig(TIM4, ENABLE);	

	TIM_SelectSlaveMode(TIM4, TIM_SlaveMode_Reset);
	TIM_UpdateRequestConfig(TIM4, TIM_UpdateSource_Regular);
	/* ----------------------TIM4 Configuration as Master for the TIM3 -----------*/
  	/* Use the TIM4 Update event  as TIM4 Trigger Output(TRGO) */
  	TIM_SelectOutputTrigger(TIM4, TIM_TRGOSource_Update);
  	/* Enable the TIM4 Master Slave Mode */
  	TIM_SelectMasterSlaveMode(TIM4, TIM_MasterSlaveMode_Enable);

  	//启动定时器4
	TIM_Cmd(TIM4, ENABLE); 
  	TIM_Cmd(TIM3, ENABLE);                  
}

/**************************实现函数********************************************
*函数原型:		uint32_t micros(void)
*功　　能:	  读取系统运行的时间 即系统运行时间差，返回单位为us 的时间数。	
输入参数：无
输出参数：处理器当前时间，从上电开始计时  单位 us
*******************************************************************************/
uint32_t micros(void)
{
 	uint32_t temp=0 ;
 	temp = TIM3->CNT; //读高16位时间
 	temp = temp<<16;
 	temp += TIM4->CNT; //读低16位时间
 	return temp;
}

void TIM2_IRQHandler(void)
{
	if ( TIM_GetITStatus(TIM2 , TIM_IT_Update) != RESET ) 
	{	
		TIM_ClearITPendingBit(TIM2 , TIM_FLAG_Update);    
  		 time++;
	}

	/*if (time%2==0)			//10ms  一次
	{
        ImuSensorProcess();
	}
	if (time%4==3)			//20ms  一次
	{
        NRFProcess();
	}
		
	if(time==1)			    //50  ms 一次
	{ 		
		UpdateObastacle();	
		UART4Proc();			
	}*/	
	
	if(time>=10)			    //50  ms 一次
	{
		time=0;	 		
	}		
}