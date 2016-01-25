/******************** 版权所有 武汉汉迪机器人科技有限公司 杜骁释 **************
* 文件名             : Time.h
* 作者               : 杜骁释
* 版本               : V1.0
* 日期               : 9/13/2012
* 描述               : 时间管理头文件
* 功能               ：提供精确延时功能				   				   	
*******************************************************************************/
#ifndef  _TIME_H
#define  _TIME_H

#include "stm32f10x.h"
void DelayMs(unsigned long N);
void DelayUs(unsigned long N);
extern volatile u32 time; // ms 计时变量

#define START_TIME()  time=0;RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);TIM_Cmd(TIM2, ENABLE)
#define STOP_TIME()   TIM_Cmd(TIM2, DISABLE);RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , DISABLE)

extern void TIM2_Init(void);
extern void TIM3_Init(void);
extern void SysTick_Init(void);
extern void SysTick_Start(void);
extern uint32_t micros(void);

#endif

