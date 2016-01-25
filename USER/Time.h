/******************** ��Ȩ���� �人���ϻ����˿Ƽ����޹�˾ ������ **************
* �ļ���             : Time.h
* ����               : ������
* �汾               : V1.0
* ����               : 9/13/2012
* ����               : ʱ�����ͷ�ļ�
* ����               ���ṩ��ȷ��ʱ����				   				   	
*******************************************************************************/
#ifndef  _TIME_H
#define  _TIME_H

#include "stm32f10x.h"
void DelayMs(unsigned long N);
void DelayUs(unsigned long N);
extern volatile u32 time; // ms ��ʱ����

#define START_TIME()  time=0;RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);TIM_Cmd(TIM2, ENABLE)
#define STOP_TIME()   TIM_Cmd(TIM2, DISABLE);RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , DISABLE)

extern void TIM2_Init(void);
extern void TIM3_Init(void);
extern void SysTick_Init(void);
extern void SysTick_Start(void);
extern uint32_t micros(void);

#endif

