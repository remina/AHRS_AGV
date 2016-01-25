
#ifndef __LED_H
#define	__LED_H

#include "stm32f10x.h"

/* the macro definition to trigger the led on or off 
 * 1 - off
 - 0 - on
 */
#define ON  0
#define OFF 1

#define LED1(a)	if (a)	\
					GPIO_SetBits(GPIOE,GPIO_Pin_0);\
					else		\
					GPIO_ResetBits(GPIOE,GPIO_Pin_0)

#define LED2(a)	if (a)	\
					GPIO_SetBits(GPIOE,GPIO_Pin_1);\
					else		\
					GPIO_ResetBits(GPIOE,GPIO_Pin_1)

//#define LED3(a)	if (a)	\
//					GPIO_SetBits(GPIOE,GPIO_Pin_14);\
//					else		\
//					GPIO_ResetBits(GPIOE,GPIO_Pin_14)
//
//#define LED4(a)	if (a)	\
//					GPIO_SetBits(GPIOE,GPIO_Pin_15);\
//					else		\
//					GPIO_ResetBits(GPIOE,GPIO_Pin_15)

extern void LED_Init(void);	 
extern void led_task(void);

#endif /* __LED_H */
