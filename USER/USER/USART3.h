#ifndef __USART3_H
#define	__USART3_H

#include "USART3Config.h"
#include "stm32f10x.h"
#include "OSQMem.h"
#include "MotorControl.h"

#define OS_MEM_USART3_MAX 		1024			//���ͻ��������ڴ��С
#define OS_MEM_USART3_BLK 		32				//ÿһ����ĳ���
#define USART3_SEND_MAX_Q	  	(OS_MEM_USART3_BLK-4)	//�����ڴ���ڵ����ռ�
#define USART3_SEND_MAX_BOX		20	   					//�����ڴ����������

#define USART3_RECV_MAX_Q	  	256					//���ջ��������ռ�


void USART3SendUpdate(void);
u8 USART3WriteDataToBuffer(u8 *buffer,u8 count);
u8 USART3DispFun(u8 *buffer);
void USART3RecvUpdate(void);

extern void UART3Proc(void);
extern void USART3_Init(unsigned long baud);
extern u8 make_frame(u8 frame_type, u8 cmd_type, u8 data[], u8 data_len, u8 frame_data[]); 

extern u8 strafe_block(float speed, u8 from, u8 to);
extern u8 back_and_turn(float speed, u8 from, u8 to);
extern u8 turn_and_go(float speed, u8 from, u8 to); 

extern RobotRate g_zigbee_manual_botrate;
extern float g_magnav_auto_botrate;
extern u8 g_from, g_to,lose_flag;
extern u8 g_magnav_action_flag; 

#endif /* __USART2_H */
