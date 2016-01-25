/******************** ��Ȩ���� �人���ϻ����˿Ƽ����޹�˾ ������ **************
* �ļ���             : MotorControl.h
* ����               : ������
* �汾               : V1.0
* ����               : 9/13/2012
* ����               : �������ͷ�ļ�
* ����               ��ʵ�������ķ�ֵ��ٶȺϳ���ֽ�			   				   	
*******************************************************************************/

#ifndef _MOTORCONTROL_H
#define _MOTORCONTROL_H

#include "global.h"
#include "stm32f10x.h"

#define CON_COEFEICIENT  626.912

typedef struct{
float fBotTranslationRateX;
float fBotTranslationRateY;
float fBotAngularRate;
}RobotRate;

typedef struct{
float fWheelSpeed_LF;
float fWheelSpeed_RF;
float fWheelSpeed_LB;
float fWheelSpeed_RB;
}WheelSpeed;

typedef struct{
double fBotDistanceX;
double fBotDistanceY;
double fBotYaw;
}RobotDistance;

extern float TempBuffer[4];
extern RobotDistance robotdistance;
//extern s32 feedback_wheelspeed[4];


extern WheelSpeed ComputeEachWheelSpeed(RobotRate brate);
extern RobotRate GetRobotRate(void);
extern void SendCmdToMotorDriver(WheelSpeed realspeed);
//void PositionControl(RobotRate* robotrate);


extern void Motor_init(void);
int abs(int a);
extern void RobotAutonomously(void);
extern void RobotAutonomously1(void);
extern void Navigating(void);
extern void set_speedctl(u8 motor_id);
extern void set_motorspeed(u8 motor_id, s32 speed);
extern void set_posctl(u8 motor_id);
extern void set_motorpos(u8 motor_id, int64_t pos);
extern void get_motorspeed(u8 motor_id);
extern void get_motorpos(u8 motor_id);
extern void set_all_speedctl(void);
extern void set_all_posctl(void);
extern void move(s32 milemeter);
extern void turn(u32 time);
extern void stop_base(void);

#define BUILD_ID(prio,deviceid,property) ((prio<<24)|(deviceid<<16)|(property))
#define GET_DEV_ID(id) ((id>>16)&0xff)
#define GET_PRIO(id)  ((id>>24)&0xff)
#define GET_PROP(id)  ((id)&0xffff)
//#define PowerOn()	GPIO_SetBits(GPIOB, GPIO_Pin_15)
//#define PowerOff()	GPIO_ResetBits(GPIOB, GPIO_Pin_15)


#define COMMON_COMMON_ACK          11
#define SERVO_STATUS_NOWPOSITION   1033
#define SERVO_STATUS_NOWSPEED      1036    

#endif

