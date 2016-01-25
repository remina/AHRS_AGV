/******************** 版权所有 武汉汉迪机器人科技有限公司 杜骁释 **************
* 文件名             : MotorControl.c
* 作者               : 杜骁释
* 版本               : V1.0
* 日期               : 9/13/2012
* 描述               : 电机控制算法实现
* 功能               ：实现麦克纳姆轮的速度合成与分解			   				   	
*******************************************************************************/

#include "MotorControl.h"
#include "CAN.h"
#include "USART1.h"
#include "math.h"
#include "time.h"

#define BOT_LENGTH       303.25 //606.5
#define BOT_WIDTH        282.5	 //565
#define WHEEL_DIAMETER   203.2       //diameter为203.2mm
#define MOTORRPM         30.0
#define GEAR_RATIO       30
//#define CON_COEFFICIENT  5.4255      // MOTORRPM / GEAR_RATIO * (PI * WHEEL_DIAMETER) / 60.0  //unit: mm/s


#define WHEEL_LFB_FORWARDDIR   0
#define WHEEL_LFB_REVERSEDIR   1
#define WHEEL_RFB_FORWARDDIR   1
#define WHEEL_RFB_REVERSEDIR   0

RobotDistance robotdistance;
volatile uint32_t  lasttime, nowtime; // 采样周期计数 单位 us

void Motor_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2;	 // output to 
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOE, &GPIO_InitStructure);	
	GPIO_SetBits(GPIOE, GPIO_Pin_2); 

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_14;	 // output to 
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOC, &GPIO_InitStructure);	
	GPIO_SetBits(GPIOC, GPIO_Pin_14); 
	
	//lasttime = micros();
  	//nowtime = micros();	  	
}

int abs(int a)
{
	if (a > 0) return a;
	else if (a < 0) return -a;
	else return a;
}

/*******************************************************************************
* 文件名	  	 : GetRobotRate
* 描述	         : 
* 输入           : 无
* 输出           : 无
* 返回           : 平台实际的速度与角速度
*******************************************************************************/
float temprate=0;
RobotRate GetRobotRate()
{
   float T = 1/(BOT_LENGTH + BOT_WIDTH);
   RobotRate realrate;
   realrate.fBotTranslationRateX =(g_RcvWheelSpeed[WHEEL_LF] + g_RcvWheelSpeed[WHEEL_RF] 
           + g_RcvWheelSpeed[WHEEL_LB] + g_RcvWheelSpeed[WHEEL_RB]) / 4 ;
   realrate.fBotTranslationRateY =	(- g_RcvWheelSpeed[WHEEL_LF] + g_RcvWheelSpeed[WHEEL_RF] 
           + g_RcvWheelSpeed[WHEEL_LB] - g_RcvWheelSpeed[WHEEL_RB]) / 4 ;
   temprate=realrate.fBotAngularRate = T *0.25* (- g_RcvWheelSpeed[WHEEL_LF] + g_RcvWheelSpeed[WHEEL_RF] 
                         - g_RcvWheelSpeed[WHEEL_LB] + g_RcvWheelSpeed[WHEEL_RB]) ;	
   
//   robotdistance.fBotDistanceX += dt*1.7*realrate.fBotTranslationRateX;
//   robotdistance.fBotDistanceY += dt*1.7*realrate.fBotTranslationRateY;
   //if(realrate.fBotAngularRate>=0)	robotdistance.fBotYaw += (dt*1.95*realrate.fBotAngularRate*180/PI); 
   //else	 robotdistance.fBotYaw += (dt*1.93*realrate.fBotAngularRate*180/PI);
    
   //robotdistance.fBotYaw += (dt*2.0*realrate.fBotAngularRate*180/PI);  
   /*robotdistance.fBotDistanceX += 0.5*realrate.fBotTranslationRateX;
   robotdistance.fBotDistanceY += 0.5*realrate.fBotTranslationRateY;
   robotdistance.fBotYaw += (0.1*realrate.fBotAngularRate)*180/PI;	*/
  
   return 	realrate;
}


/*******************************************************************************
* 文件名	  	 : ComputeEachWheelSpeed
* 描述	         : 
* 输入           : 给定速度大小和方向
* 输出           : 无
* 返回           : 每个电机的给定转速
*******************************************************************************/
WheelSpeed ComputeEachWheelSpeed(RobotRate brate)
{
  float T = BOT_LENGTH + BOT_WIDTH;

  WheelSpeed RealWheelSpeed;   //每个轮子的速度

  RealWheelSpeed.fWheelSpeed_LF = brate.fBotTranslationRateX - brate.fBotTranslationRateY - T * brate.fBotAngularRate;
  RealWheelSpeed.fWheelSpeed_RF = brate.fBotTranslationRateX + brate.fBotTranslationRateY + T * brate.fBotAngularRate;
  RealWheelSpeed.fWheelSpeed_LB = brate.fBotTranslationRateX + brate.fBotTranslationRateY - T * brate.fBotAngularRate;
  RealWheelSpeed.fWheelSpeed_RB = brate.fBotTranslationRateX - brate.fBotTranslationRateY + T * brate.fBotAngularRate;


  return RealWheelSpeed;
}

void ComputeBaseSpeed(WheelSpeed wSpeed, RobotRate *rate)
{
	float T = 1/(BOT_LENGTH + BOT_WIDTH);

	rate->fBotTranslationRateX = (wSpeed.fWheelSpeed_LF + wSpeed.fWheelSpeed_RF + wSpeed.fWheelSpeed_LB + wSpeed.fWheelSpeed_RB) / 4;
	rate->fBotTranslationRateY = (-wSpeed.fWheelSpeed_LF + wSpeed.fWheelSpeed_RF + wSpeed.fWheelSpeed_LB - wSpeed.fWheelSpeed_RB) / 4;
	rate->fBotAngularRate = (-wSpeed.fWheelSpeed_LF + wSpeed.fWheelSpeed_RF - wSpeed.fWheelSpeed_LB + wSpeed.fWheelSpeed_RB) * T / 4;
		
}

void HandleMotorMessage(int ID, int speedrate, int dir, int flag)
{
	u8 CANdata[] = {0x55, 0xAA, 0x03, 0x00, 0x00, 0x00, 0x00, 0xFF};
	CANdata[3] = flag;
	CANdata[4] = dir;
	CANdata[5] = speedrate;
	CANdata[6] = CANdata[2] + CANdata[3] + CANdata[4] + CANdata[5];
	CAN1WriteDataToBuffer(CANdata, 8, ID, CAN_RTR_DATA);
}

void set_speedctl(u8 motor_id) 
{
	u32 id = BUILD_ID(0, motor_id, 768);
	u8 data[8];
	*((u32*)data) = 0;
	CAN1WriteDataToBuffer(data, 4, id, CAN_RTR_DATA);

	id = BUILD_ID(0, motor_id, 771);
	*((u32*)data) = 2;
	CAN1WriteDataToBuffer(data, 4, id, CAN_RTR_DATA);

	id = BUILD_ID(0, motor_id, 768);
	*((u32*)data) = 1;
	CAN1WriteDataToBuffer(data, 4, id, CAN_RTR_DATA);
}

void set_posctl(u8 motor_id)
{
    u32 id = BUILD_ID(0, motor_id, 768);
	u8 data[8];
	*((u32*)data) = 0;
	CAN1WriteDataToBuffer(data, 4, id, CAN_RTR_DATA);

	id = BUILD_ID(0, motor_id, 771);
	*((u32*)data) = 4;
	CAN1WriteDataToBuffer(data, 4, id, CAN_RTR_DATA);

	id = BUILD_ID(0, motor_id, 768);
	*((u32*)data) = 1;
	CAN1WriteDataToBuffer(data, 4, id, CAN_RTR_DATA);
}

void set_all_speedctl(void)
{
	set_speedctl(1);
	set_speedctl(2);
	set_speedctl(3);
	set_speedctl(4);
}

void set_all_posctl(void)
{
	set_posctl(1);
	DelayMs(10);
	set_posctl(2);
	DelayMs(10);
	set_posctl(3);
	DelayMs(10);
	set_posctl(4);
}


void set_motorspeed(u8 motor_id, s32 speed)
{
	u32 id;
	u8 data[4];
    id = BUILD_ID(0, motor_id, 773);
//	if (speed >= 187500)
//		speed = 187500;
//	if (speed <= -187500)
//		speed = -187500;
	*((s32*)data) = speed;
	CAN1WriteDataToBuffer(data, 4, id, CAN_RTR_DATA);
}

void set_motorpos(u8 motor_id, int64_t pos)
{
	u32 id;
	u8 data[8];
	id = BUILD_ID(0, motor_id, 772);
	*((int64_t*)data) = pos;
	CAN1WriteDataToBuffer(data, 8, id, CAN_RTR_DATA);
}


//
typedef struct {
	u8 on_off:1;
	u8 ch:3;
	u8 reserved1;
	u8 reserved2;
	u16 interval;
	u32 id;
}Boost;
//void open_boost(u16 time, u32 id)
//{
//	u32 id;
//	u8 data[8];
//	Boost boost;
//	id = BUILD_ID(15, motor_id, **);
//	boost.on/off = 1;
//	boost.ch = **;
//	boost.interval = 50;
//	boost.id = 1036;  //反馈速度
//	data = &boost;
//	CAN1WriteDataBuffer(data, 8, id);
//}
//
//void close_boost(u32 id)
//{
//	u32 id;
//	u8 data[8];
//	id = BUILD_ID(15, motor_id, **);
//	boost.on/off = 0;
//	data = &boost;
//	CAN1WriteDataBuffer(data, 8, id);
//}
//
//void rotate_in_place(s16 rad)
//{
//}
//


void get_motorspeed(u8 motor_id)
{
//	CAN_msg msg;
//	msg.id = BUILD_ID(0, motor_id, 1036);
//	msg.len = 4;
//	msg.format = 1;
//	msg.type = 1;
//	*((s32*)msg.data) = 0;
//
//	CAN1WriteDataToBuffer();
//	CANSendMessage(&msg);
    
	u32 id;
	u8 data[4];
	id = BUILD_ID(1, motor_id, 1036);
	*((u32*)data) = 0;
	CAN1WriteDataToBuffer(data, 4, id, CAN_RTR_Remote); 	
}

void get_motorpos(u8 motor_id)
{
//	CAN_msg msg;
//	msg.id = BUILD_ID();
	CAN_msg msg;
	msg.id = BUILD_ID(1, motor_id, 1033);
	msg.len = 4;
	msg.format = 1;
	msg.type = 1;
//	*((u32*)msg.data) = 0;

	CANSendMessage(&msg);
		
}

void move(s32 milemeter)
{
//	set_all_posctl();
//	DelayMs(10);
	set_motorpos(1, 400000);
	set_motorpos(2, -400000);
	set_motorpos(3, 400000);
	set_motorpos(4, -400000);	
}

void turn(u32 time)
{
//	new_time = micros();	
//	if (new_time - last_time >= 1000)
//	{
//		stop();
//	}
//	else
//	static u32 last_time;
//
//	if (micros() / 1000 - last_time	>= 2000)
//		stop();		
}

void stop_base(void)
{
	set_motorspeed(1, 0);
	set_motorspeed(2, 0);
	set_motorspeed(3, 0);
	set_motorspeed(4, 0);
}

//
//void odom(void)
//{
//}

void SendCmdToMotorDriver(WheelSpeed realspeed)
{
  s32 speed;
  speed = realspeed.fWheelSpeed_LF * CON_COEFEICIENT;
  set_motorspeed(LF_MOTOR_ID, speed); 
  speed = -realspeed.fWheelSpeed_RF * CON_COEFEICIENT;
  set_motorspeed(RF_MOTOR_ID, speed);
  speed = realspeed.fWheelSpeed_LB * CON_COEFEICIENT;
  set_motorspeed(LB_MOTOR_ID, speed);
  speed = -realspeed.fWheelSpeed_RB * CON_COEFEICIENT;
  set_motorspeed(RB_MOTOR_ID, speed); 
}



