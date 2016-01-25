#include "odometry.h"
#include "stm32f10x.h"
#include "MotorControl.h"
#include "math.h"

s64 encoder_data[4];
WheelSpeed feedback_wheelspeed;

RobotRate g_robotrate;
u32 g_distance = 0;
/*void odometers(void)
{
    WheelSpeed wheelspeed;

	get_motorspeed(1);
	get_motorspeed(2);
	get_motorspeed(3);
	get_motorspeed(4);

   	wheelspeed.fWheelSpeed_LF = feedback_wheelspeed.fWheelSpeed_LF / CON_COEFEICIENT;
	wheelspeed.fWheelSpeed_RF = feedback_wheelspeed.fWheelSpeed_RF / CON_COEFEICIENT;
	wheelspeed.fWheelSpeed_LB =	feedback_wheelspeed.fWheelSpeed_LB / CON_COEFEICIENT;
	wheelspeed.fWheelSpeed_RB = feedback_wheelspeed.fWheelSpeed_RB / CON_COEFEICIENT;
	ComputeBaseSpeed(wheelspeed, &g_robotrate);	
	
	g_distance += sqrt(pow(g_robotrate.fBotTranslationRateX * 0.05, 2) + pow(g_robotrate.fBotTranslationRateY * 0.05, 2));		
}*/

