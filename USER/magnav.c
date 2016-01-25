/******************************************************************************
*  
* 磁条导航
*				   				   	
*******************************************************************************/
#include "stm32f10x.h"
#include "MotorControl.h"
#include "global.h"
#include "magnav.h"

//#define MAG_SENSOR_FRONT  1
//#define MAG_SENSOR_BACK   2
//#define MAG_SENSOR_SIDE   3


K_PD g_pd[4];
K_PD pd[4];
u8 g_pd_flag;
float error, error_p;

void mag_sensor_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD| RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, ENABLE);

	//初始化磁条传感器IO
	//Magnetic IO input		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4| GPIO_Pin_5 |GPIO_Pin_6 | GPIO_Pin_7;       
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	 // 上拉输入	GPIO_Mode_IN_FLOATING
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 |GPIO_Pin_15;       
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	 // 上拉输入	GPIO_Mode_IN_FLOATING
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4| GPIO_Pin_5 |GPIO_Pin_6| GPIO_Pin_7|GPIO_Pin_8| GPIO_Pin_9 ;       
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	 // 上拉输入	GPIO_Mode_IN_FLOATING
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;				   
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8| GPIO_Pin_9 | GPIO_Pin_10| GPIO_Pin_11 |GPIO_Pin_12| GPIO_Pin_13 | GPIO_Pin_14| GPIO_Pin_15;       
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	 // 上拉输入	GPIO_Mode_IN_FLOATING
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	//Magnetic IO input
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7| GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14;       
	GPIO_InitStructure.GPIO_Mode =GPIO_Mode_IPU;     // 上拉输入   	 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);		

	g_pd_flag = FALSE;

	pd[PID_FRONT].kp = 0.06;
	pd[PID_FRONT].kd = 0.03;
	pd[PID_BACK].kp = 0.06;
	pd[PID_BACK].kd = 0.03;
	pd[PID_LEFT].kp = -0.06;
	pd[PID_LEFT].kd = -0.03;
	pd[PID_RIGHT].kp = 0.06;
	pd[PID_RIGHT].kd = 0.03;
	error = 0;
	error_p = 0;
}

u8 last_dir=0;

void pid_cal(K_PD pd, RobotRate *rate, u8 dir, float speed)
{

//	float angluar_rate;
//	RobotRate rate;
	u8 mag_state[8] = {0};
	u8 status;
	u8 i;
	float d_error;
	extern float hold_yaw;
	extern float yaw;
	static float i_error = 0.0;
	static float hold_yaw_p = 0.0;

	if(hold_yaw != hold_yaw_p)
	{
		error_p = 0.0;
		i_error = 0.0;
	}
	error = yaw - hold_yaw;
	i_error += pd.ki*error;
	d_error = error - error_p;
	rate->fBotAngularRate = (pd.kp*error + i_error + pd.kd*d_error);	//direction control
	

	if(rate->fBotAngularRate>=0.6)
	{
		rate->fBotAngularRate=0.6;	 // limit
	}
	else if(rate->fBotAngularRate<=-0.6)
	{
		rate->fBotAngularRate=-0.6;
	}
	error_p = error;
	hold_yaw_p = hold_yaw;	
}

RobotRate mag_nav(u8 nav_dir, float speed)
{
	K_PD s_pd;
	RobotRate rate;
	WheelSpeed wheelspeed;
	u8 i = 0;
	
	if (g_pd_flag)
	{
		for (i = 0; i < 4; i++)
		{
			pd[i] = g_pd[i];
		}

		g_pd_flag = FALSE;
	}

	switch (nav_dir)
	{
	case FRONT:
		s_pd = pd[PID_FRONT];

		rate.fBotTranslationRateX = speed;
		rate.fBotTranslationRateY = 0;
//		pid_cal(pd, &rate, nav_dir);
		break;
	case BACK:
		s_pd = pd[PID_BACK];
		rate.fBotTranslationRateX = -speed;
		rate.fBotTranslationRateY = 0;
//		pid_cal(pd, &rate, nav_dir);
		break;
	case LEFT:
		s_pd = pd[PID_LEFT];
		rate.fBotTranslationRateX = 0;
		rate.fBotTranslationRateY = speed;
//		pid_cal(pd, &rate, nav_dir);
		break;
	case RIGHT:
		s_pd = pd[PID_RIGHT];
		rate.fBotTranslationRateX = 0;
		rate.fBotTranslationRateY = -speed;
//		pid_cal(s_pd, &rate, FRONT); 
		break;
	default:
		break;
	}
	if (abs((int)speed) != 0)
        pid_cal(s_pd, &rate, nav_dir, speed);
	else																												   
		rate.fBotAngularRate = 0.0;

	return rate;
}


