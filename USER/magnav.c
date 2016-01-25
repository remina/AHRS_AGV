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

u8 g_mag_sensor_data_front[8];
u8 g_mag_sensor_data_back[8];
u8 g_mag_sensor_data_left[8];
u8 g_mag_sensor_data_right[8];
u8 g_mag_sensor_front_flag;
u8 g_mag_sensor_back_flag;
u8 g_mag_sensor_left_flag;
u8 g_mag_sensor_right_flag;

float g_BotOffset_front;
float g_BotOffset_back;
float g_BotOffset_left;
float g_BotOffset_right; 

u8 g_mag_sensor_front_last_flag;
u8 g_mag_sensor_back_last_flag;
u8 g_mag_sensor_left_last_flag;
u8 g_mag_sensor_right_last_flag;

u8 g_mag_sensor_front_revert_flag;
u8 g_mag_sensor_back_revert_flag;
u8 g_mag_sensor_left_revert_flag;
u8 g_mag_sensor_right_revert_flag;


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
}

u8 sensordata[8];
u8 mag_sensor_read(u8 dir, u8 mag_sensor_data[])
{
	u8 i;
	u8 flag = 0;

	switch (dir)
	{
	case FRONT:		
		mag_sensor_data[0]=GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4);
		mag_sensor_data[1]=GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5);
	    mag_sensor_data[2]=GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6);
		mag_sensor_data[3]=GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_7); 
		mag_sensor_data[4]=GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_4);
		mag_sensor_data[5]=GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_5);
		mag_sensor_data[6]=GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0);   
		mag_sensor_data[7]=GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1);			
		break;
	case LEFT:
		mag_sensor_data[7]=GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12);
		mag_sensor_data[6]=GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13);
	    mag_sensor_data[5]=GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14);
		mag_sensor_data[4]=GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15); 
		mag_sensor_data[3]=GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_8);
		mag_sensor_data[2]=GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_9);
		mag_sensor_data[1]=GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_10);   
		mag_sensor_data[0]=GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_11);
		break;
	case BACK:
	    mag_sensor_data[0]=GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_7);
		mag_sensor_data[1]=GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_8);
	    mag_sensor_data[2]=GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_9);
		mag_sensor_data[3]=GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_10); 
		mag_sensor_data[4]=GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_11);
		mag_sensor_data[5]=GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_12);
		mag_sensor_data[6]=GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_13);   
		mag_sensor_data[7]=GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_14);	
		break;
	case RIGHT:
		mag_sensor_data[0]=GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_12);
		mag_sensor_data[1]=GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_13);
	    mag_sensor_data[2]=GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_14);
		mag_sensor_data[3]=GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_15); 
		mag_sensor_data[4]=GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_6);
		mag_sensor_data[5]=GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_7);
		mag_sensor_data[6]=GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_8);   
		mag_sensor_data[7]=GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_9);	
		break;
	default:
		break;
	}

	for (i = 0; i < 8; i++)
	{
		flag = flag || mag_sensor_data[i];
		sensordata[i] = mag_sensor_data[i];		
	}

	return flag;
}

float  BotOffset=0.0;
float  OFFSET[20]={0.0};
float  DeBotOffset=0.0;
u8 last_dir=0;
void offset_cal(u8 mag_sensor_data[], u8 sensor_num)
{		 										 
	u8 i;
	float weight[8]={1.0,1.5,2.5,3.5,3.5,2.5,1.5,1.0};
	int weight1[8]={7,5,3,1,-1,-3,-5,-7};
	float  SumWeight=0.0;
	float  SumOffset=0.0;

    for(i = 0; i < sensor_num; i++)
	{
		SumOffset+=mag_sensor_data[i]*weight[i]*weight1[i];
		SumWeight+=mag_sensor_data[i]*weight[i];
	}

	//if(mag_sensor_data[0]&&mag_sensor_data[1]&&mag_sensor_data[2]&&mag_sensor_data[3]&&mag_sensor_data[4]&&mag_sensor_data[5]&&mag_sensor_data[6]&&mag_sensor_data[7]) BotOffset=0; //灯全亮
	//else if(SumWeight==0.0&& OFFSET[0]>=0)  BotOffset=8.0;  //Lost Line
	if(SumWeight==0.0)   BotOffset=OFFSET[0];  //Lost Line
	else BotOffset=SumOffset/SumWeight;

	for(i=19;i>=1;i--)
	{
		OFFSET[i]=OFFSET[i-1];
	}
	OFFSET[0] =	BotOffset;
	DeBotOffset= (OFFSET[0]+OFFSET[1]+OFFSET[2]-OFFSET[3]-OFFSET[4]-OFFSET[5])/3;

	if(DeBotOffset>=3)	DeBotOffset=3;	  //Limit
	else if(DeBotOffset<=-3)  DeBotOffset=-3;
}


void pid_cal(K_PD pd, RobotRate *rate, u8 dir, float speed)
{

//	float angluar_rate;
//	RobotRate rate;
	u8 mag_state[8] = {0};
	u8 status;
	u8 i;

	status = mag_sensor_read(dir, mag_state);
	if (status == 0) {
		rate->fBotTranslationRateX = 0.0;
		rate->fBotTranslationRateY = 0.0;
		rate->fBotAngularRate = 0.0;
		return;
	}


   	offset_cal(mag_state, 8);

//	if (status == 0)
//	{
//		BotOffset = 0.0;
//		DeBotOffset = 0.0;
//	}
	if(last_dir!=dir)
	{		
		for(i=19;i>=1;i--)
		{
			OFFSET[i]=BotOffset;
		}
		OFFSET[0] =	BotOffset;
		DeBotOffset= 0.0;
	}

	if(dir==1||dir==2)
	{
		//if(BotOffset>=3) rate->fBotTranslationRateX=rate->fBotTranslationRateX*(4/(BotOffset+1));   //速度根据偏离的程度进行小范围内调节 系数：0.33~1.0
		//else if(BotOffset<=-3) rate->fBotTranslationRateX=rate->fBotTranslationRateX*(4/(1-BotOffset)); 
	}

    rate->fBotAngularRate = (pd.kp*BotOffset + pd.kd*DeBotOffset);	//direction control

	if(rate->fBotAngularRate>=0.6) 
		rate->fBotAngularRate=0.6;	 // limit
	else  if(rate->fBotAngularRate<=-0.6) 
		rate->fBotAngularRate=-0.6;
		
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
//	wheelspeed = ComputeEachWheelSpeed(rate);		
//    SendCmdToMotorDriver(wheelspeed);
}

void mag_sensor_update(void)
{
	g_mag_sensor_front_flag = mag_sensor_read(FRONT, g_mag_sensor_data_front);
	offset_cal(g_mag_sensor_data_front, 8);
	g_BotOffset_front = BotOffset;

	g_mag_sensor_back_flag = mag_sensor_read(BACK, g_mag_sensor_data_back);
	offset_cal(g_mag_sensor_data_back, 8);
	g_BotOffset_back = BotOffset;

	g_mag_sensor_left_flag = mag_sensor_read(LEFT, g_mag_sensor_data_left);
	offset_cal(g_mag_sensor_data_left, 8);
	g_BotOffset_left = BotOffset;

	g_mag_sensor_right_flag = mag_sensor_read(RIGHT, g_mag_sensor_data_right);
	offset_cal(g_mag_sensor_data_right, 8);
	g_BotOffset_right = BotOffset;
	
//	if (g_mag_sensor_front_flag == TRUE && g_mag_sensor_front_last_flag == FALSE)
//	{
//		g_mag_sensor_front_revert_flag = TRUE;
//	}
//	
//	if (g_mag_sensor_back_flag == TRUE && g_mag_sensor_back_last_flag == FALSE)
//	{
//		g_mag_sensor_back_revert_flag = TRUE;
//	}
//	
//	if (g_mag_sensor_left_flag == TRUE && g_mag_sensor_left_last_flag == FALSE)
//	{
//		g_mag_sensor_left_revert_flag = TRUE;
//	}
//	
//	if (g_mag_sensor_right_flag == TRUE && g_mag_sensor_right_last_flag == FALSE)
//	{
//		g_mag_sensor_right_revert_flag = TRUE;
//	}
//	
//	g_mag_sensor_front_last_flag = g_mag_sensor_front_flag;
//	g_mag_sensor_back_last_flag = g_mag_sensor_back_flag;
//	g_mag_sensor_left_last_flag = g_mag_sensor_left_flag;
//	g_mag_sensor_right_last_flag = g_mag_sensor_right_flag;			
}

//u8 move_base(u8 from, u8 to)
//{
//		
//	mag_nav(FRONT, 200);
//
//	while ()
//	{
//		
//	}
//
//	if (from > to)
//		mag_nav(LEFT, 200);
//	else 
//		mag_nav(RIGHT, 200);
//
//}



