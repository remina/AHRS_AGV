#include "stm32f10x.h"
//#include "global.h"
#include "MotorControl.h"
#include "sensors.h"
#include "global.h"
#include "USART3.h"
#include "USART2.h"
#include "UART4.h"
#include "magnav.h"
#include "odometry.h"

u32 g_last_dis = 0;

int g_front_distance=1350;
int g_mid_distance=2800;
int g_back_distance=1400;
int g_mid_distance_1=2800;
//磁条引线长度  前24.6cm   后 24.7cm

/* AMP-I arbitration struct*/
typedef struct layer LAYER;

struct layer {
	u8 cmd;
	RobotRate robotrate;
	u8 flag;
};

RobotRate zerorate = {0.0, 0.0, 0.0};

LAYER bluetooth_manual, zigbee_manual, magnav, stop;

#define AMP_STATUS_SIZE 4
LAYER *amp[AMP_STATUS_SIZE] = {&bluetooth_manual, &zigbee_manual,  &magnav, &stop};

LAYER stop;                   // the default layer
LAYER *this_layer = &stop;    // output, layer chosen by arbitrator()
LAYER *this_layer;    
LAYER **p_amp;                // pointer to amp priority list
int amp_size;                 // number of tasks in priority list

u8 test_flag = 0;
u8 move_all_flag = 0;
u8 move_target_flag = 0;

u8 arbitrate; //global flag to enable subsumption
u8 halt;      //global flag to halt robot

u8 rfid_back1[] = {0x55, 0xaa};
u8 rfid_back2[] = {0xff};

int	amp_init()                   // make amp the active p_amp
{
	arbitrate = 1;
	p_amp = &amp[0];	         // global amp priority list pointer
	amp_size = AMP_STATUS_SIZE;  // number of taks in job1 list
	stop.robotrate.fBotTranslationRateX = 0.0;
	stop.robotrate.fBotTranslationRateY = 0.0;
	stop.robotrate.fBotAngularRate = 0.0;
	stop.flag=TRUE;
	return 0;
}

u8 base_cmd(LAYER *layer)
{
	extern u8 g_done_flag; 
	RobotRate robotrate;
	WheelSpeed realspeed;

	robotrate = layer->robotrate;
	realspeed = ComputeEachWheelSpeed(robotrate);
	SendCmdToMotorDriver(realspeed);
//	g_done_flag = FALSE;
}

void arbitrator()
{
	u8 i = 0;

	if (arbitrate) {
		for (i = 0; i <	amp_size; i++)
		{
			if (p_amp[i]->flag) break;
		}
		this_layer = amp[i];
		
		base_cmd(this_layer);	
	}
}

void bluetooth_task(void)
{
	extern u8 g_bt_manual_flag;
	extern u8 g_dir;

	if(test_flag)
	{
		bluetooth_manual.cmd  = ALLDIR;
		bluetooth_manual.robotrate.fBotTranslationRateX = g_bt_manual_botrate;
		bluetooth_manual.robotrate.fBotTranslationRateY = 0;
		bluetooth_manual.robotrate.fBotAngularRate = 0.0;
		bluetooth_manual.flag = TRUE;
	}
	else if(move_all_flag)
	{
		bluetooth_manual.cmd  = ALLDIR;
		bluetooth_manual.robotrate.fBotTranslationRateX = 0;
		bluetooth_manual.robotrate.fBotTranslationRateY = 0;
		bluetooth_manual.robotrate.fBotAngularRate = g_bt_manual_botrate;
		bluetooth_manual.flag = TRUE;
	}
	else if(g_bt_manual_flag)
	{
		bluetooth_manual.cmd  = ALLDIR;
		bluetooth_manual.robotrate = mag_nav(g_dir, g_bt_manual_botrate);				
		bluetooth_manual.flag = TRUE;					
	}
	else
		bluetooth_manual.flag = FALSE;				
}

/*void zigbee_manual_task(void)
{
	extern u8 g_zigbee_manual_flag;
	extern u8 g_done_flag;
	if (g_zigbee_manual_flag && g_done_flag)
	{
		zigbee_manual.cmd = ALLDIR;
		zigbee_manual.robotrate = g_zigbee_manual_botrate;
		zigbee_manual.flag = TRUE;
//		g_done_flag = FALSE;		
	}
	else
	{
		zigbee_manual.flag = FALSE;	
	}	
}
*/
u8 Mag_Sensor_Checked(u8 mag_sensor_data[], u8 sensor_num)
{
	u8 i;
	u8 flag;
	u8 mag_sum_data=0;
	for (i = 0; i < 8; i++)
	{
		mag_sum_data += mag_sensor_data[i];		
	}
	if(mag_sum_data>=sensor_num)  flag=1;
	else flag=0;

	return flag;
}

u8 Mag_Sensor_Checked1(u8 mag_sensor_data[], u8 sensor_from, u8 sensor_to)
{
	u8 i;
	u8 flag=0;
	for (i = sensor_from; i <= sensor_to; i++)
	{	
		if(mag_sensor_data[i]==1)  flag=1;
	}
	return flag;
}

u8 u3_data_test[5] = {0x55, 0xAA, 0, 0, 0};


void magnav_task(void)
{
	extern u8 g_magnav_auto_flag;
	extern u8 g_done_flag;
	extern u8 g_dir;
	int rate;

	/*test data*/
	
	if(g_magnav_action_flag)
	{
		magnav.cmd = ALLDIR;
		magnav.robotrate = mag_nav(g_dir, g_magnav_auto_botrate);
		magnav.flag	= TRUE;
	}
	else
		magnav.flag = FALSE;
}



void sensors(void)
{
	
	bluetooth_task();
//	zigbee_manual_task();
//	origin_pos_task();
//	magnav_task();
	arbitrator();
}

