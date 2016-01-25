#ifndef	_MAGNAV_H	 
#define _MAGNAV_H

#include "MotorControl.h"

typedef struct {
	float kp;
	float kd;
} K_PD;

extern K_PD g_pd[4];
extern K_PD pd[4];

extern u8 g_pd_flag;
extern void mag_sensor_init(void);
extern RobotRate mag_nav(u8 nav_dir, float speed);
extern u8 mag_sensor_read(u8 dir, u8 mag_sensor_data[]);
extern void offset_cal(u8 mag_sensor_data[], u8 sensor_num);
extern void mag_sensor_update(void);

extern u8 g_mag_sensor_data_front[8];
extern u8 g_mag_sensor_data_back[8];
extern u8 g_mag_sensor_data_left[8];
extern u8 g_mag_sensor_data_right[8];
extern u8 g_mag_sensor_front_flag;
extern u8 g_mag_sensor_back_flag;
extern u8 g_mag_sensor_left_flag;
extern u8 g_mag_sensor_right_flag;

extern u8 g_mag_sensor_front_revert_flag;
extern u8 g_mag_sensor_back_revert_flag;
extern u8 g_mag_sensor_left_revert_flag;
extern u8 g_mag_sensor_right_revert_flag;

extern float g_BotOffset_front;
extern float g_BotOffset_back;
extern float g_BotOffset_left;
extern float g_BotOffset_right; 

extern float BotOffset;

#endif

