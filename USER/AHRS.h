#ifndef __AHRS_H
#define	__AHRS_H

#include "stm32f10x.h"
#include "USART1.h"

//*****************************************Macros for AHRS *************************************************//
#define FIRST_FRAMEHEADER    0x55
#define ACC_METER    		 0x51
#define GYRO        		 0x52
#define ANGLE_OUTPUT         0x53
#define MAG_METER            0x54

#define NO_START_RCV         0
#define START_RCV            1

#define g                    9.9

#define total                2147483648      //32bit timer's total count

#define sign(x)              (x > 0?1:-1)

extern float yaw;
extern float pitch;
extern float roll;

extern float ex;
extern float ey;
extern float ez;

//********************************************for AHRS computing******************************************//
//*********************************quatanion structure************************************//
typedef struct 
{
	float q0;
	float q1;
	float q2;
	float q3;
}qua;

//*********************************structure for sensor data******************************//
//*********************************raw data(directly from sensor,without unit convert into g/rads/mgausse)
typedef struct
{
	s16 acc_x;
	s16 acc_y;
	s16 acc_z;
	s16 gyro_x;
	s16 gyro_y;
	s16 gyro_z;
	s16 mag_x;
	s16 mag_y;
	s16 mag_z;
}raw_data;

//***************************raw data(unit convert into g/rads/mgausse)
typedef struct
{
	float acc_x;
	float acc_y;
	float acc_z;
	float gyro_x;
	float gyro_y;
	float gyro_z;
	float mag_x;
	float mag_y;
	float mag_z;
}raw1_data;

//************************data for calculating*************************************************
typedef struct
{
	float acc_x;
	float acc_y;
	float acc_z;
	float gyro_x;
	float gyro_y;
	float gyro_z;
	float mag_x;
	float mag_y;
	float mag_z;
}sensor_data;

//*******************************math functions*************************************************//
float invSqrt(float x) ;

void qua_norm(qua *q);
void norm(float *x, float *y, float *z);

//******************************************functions for AHRS*********************************************************//
void SensorDataProcess(u8 type, raw_data *raw, raw1_data *raw1);
float invSqrt(float x);
u8 AHRSCheckDataFrame(void);
void SensorInitial(u8 type, qua *q);
void AHRS_iteration(u8 type, qua *q , sensor_data *s);
void AHRS_computeEuler(qua *q);
void AHRS_compute();


#endif /* _AHRS_H */


