#ifndef __AHRS_H
#define	__AHRS_H

#include "stm32f10x.h"
#include "USART1.h"

//*****************************************Macros for AHRS *************************************************//
#define FIRST_FRAMEHEADER    0x55
#define ACC_METER    		 		 0x51
#define GYRO        		 		 0x52
#define ANGLE_OUTPUT         0x53
#define MAG_METER            0x54

#define NO_START_RCV         0
#define START_RCV            1

#define g                    9.9

#define total                2147483648      //32bit timer's total count

extern float yaw;
//******************************************functions for AHRS*********************************************************//
void SensorDataProcess(u8 type);
float invSqrt(float x);
u8 AHRSCheckDataFrame(void);
void SensorInitial(u8 type);
void AHRS_iteration(u8 type);
void AHRS_computeEuler(void);
void AHRS_compute(void);

#endif /* _AHRS_H */


