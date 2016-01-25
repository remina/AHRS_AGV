#ifndef _ODOMETRY_H
#define _ODOMETRY_H
#include "global.h"

extern s64 encoder_data[4];

extern WheelSpeed feedback_wheelspeed;
extern u32 g_distance;

extern void odometers(void);

#endif

