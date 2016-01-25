#include "UART4.h"
#include "USART2.h"
#include "global.h"
#include "sensors.h"
#include "FLASH.h"
#include "stm32f10x.h"
#include "AHRS.h"
#include "Time.h"
#include "math.h"

//*****************************************variebles for AHRS ****************************************************//
bool cut_off = false;
u8 rawdata[11] = {0};
float gyro_x_f = 0.0f, gyro_y_f = 0.0f, gyro_z_f = 0.0f; 
float b_x = -1.0f, b_y = 0.0f, b_z = 0.0f;
float h_x = -1.0f, h_y = 0.0f, h_z = 0.0f;

qua SEq = {1.0, 0.0, 0.0, 0.0};
qua acc_q = {1.0, 0.0, 0.0, 0.0};
qua mag_q = {1.0, 0.0, 0.0, 0.0};
qua a_m_q = {1.0, 0.0, 0.0, 0.0};
raw_data data_first = {0, 0, 0, 0, 0, 0, 0, 0, 0};
raw1_data data_second = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
sensor_data s_data = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

float twoKp_z = 260.0f, twoKp_x = 260.0, twoKp_y = 260.0, twoKi_z = 10.0, twoKi_x = 10.0, twoKi_y = 10.0;
float vx = 0.0f, vy = 0.0f, vz = 0.0f, wx = 0.0f, wy = 0.0f, wz = 0.0f;
static int cut_count = 0;
float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;

//for initial quaternion
float buffer1[8] ={0},buffer2[8] ={0},buffer3[8] ={0},buffer4[8] ={0},buffer5[8] ={0},buffer6[8] ={0},buffer7[8] ={0},buffer8[8] ={0},buffer9[8] ={0};

u8  counter = 0;
float bias = 0.0f;

bool qua_init = false;
bool cal_ready = false;

u8 counter1;
u8 counter2;
u8 counter3;

float w_x_bias = 0.0f, w_y_bias = 0.0f, w_z_bias = 0.0f;
float a_x_bias = 0.0f, a_y_bias = 0.0f, a_z_bias = -1.0f, m_x_bias = -1.0f, m_y_bias = 0.0f, m_z_bias = 0.0f;
//**********************************variable for AHRS****************************************//
u8 AHRSRecvState = 0;

float ex = 0.0f, ey = 0.0f, ez = 0.0f;
float integralFb_x = 0.0f, integralFb_y = 0.0f, integralFb_z = 0.0f;

extern u8 UART4RecvPtrW, UART4RecvPtrR;
extern u8 UART4RecvBufStart, UART4RecvBufEnd;
extern u8 UART4RecvFrameLen;

float t1 = 0.0f, t2 = 0.0f, interval = 0.0f;

//****************************************************functions for AHRS ****************************************************//
//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x) 
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

void qua_norm(qua *q)
{
	float norm = invSqrt(q->q0 * q->q0 + q->q1 * q->q1 + q->q2 * q->q2 + q->q3 * q->q3);
	q->q0 *= norm;
	q->q1 *= norm;
	q->q2 *= norm;
	q->q3 *= norm;
}

void norm(float *x, float *y, float *z)
{
	float norm = invSqrt(*x * *x + *y * *y + *z * *z);
	*x *= norm;
	*y *= norm;
	*z *= norm;
}

void vector2qua(float from_x, float from_y, float from_z, float to_x, float to_y, float to_z, qua *q)
{
	float norm_from = invSqrt(from_x * from_x + from_y * from_y + from_z * from_z);
	float norm_to = invSqrt(to_x * to_x + to_y * to_y + to_z * to_z);
	float cos_theta = (from_x * to_x + from_y * to_y + from_z * to_z) * norm_from * norm_to;
	//float cos_halftheta = sqrt((1 + cos_theta)/2.0);
	float sin_halftheta = sqrt((1 - cos_theta)/2.0);

	float cross_x = from_y * to_z - from_z * to_y;
	float cross_y = from_z * to_x - from_x * to_z;
	float cross_z = from_x * to_y - from_y * to_x;

	norm(&cross_x, &cross_y, &cross_z);

	q->q0 = sqrt((1 + cos_theta)/2.0);
	q->q1 = cross_x * sin_halftheta;
	q->q2 = cross_y * sin_halftheta;
	q->q3 = cross_z * sin_halftheta;
}

void vector_corss(float from_x, float from_y, float from_z, float to_x, float to_y, float to_z, float *out_x, float *out_y, float *out_z)
{
	*out_x = from_y * to_z - from_z * to_y;
	*out_y = from_z * to_x - from_x * to_z;
	*out_z = from_x * to_y - from_y * to_x;
}

void vector_add(float from_x, float from_y, float from_z, float to_x, float to_y, float to_z, float *out_x, float *out_y, float *out_z)
{
	*out_x = from_x + to_x;
	*out_y = from_y + to_y;
	*out_z = from_z + to_z;
}

void qua_multiply(qua *from, qua *to, qua *out)
{
	out->q0 = from->q0 * to->q0 - from->q1 * to->q1 - from->q2 * to->q2 - from->q3 * to->q3;
	out->q1 = from->q0 * to->q1 + from->q1 * to->q0 + from->q2 * to->q3 - from->q3 * to->q2;
	out->q2 = from->q0 * to->q2 + from->q2 * to->q0 + from->q3 * to->q1 - from->q1 * to->q3;
	out->q3 = from->q0 * to->q3 + from->q3 * to->q0 + from->q1 * to->q2 - from->q2 * to->q1;
}

//************************************AHRS raw data fream checking*********************//
u8 AHRSCheckDataFrame(void)
{
  u8 i = 0, j = 0, k = 0;
  s8 flag = -1;
  while (UART4RecvPtrR != UART4RecvPtrW) 
  {
    if (AHRSRecvState == NO_START_RCV) 
	{
      k = 0;
      i = UART4RecvPtrR;
      if ((u8)UART4RecvBuffer[i] == FIRST_FRAMEHEADER) 
			{
        k++;
      }
      
      i = (UART4RecvPtrR + 1);
      if ((u8)UART4RecvBuffer[i] == ACC_METER || (u8)UART4RecvBuffer[i] == GYRO || (u8)UART4RecvBuffer[i] == MAG_METER || (u8)UART4RecvBuffer[i] == ANGLE_OUTPUT) 
	  {
        k++;
      }

      if (k == 2) 
	  {
        //获取有效数据的起始地址和末尾地址（末尾地址指向整个数据帧的帧尾，校验和）
        AHRSRecvState = START_RCV;
        UART4RecvBufStart = UART4RecvPtrR;
        UART4RecvBufEnd = (UART4RecvPtrR + UART4RecvFrameLen);
      }      
    } 
	else 
	{
    if(UART4RecvPtrR == UART4RecvBufEnd) 
	  {
        AHRSRecvState = NO_START_RCV;
        
        j = UART4RecvBufStart;
        k = 0;
        for(i = 0; i < UART4RecvFrameLen; i++) 
				{
          k += UART4RecvBuffer[j];
         
          j = (j + 1);
        }
        if(k == UART4RecvBuffer[UART4RecvBufEnd]) 
				{
					flag = UART4RecvBuffer[UART4RecvBufStart + 1];
					k = 0;
					for(i = 0; i < (UART4RecvFrameLen + 1); i++) 
				  {
						j = UART4RecvBufStart + i;
						rawdata[i] = UART4RecvBuffer[j];
          }
					return (flag);
				}
     }
   }
		//waitting for RecvPtrR is heading to buffer end (UART4RecvPtrR ++)
    UART4RecvPtrR ++;//to find frame head,while recbuf is not overflow
  }
//  return (flag);	
}


//****************************************************************************************/
void SensorDataProcess(u8 type, raw_data *raw, raw1_data *raw1)
{
	//turn raw sensor datas into processed ones
	switch(type)
	{
		case 0x52:
		{					
			raw->gyro_x = rawdata[3] << 8;
			raw->gyro_x += rawdata[2];
			raw->gyro_y = rawdata[5] << 8;
			raw->gyro_y += rawdata[4];
			raw->gyro_z = rawdata[7] << 8;
			raw->gyro_z += rawdata[6];
			
			//turn degreen into rad
			raw1->gyro_x= 200.0f * raw->gyro_x / 32768.0 * 2000.0 / 180.0 * PI;
			raw1->gyro_y = 200.0f * raw->gyro_y / 32768.0 * 2000.0 / 180.0 * PI;
			raw1->gyro_z = -260.0f * raw->gyro_z / 32768.0 * 2000.0 / 180.0 * PI;
			if(fabs(raw1->gyro_x) < 0.2) raw1->gyro_x = 0.0f;
			if(fabs(raw1->gyro_y) < 0.2) raw1->gyro_y = 0.0f;
			if(fabs(raw1->gyro_z) < 0.2) raw1->gyro_z = 0.0f;
			break;
		}
		case 0x51:
		{		
			
			raw->acc_x= rawdata[3] << 8;
			raw->acc_x+= rawdata[2];
			raw->acc_y= rawdata[5] << 8;
			raw->acc_y+= rawdata[4];
			raw->acc_z= rawdata[7] << 8;
			raw->acc_z+= rawdata[6];			
			//turn into g
			raw1->acc_x= -1.0f * raw->acc_x/ 32768.0 * 16.0 ;
			raw1->acc_y= raw->acc_y/ 32768.0 * 16.0 ;
			raw1->acc_z= raw->acc_z/ 32768.0 * 16.0 ;
			break;
		}
		case 0x54:
		{
		
			raw->mag_x= rawdata[3] << 8;
			raw->mag_x+= rawdata[2];
			raw->mag_y= rawdata[5] << 8;
			raw->mag_y+= rawdata[4];
			raw->mag_z= rawdata[7] << 8;
			raw->mag_z+= rawdata[6];		
			
			raw1->mag_x= -1.0f * raw->mag_x;
			raw1->mag_y= raw->mag_y;
			raw1->mag_z= raw->mag_z;
			break;
		}
		default:;
	}
}


//*****************************sensor initialazing for AHRS computating****************************//
void SensorInitial(u8 type, qua *q)
{
	float m_x_t = 0.0f;
	float m_y_t = 0.0f;
	u8 flag = type;
	float sum = 0.0f;
	u8 j = 0;
	
	if(flag == ACC_METER || flag == GYRO || flag == ANGLE_OUTPUT || flag == MAG_METER)
	{		
		if(!qua_init)
		{
			SensorDataProcess(flag, &data_first, &data_second);
			switch(flag)
			{
				case 0x52:
				{
					buffer1[counter1] = data_second.gyro_x;
					buffer2[counter1] = data_second.gyro_y;
					buffer3[counter1] = data_second.gyro_z;
					counter1++;
					break;
				}
				case 0x51:
				{
					buffer4[counter2] = data_second.acc_x;
					buffer5[counter2] = data_second.acc_y;
					buffer6[counter2] = data_second.acc_z;
					counter2++;
					break;
				}
				case 0x54:
				{
					buffer7[counter3] = data_second.mag_x;
					buffer8[counter3] = data_second.mag_y;
					buffer9[counter3] = data_second.mag_z;
					counter3++;
					break;
				}
				default:
				{
					counter1++;
					counter2++;
					counter3++;
				}
			}
			if((counter1 > 6) && (counter2 > 6) && (counter3 > 6))
			{
				//为了速度加快，加入一个粗糙的加权均值滤波
				for(j = 0;j < counter1;j++)
				{
					sum += buffer1[j] * (j + 1);
				}
				bias = sum / 36.0f;
				sum = 0;
				w_x_bias = bias;
	
				for(j = 0;j < counter1;j++)
				{
					sum += buffer2[j] * (j + 1);
				}
				bias = sum / 36.0f;
				sum = 0;
				w_y_bias = bias;
				
				for(j = 0;j < counter1;j++)
				{
					sum += buffer3[j] * (j + 1);
				}
				bias = sum / 36.0f;
				sum = 0;
				w_z_bias = bias;

				for(j = 0;j < counter2;j++)
				{
					sum += buffer4[j];
				}
				bias = sum / (float)counter2;
				sum = 0;
				a_x_bias = bias;

				for(j = 0;j < counter2;j++)
				{
					sum += buffer5[j];
				}
				bias = sum / (float)counter2;
				sum = 0;
				a_y_bias = bias;

				for(j = 0;j < counter2;j++)
				{
					sum += buffer6[j];
				}
				bias = sum / (float)counter2;
				sum = 0;
				a_z_bias = bias;
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
				for(j = 0;j < counter3;j++)
				{
					sum += buffer7[j];
				}
				bias = sum / (float)counter3;
				sum = 0;
				m_x_bias = bias;

				for(j = 0;j < counter3;j++)
				{
					sum += buffer8[j];
				}
				bias = sum / (float)counter3;
				sum = 0;
				m_y_bias = bias;

				for(j = 0;j < counter3;j++)
				{
					sum += buffer9[j];
				}
				bias = sum / (float)counter3;
				sum = 0;
				m_z_bias = bias;
				qua_init = true;
			}
		}
		if(qua_init)
		{
			// normalise the accelerometer measurement
			norm(&a_x_bias, &a_y_bias, &a_z_bias);
			// normacclise the maccgnetometer meaccsurement
			norm(&m_x_bias, &m_y_bias, &m_z_bias);
			
			roll = PI - fabs(atan2(a_y_bias, a_z_bias));
			pitch = asin(a_x_bias);              
			m_x_t = m_x_bias * cos(roll) + m_y_bias * sin(roll) * sin(pitch) - m_z_bias * sin(roll) * cos(pitch);
			m_y_t = m_y_bias * cos(pitch) + m_z_bias * sin(pitch);
			
			if(m_x_t < 0) {m_x_t = -m_x_t;}
			if(m_y_t < 0) {m_y_t = -m_y_t;}
			if(m_y_bias > 0 && m_x_bias < 0) 
			{
				yaw = atan(m_y_t / m_x_t);
			}
			else if(m_y_bias < 0 && m_x_bias < 0)
			{
				yaw = -1.0f * atan(m_y_t / m_x_t);
			}
			else if(m_y_bias > 0 && m_x_bias > 0)
			{
				yaw = PI - atan(m_y_t / m_x_t);
			}
			else
			{
				yaw = -1.0f * (PI - atan(m_y_t / m_x_t));
			}
                     
			q->q0= cos(0.5 * roll) * cos(0.5 * pitch) * cos(0.5 * yaw) + sin(0.5 * roll) * sin(0.5 * pitch) * sin(0.5 * yaw);  
			q->q1= sin(0.5 * roll) * cos(0.5 * pitch) * cos(0.5 * yaw) - cos(0.5 * roll) * sin(0.5 * pitch) * sin(0.5 * yaw);  
			q->q2 = cos(0.5 * roll) * sin(0.5 * pitch) * cos(0.5 * yaw) + sin(0.5 * roll) * cos(0.5 * pitch) * sin(0.5 * yaw);  
			q->q3= cos(0.5 * roll) * cos(0.5 * pitch) * sin(0.5 * yaw) - sin(0.5 * roll) * sin(0.5 * pitch) * cos(0.5 * yaw);                                         
			counter1 = 0;counter2 = 0;counter3 = 0;
		}                                                  
	}
}



//*****************************AHRS quatanion iteration ************************************************//
void AHRS_iteration(u8 type, qua *q, sensor_data *s)
{
	u8 j = 0;
	float sum = 0.0f;
	u8 flag = type;
	float q0q1 = 0.0, q0q2 = 0.0, q0q3 = 0.0, q1q1 = 0.0, q1q2 = 0.0, 
		  q1q3 = 0.0, q2q2 = 0.0, q2q3 = 0.0, q3q3 = 0.0; 
	
	t1 = micros();
	if(flag == ACC_METER || flag == GYRO || flag == ANGLE_OUTPUT || flag == MAG_METER)
	{	
		SensorDataProcess(flag, &data_first, &data_second);
		
		buffer1[counter] = data_second.gyro_x;
		buffer2[counter] = data_second.gyro_y;
		buffer3[counter] = data_second.gyro_z;
		buffer4[counter] = data_second.acc_x;
		buffer5[counter] = data_second.acc_y;
		buffer6[counter] = data_second.acc_z;
		buffer7[counter] = data_second.mag_x;
		buffer8[counter] = data_second.mag_y;
		buffer9[counter] = data_second.mag_z;
		
		counter ++;
		
		if(counter > 6) counter = 0;
		
		//mean filter
		for( j = 0;j < 6;j++)
		{
			sum += buffer1[j] * (j + 1);
		}
		s->gyro_x = sum / 36.0f - w_x_bias;
		sum = 0;
				
		for( j = 0;j < 6;j++)
		{
			sum += buffer2[j] * (j + 1);
		}
		s->gyro_y = sum / 36.0f - w_y_bias;
		sum = 0;
		
		for( j = 0;j < 6;j++)
		{
			sum += buffer3[j] * (j + 1);
		}
		s->gyro_z = sum / 36.0f - w_z_bias;
		sum = 0;
					
		for( j = 0;j < 6;j++)
		{
			sum += buffer4[j];
		}
		s->acc_x = sum / 6.0f;
		sum = 0;
			
		for( j = 0;j < 6;j++)
		{
			sum += buffer5[j];
		}
		s->acc_y = sum / 6.0f;
		sum = 0;
					
		for( j = 0;j < 6;j++)
		{
			sum += buffer6[j];
		}
		s->acc_z = sum / 6.0f;
		sum = 0;

		for( j = 0;j < 6;j++)
		{
			sum += buffer7[j];
		}
		s->mag_x = sum / 6.0f;
		sum = 0;

		for( j = 0;j < 6;j++)
		{
			sum += buffer8[j];
		}
		s->mag_y = sum / 6.0f;
		sum = 0;
					
		for( j = 0;j < 6;j++)
		{
			sum += buffer9[j];
		}
		s->mag_z = sum / 6.0f;
		sum = 0;
		cal_ready = true;
	}
	
	//**************************update quanion***************************//
	if(cal_ready)
	{ 
		//abort adjust without MAG,only using gravity
		if(cut_off) 
		{
			 //the interial calculation takes about 400ms,if need 8s to cutoff mag so, count should be 
			cut_count++;
			if(cut_count == 800)
			{
				cut_count = 0;
				cut_off = false;
			}				
			else
			{
				norm(&(s->acc_x), &(s->acc_y), &(s->acc_z));
			
				// Auxiliary variables to avoid repeated arithmetic
				q0q1 = q->q0 * q->q1;
				q0q2 = q->q0 * q->q2;
				q0q3 = q->q0 * q->q3;
				q1q1 = q->q1 * q->q1;
				q1q2 = q->q1 * q->q2;
				q1q3 = q->q1 * q->q3;
				q2q2 = q->q2 * q->q2;
				q2q3 = q->q2 * q->q3;
				q3q3 = q->q3 * q->q3; 
			
				// Estimated direction of gravity (????á|?òá?[0 0 0 1g]óée?μ×aμ?s?￡￡?μ?ê?′?′|ó|???a[0 0 0 -1]￡?
				vx = 2.0 * (q1q3 - q0q2) * -1.0f;
				vy = 2.0 * (q0q1 + q2q3);
				vz = 2.0 * (0.5f - q1q1 - q2q2);
			
				// Error is sum of cross product between estimated direction and measured direction of field vectors(2?3?±í?ó2?￡?′?3?ó???á|3?2?3??óoí)
				//maybe ex,ey,ex is in degreen ,which should be turn into rads(since ex,ey,ez is much too huge, making w_x,w_y,w_z turnning all the time	)
				ex = (s->acc_y * vz - s->acc_z * vy);
				ey = (s->acc_z * vx - s->acc_x * vz);
				ez = (s->acc_x * vy - s->acc_y * vx);
			
				if(fabs(ex) < 0.001){ex = 0.0f;}
				if(fabs(ey) < 0.001){ey = 0.0f;}
				if(fabs(ez) < 0.001){ez = 0.0f;}
			
				// Compute and apply integral feedback if enabled(ó?2?3??ó2?PIDT?yíó?Yò?)
				if(ex != 0.0f && ey != 0.0f && ez != 0.0f) 
				{
					integralFb_x += twoKi_x * ex ;	// integral error scaled by Ki
					integralFb_y += twoKi_y * ey ;
					integralFb_z += twoKi_z * ez ;
		
					s->gyro_x+= integralFb_x;	// apply integral feedback
					s->gyro_y+= integralFb_y;
					s->gyro_z+= integralFb_z;
				}
				else 
				{
					integralFb_x = 0.0f;	// prevent integral gyroindup
					integralFb_y = 0.0f;
					integralFb_z = 0.0f;
				}

				// Apply proportional feedback
				s->gyro_x+= twoKp_x * ex;
				s->gyro_y+= twoKp_y * ey;
				s->gyro_z += twoKp_z * ez;
	
				gyro_x_f = s->gyro_x;
				gyro_y_f = s->gyro_y;
				gyro_z_f = s->gyro_z;
			}
		}
		else                             							 //adjust with MAG
		{			
			// normalise the accelerometer measurement
			norm(&(s->acc_x), &(s->acc_y), &(s->acc_z));

			// normacclise the maccgnetometer meaccsurement
			norm(&(s->mag_x), &(s->mag_y), &(s->mag_z));
		
			// Auxiliary variables to avoid repeated arithmetic
			q0q1 = q->q0 * q->q1;
			q0q2 = q->q0 * q->q2;
			q0q3 = q->q0 * q->q3;
			q1q1 = q->q1 * q->q1;
			q1q2 = q->q1 * q->q2;
			q1q3 = q->q1 * q->q3;
			q2q2 = q->q2 * q->q2;
			q2q3 = q->q2 * q->q3;
			q3q3 = q->q3 * q->q3; 
	
			// Reference direction of Earth's magnetic field
			h_x = 2.0f * (s->mag_x * (0.5f - q2q2 - q3q3) + s->mag_y * (q1q2 - q0q3) + s->mag_z * (q1q3 + q0q2));
			h_y = 2.0f * (s->mag_x * (q1q2 + q0q3) + s->mag_y * (0.5f - q1q1 - q3q3) + s->mag_z * (q2q3 - q0q1));
			b_x = (float)(sqrt((h_x * h_x) + (h_y * h_y)));
			b_z = 2.0f * (s->mag_x * (q1q3 - q0q2) + s->mag_y * (q2q3 + q0q1) + s->mag_z * (0.5f - q1q1 - q2q2));
			
			// Estimated direction of gravity 
			vx = 2.0 * (q1q3 - q0q2) * -1.0f;
			vy = 2.0 * (q0q1 + q2q3) ;
			vz = 2.0 * (0.5f - q1q1 - q2q2);
			// Estimated direction of magnetic 
			wx = -1.0f * 2.0 * (b_x * (0.5f - q2q2 - q3q3) + b_z * (q1q3 - q0q2));
			wy =  2.0 * (b_x * (q1q2 - q0q3) + b_z * (q0q1 + q2q3));
			wz =  2.0 * (b_x * (q0q2 + q1q3) + b_z * (0.5f - q1q1 - q2q2)); 
			
			// Error is sum of cross product between estimated direction and measured direction of field vectors
			ex = (s->acc_y * vz - s->acc_z * vy) + (s->mag_y * wz - s->mag_z * wy);
			ey = (s->acc_z * vx - s->acc_x * vz) + (s->mag_z * wx - s->mag_x * wz);
			ez = (s->acc_x * vy - s->acc_y * vx) + (s->mag_x * wy - s->mag_y * wx);
			
			if(fabs(ex) < 0.001){ex = 0.0f;}
			if(fabs(ey) < 0.001){ey = 0.0f;}
			if(fabs(ez) < 0.001){ez = 0.0f;}
			
			// Compute and apply integral feedback if enabled(ó?2?3??ó2?PIDT?yíó?Yò?)
			if(ex != 0.0f && ey != 0.0f && ez != 0.0f) 
			{
				integralFb_x += twoKi_x * ex ;	// integral error scaled by Ki
				integralFb_y += twoKi_y * ey ;
				integralFb_z += twoKi_z * ez ;
		
				s->gyro_x += integralFb_x;	// apply integral feedback
				s->gyro_y += integralFb_y;
				s->gyro_z += integralFb_z;
			}
			else 
			{
				integralFb_x = 0.0f;	// prevent integral gyroindup
				integralFb_y = 0.0f;
				integralFb_z = 0.0f;
			}

			// Apply proportional feedback
			s->gyro_x += twoKp_x * ex;
			s->gyro_y += twoKp_y * ey;
			s->gyro_z += twoKp_z * ez;
	
			gyro_x_f = s->gyro_x;
			gyro_y_f = s->gyro_y;
			gyro_z_f = s->gyro_z;
		}

		//quatanion interation
		t2 = micros();
		if(t2 > t1)
		{
			interval = (t2 -t1) * 0.000001;
		}
		else
		{
			interval = (float)((total + t2 - t1) * 0.000001);
		}
		
		q->q0+= (-q->q1 * gyro_x_f - q->q2 * gyro_y_f * -1.0f - q->q3 * gyro_z_f * -1.0f) * interval / 2.0f;
		q->q1+= (q->q0 * gyro_x_f + q->q2 * gyro_z_f * -1.0f - q->q3 * gyro_y_f * -1.0f) *  interval / 2.0f;
		q->q2+= (q->q0 * gyro_y_f * -1.0f - q->q1 * gyro_z_f * -1.0f + q->q3 * gyro_x_f) *  interval / 2.0f;
		q->q3+= (q->q0 * gyro_z_f * -1.0f + q->q1 * gyro_y_f * -1.0f - q->q2 * gyro_x_f) *  interval / 2.0f; 

		// Normalise quaternion
		qua_norm(q);
	}
}


//*****************************AHRS quatanion iteration by qua************************************************//
void AHRS_iteration_qua(u8 type, qua *q, sensor_data *s)
{
	u8 j = 0;
	float sum = 0.0f;
	u8 flag = type;
	float hx_real = 0.0, hy_real = 0.0, hz_real = 0.0,       //h为法线 
		  cx_real = 0.0, cy_real = 0.0, cz_real = 0.0; 	 //c为对角线
	float hx_img = 0.0, hy_img = 0.0, hz_img = 0.0,       //h为法线 
		  cx_img = 0.0, cy_img = 0.0, cz_img = 0.0; 	 //c为对角线	
	float q0q1 = 0.0, q0q2 = 0.0, q0q3 = 0.0, q1q1 = 0.0, q1q2 = 0.0, 
		  q1q3 = 0.0, q2q2 = 0.0, q2q3 = 0.0, q3q3 = 0.0; 
	t1 = micros();
	if(flag == ACC_METER || flag == GYRO || flag == ANGLE_OUTPUT || flag == MAG_METER)
	{	
		SensorDataProcess(flag, &data_first, &data_second);
		
		buffer1[counter] = data_second.gyro_x;
		buffer2[counter] = data_second.gyro_y;
		buffer3[counter] = data_second.gyro_z;
		buffer4[counter] = data_second.acc_x;
		buffer5[counter] = data_second.acc_y;
		buffer6[counter] = data_second.acc_z;
		buffer7[counter] = data_second.mag_x;
		buffer8[counter] = data_second.mag_y;
		buffer9[counter] = data_second.mag_z;
		
		counter ++;
		
		if(counter > 6) counter = 0;
		
		//mean filter
		for( j = 0;j < 6;j++)
		{
			sum += buffer1[j] * (j + 1);
		}
		s->gyro_x = sum / 36.0f - w_x_bias;
		sum = 0;
				
		for( j = 0;j < 6;j++)
		{
			sum += buffer2[j] * (j + 1);
		}
		s->gyro_y = sum / 36.0f - w_y_bias;
		sum = 0;
		
		for( j = 0;j < 6;j++)
		{
			sum += buffer3[j] * (j + 1);
		}
		s->gyro_z = sum / 36.0f - w_z_bias;
		sum = 0;
					
		for( j = 0;j < 6;j++)
		{
			sum += buffer4[j];
		}
		s->acc_x = sum / 6.0f;
		sum = 0;
			
		for( j = 0;j < 6;j++)
		{
			sum += buffer5[j];
		}
		s->acc_y = sum / 6.0f;
		sum = 0;
					
		for( j = 0;j < 6;j++)
		{
			sum += buffer6[j];
		}
		s->acc_z = sum / 6.0f;
		sum = 0;

		for( j = 0;j < 6;j++)
		{
			sum += buffer7[j];
		}
		s->mag_x = sum / 6.0f;
		sum = 0;

		for( j = 0;j < 6;j++)
		{
			sum += buffer8[j];
		}
		s->mag_y = sum / 6.0f;
		sum = 0;
					
		for( j = 0;j < 6;j++)
		{
			sum += buffer9[j];
		}
		s->mag_z = sum / 6.0f;
		sum = 0;
		cal_ready = true;
	}
	
	//**************************update quanion***************************//
	if(cal_ready)
	{ 	
		// normalise the accelerometer measurement
		norm(&(s->acc_x), &(s->acc_y), &(s->acc_z));

		// normacclise the maccgnetometer meaccsurement
		norm(&(s->mag_x), &(s->mag_y), &(s->mag_z));
			
		// Auxiliary variables to avoid repeated arithmetic
		q0q1 = q->q0 * q->q1;
		q0q2 = q->q0 * q->q2;
		q0q3 = q->q0 * q->q3;
		q1q1 = q->q1 * q->q1;
		q1q2 = q->q1 * q->q2;
		q1q3 = q->q1 * q->q3;
		q2q2 = q->q2 * q->q2;
		q2q3 = q->q2 * q->q3;
		q3q3 = q->q3 * q->q3; 
			
		// Reference direction of Earth's magnetic field
		h_x = 2.0f * (s->mag_x * (0.5f - q2q2 - q3q3) + s->mag_y * (q1q2 - q0q3) + s->mag_z * (q1q3 + q0q2));
		h_y = 2.0f * (s->mag_x * (q1q2 + q0q3) + s->mag_y * (0.5f - q1q1 - q3q3) + s->mag_z * (q2q3 - q0q1));
		b_x = -1.0f* (float)(sqrt((h_x * h_x) + (h_y * h_y)));
		b_z = -2.0f * (s->mag_x * (q1q3 - q0q2) + s->mag_y * (q2q3 + q0q1) + s->mag_z * (0.5f - q1q1 - q2q2));
			
		//Estimated direction of gravity(都做了归一化了，不用担心单位)
		vx = 0.0;
		vy = 0.0;
		vz = -1.0;
		// Estimated direction of magnetic(earth's) 
		norm(&b_x, &b_y, &b_z);

		//计算法线和对角线 用两种acc和mag分别算h和c
		vector_corss(vx, vy, vz, b_x, b_y, b_z, &hx_img, &hy_img, &hz_img);
		vector_add(vx, vy, vz, b_x, b_y, b_z, &cx_img, &cy_img, &cz_img);
		vector_corss(s->acc_x, s->acc_y, s->acc_z, s->mag_x, s->mag_y, s->mag_z, &hx_real, &hy_real, &hz_real);
		vector_add(s->acc_x, s->acc_y, s->acc_z, s->mag_x, s->mag_y, s->mag_z, &cx_real, &cy_real, &cz_real);			
			
		//use (vx,vy,vz) & (b_x,b_y,b_z)as to,s->acc & s->mag as from,caculate acc_q & mag_q(因为我一直求的四元数是SEq.而非ESq)
		vector2qua(hx_real, hy_real, hz_real, hx_img, hy_img, hz_img, &acc_q);
		vector2qua(cx_real, cy_real, cz_real, cx_img, cy_img, cz_img, &mag_q);
		

		//quatanion interation
		qua_multiply(&acc_q, &mag_q, &a_m_q);
		qua_norm(&a_m_q);
		q->q0+= (-q->q1 * s->gyro_x - q->q2 * s->gyro_y * -1.0f - q->q3 * s->gyro_z) * interval / 2.0f;
		q->q1+= (q->q0 * s->gyro_x + q->q2 * s->gyro_z - q->q3 * s->gyro_y * -1.0f) *  interval / 2.0f;
		q->q2+= (q->q0 * s->gyro_y * -1.0f - q->q1 * s->gyro_z + q->q3 * s->gyro_x) *  interval / 2.0f;
		q->q3+= (q->q0 * s->gyro_z + q->q1 * s->gyro_y * -1.0f - q->q2 * s->gyro_x) *  interval / 2.0f; 

		// Normalise quaternion
		qua_norm(q);

		//sensor fusing (complementary filter)暂定0.02
		q->q0 = 0.95 * q->q0 + a_m_q.q0 * 0.05;
		q->q1 = 0.95 * q->q1 + a_m_q.q1 * 0.05;
		q->q2 = 0.95 * q->q2 + a_m_q.q2 * 0.05;
		q->q3 = 0.95 * q->q3 + a_m_q.q3 * 0.05;
		//time taken
		t2 = micros();
		if(t2 > t1)
		{
			interval = (t2 -t1) * 0.000001;
		}
		else
		{
			interval = (float)((total + t2 - t1) * 0.000001);
		}
	}
}

void AHRS_computeEuler(qua *q)
{
	pitch = -1.0f * asin(-2*q->q1*q->q3 + 2*q->q0*q->q2) / PI * 180.0;	 
  roll  = atan2(2*q->q2*q->q3 + 2*q->q0*q->q1,-2*q->q1*q->q1 - 2*q->q2*q->q2 + 1) / PI * 180;
	yaw   = atan2(2*q->q1*q->q2 + 2*q->q0*q->q3,-2*q->q2*q->q2 - 2*q->q3*q->q3 + 1) / PI * 180;
}



//*****************************AHRS computating task*************************************************//
void AHRS_compute() 
{
  u8 flag;
  flag = AHRSCheckDataFrame();
  AHRS_iteration_qua(flag, &SEq, &s_data);
  AHRS_computeEuler(&SEq);
}











