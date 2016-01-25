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
s16 acc_x_raw = 0, acc_y_raw = 0, acc_z_raw = 0, gyro_x_raw = 0, gyro_y_raw = 0,
	gyro_z_raw = 0, mag_x_raw = 0, mag_y_raw = 0, mag_z_raw = 0;
u8 rawdata[11] = {0};
float acc_x = 0.0f, acc_y = 0.0f, acc_z = 0.0f, gyro_x = 0.0f, gyro_y = 0.0f, gyro_z = 0.0f, mag_x = 0.0f, mag_y = 0.0f,
	  mag_z = 0.0f, gyro_x_0 = 0.0f, gyro_y_0 = 0.0f, gyro_z_0 = 0.0f, gyro_x_f = 0.0f, gyro_y_f = 0.0f, gyro_z_f = 0.0f; 
float gyro_x_p = 0.0f, gyro_y_p = 0.0f, gyro_z_p = 0.0f;
float a_x = 0.0f, a_y = 0.0f, a_z = -1.0f, w_x = 0.0f, w_y = 0.0f, w_z = 0.0f, m_x = -1.0f, m_y = 0.0f, m_z = 0.0f;
float b_x = -1.0f, b_z = 0.0f;
float h_x = -1.0f, h_y = 0.0f, h_z = 0.0f;

float SEq_1 = 1.0f, SEq_2 = 0.0f, SEq_3 = 0.0f, SEq_4 = 0.0f;
//float SEq_1_n = 1.0f, SEq_2_n = 0.0f, SEq_3_n = 0.0f, SEq_4_n = 0.0f;     //用于构造梯形结构解微分方程


float twoKp_z = 260.0f, twoKp_x = 260.0, twoKp_y = 260.0, twoKi_z = 10.0, twoKi_x = 10.0, twoKi_y = 10.0, twoKd_x = 0.0, twoKd_y = 0.0, twoKd_z = 0.0;
float vx = 0.0f, vy = 0.0f, vz = 0.0f, wx = 0.0f, wy = 0.0f, wz = 0.0f;
static char cut_count;
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
float pex = 0.0f, pey = 0.0f, pez = 0.0f;
float dex = 0.0f, dey = 0.0f, dez = 0.0f;

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
//		UART4WriteDataToBuffer(UART4RecvBuffer + UART4RecvBufStart, UART4RecvFrameLen);
        if(k == UART4RecvBuffer[UART4RecvBufEnd]) 
				{
          //UART4RecvBufStart ++;
					flag = UART4RecvBuffer[UART4RecvBufStart + 1];
					k = 0;
					for(i = 0; i < (UART4RecvFrameLen + 1); i++) 
				  {
						j = UART4RecvBufStart + i;
						rawdata[i] = UART4RecvBuffer[j];
          }
					return (flag);
					//UART4RecvBufStart = UART4RecvBuffer[UART4RecvBufEnd] + 1;
				}
     }
   }
		//waitting for RecvPtrR is heading to buffer end (UART4RecvPtrR ++)
    UART4RecvPtrR ++;//to find frame head,while recbuf is not overflow
  }
//  return (flag);	
}


//****************************************************************************************/
void SensorDataProcess(u8 type)
{
	//turn raw sensor datas into processed ones
	switch(type)
	{
		case 0x52:
		{					
			gyro_x_raw = rawdata[3] << 8;
			gyro_x_raw += rawdata[2];
			gyro_y_raw = rawdata[5] << 8;
			gyro_y_raw += rawdata[4];
			gyro_z_raw = rawdata[7] << 8;
			gyro_z_raw += rawdata[6];
			
			//turn degreen into rad
			w_x = 200.0f * gyro_x_raw / 32768.0 * 2000.0 / 180.0 * PI;
			w_y = 200.0f * gyro_y_raw / 32768.0 * 2000.0 / 180.0 * PI;
			w_z = -200.0f * gyro_z_raw / 32768.0 * 2000.0 / 180.0 * PI;
			if(fabs(w_x) < 0.2) w_x = 0.0f;
			if(fabs(w_y) < 0.2) w_y = 0.0f;
			if(fabs(w_z) < 0.2) w_z = 0.0f;

			break;
		}
		case 0x51:
		{		
			
			acc_x_raw = rawdata[3] << 8;
			acc_x_raw += rawdata[2];
			acc_y_raw = rawdata[5] << 8;
			acc_y_raw += rawdata[4];
			acc_z_raw = rawdata[7] << 8;
			acc_z_raw += rawdata[6];
			
			//turn into g
			a_x = -1.0f * acc_x_raw / 32768.0 * 16.0 ;
			a_y = acc_y_raw / 32768.0 * 16.0 ;
			a_z = acc_z_raw / 32768.0 * 16.0 ;
			break;
		}
		case 0x54:
		{
		
			mag_x_raw = rawdata[3] << 8;
			mag_x_raw += rawdata[2];
			mag_y_raw = rawdata[5] << 8;
			mag_y_raw += rawdata[4];
			mag_z_raw = rawdata[7] << 8;
			mag_z_raw += rawdata[6];		
			
			m_x = -1.0f * mag_x_raw;
			m_y = mag_y_raw;
			m_z = mag_z_raw;
			break;
		}
		default:;
	}
}


//*****************************sensor initialazing for AHRS computating****************************//
void SensorInitial(u8 type)
{
	u8 j = 0;
	float norm = 0.0f;
	float m_x_t = 0.0f;
	float m_y_t = 0.0f;
	u8 flag = type;
	float sum = 0.0f;
	
	/*u8 counter1 = 0;
	u8 counter2 = 0;
	u8 counter3 = 0;*/
	if(flag == ACC_METER || flag == GYRO || flag == ANGLE_OUTPUT || flag == MAG_METER)
	{		
		if(!qua_init)
		{
			SensorDataProcess(flag);
			switch(flag)
			{
				case 0x52:
				{
					buffer1[counter1] = w_x;
					buffer2[counter1] = w_y;
					buffer3[counter1] = w_z;
					counter1++;
					break;
				}
				case 0x51:
				{
					buffer4[counter2] = a_x;
					buffer5[counter2] = a_y;
					buffer6[counter2] = a_z;
					counter2++;
					break;
				}
				case 0x54:
				{
					buffer7[counter3] = m_x;
					buffer8[counter3] = m_y;
					buffer9[counter3] = m_z;
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
			norm = invSqrt(a_x_bias * a_x_bias + a_y_bias * a_y_bias + a_z_bias * a_z_bias);
			a_x_bias *= norm;
			a_y_bias *= norm;
			a_z_bias *= norm;
			// normacclise the maccgnetometer meaccsurement
			norm = invSqrt(m_x_bias * m_x_bias + m_y_bias * m_y_bias + m_z_bias * m_z_bias);
			m_x_bias *= norm;
			m_y_bias *= norm;
			m_z_bias *= norm;
			
			roll = atan2(a_y_bias, a_z_bias);
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
                     
			SEq_1 = cos(0.5 * roll) * cos(0.5 * pitch) * cos(0.5 * yaw) + sin(0.5 * roll) * sin(0.5 * pitch) * sin(0.5 * yaw);  
			SEq_2 = sin(0.5 * roll) * cos(0.5 * pitch) * cos(0.5 * yaw) - cos(0.5 * roll) * sin(0.5 * pitch) * sin(0.5 * yaw);  
			SEq_3 = cos(0.5 * roll) * sin(0.5 * pitch) * cos(0.5 * yaw) + sin(0.5 * roll) * cos(0.5 * pitch) * sin(0.5 * yaw);  
			SEq_4 = cos(0.5 * roll) * cos(0.5 * pitch) * sin(0.5 * yaw) - sin(0.5 * roll) * sin(0.5 * pitch) * cos(0.5 * yaw);                                         
			counter1 = 0;counter2 = 0;counter3 = 0;
		}                                                  
	}
}



//*****************************AHRS quatanion iteration ************************************************//
void AHRS_iteration(u8 type)
{
	//*******************************???ˉ′°?ú??2¨*******************************************************//
	u8 j = 0;
	float norm = 0.0f, sum = 0.0f;
	u8 flag = type;
	float q0q1 = 0.0, q0q2 = 0.0, q0q3 = 0.0, q1q1 = 0.0, q1q2 = 0.0, 
		  q1q3 = 0.0, q2q2 = 0.0, q2q3 = 0.0, q3q3 = 0.0; 
	
	t1 = micros();
	if(flag == ACC_METER || flag == GYRO || flag == ANGLE_OUTPUT || flag == MAG_METER)
	{	
		SensorDataProcess(flag);
		
		buffer1[counter] = w_x;
		buffer2[counter] = w_y;
		buffer3[counter] = w_z;
		buffer4[counter] = a_x;
		buffer5[counter] = a_y;
		buffer6[counter] = a_z;
		buffer7[counter] = m_x;
		buffer8[counter] = m_y;
		buffer9[counter] = m_z;
		
		counter ++;
		
		if(counter > 6) counter = 0;
		
		//mean filter
		for( j = 0;j < 6;j++)
		{
			sum += buffer1[j] * (j + 1);
		}
		gyro_x = sum / 36.0f - w_x_bias;
		sum = 0;
				
		for( j = 0;j < 6;j++)
		{
			sum += buffer2[j] * (j + 1);
		}
		gyro_y = sum / 36.0f - w_y_bias;
		sum = 0;
		
		for( j = 0;j < 6;j++)
		{
			sum += buffer3[j] * (j + 1);
		}
		gyro_z = sum / 36.0f - w_z_bias;
		sum = 0;
					
		for( j = 0;j < 6;j++)
		{
			sum += buffer4[j];
		}
		acc_x = sum / 6.0f;
		sum = 0;
			
		for( j = 0;j < 6;j++)
		{
			sum += buffer5[j];
		}
		acc_y = sum / 6.0f;
		sum = 0;
					
		for( j = 0;j < 6;j++)
		{
			sum += buffer6[j];
		}
		acc_z = sum / 6.0f;
		sum = 0;

		for( j = 0;j < 6;j++)
		{
			sum += buffer7[j];
		}
		mag_x = sum / 6.0f;
		sum = 0;

		for( j = 0;j < 6;j++)
		{
			sum += buffer8[j];
		}
		mag_y = sum / 6.0f;
		sum = 0;
					
		for( j = 0;j < 6;j++)
		{
			sum += buffer9[j];
		}
		mag_z = sum / 6.0f;
		sum = 0;
		cal_ready = true;
	}
	
	//**************************update quanion***************************//
	if(cal_ready)
	{ 
		//if((fabs(gyro_x - gyro_x_p) > 20.0) || (fabs(gyro_y - gyro_y_p) > 20.0) || (fabs(gyro_z - gyro_z_p) > 20.0) || !cut_off) cut_off = true;	
		if(fabs(gyro_z - gyro_z_p) > 5.0 || fabs(gyro_z_p - gyro_z) > 5.0) cut_off = true;
		gyro_x_p = gyro_x;
		gyro_y_p = gyro_y;
		gyro_z_p = gyro_z;
		//abort adjust without MAG,only using gravity
		if(cut_off) 
		{
			 //the interial calculation takes about 400ms,if need 8s to cutoff mag so, count should be 
			cut_count++;
			if(cut_count == (char)250)
			{
				cut_count = 0;
				cut_off = false;
			}				
			else
			{
				norm = invSqrt(acc_x * acc_x + acc_y * acc_y + acc_z * acc_z);
				acc_x *= norm;
				acc_y *= norm;
				acc_z *= norm;
			
				// Auxiliary variables to avoid repeated arithmetic
				//q0q0 = SEq_1 * SEq_1;
				q0q1 = SEq_1 * SEq_2;
				q0q2 = SEq_1 * SEq_3;
				q0q3 = SEq_1 * SEq_4;
				q1q1 = SEq_2 * SEq_2;
				q1q2 = SEq_2 * SEq_3;
				q1q3 = SEq_2 * SEq_4;
				q2q2 = SEq_3 * SEq_3;
				q2q3 = SEq_3 * SEq_4;
				q3q3 = SEq_4 * SEq_4; 
			
				// Estimated direction of gravity (????á|?òá?[0 0 0 1g]óée?μ×aμ?s?￡￡?μ?ê?′?′|ó|???a[0 0 0 -1]￡?
				vx = 2.0 * (q1q3 - q0q2) * -1.0f;
				vy = 2.0 * (q0q1 + q2q3);
				vz = 2.0 * (0.5f - q1q1 - q2q2);
			
				// Error is sum of cross product between estimated direction and measured direction of field vectors(2?3?±í?ó2?￡?′?3?ó???á|3?2?3??óoí)
				//maybe ex,ey,ex is in degreen ,which should be turn into rads(since ex,ey,ez is much too huge, making w_x,w_y,w_z turnning all the time	)
				ex = (acc_y * vz - acc_z * vy);
				ey = (acc_z * vx - acc_x * vz);
				ez = (acc_x * vy - acc_y * vx);
			
				if(fabs(ex) < 0.001){ex = 0.0f;}
				if(fabs(ey) < 0.001){ey = 0.0f;}
				if(fabs(ez) < 0.001){ez = 0.0f;}
			
				dex = ex - pex;
				dey = ey - pey;
				dez = ez - pez;
			
				pex = ex;
				pey = ey;
				pez = ez;
				// Compute and apply integral feedback if enabled(ó?2?3??ó2?PIDT?yíó?Yò?)
				if(ex != 0.0f && ey != 0.0f && ez != 0.0f) 
				{
					integralFb_x += twoKi_x * ex ;	// integral error scaled by Ki
					integralFb_y += twoKi_y * ey ;
					integralFb_z += twoKi_z * ez ;
		
					gyro_x += integralFb_x;	// apply integral feedback
					gyro_y += integralFb_y;
					gyro_z += integralFb_z;
				}
				else 
				{
					integralFb_x = 0.0f;	// prevent integral gyroindup
					integralFb_y = 0.0f;
					integralFb_z = 0.0f;
				}

				// Apply proportional feedback
				gyro_x += twoKp_x * ex;
				gyro_y += twoKp_y * ey;
				gyro_z += twoKp_z * ez;

				// Apply  Derivative feedback
				gyro_x += twoKd_x * dex;
				gyro_y += twoKd_y * dey;
				gyro_z += twoKd_z * dez;
	
				gyro_x_f = gyro_x;
				gyro_y_f = gyro_y;
				gyro_z_f = gyro_z;
			}
		}
		else                             							 //adjust with MAG
		{			
			// normalise the accelerometer measurement
			norm = invSqrt(acc_x * acc_x + acc_y * acc_y + acc_z * acc_z);
			acc_x *= norm;
			acc_y *= norm;
			acc_z *= norm;
			// normacclise the maccgnetometer meaccsurement
			norm = invSqrt(mag_x * mag_x + mag_y * mag_y + mag_z * mag_z);
			mag_x *= norm;
			mag_y *= norm;
			mag_z *= norm;
			
			// Auxiliary variables to avoid repeated arithmetic
			//q0q0 = SEq_1 * SEq_1;
			q0q1 = SEq_1 * SEq_2;
			q0q2 = SEq_1 * SEq_3;
			q0q3 = SEq_1 * SEq_4;
			q1q1 = SEq_2 * SEq_2;
			q1q2 = SEq_2 * SEq_3;
			q1q3 = SEq_2 * SEq_4;
			q2q2 = SEq_3 * SEq_3;
			q2q3 = SEq_3 * SEq_4;
			q3q3 = SEq_4 * SEq_4; 
			
			// Reference direction of Earth's magnetic field(í?é?μ?3ìDòè?????óD?D??b_xμ??y?o￡??ò???òμ?′?3?ó|???ú2é?ˉê±3?ò?-1￡?ò?±￡?¤′?á|ó??y·??òò??á)
			// ′??D?÷2a3?μ?′?á|￡?′óS?μ×aμ?e?μ￡¨ESq X Sd X SEq￡?
			h_x = 2.0f * (mag_x * (0.5f - q2q2 - q3q3) + mag_y * (q1q2 - q0q3) + mag_z * (q1q3 + q0q2));
			h_y = 2.0f * (mag_x * (q1q2 + q0q3) + mag_y * (0.5f - q1q1 - q3q3) + mag_z * (q2q3 - q0q1));
			b_x = (float)(sqrt((h_x * h_x) + (h_y * h_y)));
			b_z = 2.0f * (mag_x * (q1q3 - q0q2) + mag_y * (q2q3 + q0q1) + mag_z * (0.5f - q1q1 - q2q2));
			
			// Estimated direction of gravity (????á|?òá?[0 0 0 1g]óée?μ×aμ?s?￡￡?μ?ê?′?′|ó|???a[0 0 0 -1]￡?
			vx = 2.0 * (q1q3 - q0q2) * -1.0f;
			vy = 2.0 * (q0q1 + q2q3) ;
			vz = 2.0 * (0.5f - q1q1 - q2q2);
			// Estimated direction of magnetic 
			wx = -1.0f * 2.0 * (b_x * (0.5f - q2q2 - q3q3) + b_z * (q1q3 - q0q2));
			wy =  2.0 * (b_x * (q1q2 - q0q3) + b_z * (q0q1 + q2q3));
			wz =  2.0 * (b_x * (q0q2 + q1q3) + b_z * (0.5f - q1q1 - q2q2)); 
			
			// Error is sum of cross product between estimated direction and measured direction of field vectors(2?3?±í?ó2?￡?′?3?ó???á|3?2?3??óoí)
			//maybe ex,ey,ex is in degreen ,which should be turn into rads(since ex,ey,ez is much too huge, making w_x,w_y,w_z turnning all the time	)
			ex = (acc_y * vz - acc_z * vy) + (mag_y * wz - mag_z * wy);
			ey = (acc_z * vx - acc_x * vz) + (mag_z * wx - mag_x * wz);
			ez = (acc_x * vy - acc_y * vx) + (mag_x * wy - mag_y * wx);
			
			if(fabs(ex) < 0.001){ex = 0.0f;}
			if(fabs(ey) < 0.001){ey = 0.0f;}
			if(fabs(ez) < 0.001){ez = 0.0f;}
			
			dex = ex - pex;
			dey = ey - pey;
			dez = ez - pez;
			
			pex = ex;
			pey = ey;
			pez = ez;
			// Compute and apply integral feedback if enabled(ó?2?3??ó2?PIDT?yíó?Yò?)
			if(ex != 0.0f && ey != 0.0f && ez != 0.0f) 
			{
				integralFb_x += twoKi_x * ex ;	// integral error scaled by Ki
				integralFb_y += twoKi_y * ey ;
				integralFb_z += twoKi_z * ez ;
		
				gyro_x += integralFb_x;	// apply integral feedback
				gyro_y += integralFb_y;
				gyro_z += integralFb_z;
			}
			else 
			{
				integralFb_x = 0.0f;	// prevent integral gyroindup
				integralFb_y = 0.0f;
				integralFb_z = 0.0f;
			}

			// Apply proportional feedback
			gyro_x += twoKp_x * ex;
			gyro_y += twoKp_y * ey;
			gyro_z += twoKp_z * ez;

			// Apply  Derivative feedback
			gyro_x += twoKd_x * dex;
			gyro_y += twoKd_y * dey;
			gyro_z += twoKd_z * dez;
	
			gyro_x_f = gyro_x;
			gyro_y_f = gyro_y;
			gyro_z_f = gyro_z;
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
		
		SEq_1 += (-SEq_2 * gyro_x_f - SEq_3 * gyro_y_f * -1.0f - SEq_4 * gyro_z_f * -1.0f) * interval / 2.0f;
		SEq_2 += (SEq_1 * gyro_x_f + SEq_3 * gyro_z_f * -1.0f - SEq_4 * gyro_y_f * -1.0f) *  interval / 2.0f;
		SEq_3 += (SEq_1 * gyro_y_f * -1.0f - SEq_2 * gyro_z_f * -1.0f + SEq_4 * gyro_x_f) *  interval / 2.0f;
		SEq_4 += (SEq_1 * gyro_z_f * -1.0f + SEq_2 * gyro_y_f * -1.0f - SEq_3 * gyro_x_f) *  interval / 2.0f; 

		// Normalise quaternion
		norm = invSqrt(SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4);
		SEq_1 *= norm;
		SEq_2 *= norm;
		SEq_3 *= norm;
		SEq_4 *= norm;	
	}
}

void AHRS_computeEuler(void)
{
	u8 ack_frame[8];
	pitch = -1.0f * asin(-2*SEq_2*SEq_4 + 2*SEq_1*SEq_3) / PI * 180.0;	 
    roll  = atan2(2*SEq_3*SEq_4 + 2*SEq_1*SEq_2,-2*SEq_2*SEq_2 - 2*SEq_3*SEq_3 + 1) / PI * 180;
	yaw   = atan2(2*SEq_2*SEq_3 + 2*SEq_1*SEq_4,-2*SEq_3*SEq_3 - 2*SEq_4*SEq_4 + 1) / PI * 180;
}



//*****************************AHRS computating task*************************************************//
void AHRS_compute(void) 
{
  /*char output1[] = {0};
  char output2[] = {0};
  char output3[] = {0};
  char output4[] = "yaw = ";
  char output5[] = "pitch = ";
  char output6[] = "roll = ";
  char output7[] = "  ,\r\n";
  char output8[] = "  ,";
	char output9[] = {0};
	char output10[] = "interval is ";*/
  u8 flag;
  /*if(UART4RecvPtrR != UART4RecvPtrW) 
  {*/
		flag = AHRSCheckDataFrame();
    /*if(flag == ACC_METER || flag == GYRO || flag == ANGLE_OUTPUT || flag == MAG_METER) 
		{*/
			AHRS_iteration(flag);
			AHRS_computeEuler();
  //  } 
	
/*sprintf(output1, "%f", yaw); 
	sprintf(output2, "%f", pitch); 
	sprintf(output3, "%f", roll); 
	sprintf(output9, "%f", interval);
	
	UART4DispFun((unsigned char *)output4); 
	UART4DispFun((unsigned char *)output1); 
	UART4DispFun((unsigned char *)output8);
	
	UART4DispFun((unsigned char *)output5);
    UART4DispFun((unsigned char *)output2);
	UART4DispFun((unsigned char *)output8);

	UART4DispFun((unsigned char *)output6); 
	UART4DispFun((unsigned char *)output3); 
	UART4DispFun((unsigned char *)output8); 
		
	UART4DispFun((unsigned char *)output10); 
	UART4DispFun((unsigned char *)output9); 
	UART4DispFun((unsigned char *)output7); */
	
  //} 
}











