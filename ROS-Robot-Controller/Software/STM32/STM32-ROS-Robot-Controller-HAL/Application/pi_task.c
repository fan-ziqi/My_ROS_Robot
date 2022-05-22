/*
	Copyright 2022 Fan Ziqi

	Licensed under the Apache License, Version 2.0 (the "License");
	you may not use this file except in compliance with the License.
	You may obtain a copy of the License at

		http://www.apache.org/licenses/LICENSE-2.0

	Unless required by applicable law or agreed to in writing, software
	distributed under the License is distributed on an "AS IS" BASIS,
	WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
	See the License for the specific language governing permissions and
	limitations under the License.
*/

#include "main.h"
#include "bsp_define.h"
#include "bsp_usart.h"
#include "bsp_imu.h"
#include "string.h"

#define LOG_TAG    "PI"
#include "bsp_log.h"

extern UART_HandleTypeDef huart2;

extern struct imu_data robot_imu_dmp_data;
extern int16_t robot_odom[]; //里程计数据，绝对值和变化值，x y yaw dx dy dyaw
extern int16_t encoder_delta[];	//编码器变化值
extern int16_t encoder_delta_target[];  //编码器目标变化值
extern float battery_voltage;

int16_t SendData[24] = {0};

void send_to_pi(void)
{
	//陀螺仪角速度 = (gyro/32768) * 2000 ?s
	SendData[0] = robot_imu_dmp_data.gyro.x;
	SendData[1] = robot_imu_dmp_data.gyro.y;
	SendData[2] = robot_imu_dmp_data.gyro.z;
	//加速度 = (acc/32768) * 2G  
	SendData[3] = robot_imu_dmp_data.accel.x;
	SendData[4] = robot_imu_dmp_data.accel.y;
	SendData[5] = robot_imu_dmp_data.accel.z;
	//姿态角度 = (angle/100)
	SendData[6] = robot_imu_dmp_data.pitch;
	SendData[7] = robot_imu_dmp_data.roll;
	SendData[8] = robot_imu_dmp_data.yaw;
	//里程计坐标 x(m) y(m) yaw(rad)  odom_frame
	SendData[9]  = robot_odom[0];
	SendData[10] = robot_odom[1];
	SendData[11] = robot_odom[2];
	//里程计坐标变化量  d_x(m) d_y(m) d_yaw(rad)  base_frame
	SendData[12] = robot_odom[3];
	SendData[13] = robot_odom[4];
	SendData[14] = robot_odom[5];
	//编码器当前值
	SendData[15] = encoder_delta[0];
	SendData[16] = encoder_delta[1];
	SendData[17] = encoder_delta[2];
	SendData[18] = encoder_delta[3];
	//编码器目标值
	SendData[19] = encoder_delta_target[0];
	SendData[20] = encoder_delta_target[1];
	SendData[21] = encoder_delta_target[2];
	SendData[22] = encoder_delta_target[3];
	//电池
	SendData[23] = (int16_t)(battery_voltage*100);
	
	//发送串口数据
	USART_Send_Pack(&huart2, SendData, 24, 06);
	
//	//清空数组
//	memset(SendData,0,sizeof(SendData));
}

void pi_task(void const * argument)
{
	while(1)
	{
		send_to_pi();
		osDelay(20);
	}
}

