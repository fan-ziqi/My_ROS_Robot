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
#include "bsp_imu.h"

#define LOG_TAG    "IMU"
#include "bsp_log.h"

struct imu_data robot_imu_dmp_data;

struct xyz_data acc;
struct xyz_data gyro;

//10ms
void imu_task(void const * argument)
{
	MPU6050_Init();    //MPU6050≥ı ºªØ
	
	while(1)
	{
		MPU6050_DMP_GetData(&robot_imu_dmp_data);
		
		osDelay(10);
	}
}

