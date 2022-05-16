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
		
		osDelay(20);
	}
}

