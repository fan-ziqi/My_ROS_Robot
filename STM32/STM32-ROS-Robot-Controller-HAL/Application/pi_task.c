#include "main.h"
#include "bsp_define.h"
#include "bsp_usart.h"
#include "bsp_imu.h"
#include "string.h"

#define LOG_TAG    "PI"
#include "bsp_log.h"

extern UART_HandleTypeDef huart2;

extern struct imu_data robot_imu_dmp_data;
extern float battery_voltage;

int16_t SendData[24] = {0};

void send_to_pi(void)
{
	//�����ǽ��ٶ� = (gyro/32768) * 2000 ?s
	SendData[0] = robot_imu_dmp_data.gyro.x;
	SendData[1] = robot_imu_dmp_data.gyro.y;
	SendData[2] = robot_imu_dmp_data.gyro.z;
	//���ٶ� = (acc/32768) * 2G  
	SendData[3] = robot_imu_dmp_data.accel.x;
	SendData[4] = robot_imu_dmp_data.accel.y;
	SendData[5] = robot_imu_dmp_data.accel.z;
	//��̬�Ƕ� = (angle/100)
	SendData[6] = robot_imu_dmp_data.pitch;
	SendData[7] = robot_imu_dmp_data.roll;
	SendData[8] = robot_imu_dmp_data.yaw;
//	//��̼����� x(m) y(m) yaw(rad)  odom_frame
//	SendData[9] = ;
//	SendData[10] = ;
//	SendData[11] = ;
//	//��̼�����仯��  d_x(m) d_y(m) d_yaw(rad)  base_frame
//	SendData[12] = ;
//	SendData[13] = ;
//	SendData[14] = ;
//	//��������ǰֵ��Ŀ��ֵ
//	SendData[15] = ;
//	SendData[16] = ;
//	SendData[17] = ;
//	SendData[18] = ;
//	
//	SendData[19] = ;
//	SendData[20] = ;
//	SendData[21] = ;
//	SendData[22] = ;
	//���
	SendData[23] = (int16_t)(battery_voltage*100);
	//���ʹ�������
	
	USART_Send_Pack(&huart2, SendData, 24, 06);
	
//	//�������
//	memset(SendData,0,sizeof(SendData));
}

void pi_task(void const * argument)
{
	while(1)
	{
		send_to_pi();
		osDelay(10);
	}
}

