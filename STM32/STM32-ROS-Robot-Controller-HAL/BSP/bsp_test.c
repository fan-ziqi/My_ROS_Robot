#include "main.h"
#include "bsp_test.h"
#include "bsp_define.h"

#include "bsp_delay.h"
#include "bsp_led.h"
#include "bsp_vin.h"
#include "bsp_key.h"
#include "bsp_imu.h"
#include "bsp_motor.h"
#include "bsp_encoder.h"
#include "bsp_pid.h"
#include "bsp_cmd.h"

#include <cstring>

#define LOG_TAG    "TEST"
#include "bsp_log.h"

extern char CMD_Buffer[];
extern int CMD_Buffer_Count;
extern int CMD_Flag;

//LED����
void Test_LED(void)
{
	printf("LED����\r\n");
	
	//LED_Green����0.5S
	LED_Green_On();
	delay_ms(500);
	LED_Green_Off();               
	delay_ms(500);
	
	//LED_Blue����0.5S
	LED_Blue_On();
	delay_ms(500);
	LED_Blue_Off();
	delay_ms(500);
	
	LED_Blue_On();
	
	while (1)
	{	
		//LED��ת
		LED_Green_Toggle();
		LED_Blue_Toggle();
		
		cmd_exit();
		osDelay(100);
	}
}

//�����������
void Test_Debug(void)
{
	uint8_t count=0;
	
	while (1)
	{	
		//���Դ��������Ϣ		
		printf("printf���ԣ�%d \r\n",count);
		
		//����LOGģ��
		LOG_I("LOG���ԣ�%d \r\n",count);
		
		count++;
		
		cmd_exit();
		osDelay(100);
	}
}

//�����ѹ��⣬ͨ���������
void Test_VIN(void)
{
	printf("��ѹ����\r\n");
	
	float battery_voltage;
	
	//�ȴ���ѹ�ȶ�
	osDelay(1000);
	
	//VIN����ʼ��
	VIN_Init();
	
	while (1) 
	{	
		battery_voltage = get_battery_voltage() + 0.13214f;
		LOG_D("��ǰ��ѹ = %f\r\n", battery_voltage);
		
		cmd_exit();
		osDelay(100);
	}
}

void Test_KEY(void)
{
	printf("��������\r\n");
	
	extern int key1_state,key2_state;

	while(1) 
	{	
		Key_Scan();
		//KEY1����LED1�����ɿ�LED1��
		if(key1_state == KEY_DOWN)
		{
			LED_Green_On();  
		}
		if(key1_state == KEY_UP)
		{
			LED_Green_Off();
		}
		//KEY2�������ɿ�LED2��ת
		if(key2_state == KEY_DOWNUP)
		{
			LED_Blue_Toggle(); 
		}
		
		cmd_exit();
		osDelay(100);
	}
}	

void Test_MPU6050(void)
{
	printf("MPU6050����\r\n");
	
	struct xyz_data robot_accel_xyz_data;
	struct xyz_data robot_gyro_xyz_data;

	struct imu_data robot_imu_dmp_data;
	
	MPU6050_Init();    //MPU6050��ʼ��
	MPU6050_DMP_Init();	//DMP��ʼ��
	
	while (1)
	{	
		MPU6050_GetAccelData(&robot_accel_xyz_data);  //��ȡ������ٶ�����
		MPU6050_GetGyroData(&robot_gyro_xyz_data);  //��ȡ��������������		
		printf("%d %d %d %d %d %d \r\n",
			robot_accel_xyz_data.x,
			robot_accel_xyz_data.y,
			robot_accel_xyz_data.z,
			robot_gyro_xyz_data.x,
			robot_gyro_xyz_data.y,
			robot_gyro_xyz_data.z);		
		
		MPU6050_DMP_GetData(&robot_imu_dmp_data); //��ȡDMP����
		printf("%d %d %d %d %d %d %d %d %d \r\n",
			robot_imu_dmp_data.accel.x,
			robot_imu_dmp_data.accel.y,
			robot_imu_dmp_data.accel.z,
			robot_imu_dmp_data.gyro.x,
			robot_imu_dmp_data.gyro.y,
			robot_imu_dmp_data.gyro.z,
			robot_imu_dmp_data.pitch,
			robot_imu_dmp_data.roll,
			robot_imu_dmp_data.yaw);
		
		cmd_exit();
		osDelay(100);
	}
}

//4·���PWM�ٶȿ��ƣ�������ת�ͱ��ٷ�ת��������
void Test_Motor_PWM(void)
{
	Delay_Init();
	
	int speed;
	
	//�����ʼ��
	Motor_Init();
	
	for(speed=0; speed>=-1000; speed--)
	{
		MOTOR_SetSpeed(1, speed); 
		MOTOR_SetSpeed(2, speed); 
		MOTOR_SetSpeed(3, speed); 
		MOTOR_SetSpeed(4, speed); 
		delay_ms(5);
	}

	while (1) 
	{	
		//���Ƶ��ת��
		for(speed=-1000; speed<=1000; speed++)
		{
			MOTOR_SetSpeed(1, speed); 
			MOTOR_SetSpeed(2, speed); 
			MOTOR_SetSpeed(3, speed); 
			MOTOR_SetSpeed(4, speed); 
			delay_ms(5);
		}
		for(speed=1000; speed>=-1000; speed--)
		{
			MOTOR_SetSpeed(1, speed); 
			MOTOR_SetSpeed(2, speed); 
			MOTOR_SetSpeed(3, speed); 
			MOTOR_SetSpeed(4, speed); 
			delay_ms(5);
		}
	}
}	

//��·������300ms����,�ֶ�ת�����,ͨ���������
void Test_Motor_Encoder(void)
{
	Delay_Init();
	
	//������������ʼ��
	Encoder_Init();
	
	//��ʾ��Ϣ
	printf("��·����������\r\n");
	
	while (1) 
	{
		printf("Encoder: [1]%5d [2]%5d [3]%5d [4]%5d \r\n", 
			Encoder_Get_Counter(1),
			Encoder_Get_Counter(2),
			Encoder_Get_Counter(3),
			Encoder_Get_Counter(4));
		delay_ms(300);
	}
}

void Test_Motor_PID(void)
{
	int16_t encoder_delta[4];	//��������Ա仯ֵ,����ʵ���ٶ�
	int16_t encoder_delta_target[4] = {0}; //������Ŀ��ֵ������Ŀ���ٶ�
	int16_t motor_pwm[4];  //���PWM�ٶ�

//	uint8_t comdata[32];
	
	Delay_Init();
	
	//�����ʼ��
	Motor_Init();
	
	//������������ʼ��
	Encoder_Init();
	
	while (1) 
	{	
		//��ȡ�������仯ֵ
		encoder_delta[0] = -(Encoder_Get_Counter(1) - 30000);
		encoder_delta[1] =  (Encoder_Get_Counter(2) - 30000);
		encoder_delta[2] = -(Encoder_Get_Counter(3) - 30000);
		encoder_delta[3] =  (Encoder_Get_Counter(4) - 30000);
		
		//���ñ������м�ֵ
		Encoder_Set_Counter(1, 30000);
		Encoder_Set_Counter(2, 30000);
		Encoder_Set_Counter(3, 30000);
		Encoder_Set_Counter(4, 30000);
		
		//����PID���ƺ���ٶ�
		motor_pwm[0] = PID_Motor_Control(encoder_delta_target[0], encoder_delta[0]);
		motor_pwm[1] = PID_Motor_Control(encoder_delta_target[1], encoder_delta[1]);
		motor_pwm[2] = PID_Motor_Control(encoder_delta_target[2], encoder_delta[2]);
		motor_pwm[3] = PID_Motor_Control(encoder_delta_target[3], encoder_delta[3]);

		MOTOR_SetSpeed(1, motor_pwm[0]);
		MOTOR_SetSpeed(1, motor_pwm[1]);
		MOTOR_SetSpeed(1, motor_pwm[2]);
		MOTOR_SetSpeed(1, motor_pwm[3]);
		
//		//��ȡUSB��������
//		if( UART_DB_GetData(comdata))  
//		{
//			//�������ٶȿ���
//			if(comdata[0] == 0x01)
//			{
//				encoder_delta_target = (int16_t)((comdata[1]<<8) | comdata[2]);	
//			}

//			//�ٶȿ���PID����
//			if(comdata[0] == 0x02)
//			{
//				motor_kp = (int16_t)((comdata[1]<<8) | comdata[2]);
//				motor_ki = (int16_t)((comdata[3]<<8) | comdata[4]);
//				motor_kd = (int16_t)((comdata[5]<<8) | comdata[6]);
//			}

//		}
//		
//		//���ڷ��͵��Ŀ��ת�ٺ�ʵ��ת�٣�����������
//		comdata[0] = (u8)( encoder_delta_target >> 8 );  
//		comdata[1] = (u8)( encoder_delta_target );
//		comdata[2] = (u8)( encoder_delta >> 8 );
//		comdata[3] = (u8)( encoder_delta );
//	 
//		UART_DB_SendPacket(comdata, 4, 0x03);
				 
		LED_Green_Toggle();
	}
}
