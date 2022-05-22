/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include <stdio.h>
#include <math.h>   

#include "sys.h"    //ϵͳ����
#include "delay.h"  //�����ʱ
#include "led.h"    //LED�ƿ���
#include "vin.h"    //�����ѹ���
#include "uart_db.h"  //���Դ���
#include "uart_pi.h"  //��ݮ�ɴ���
#include "motor.h"    //ֱ��������ٿ���
#include "encoder.h"  //����������
#include "tim.h"      //��ʱ��
#include "mpu6050.h"  //IMU���ٶ������ǲ���
#include "mpu6050_dmp.h"  //DMP���ܺ���
#include "pid.h"        //PID����
#include "kinematics.h" //�˶�ѧ����

#include "test.h"


#define ENCODER_MID_VALUE  30000  
#define VBAT_VOL_CHG    1130 
#define VBAT_VOL_OFF    1000   


int16_t encoder[4];	//�������ۼ�ֵ
int16_t encoder_delta[4];	//�������仯ֵ
int16_t encoder_delta_target[4] = {0};  //������Ŀ��仯ֵ
int16_t robot_odom[6] = {0}; //��̼����ݣ�����ֵ�ͱ仯ֵ��x y yaw dx dy dyaw
int16_t motor_pwm[4];  //���PWM
uint16_t bat_vcc;  //��ص�ѹ
int16_t mpu_data[10];  //�����ǣ����ٶȣ���̬��

int16_t robot_target_speed[3] = {0};  //������Ŀ���ٶ� X Y Yaw
int16_t robot_params[2] = {1000,1000};  //�����˲���

//��Ҫ��������
void ROBOT_GetImuData(void);  //��ȡMPU6050����
void ROBOT_MoveCtl(void);  //�������˶����ƺ���  
void ROBOT_BatteryManagement(void);  //�����˵�ع���
void ROBOT_SendDataToPi(void);  //�����˷������ݵ���ݮ��

/**
  * @��  ��  ����������
  * @��  ��  ��
  * @����ֵ  ��
  */


int main(void)
{

//	Test_LED();
//	Test_VIN();
//	Test_KEY();
//	Test_MPU6050();
//	Test_Servo();
//	Test_Motor_PWM();
//	Test_Motor_Encoder();
//	Test_Motor_PID();
//	Test_USART_PI();

	
	uint8_t cnt = 1;  //��ʱ������

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);   //�����жϷ���

	//�����˳�ʼ��
	MOTOR_Init(10);
	DELAY_Init();  
	JTAG_Set(JTAG_SWD_DISABLE);     
	JTAG_Set(SWD_ENABLE);          
	LED_Init();
	VIN_Init();
	UART_DB_Init(115200);
	UART_PI_Init(115200);

	//��������ʼ��
	ENCODER_Motor1_Init(ENCODER_MID_VALUE*2);  
	ENCODER_Motor2_Init(ENCODER_MID_VALUE*2);  
	ENCODER_Motor3_Init(ENCODER_MID_VALUE*2);  
	ENCODER_Motor4_Init(ENCODER_MID_VALUE*2);  
	ENCODER_Motor1_SetCounter(ENCODER_MID_VALUE); 
	ENCODER_Motor2_SetCounter(ENCODER_MID_VALUE); 
	ENCODER_Motor3_SetCounter(ENCODER_MID_VALUE); 
	ENCODER_Motor4_SetCounter(ENCODER_MID_VALUE);
	
    //������ʾ��Ϣ
	LED_Blue_On();
	LED_Green_On();
	delay_ms(100);	
	LED_Blue_Off();
	LED_Green_Off();
	delay_ms(300);

	MPU6050_Init();    //MPU6050��ʼ��
	MPU6050_DMP_Init();	//DMP��ʼ��
	TIM6_Init(10000);
	TIM6_Cmd(ENABLE);
	
	
	//�̵Ƶ�������ʾ����
	LED_Green_On();

	while(1)
	{
		//100HZ����Ƶ��
		if(TIM6_CheckIrqStatus())
		{		
			
			//�����˻�ȡPMU6050����
			ROBOT_GetImuData();
			
			//50HZִ��Ƶ��
			if(cnt%2 == 0)
			{
				
				//�������˶�����
				ROBOT_MoveCtl();
			
				//�����˵�������
				ROBOT_BatteryManagement();
			
				//�����˷������ݵ���ݮ��
				ROBOT_SendDataToPi();
				
			}
			
			//�������ۼ�
			cnt++;
		}
	}
}

/**
  * @��  ��  �����˻�ȡMPU6050 ���ٶ���������̬����
  * @��  ��  ��
  * @����ֵ  ��
  */
void ROBOT_GetImuData(void)
{
	  //�������壬����Ƕ�
    MPU6050_DMP_GetData(mpu_data);
	
	//TODO:��дMPU6050�Լ�DMP��
	
}
/**
  * @��  ��  �������˶����ƺ���
  * @��  ��  ��
  * @����ֵ  ��
  */
void ROBOT_MoveCtl(void)
{ 
	  
	//��ȡ�������仯ֵ
	encoder_delta[0] = -(ENCODER_Motor1_GetCounter()-ENCODER_MID_VALUE);
	encoder_delta[1] = ENCODER_Motor2_GetCounter()  -ENCODER_MID_VALUE;
	encoder_delta[2] = -(ENCODER_Motor3_GetCounter()-ENCODER_MID_VALUE);
	encoder_delta[3] = ENCODER_Motor4_GetCounter()  -ENCODER_MID_VALUE;
		
	//���ñ������м�ֵ
	ENCODER_Motor1_SetCounter(ENCODER_MID_VALUE);
	ENCODER_Motor2_SetCounter(ENCODER_MID_VALUE);
	ENCODER_Motor3_SetCounter(ENCODER_MID_VALUE);
	ENCODER_Motor4_SetCounter(ENCODER_MID_VALUE);
		
	//����������ۼ�ֵ
	encoder[0] = encoder[0] + encoder_delta[0];
	encoder[1] = encoder[1] + encoder_delta[1];
	encoder[2] = encoder[2] + encoder_delta[2];
	encoder[3] = encoder[3] + encoder_delta[3];
		
	//�˶�ѧ����
	Kinematics_Forward(encoder,robot_odom);  //�����˶�ѧ����
	Kinematics_Inverse(robot_target_speed, encoder_delta_target);  //�����˶�ѧ����

	//���PID�ٶȿ���
	motor_pwm[0] = PID_MotorVelocityCtlA(encoder_delta_target[0], encoder_delta[0]);   
	motor_pwm[1] = PID_MotorVelocityCtlB(encoder_delta_target[1], encoder_delta[1]);   
	motor_pwm[2] = PID_MotorVelocityCtlC(encoder_delta_target[2], encoder_delta[2]);   
	motor_pwm[3] = PID_MotorVelocityCtlD(encoder_delta_target[3], encoder_delta[3]);  
					
					
	MOTOR1_SetSpeed(motor_pwm[0]);
	MOTOR2_SetSpeed(motor_pwm[1]);  
	MOTOR3_SetSpeed(motor_pwm[2]);
	MOTOR4_SetSpeed(motor_pwm[3]); 
		
}

/**
  * @��  ��  �����˵�ع���
  * @��  ��  ��
  * @����ֵ  ��
  */
void ROBOT_BatteryManagement(void)
{
	//��������
	static uint16_t bat_vol_cnt = 0;   
	
	//�ɼ���ص�ѹ
	bat_vcc = (uint16_t)100*Get_VCC();		
		
//	printf("��ǰ��ѹ =%d V \r\n",bat_vcc);
	
	if(bat_vcc < VBAT_VOL_CHG)  //1050
	{

		LED_Blue_Toggle();
		
		if(bat_vcc < VBAT_VOL_OFF) //990
		{
			bat_vol_cnt++;
      
			//��ѹ����С��VBAT_VOL_OFF 4�룬����ͣ������״̬
			if(bat_vol_cnt > 200 )
			{
				
				//��Ƴ���������ͣ��״̬�����ֹͣת��
				LED_Blue_On();
				LED_Green_Off();
				MOTOR1_SetSpeed(0);  
				MOTOR2_SetSpeed(0);  
				MOTOR3_SetSpeed(0);  
				MOTOR4_SetSpeed(0);  
				
				//����
				while(1)
				{	
					delay_ms(100);
				}								
			}
		}
		else
		{
			bat_vol_cnt = 0;
		}
	}
	else
	{
		LED_Blue_Off();
	}
}


/**
  * @��  ��  �����˷������ݵ���ݮ��
  * @��  ��  ��
  * @����ֵ  ��
  */
//��������ݾ�Ϊu16���ͣ�Ҳ����ʮ�������65535�����������1111111111111111��ʮ���������ffff
//ÿ������ֳɸ߰�λ�͵Ͱ�λ����ֺ����������Ϊu8��254��11111111��ff��
void ROBOT_SendDataToPi(void)
{
	//���ڷ�������
	static uint8_t comdata[60]; 			
	
	//�����ǽ��ٶ� = (gyro/32768) * 2000 ?s
	comdata[0] = (u8)( mpu_data[0] >> 8 );  
	comdata[1] = (u8)( mpu_data[0] );
	comdata[2] = (u8)( mpu_data[1] >> 8 );
	comdata[3] = (u8)( mpu_data[1] );
	comdata[4] = (u8)( mpu_data[2] >> 8 );
	comdata[5] = (u8)( mpu_data[2] );
	
	//���ٶ� = (acc/32768) * 2G  
	comdata[6] = (u8)( mpu_data[3] >> 8 );
	comdata[7] = (u8)( mpu_data[3] );
	comdata[8] = (u8)( mpu_data[4] >> 8 );
	comdata[9] = (u8)( mpu_data[4] );
	comdata[10] = (u8)( mpu_data[5] >> 8 );
	comdata[11] = (u8)( mpu_data[5] );
	
	//��̬�Ƕ� = (angle/100)
	comdata[12] = (u8)( mpu_data[6] >> 8 ); 
	comdata[13] = (u8)( mpu_data[6] );
	comdata[14] = (u8)( mpu_data[7] >> 8 );
	comdata[15] = (u8)( mpu_data[7] );
	comdata[16] = (u8)( mpu_data[8] >> 8 );
	comdata[17] = (u8)( mpu_data[8] );
	
	//��̼����� x(m) y(m) yaw(rad)  odom_frame
	comdata[18] = (u8)( robot_odom[0] >> 8 );
	comdata[19] = (u8)( robot_odom[0] );
	comdata[20] = (u8)( robot_odom[1] >> 8 );
	comdata[21] = (u8)( robot_odom[1] );
	comdata[22] = (u8)( robot_odom[2] >> 8 );
	comdata[23] = (u8)( robot_odom[2] );
	
	//��̼�����仯��  d_x(m) d_y(m) d_yaw(rad)  base_frame
	comdata[24] = (u8)( robot_odom[3] >> 8 );
	comdata[25] = (u8)( robot_odom[3] );
	comdata[26] = (u8)( robot_odom[4] >> 8 );
	comdata[27] = (u8)( robot_odom[4] );
	comdata[28] = (u8)( robot_odom[5] >> 8 );
	comdata[29] = (u8)( robot_odom[5] );
		
	//��������ǰֵ��Ŀ��ֵ
	comdata[30] = (u8)( encoder_delta[0] >> 8 );  
	comdata[31] = (u8)( encoder_delta[0] );
	comdata[32] = (u8)( encoder_delta[1] >> 8 );
	comdata[33] = (u8)( encoder_delta[1] );
	comdata[34] = (u8)( encoder_delta[2] >> 8 );
	comdata[35] = (u8)( encoder_delta[2] );
	comdata[36] = (u8)( encoder_delta[3] >> 8 );
	comdata[37] = (u8)( encoder_delta[3] );
	
	comdata[38] = (u8)( encoder_delta_target[0] >> 8 );  
	comdata[39] = (u8)( encoder_delta_target[0] );
	comdata[40] = (u8)( encoder_delta_target[1] >> 8 );
	comdata[41] = (u8)( encoder_delta_target[1] );
	comdata[42] = (u8)( encoder_delta_target[2] >> 8 );
	comdata[43] = (u8)( encoder_delta_target[2] );
	comdata[44] = (u8)( encoder_delta_target[3] >> 8 );
	comdata[45] = (u8)( encoder_delta_target[3] );
	
	//������
	comdata[46] = (u8)( bat_vcc >> 8 );
	comdata[47] = (u8)( bat_vcc );
		
	//���ʹ�������
	UART_PI_SendPacket(comdata, 48, 0x06);
	
//	UART_DB_SendPacket(comdata, 48, 0x06);
}


