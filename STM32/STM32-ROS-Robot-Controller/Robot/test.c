#include "test.h"
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include <stdio.h>

#include "sys.h"    //ϵͳ����
#include "delay.h"  //�����ʱ
#include "led.h"    //LED�ƿ���
#include "vin.h"    //�����ѹ���
#include "key.h"    //������� 
#include "uart_db.h"  //���Դ���
#include "uart_pi.h"  //��ݮ�ɴ���
#include "motor.h"    //ֱ��������ٿ���
#include "encoder.h"  //����������
#include "servo.h"    //�������
#include "tim.h"      //��ʱ��
#include "mpu6050.h"  //IMU���ٶ������ǲ���
#include "mpu6050_dmp.h"  //DMP���ܺ���

/******************************************************************************
      ��������  �嵥
			
* LED��˸�������������Դ���Printf�������
* VIN�����ѹ������̣����׵�����
* KEY������������̣��������
* MPU6050���ݲɼ�����
* �����������
* ֱ�����PWM�ٶȿ�������
* ���AB��������������
* ֱ�����PID��������
* STM32����ݮ��ͨ��

*******************************************************************************/

/******************************************************************************
LED_Green����0.5S,LED_Blue����0.5S,LED��ת,�������
*******************************************************************************/
void Test_LED(void)
{
	uint8_t i=0;
	
	//�����ж����ȼ�����
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);    

	//JTAG������
	JTAG_Set(JTAG_SWD_DISABLE);  //�ر�JTAG�ӿ� 
	JTAG_Set(SWD_ENABLE);  //��SWD�ӿ� �������������SWD�ӿڵ��� 
	
	//�����ʱ��ʼ��
	DELAY_Init(); 	
	LED_Init();  //LED��ʼ��

	//���Դ��ڳ�ʼ��
	UART_DB_Init(115200); //���Դ���
	printf("  \r\n"); //����ո�CPUBUG
	
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
		//���Դ��������Ϣ		
		printf("Printf������ԣ�%d \r\n",i);
		i++;
		delay_ms(100);
		
		//LED��ת
		LED_Green_Toggle();
		LED_Blue_Toggle();
	}
}


/******************************************************************************
�����ѹ��⣬ͨ�����������ѹֵ
ADC2ͨ��8
*******************************************************************************/
void Test_VIN(void)
{
	float bat_vcc;
	
	//�����ж����ȼ�����
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);    

	//JTAG������
	JTAG_Set(JTAG_SWD_DISABLE);  //�ر�JTAG�ӿ� 
	JTAG_Set(SWD_ENABLE);  //��SWD�ӿ� �������������SWD�ӿڵ��� 
	
	//�����ʱ��ʼ��
	DELAY_Init(); 	
	
	//���Դ��ڳ�ʼ��
	UART_DB_Init(115200); //���Դ���
	printf("  \r\n"); //����ո�CPUBUG
	
	//VIN����ʼ��
	VIN_Init();
	
	//��ʾ��Ϣ
	printf("ADC2ͨ��8��ѹ����\r\n");
	
	while (1) 
	{	
		bat_vcc = Get_VCC();
		printf("��ǰ��ѹ =%6.2f V \r\n",bat_vcc);				//��ӡ��ǰ��ѹ������С�������λ
	}
}


/******************************************************************************
KEY1����LED1�����ɿ�LED1��
KEY2�������ɿ�LED2��ת
*******************************************************************************/
void Test_KEY(void)
{
	extern int key1_state,key2_state;
	//�����ж����ȼ�����
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);    

	//JTAG������
	JTAG_Set(JTAG_SWD_DISABLE);  //�ر�JTAG�ӿ� 
	JTAG_Set(SWD_ENABLE);  //��SWD�ӿ� �������������SWD�ӿڵ��� 
	
	//�����ʱ��ʼ��
	DELAY_Init(); 	
	LED_Init();  	
	
	//���Դ��ڳ�ʼ��
	UART_DB_Init(115200); //���Դ���
	printf("  \r\n"); //����ո�CPUBUG
	
	//��ʼ��
	KEY_Init();
	
	while (1) 
	{	
		KEY_Scan();
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
		delay_ms(10);
	}
}	


/******************************************************************************
�������ƣ���ȡMPU6050���ݲ����
����˵������ȡMPU6050���ݣ�ͨ������Printf���
����˵�����������������ٶȣ��������������ݣ�ת�����������۲����ݱ仯
         ʹ�����ǵ�X-PrintfScope��������Թ۲쵽��������
*******************************************************************************/
void Test_MPU6050(void)
{
	int16_t acc[3],gyro[3]; //IMU����
	
	//�����ж����ȼ�����
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);    

	//JTAG������
	JTAG_Set(JTAG_SWD_DISABLE);  //�ر�JTAG�ӿ� 
	JTAG_Set(SWD_ENABLE);  //��SWD�ӿ� �������������SWD�ӿڵ��� 
	
	//�����ʱ��ʼ��
	DELAY_Init(); 	
	LED_Init();  //LED��ʼ��

	//���Դ��ڳ�ʼ��
	UART_DB_Init(115200); //���Դ���
	printf("  \r\n"); //����ո�CPUBUG
	
	//MPU6050��ʼ��  
	MPU6050_Init();    
	MPU6050_SetAccRange(ACC_RANGE_2G);    //���ü��ٶ�����
	MPU6050_SetGyroRange(GYRO_RANGE_2000); //��������������
	MPU6050_SetGyroSmplRate(200);            //���������ǲ�����
	MPU6050_SetDLPF(DLPF_ACC94_GYRO98);   //���õ�ͨ�˲�������
	
	while (1)
	{	
		//������̬�������ǡ����ٶ�����
		MPU6050_GetAccData(acc);  //��ȡ������ٶ�����
		MPU6050_GetGyroData(gyro);  //��ȡ��������������		
		
		//���Դ��������Ϣ
		printf("%d %d %d %d %d %d \r\n",
		acc[0],acc[1],acc[2],gyro[0],gyro[1],gyro[2]);		
		
//		mpu6050_send_data(acc[0],acc[1],acc[2]);
		

	}
}


/******************************************************************************
����8·���60�ȡ�90�ȡ�120�ȼ���˶�
*******************************************************************************/
void Test_Servo(void)
{
	//�����ж����ȼ�����
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  
	
	//���Դ��ڳ�ʼ��
	UART_DB_Init(115200); //���Դ���
	printf("  \r\n"); //����ո�CPUBUG
	
	//JTAG������
	JTAG_Set(JTAG_SWD_DISABLE);  //�ر�JTAG�ӿ� 
	JTAG_Set(SWD_ENABLE);  //��SWD�ӿ� �������������SWD�ӿڵ��� 		
	
	//�����ʱ��ʼ��
	DELAY_Init();
	LED_Init();
	
	//��ʼ��
	SERVO_AB_Init();
	SERVO_CD_Init();
	SERVO_EF_Init();
	SERVO_GH_Init();
	
	//��ʾ��Ϣ
	printf("��·������Ʋ���\r\n");

	while (1) 
	{		
		printf("*60��...... \r\n");		
		SERVO_A_SetAngle(600);
		SERVO_B_SetAngle(600);
		SERVO_C_SetAngle(600);
		SERVO_D_SetAngle(600);
		SERVO_E_SetAngle(600);
		SERVO_F_SetAngle(600);
		SERVO_G_SetAngle(600);
		SERVO_H_SetAngle(600);
		delay_ms(1000);
		
		printf("*90��...... \r\n");
		SERVO_A_SetAngle(900);
		SERVO_B_SetAngle(900);
		SERVO_C_SetAngle(900);
		SERVO_D_SetAngle(900);
		SERVO_E_SetAngle(900);
		SERVO_F_SetAngle(900);
		SERVO_G_SetAngle(900);
		SERVO_H_SetAngle(900);
		delay_ms(1000);
		
		printf("*120��...... \r\n");
		SERVO_A_SetAngle(1200);
		SERVO_B_SetAngle(1200);
		SERVO_C_SetAngle(1200);
		SERVO_D_SetAngle(1200);
		SERVO_E_SetAngle(1200);
		SERVO_F_SetAngle(1200);
		SERVO_G_SetAngle(1200);
		SERVO_H_SetAngle(1200);
		delay_ms(1000);
	}
}		



/******************************************************************************
����4·ֱ�����PWM����,������ת�ͱ��ٷ�ת��������
*******************************************************************************/
void Test_Motor_PWM(void)
{
	int temp;
	
	//�����ж����ȼ�����
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  
	
	//JTAG������
	JTAG_Set(JTAG_SWD_DISABLE);  //�ر�JTAG�ӿ� 
	JTAG_Set(SWD_ENABLE);  //��SWD�ӿ� �������������SWD�ӿڵ��� 		
	
	//�����ʱ��ʼ��
	DELAY_Init(); 	
	LED_Init();  
	
	//��ʼ��
	MOTOR_Init(10);  //���õ������PWMƵ��Ϊ10K
	
	for(temp=0; temp>=-2000; temp--)
	{
		MOTOR1_SetSpeed(temp); 
		MOTOR2_SetSpeed(temp); 
		MOTOR3_SetSpeed(temp); 
		MOTOR4_SetSpeed(temp); 
		delay_ms(5);
	}

	while (1) 
	{	
		//���Ƶ��ת��
		for(temp=-2000; temp<=2000; temp++)
		{
			MOTOR1_SetSpeed(temp); 
			MOTOR2_SetSpeed(temp); 
			MOTOR3_SetSpeed(temp); 
			MOTOR4_SetSpeed(temp); 
			delay_ms(5);
		}
		for(temp=2000; temp>=-2000; temp--)
		{
			MOTOR1_SetSpeed(temp); 
			MOTOR2_SetSpeed(temp); 
			MOTOR3_SetSpeed(temp); 
			MOTOR4_SetSpeed(temp); 
			delay_ms(5);
		}
	}
}	


/******************************************************************************
��·������300ms����,�ֶ�ת�����,�������
*******************************************************************************/
void Test_Motor_Encoder(void)
{
	//�����ж����ȼ�����
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  
	
	//���Դ��ڳ�ʼ��
	UART_DB_Init(115200); //���Դ���
	printf("  \r\n"); //����ո�CPUBUG
	
	//JTAG������
	JTAG_Set(JTAG_SWD_DISABLE);  //�ر�JTAG�ӿ� 
	JTAG_Set(SWD_ENABLE);  //��SWD�ӿ� �������������SWD�ӿڵ��� 		
	
	//�����ʱ��ʼ��
	DELAY_Init(); 	
	LED_Init();   
	
	//������������ʼ��
	ENCODER_Motor1_Init(60000);  
	ENCODER_Motor2_Init(60000); 
	ENCODER_Motor3_Init(60000);  
	ENCODER_Motor4_Init(60000); 
	
	//�趨�м�ֵ30000
	ENCODER_Motor1_SetCounter(30000); 
	ENCODER_Motor2_SetCounter(30000); 
	ENCODER_Motor3_SetCounter(30000); 
	ENCODER_Motor4_SetCounter(30000); 
	
	//��ʾ��Ϣ
	printf("��·����������\r\n");
	
	while (1) 
	{
		printf("*AB:%5d CD:%5d EF:%5d GH:%5d \r\n",
		ENCODER_Motor1_GetCounter(),ENCODER_Motor2_GetCounter(),
		ENCODER_Motor3_GetCounter(),ENCODER_Motor4_GetCounter());  
		delay_ms(300);
	}
}	


/******************************************************************************
�������ƣ�ֱ�����ٵ��PID�ٶȿ���
����˵����ֱ������ٶ�PID�������̣���ͨ��X-PrintfScope�۲�Ŀ��ֵ��ʵ��ֵ����
����˵����Ӳ�����ӣ�������ӿ��ư���A�ӿڣ�����������AB�ӿ�
         ͨ��X-PrintfScope����۲첨�����ߣ������е��ת���趨��PID��������
ע��������˻������е�����������֣��������ʺ�����һ�֣���һ����Ҫ�������ת��
				 ��������ο���Ƶ�̡̳�
*******************************************************************************/
void Test_Motor_PID(void)
{
//	int16_t encoder;	//����������ֵ
	int16_t encoder_delta;	//��������Ա仯ֵ,����ʵ���ٶ�
		
	int16_t encoder_delta_target = 0; //������Ŀ��ֵ������Ŀ���ٶ�
	int16_t motor_pwm;  //���PWM�ٶ�

	int16_t motor_kp=600;  //PID����
	int16_t motor_ki=0;  //PID����
	int16_t motor_kd=400;  //PID����
	
	
	
	int32_t bias,bias_last,bias_integral = 0;
	uint8_t comdata[32];
	
	//�����ж����ȼ�����
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);    
	
	//��ʼ�����
	MOTOR_Init(10);  //���õ������PWMƵ��Ϊ10K

	//JTAG������
	JTAG_Set(JTAG_SWD_DISABLE);  //�ر�JTAG�ӿ� 
	JTAG_Set(SWD_ENABLE);  //��SWD�ӿ� �������������SWD�ӿڵ��� 
	
	//�����ʱ��ʼ��
	DELAY_Init(); 	
	LED_Init();  //LED��ʼ��
	
	//���Դ��ڳ�ʼ��
	UART_DB_Init(115200); //���Դ���
	
	//��ʱ����ʼ��
	TIM6_Init(40000);//���ö�ʱ�����ڶ�ʱʱ��40ms,25HZ

	//��������ʼ��
	ENCODER_Motor1_Init(30000*2);  //������������ʼ��
	ENCODER_Motor1_SetCounter(30000); 		//���ñ�������ʼֵ
	
	while (1) 
	{	
		//ִ�����ڣ�40ms��25Hz
		if(TIM6_CheckIrqStatus())
		{			
			//����������ٶ�
			encoder_delta = -(ENCODER_Motor1_GetCounter()-30000);
	
			//���ñ�������ʼ�м�ֵ
			ENCODER_Motor1_SetCounter(30000);
			
			//PID����
			bias = encoder_delta_target - encoder_delta;
			bias_integral += bias;
			motor_pwm += motor_kp*bias*0.01f + motor_kd*(bias-bias_last)*0.01f + motor_ki*bias_integral*0.01f;
			
			bias_last = bias;
			
			//����������
			if(motor_pwm>2000)   motor_pwm = 2000;
			if(motor_pwm<-2000)  motor_pwm = -2000;			
			
			MOTOR1_SetSpeed(-motor_pwm);
			
			//��ȡUSB��������
			if( UART_DB_GetData(comdata))  
			{
				//�������ٶȿ���
				if(comdata[0] == 0x01)
				{
					encoder_delta_target = (int16_t)((comdata[1]<<8) | comdata[2]);	
				}

				//�ٶȿ���PID����
				if(comdata[0] == 0x02)
				{
					motor_kp = (int16_t)((comdata[1]<<8) | comdata[2]);
					motor_ki = (int16_t)((comdata[3]<<8) | comdata[4]);
					motor_kd = (int16_t)((comdata[5]<<8) | comdata[6]);
				}

			}
			
			//���ڷ��͵��Ŀ��ת�ٺ�ʵ��ת�٣�����������
			comdata[0] = (u8)( encoder_delta_target >> 8 );  
			comdata[1] = (u8)( encoder_delta_target );
			comdata[2] = (u8)( encoder_delta >> 8 );
			comdata[3] = (u8)( encoder_delta );
		 
			UART_DB_SendPacket(comdata, 4, 0x03);
				 
			LED_Green_Toggle();
		}
	}
}

/******************************************************************************
�������ƣ�OpenCRP����ݮ�ɴ���ͨ��
����˵����OpenCRPͨ������ݮ��40PIN�������ӵĴ��ڣ����д���ͨ��
����˵���������̲�����ͨ��������ݮ�ɹ۲����󣬿��Խ����ͽ������Ŷ̽ӣ�ʵ���Է������ղ���
         ͨ�Ų���X-Protocol����ʹ�����˴��ڵ������ֵ��ԣ����������Ƶ�̳�
*******************************************************************************/
void Test_USART_PI(void)
{
	uint8_t senddata[32];
	uint8_t receivedata[32];
	
	//�����ж����ȼ�����
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);    
	
	//JTAG������
	JTAG_Set(JTAG_SWD_DISABLE);  //�ر�JTAG�ӿ� 
	JTAG_Set(SWD_ENABLE);  //��SWD�ӿ� �������������SWD�ӿڵ��� 
	
	//�����ʱ��ʼ��
	DELAY_Init(); 	
	LED_Init();  //LED��ʼ��
	
	//���Դ��ڳ�ʼ��
	UART_DB_Init(115200); //���Դ���
	UART_PI_Init(115200); //��ݮ�ɴ���
	
	while (1) 
	{	
			//�鿴��ݮ�ɴ����Ƿ���յ����ݣ�������յ����ݣ�ͨ��USB�����������
			if( UART_PI_GetData(receivedata))   
			{
				printf("ID:%d  Data:%d %d %d %d \r\n",receivedata[0],receivedata[1],receivedata[2],receivedata[3],receivedata[4]);		
			}

			//��װҪ���͵�����
			senddata[0] = 1;  
			senddata[1] = 10;
			senddata[2] = 100;
			senddata[3] = 200;

			//��������
			UART_PI_SendPacket(senddata, 4, 0x03);
			
			//��ʱ100ms			 
			delay_ms(100);
			LED_Green_Toggle();
	}
}
