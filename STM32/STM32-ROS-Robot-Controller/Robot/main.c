/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include <stdio.h>
#include <math.h>   

#include "sys.h"    //系统设置
#include "delay.h"  //软件延时
#include "led.h"    //LED灯控制
#include "vin.h"    //输入电压检测
#include "uart_db.h"  //调试串口
#include "uart_pi.h"  //树莓派串口
#include "motor.h"    //直流电机调速控制
#include "encoder.h"  //编码器控制
#include "tim.h"      //定时器
#include "mpu6050.h"  //IMU加速度陀螺仪测量
#include "mpu6050_dmp.h"  //DMP功能函数
#include "pid.h"        //PID控制
#include "kinematics.h" //运动学解析

#include "test.h"


#define ENCODER_MID_VALUE  30000  
#define VBAT_VOL_CHG    1130 
#define VBAT_VOL_OFF    1000   


int16_t encoder[4];	//编码器累加值
int16_t encoder_delta[4];	//编码器变化值
int16_t encoder_delta_target[4] = {0};  //编码器目标变化值
int16_t robot_odom[6] = {0}; //里程计数据，绝对值和变化值，x y yaw dx dy dyaw
int16_t motor_pwm[4];  //电机PWM
uint16_t bat_vcc;  //电池电压
int16_t mpu_data[10];  //陀螺仪，加速度，姿态角

int16_t robot_target_speed[3] = {0};  //机器人目标速度 X Y Yaw
int16_t robot_params[2] = {1000,1000};  //机器人参数

//主要函数声明
void ROBOT_GetImuData(void);  //读取MPU6050数据
void ROBOT_MoveCtl(void);  //机器人运动控制函数  
void ROBOT_BatteryManagement(void);  //机器人电池管理
void ROBOT_SendDataToPi(void);  //机器人发送数据到树莓派

/**
  * @简  述  程序主函数
  * @参  数  无
  * @返回值  无
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

	
	uint8_t cnt = 1;  //定时器计数

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);   //设置中断分组

	//机器人初始化
	MOTOR_Init(10);
	DELAY_Init();  
	JTAG_Set(JTAG_SWD_DISABLE);     
	JTAG_Set(SWD_ENABLE);          
	LED_Init();
	VIN_Init();
	UART_DB_Init(115200);
	UART_PI_Init(115200);

	//编码器初始化
	ENCODER_Motor1_Init(ENCODER_MID_VALUE*2);  
	ENCODER_Motor2_Init(ENCODER_MID_VALUE*2);  
	ENCODER_Motor3_Init(ENCODER_MID_VALUE*2);  
	ENCODER_Motor4_Init(ENCODER_MID_VALUE*2);  
	ENCODER_Motor1_SetCounter(ENCODER_MID_VALUE); 
	ENCODER_Motor2_SetCounter(ENCODER_MID_VALUE); 
	ENCODER_Motor3_SetCounter(ENCODER_MID_VALUE); 
	ENCODER_Motor4_SetCounter(ENCODER_MID_VALUE);
	
    //开机提示信息
	LED_Blue_On();
	LED_Green_On();
	delay_ms(100);	
	LED_Blue_Off();
	LED_Green_Off();
	delay_ms(300);

	MPU6050_Init();    //MPU6050初始化
	MPU6050_DMP_Init();	//DMP初始化
	TIM6_Init(10000);
	TIM6_Cmd(ENABLE);
	
	
	//绿灯点亮，提示运行
	LED_Green_On();

	while(1)
	{
		//100HZ控制频率
		if(TIM6_CheckIrqStatus())
		{		
			
			//机器人获取PMU6050数据
			ROBOT_GetImuData();
			
			//50HZ执行频率
			if(cnt%2 == 0)
			{
				
				//机器人运动控制
				ROBOT_MoveCtl();
			
				//机器人电量管理
				ROBOT_BatteryManagement();
			
				//机器人发送数据到树莓派
				ROBOT_SendDataToPi();
				
			}
			
			//计数器累加
			cnt++;
		}
	}
}

/**
  * @简  述  机器人获取MPU6050 加速度陀螺仪姿态数据
  * @参  数  无
  * @返回值  无
  */
void ROBOT_GetImuData(void)
{
	  //变量定义，舵机角度
    MPU6050_DMP_GetData(mpu_data);
	
	//TODO:重写MPU6050以及DMP库
	
}
/**
  * @简  述  机器人运动控制函数
  * @参  数  无
  * @返回值  无
  */
void ROBOT_MoveCtl(void)
{ 
	  
	//获取编码器变化值
	encoder_delta[0] = -(ENCODER_Motor1_GetCounter()-ENCODER_MID_VALUE);
	encoder_delta[1] = ENCODER_Motor2_GetCounter()  -ENCODER_MID_VALUE;
	encoder_delta[2] = -(ENCODER_Motor3_GetCounter()-ENCODER_MID_VALUE);
	encoder_delta[3] = ENCODER_Motor4_GetCounter()  -ENCODER_MID_VALUE;
		
	//设置编码器中间值
	ENCODER_Motor1_SetCounter(ENCODER_MID_VALUE);
	ENCODER_Motor2_SetCounter(ENCODER_MID_VALUE);
	ENCODER_Motor3_SetCounter(ENCODER_MID_VALUE);
	ENCODER_Motor4_SetCounter(ENCODER_MID_VALUE);
		
	//计算编码器累加值
	encoder[0] = encoder[0] + encoder_delta[0];
	encoder[1] = encoder[1] + encoder_delta[1];
	encoder[2] = encoder[2] + encoder_delta[2];
	encoder[3] = encoder[3] + encoder_delta[3];
		
	//运动学解析
	Kinematics_Forward(encoder,robot_odom);  //正向运动学解析
	Kinematics_Inverse(robot_target_speed, encoder_delta_target);  //逆向运动学解析

	//电机PID速度控制
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
  * @简  述  机器人电池管理
  * @参  数  无
  * @返回值  无
  */
void ROBOT_BatteryManagement(void)
{
	//计数变量
	static uint16_t bat_vol_cnt = 0;   
	
	//采集电池电压
	bat_vcc = (uint16_t)100*Get_VCC();		
		
//	printf("当前电压 =%d V \r\n",bat_vcc);
	
	if(bat_vcc < VBAT_VOL_CHG)  //1050
	{

		LED_Blue_Toggle();
		
		if(bat_vcc < VBAT_VOL_OFF) //990
		{
			bat_vol_cnt++;
      
			//电压持续小于VBAT_VOL_OFF 4秒，进入停机保护状态
			if(bat_vol_cnt > 200 )
			{
				
				//红灯常亮，进入停机状态，电机停止转动
				LED_Blue_On();
				LED_Green_Off();
				MOTOR1_SetSpeed(0);  
				MOTOR2_SetSpeed(0);  
				MOTOR3_SetSpeed(0);  
				MOTOR4_SetSpeed(0);  
				
				//报警
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
  * @简  述  机器人发送数据到树莓派
  * @参  数  无
  * @返回值  无
  */
//这里的数据均为u16类型，也就是十进制最大65535，二进制最大1111111111111111，十六进制最大ffff
//每个数拆分成高八位和低八位，拆分后的数据类型为u8（254、11111111、ff）
void ROBOT_SendDataToPi(void)
{
	//串口发送数据
	static uint8_t comdata[60]; 			
	
	//陀螺仪角速度 = (gyro/32768) * 2000 ?s
	comdata[0] = (u8)( mpu_data[0] >> 8 );  
	comdata[1] = (u8)( mpu_data[0] );
	comdata[2] = (u8)( mpu_data[1] >> 8 );
	comdata[3] = (u8)( mpu_data[1] );
	comdata[4] = (u8)( mpu_data[2] >> 8 );
	comdata[5] = (u8)( mpu_data[2] );
	
	//加速度 = (acc/32768) * 2G  
	comdata[6] = (u8)( mpu_data[3] >> 8 );
	comdata[7] = (u8)( mpu_data[3] );
	comdata[8] = (u8)( mpu_data[4] >> 8 );
	comdata[9] = (u8)( mpu_data[4] );
	comdata[10] = (u8)( mpu_data[5] >> 8 );
	comdata[11] = (u8)( mpu_data[5] );
	
	//姿态角度 = (angle/100)
	comdata[12] = (u8)( mpu_data[6] >> 8 ); 
	comdata[13] = (u8)( mpu_data[6] );
	comdata[14] = (u8)( mpu_data[7] >> 8 );
	comdata[15] = (u8)( mpu_data[7] );
	comdata[16] = (u8)( mpu_data[8] >> 8 );
	comdata[17] = (u8)( mpu_data[8] );
	
	//里程计坐标 x(m) y(m) yaw(rad)  odom_frame
	comdata[18] = (u8)( robot_odom[0] >> 8 );
	comdata[19] = (u8)( robot_odom[0] );
	comdata[20] = (u8)( robot_odom[1] >> 8 );
	comdata[21] = (u8)( robot_odom[1] );
	comdata[22] = (u8)( robot_odom[2] >> 8 );
	comdata[23] = (u8)( robot_odom[2] );
	
	//里程计坐标变化量  d_x(m) d_y(m) d_yaw(rad)  base_frame
	comdata[24] = (u8)( robot_odom[3] >> 8 );
	comdata[25] = (u8)( robot_odom[3] );
	comdata[26] = (u8)( robot_odom[4] >> 8 );
	comdata[27] = (u8)( robot_odom[4] );
	comdata[28] = (u8)( robot_odom[5] >> 8 );
	comdata[29] = (u8)( robot_odom[5] );
		
	//编码器当前值和目标值
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
	
	//编码器
	comdata[46] = (u8)( bat_vcc >> 8 );
	comdata[47] = (u8)( bat_vcc );
		
	//发送串口数据
	UART_PI_SendPacket(comdata, 48, 0x06);
	
//	UART_DB_SendPacket(comdata, 48, 0x06);
}


