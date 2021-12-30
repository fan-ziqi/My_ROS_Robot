#include "test.h"
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include <stdio.h>

#include "sys.h"    //系统设置
#include "delay.h"  //软件延时
#include "led.h"    //LED灯控制
#include "vin.h"    //输入电压检测
#include "key.h"    //按键检测 
#include "uart_db.h"  //调试串口
#include "uart_pi.h"  //树莓派串口
#include "motor.h"    //直流电机调速控制
#include "encoder.h"  //编码器控制
#include "servo.h"    //舵机控制
#include "tim.h"      //定时器
#include "mpu6050.h"  //IMU加速度陀螺仪测量
#include "mpu6050_dmp.h"  //DMP功能函数

/******************************************************************************
      基础例程  清单
			
* LED闪烁，蜂鸣器，调试串口Printf输出例程
* VIN输入电压检测例程，简易电量计
* KEY按键检测检测例程，软件消抖
* MPU6050数据采集例程
* 舵机控制例程
* 直流电机PWM速度控制例程
* 电机AB正交编码器例程
* 直流电机PID调速例程
* STM32与树莓派通信

*******************************************************************************/

/******************************************************************************
LED_Green点亮0.5S,LED_Blue点亮0.5S,LED反转,串口输出
*******************************************************************************/
void Test_LED(void)
{
	uint8_t i=0;
	
	//设置中断优先级分组
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);    

	//JTAG口设置
	JTAG_Set(JTAG_SWD_DISABLE);  //关闭JTAG接口 
	JTAG_Set(SWD_ENABLE);  //打开SWD接口 可以利用主板的SWD接口调试 
	
	//软件延时初始化
	DELAY_Init(); 	
	LED_Init();  //LED初始化

	//调试串口初始化
	UART_DB_Init(115200); //调试串口
	printf("  \r\n"); //输出空格，CPUBUG
	
	//LED_Green点亮0.5S
	LED_Green_On();
	delay_ms(500);
	LED_Green_Off();               
	delay_ms(500);
	
	//LED_Blue点亮0.5S
	LED_Blue_On();
	delay_ms(500);
	LED_Blue_Off();
	delay_ms(500);
	
	LED_Blue_On();
	
	while (1)
	{	
		//调试串口输出信息		
		printf("Printf输出测试：%d \r\n",i);
		i++;
		delay_ms(100);
		
		//LED反转
		LED_Green_Toggle();
		LED_Blue_Toggle();
	}
}


/******************************************************************************
输入电压检测，通过串口输出电压值
ADC2通道8
*******************************************************************************/
void Test_VIN(void)
{
	float bat_vcc;
	
	//设置中断优先级分组
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);    

	//JTAG口设置
	JTAG_Set(JTAG_SWD_DISABLE);  //关闭JTAG接口 
	JTAG_Set(SWD_ENABLE);  //打开SWD接口 可以利用主板的SWD接口调试 
	
	//软件延时初始化
	DELAY_Init(); 	
	
	//调试串口初始化
	UART_DB_Init(115200); //调试串口
	printf("  \r\n"); //输出空格，CPUBUG
	
	//VIN检测初始化
	VIN_Init();
	
	//提示信息
	printf("ADC2通道8电压测试\r\n");
	
	while (1) 
	{	
		bat_vcc = Get_VCC();
		printf("当前电压 =%6.2f V \r\n",bat_vcc);				//打印当前电压，保留小数点后两位
	}
}


/******************************************************************************
KEY1按下LED1亮，松开LED1灭
KEY2按下再松开LED2翻转
*******************************************************************************/
void Test_KEY(void)
{
	extern int key1_state,key2_state;
	//设置中断优先级分组
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);    

	//JTAG口设置
	JTAG_Set(JTAG_SWD_DISABLE);  //关闭JTAG接口 
	JTAG_Set(SWD_ENABLE);  //打开SWD接口 可以利用主板的SWD接口调试 
	
	//软件延时初始化
	DELAY_Init(); 	
	LED_Init();  	
	
	//调试串口初始化
	UART_DB_Init(115200); //调试串口
	printf("  \r\n"); //输出空格，CPUBUG
	
	//初始化
	KEY_Init();
	
	while (1) 
	{	
		KEY_Scan();
		//KEY1按下LED1亮，松开LED1灭
		if(key1_state == KEY_DOWN)
		{
			LED_Green_On();  
		}
		if(key1_state == KEY_UP)
		{
			LED_Green_Off();
		}
		//KEY2按下再松开LED2翻转
		if(key2_state == KEY_DOWNUP)
		{
			LED_Blue_Toggle(); 
		}
		delay_ms(10);
	}
}	


/******************************************************************************
例程名称：读取MPU6050数据并输出
例程说明：读取MPU6050数据，通过串口Printf输出
操作说明：串口输出三轴加速度，三轴陀螺仪数据，转动控制器，观察数据变化
         使用我们的X-PrintfScope软件，可以观察到波形数据
*******************************************************************************/
void Test_MPU6050(void)
{
	int16_t acc[3],gyro[3]; //IMU数据
	
	//设置中断优先级分组
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);    

	//JTAG口设置
	JTAG_Set(JTAG_SWD_DISABLE);  //关闭JTAG接口 
	JTAG_Set(SWD_ENABLE);  //打开SWD接口 可以利用主板的SWD接口调试 
	
	//软件延时初始化
	DELAY_Init(); 	
	LED_Init();  //LED初始化

	//调试串口初始化
	UART_DB_Init(115200); //调试串口
	printf("  \r\n"); //输出空格，CPUBUG
	
	//MPU6050初始化  
	MPU6050_Init();    
	MPU6050_SetAccRange(ACC_RANGE_2G);    //设置加速度量程
	MPU6050_SetGyroRange(GYRO_RANGE_2000); //设置陀螺仪量程
	MPU6050_SetGyroSmplRate(200);            //设置陀螺仪采样率
	MPU6050_SetDLPF(DLPF_ACC94_GYRO98);   //设置低通滤波器带宽
	
	while (1)
	{	
		//更新姿态、陀螺仪、加速度数据
		MPU6050_GetAccData(acc);  //读取三轴加速度数据
		MPU6050_GetGyroData(gyro);  //读取三轴陀螺仪数据		
		
		//调试串口输出信息
		printf("%d %d %d %d %d %d \r\n",
		acc[0],acc[1],acc[2],gyro[0],gyro[1],gyro[2]);		
		
//		mpu6050_send_data(acc[0],acc[1],acc[2]);
		

	}
}


/******************************************************************************
控制8路舵机60度、90度、120度间隔运动
*******************************************************************************/
void Test_Servo(void)
{
	//设置中断优先级分组
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  
	
	//调试串口初始化
	UART_DB_Init(115200); //调试串口
	printf("  \r\n"); //输出空格，CPUBUG
	
	//JTAG口设置
	JTAG_Set(JTAG_SWD_DISABLE);  //关闭JTAG接口 
	JTAG_Set(SWD_ENABLE);  //打开SWD接口 可以利用主板的SWD接口调试 		
	
	//软件延时初始化
	DELAY_Init();
	LED_Init();
	
	//初始化
	SERVO_AB_Init();
	SERVO_CD_Init();
	SERVO_EF_Init();
	SERVO_GH_Init();
	
	//提示信息
	printf("八路舵机控制测试\r\n");

	while (1) 
	{		
		printf("*60度...... \r\n");		
		SERVO_A_SetAngle(600);
		SERVO_B_SetAngle(600);
		SERVO_C_SetAngle(600);
		SERVO_D_SetAngle(600);
		SERVO_E_SetAngle(600);
		SERVO_F_SetAngle(600);
		SERVO_G_SetAngle(600);
		SERVO_H_SetAngle(600);
		delay_ms(1000);
		
		printf("*90度...... \r\n");
		SERVO_A_SetAngle(900);
		SERVO_B_SetAngle(900);
		SERVO_C_SetAngle(900);
		SERVO_D_SetAngle(900);
		SERVO_E_SetAngle(900);
		SERVO_F_SetAngle(900);
		SERVO_G_SetAngle(900);
		SERVO_H_SetAngle(900);
		delay_ms(1000);
		
		printf("*120度...... \r\n");
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
控制4路直流电机PWM调速,变速正转和变速反转交替运行
*******************************************************************************/
void Test_Motor_PWM(void)
{
	int temp;
	
	//设置中断优先级分组
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  
	
	//JTAG口设置
	JTAG_Set(JTAG_SWD_DISABLE);  //关闭JTAG接口 
	JTAG_Set(SWD_ENABLE);  //打开SWD接口 可以利用主板的SWD接口调试 		
	
	//软件延时初始化
	DELAY_Init(); 	
	LED_Init();  
	
	//初始化
	MOTOR_Init(10);  //设置电机控制PWM频率为10K
	
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
		//控制电机转动
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
四路编码器300ms采样,手动转动电机,串口输出
*******************************************************************************/
void Test_Motor_Encoder(void)
{
	//设置中断优先级分组
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  
	
	//调试串口初始化
	UART_DB_Init(115200); //调试串口
	printf("  \r\n"); //输出空格，CPUBUG
	
	//JTAG口设置
	JTAG_Set(JTAG_SWD_DISABLE);  //关闭JTAG接口 
	JTAG_Set(SWD_ENABLE);  //打开SWD接口 可以利用主板的SWD接口调试 		
	
	//软件延时初始化
	DELAY_Init(); 	
	LED_Init();   
	
	//正交编码器初始化
	ENCODER_Motor1_Init(60000);  
	ENCODER_Motor2_Init(60000); 
	ENCODER_Motor3_Init(60000);  
	ENCODER_Motor4_Init(60000); 
	
	//设定中间值30000
	ENCODER_Motor1_SetCounter(30000); 
	ENCODER_Motor2_SetCounter(30000); 
	ENCODER_Motor3_SetCounter(30000); 
	ENCODER_Motor4_SetCounter(30000); 
	
	//提示信息
	printf("四路编码器测试\r\n");
	
	while (1) 
	{
		printf("*AB:%5d CD:%5d EF:%5d GH:%5d \r\n",
		ENCODER_Motor1_GetCounter(),ENCODER_Motor2_GetCounter(),
		ENCODER_Motor3_GetCounter(),ENCODER_Motor4_GetCounter());  
		delay_ms(300);
	}
}	


/******************************************************************************
例程名称：直流减速电机PID速度控制
例程说明：直流电机速度PID控制例程，可通过X-PrintfScope观察目标值与实际值曲线
操作说明：硬件连接，电机连接控制板电机A接口，编码器连接AB接口
         通过X-PrintfScope软件观察波形曲线，并进行电机转速设定和PID参数调节
注意事项：塔克机器人中电机有左右两种，此例程适合其中一种，另一种需要调整电机转向。
				 具体操作参考视频教程。
*******************************************************************************/
void Test_Motor_PID(void)
{
//	int16_t encoder;	//编码器绝对值
	int16_t encoder_delta;	//编码器相对变化值,代表实际速度
		
	int16_t encoder_delta_target = 0; //编码器目标值，代表目标速度
	int16_t motor_pwm;  //电机PWM速度

	int16_t motor_kp=600;  //PID参数
	int16_t motor_ki=0;  //PID参数
	int16_t motor_kd=400;  //PID参数
	
	
	
	int32_t bias,bias_last,bias_integral = 0;
	uint8_t comdata[32];
	
	//设置中断优先级分组
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);    
	
	//初始化电机
	MOTOR_Init(10);  //设置电机控制PWM频率为10K

	//JTAG口设置
	JTAG_Set(JTAG_SWD_DISABLE);  //关闭JTAG接口 
	JTAG_Set(SWD_ENABLE);  //打开SWD接口 可以利用主板的SWD接口调试 
	
	//软件延时初始化
	DELAY_Init(); 	
	LED_Init();  //LED初始化
	
	//调试串口初始化
	UART_DB_Init(115200); //调试串口
	
	//定时器初始化
	TIM6_Init(40000);//设置定时器周期定时时间40ms,25HZ

	//编码器初始化
	ENCODER_Motor1_Init(30000*2);  //正交编码器初始化
	ENCODER_Motor1_SetCounter(30000); 		//设置编码器初始值
	
	while (1) 
	{	
		//执行周期（40ms）25Hz
		if(TIM6_CheckIrqStatus())
		{			
			//计算编码器速度
			encoder_delta = -(ENCODER_Motor1_GetCounter()-30000);
	
			//设置编码器初始中间值
			ENCODER_Motor1_SetCounter(30000);
			
			//PID控制
			bias = encoder_delta_target - encoder_delta;
			bias_integral += bias;
			motor_pwm += motor_kp*bias*0.01f + motor_kd*(bias-bias_last)*0.01f + motor_ki*bias_integral*0.01f;
			
			bias_last = bias;
			
			//限制最大输出
			if(motor_pwm>2000)   motor_pwm = 2000;
			if(motor_pwm<-2000)  motor_pwm = -2000;			
			
			MOTOR1_SetSpeed(-motor_pwm);
			
			//获取USB串口数据
			if( UART_DB_GetData(comdata))  
			{
				//左右轮速度控制
				if(comdata[0] == 0x01)
				{
					encoder_delta_target = (int16_t)((comdata[1]<<8) | comdata[2]);	
				}

				//速度控制PID参数
				if(comdata[0] == 0x02)
				{
					motor_kp = (int16_t)((comdata[1]<<8) | comdata[2]);
					motor_ki = (int16_t)((comdata[3]<<8) | comdata[4]);
					motor_kd = (int16_t)((comdata[5]<<8) | comdata[6]);
				}

			}
			
			//串口发送电机目标转速和实际转速，编码器数据
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
例程名称：OpenCRP与树莓派串口通信
例程说明：OpenCRP通过与树莓派40PIN插针连接的串口，进行串口通信
操作说明：此例程不方便通过连接树莓派观察现象，可以将发送接收引脚短接，实现自发，自收操作
         通信采用X-Protocol，可使用塔克串口调试助手调试，具体详见视频教程
*******************************************************************************/
void Test_USART_PI(void)
{
	uint8_t senddata[32];
	uint8_t receivedata[32];
	
	//设置中断优先级分组
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);    
	
	//JTAG口设置
	JTAG_Set(JTAG_SWD_DISABLE);  //关闭JTAG接口 
	JTAG_Set(SWD_ENABLE);  //打开SWD接口 可以利用主板的SWD接口调试 
	
	//软件延时初始化
	DELAY_Init(); 	
	LED_Init();  //LED初始化
	
	//调试串口初始化
	UART_DB_Init(115200); //调试串口
	UART_PI_Init(115200); //树莓派串口
	
	while (1) 
	{	
			//查看树莓派串口是否接收到数据，如果接收到数据，通过USB串口输出数据
			if( UART_PI_GetData(receivedata))   
			{
				printf("ID:%d  Data:%d %d %d %d \r\n",receivedata[0],receivedata[1],receivedata[2],receivedata[3],receivedata[4]);		
			}

			//封装要发送的数据
			senddata[0] = 1;  
			senddata[1] = 10;
			senddata[2] = 100;
			senddata[3] = 200;

			//发送数据
			UART_PI_SendPacket(senddata, 4, 0x03);
			
			//延时100ms			 
			delay_ms(100);
			LED_Green_Toggle();
	}
}
