#include "main.h"
#include "bsp_define.h"
#include "bsp_vin.h"
#include "bsp_other.h"
#include "bsp_motor.h"
#include "bsp_encoder.h"

#define LOG_TAG    "OTHER"
#include "bsp_log.h"

extern int battery_status;
extern float battery_voltage;

void other_task(void const * argument)
{
	while(1)
	{
//		//全部初始化完成且电量充足，绿灯点亮，蓝灯熄灭，提示运行
//		if(battery_status == IS_FULL)
//		{
//			LOG_D("Battery Is Full\r\n");
//		}
//		if(battery_status == NEED_CHARGE)
//		{
//			LOG_D("Battery Need Charge\r\n");
//		}
//		if(battery_status == NEED_POWEROFF)
//		{
//			LOG_D("Battery Need PowerOff\r\n");
//			while(1) ;
//		}
//		LOG_D("System Running...\r\n");
//		LOG_D("当前电压 = %f\r\n", battery_voltage);
//		MOTOR_SetSpeed(1, 200);
//		MOTOR_SetSpeed(2, 200);
//		MOTOR_SetSpeed(3, 200);
//		MOTOR_SetSpeed(4, 200);
//		osDelay(2000);
//		MOTOR_SetSpeed(1, -200);
//		MOTOR_SetSpeed(2, -200);
//		MOTOR_SetSpeed(3, -200);
//		MOTOR_SetSpeed(4, -200);

//		LOG_I("Encoder: [1]%5d [2]%5d [3]%5d [4]%5d \r\n", 
//					Encoder_Get_Counter(1),
//					Encoder_Get_Counter(2),
//					Encoder_Get_Counter(3),
//					Encoder_Get_Counter(4));


		osDelay(10);
	}
}

void uart_pi_task(void const * argument)
{
	while(1)
	{
		osDelay(1);
	}
}

void uart_debug_task(void const * argument)
{
	while(1)
	{
		osDelay(1);
	}
}

void robot_move_task(void const * argument)
{
	while(1)
	{
		osDelay(1);
	}
}

