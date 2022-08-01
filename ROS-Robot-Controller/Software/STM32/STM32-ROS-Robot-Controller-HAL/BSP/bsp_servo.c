#include "main.h"
#include "bsp_servo.h"
#include "bsp_define.h"

extern TIM_HandleTypeDef htim3;

void ServoInit(void)
{
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);	
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);	
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);	
}

void ServoCtrl(unsigned int id,float num)
{
	float angle;
	switch(id)
	{
		//注：如若修改舵机的初始角，可以修改下文(num+...)，单位为角度
		case SERVO1: angle = (num+90)*(100.0f/180.0f)*20+500; break;
		case SERVO2: angle = (num)*(100.0f/180.0f)*20+500; break;
		case SERVO3: angle = (num)*(100.0f/180.0f)*20+500; break;

		default:break;
	}
	switch(id)
	{
		case SERVO1: MOTOR_SERVO1((int)angle); break;
		case SERVO2: MOTOR_SERVO2((int)angle); break;
		case SERVO3: MOTOR_SERVO3((int)angle); break;

		default:break;
	}

}
