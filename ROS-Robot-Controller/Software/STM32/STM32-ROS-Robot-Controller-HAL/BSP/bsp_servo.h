#ifndef BSP_SERVO_H
#define BSP_SERVO_H
#include "bsp_define.h"

#define MOTOR_SERVO1(angle)   __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,angle);
#define MOTOR_SERVO2(angle)   __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,angle);
#define MOTOR_SERVO3(angle)   __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3,angle);

typedef enum
{
	SERVO1 = 1u,
	SERVO2,
	SERVO3,
}id;

void ServoInit(void);
void ServoCtrl(unsigned int id,float num);

#endif
