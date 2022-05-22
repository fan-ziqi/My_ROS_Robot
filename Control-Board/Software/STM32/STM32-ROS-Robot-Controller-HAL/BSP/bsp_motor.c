/*
	Copyright 2022 Fan Ziqi

	Licensed under the Apache License, Version 2.0 (the "License");
	you may not use this file except in compliance with the License.
	You may obtain a copy of the License at

		http://www.apache.org/licenses/LICENSE-2.0

	Unless required by applicable law or agreed to in writing, software
	distributed under the License is distributed on an "AS IS" BASIS,
	WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
	See the License for the specific language governing permissions and
	limitations under the License.
*/

#include "main.h"
#include "bsp_motor.h"
#include "bsp_define.h"

#define LOG_TAG    "MOTOR"
#include "bsp_log.h"

extern TIM_HandleTypeDef htim8;

void Motor_Init(void)
{
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_4);
	
	LOG_I("MOTOR Init Success\r\n");
}	

static void MOTOR1_SetSpeed(int16_t speed)
{
	uint16_t temp;

	if(speed > 0)
	{
		HAL_GPIO_WritePin(M1IN1_GPIO_Port, M1IN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(M1IN2_GPIO_Port, M1IN2_Pin, GPIO_PIN_RESET);
		temp = speed;	
	}
	else if(speed < 0)
	{
		HAL_GPIO_WritePin(M1IN1_GPIO_Port, M1IN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(M1IN2_GPIO_Port, M1IN2_Pin, GPIO_PIN_SET);
		temp = (-speed);
	}
	else
	{
		HAL_GPIO_WritePin(M1IN1_GPIO_Port, M1IN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(M1IN2_GPIO_Port, M1IN2_Pin, GPIO_PIN_RESET);
		temp = 0;
	}
	
	if(temp>1000) temp = 1000;
	
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, temp);
}

static void MOTOR2_SetSpeed(int16_t speed)
{
	uint16_t temp;

	if(speed > 0)
	{
		HAL_GPIO_WritePin(M2IN1_GPIO_Port, M2IN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(M2IN2_GPIO_Port, M2IN2_Pin, GPIO_PIN_SET);
		temp = speed;	
	}
	else if(speed < 0)
	{
		HAL_GPIO_WritePin(M2IN1_GPIO_Port, M2IN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(M2IN2_GPIO_Port, M2IN2_Pin, GPIO_PIN_RESET);
		temp = (-speed);
	}
	else
	{
		HAL_GPIO_WritePin(M2IN1_GPIO_Port, M2IN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(M2IN2_GPIO_Port, M2IN2_Pin, GPIO_PIN_RESET);
		temp = 0;
	}
	
	if(temp>1000) temp = 1000;
	
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, temp);
}

static void MOTOR3_SetSpeed(int16_t speed)
{
	uint16_t temp;

	if(speed > 0)
	{
		HAL_GPIO_WritePin(M3IN1_GPIO_Port, M3IN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(M3IN2_GPIO_Port, M3IN2_Pin, GPIO_PIN_RESET);
		temp = speed;	
	}
	else if(speed < 0)
	{
		HAL_GPIO_WritePin(M3IN1_GPIO_Port, M3IN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(M3IN2_GPIO_Port, M3IN2_Pin, GPIO_PIN_SET);
		temp = (-speed);
	}
	else
	{
		HAL_GPIO_WritePin(M3IN1_GPIO_Port, M3IN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(M3IN2_GPIO_Port, M3IN2_Pin, GPIO_PIN_RESET);
		temp = 0;
	}
	
	if(temp>1000) temp = 1000;
	
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, temp);
}

static void MOTOR4_SetSpeed(int16_t speed)
{
	uint16_t temp;

	if(speed > 0)
	{
		HAL_GPIO_WritePin(M4IN1_GPIO_Port, M4IN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(M4IN2_GPIO_Port, M4IN2_Pin, GPIO_PIN_SET);
		temp = speed;	
	}
	else if(speed < 0)
	{
		HAL_GPIO_WritePin(M4IN1_GPIO_Port, M4IN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(M4IN2_GPIO_Port, M4IN2_Pin, GPIO_PIN_RESET);
		temp = (-speed);
	}
	else
	{
		HAL_GPIO_WritePin(M4IN1_GPIO_Port, M4IN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(M4IN2_GPIO_Port, M4IN2_Pin, GPIO_PIN_RESET);
		temp = 0;
	}
	
	if(temp>1000) temp = 1000;
	
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, temp);
}


void MOTOR_SetSpeed(int8_t Motor_Num, int16_t speed)
{
	switch(Motor_Num)
	{
		case 1: MOTOR1_SetSpeed(speed); break;
		case 2: MOTOR2_SetSpeed(speed); break;
		case 3: MOTOR3_SetSpeed(speed); break;
		case 4: MOTOR4_SetSpeed(speed); break;
		default: LOG_E("Motor_Num[%d] ERROR\r\n", Motor_Num); break;
	}
}


