#include "main.h"
#include "bsp_encoder.h"
#include "bsp_define.h"

#define LOG_TAG    "ENCODER"
#include "bsp_log.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;

void Encoder_Init(void)
{
	//±àÂëÆ÷Ä£Ê½Æô¶¯
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
	
	Encoder_Set_Counter(1, ENCODER_MID_VALUE);
	Encoder_Set_Counter(2, ENCODER_MID_VALUE);
	Encoder_Set_Counter(3, ENCODER_MID_VALUE);
	Encoder_Set_Counter(4, ENCODER_MID_VALUE);
	
	LOG_I("Encoder Init Success\r\n");
}


void Encoder_Set_Counter(int8_t Motor_Num, int16_t count)
{
	switch(Motor_Num)
	{
		case 1: __HAL_TIM_SET_COUNTER(&htim2, count); break;
		case 2: __HAL_TIM_SET_COUNTER(&htim3, count); break;
		case 3: __HAL_TIM_SET_COUNTER(&htim4, count); break;
		case 4: __HAL_TIM_SET_COUNTER(&htim5, count); break;
		default: 
		{
			LOG_E("Motor_Num ERROR\r\n");
			break;
		}
	}
}


uint16_t Encoder_Get_Counter(int8_t Motor_Num)
{
	uint16_t counter = 0;
	switch(Motor_Num)
	{
		case 1: counter = __HAL_TIM_GetCounter(&htim2); break;
		case 2: counter = __HAL_TIM_GetCounter(&htim3); break;
		case 3: counter = __HAL_TIM_GetCounter(&htim4); break;
		case 4: counter = __HAL_TIM_GetCounter(&htim5); break;
		default: 
		{
			LOG_E("Motor_Num ERROR\r\n");
			counter = 0;
			break;
		}
	}
	return counter;
}

uint16_t Encoder_Get_Dir(int8_t Motor_Num)
{
	uint16_t direction = 1;
	switch(Motor_Num)
	{
		case 1: direction = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2); break;
		case 2: direction = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3); break;
		case 3: direction = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim4); break;
		case 4: direction = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim5); break;
		default: 
		{
			LOG_E("Motor_Num ERROR\r\n");
			direction = 1;
			break;
		}
	}
	return direction;
}
