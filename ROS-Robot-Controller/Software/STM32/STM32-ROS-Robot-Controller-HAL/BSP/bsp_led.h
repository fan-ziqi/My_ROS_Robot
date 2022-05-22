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

#ifndef BSP_LED_H
#define BSP_LED_H
#include "main.h"

#define LED_Green_Off()  	   		HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET)     //绿色LED熄灭
#define LED_Green_On()		    	HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET)   //绿色LED点亮
#define LED_Green_Toggle()    	HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin)                  //绿色LED翻转

#define LED_Blue_Off()  	     	HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET)       //蓝色LED熄灭
#define LED_Blue_On()		       	HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET)     //蓝色LED点亮
#define LED_Blue_Toggle()      	HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin)                    //蓝色LED翻转

void LED_Init(void);

#endif
