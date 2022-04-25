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
