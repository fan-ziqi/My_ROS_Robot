#ifndef BSP_LED_H
#define BSP_LED_H
#include "main.h"

#define LED_Green_Off()  	   		HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET)     //��ɫLEDϨ��
#define LED_Green_On()		    	HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET)   //��ɫLED����
#define LED_Green_Toggle()    	HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin)                  //��ɫLED��ת

#define LED_Blue_Off()  	     	HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET)       //��ɫLEDϨ��
#define LED_Blue_On()		       	HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET)     //��ɫLED����
#define LED_Blue_Toggle()      	HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin)                    //��ɫLED��ת

void LED_Init(void);

#endif
