/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LED_H
#define __LED_H

/* Includes ------------------------------------------------------------------*/	 
#include "stm32f10x.h"

void LED_Init(void);


//X-SOFT接口函数
void LED_Init(void);

#define LED_Green_Off()  	   GPIO_SetBits(GPIOD, GPIO_Pin_2)      //LEDG绿色熄灭
#define LED_Green_On()		     GPIO_ResetBits(GPIOD, GPIO_Pin_2)    //LEDG绿色点亮
#define LED_Green_Toggle()    GPIO_WriteBit(GPIOD, GPIO_Pin_2, (BitAction) (1 - GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_2)))	//LEDG绿色状态翻转

#define LED_Blue_Off()  	     GPIO_SetBits(GPIOC, GPIO_Pin_12)      //LEDG绿色熄灭
#define LED_Blue_On()		       GPIO_ResetBits(GPIOC, GPIO_Pin_12)    //LEDG绿色点亮
#define LED_Blue_Toggle()      GPIO_WriteBit(GPIOC, GPIO_Pin_12, (BitAction) (1 - GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_12)))	//LEDG绿色状态翻转
#endif 

