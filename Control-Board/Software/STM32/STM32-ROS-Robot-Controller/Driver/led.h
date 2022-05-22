/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LED_H
#define __LED_H

/* Includes ------------------------------------------------------------------*/	 
#include "stm32f10x.h"

void LED_Init(void);


//X-SOFT�ӿں���
void LED_Init(void);

#define LED_Green_Off()  	   GPIO_SetBits(GPIOD, GPIO_Pin_2)      //LEDG��ɫϨ��
#define LED_Green_On()		     GPIO_ResetBits(GPIOD, GPIO_Pin_2)    //LEDG��ɫ����
#define LED_Green_Toggle()    GPIO_WriteBit(GPIOD, GPIO_Pin_2, (BitAction) (1 - GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_2)))	//LEDG��ɫ״̬��ת

#define LED_Blue_Off()  	     GPIO_SetBits(GPIOC, GPIO_Pin_12)      //LEDG��ɫϨ��
#define LED_Blue_On()		       GPIO_ResetBits(GPIOC, GPIO_Pin_12)    //LEDG��ɫ����
#define LED_Blue_Toggle()      GPIO_WriteBit(GPIOC, GPIO_Pin_12, (BitAction) (1 - GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_12)))	//LEDG��ɫ״̬��ת
#endif 

