/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DELAY_H
#define __DELAY_H

/* Includes ------------------------------------------------------------------*/	 
#include "stm32f10x.h"

//OpenCRP�����ӿں���
void DELAY_Init(void); //��ʱ��ʼ��
void Delayus(__IO uint16_t us);  //���΢����ʱ
void delay_ms(__IO uint16_t ms);  //���������ʱ

#endif 
