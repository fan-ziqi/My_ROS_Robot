/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DELAY_H
#define __DELAY_H

/* Includes ------------------------------------------------------------------*/	 
#include "stm32f10x.h"

//OpenCRP驱动接口函数
void DELAY_Init(void); //延时初始化
void Delayus(__IO uint16_t us);  //软件微妙延时
void delay_ms(__IO uint16_t ms);  //软件毫秒延时

#endif 
