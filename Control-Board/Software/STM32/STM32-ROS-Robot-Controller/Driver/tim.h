/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TIM_H
#define __TIM_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

//OpenCRP驱动接口函数
void TIM6_Init(uint16_t cnt_us);  //TIM6初始化
void TIM6_Cmd(FunctionalState NewState); //TIM6定时器开启关闭
uint8_t TIM6_CheckIrqStatus(void);//标志位确认

#endif
