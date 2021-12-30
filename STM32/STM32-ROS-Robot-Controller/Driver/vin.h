/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __VIN_H
#define __VIN_H

/* Includes ------------------------------------------------------------------*/	 
#include "stm32f10x.h"

//OpenCRP驱动接口函数
void VIN_Init(void);  //VIN 输入电压检测初始化
float Get_VCC(void);

u16  Get_adc2(u8 ch); 
u16 Get_adc2_Average(u8 ch,u8 times);

#endif
