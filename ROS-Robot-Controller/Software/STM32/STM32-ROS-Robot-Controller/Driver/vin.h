/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __VIN_H
#define __VIN_H

/* Includes ------------------------------------------------------------------*/	 
#include "stm32f10x.h"

//OpenCRP�����ӿں���
void VIN_Init(void);  //VIN �����ѹ����ʼ��
float Get_VCC(void);

u16  Get_adc2(u8 ch); 
u16 Get_adc2_Average(u8 ch,u8 times);

#endif
