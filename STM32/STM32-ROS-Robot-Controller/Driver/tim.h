/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TIM_H
#define __TIM_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

//OpenCRP�����ӿں���
void TIM6_Init(uint16_t cnt_us);  //TIM6��ʼ��
void TIM6_Cmd(FunctionalState NewState); //TIM6��ʱ�������ر�
uint8_t TIM6_CheckIrqStatus(void);//��־λȷ��

#endif
