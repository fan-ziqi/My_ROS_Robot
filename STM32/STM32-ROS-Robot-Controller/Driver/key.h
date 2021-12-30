/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __KEY_H
#define __KEY_H

/* Includes ------------------------------------------------------------------*/	 
#include "stm32f10x.h"

typedef enum
{
	KEY_DOWN=1,
	KEY_UP,
	KEY_DOWNUP,
} KEY_STATE;

void KEY_Init(void);  //������ʼ��
void KEY_Scan(void);  //����ɨ��

#endif 
