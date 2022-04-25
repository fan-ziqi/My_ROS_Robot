#ifndef BSP_KEY_H
#define BSP_KEY_H
#include "main.h"

typedef enum
{
	KEY_DOWN=1,
	KEY_UP,
	KEY_DOWNUP,
} KEY_STATE;

#define KEY1 HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin)
#define KEY2 HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin)

void Key_Scan(void);

#endif
