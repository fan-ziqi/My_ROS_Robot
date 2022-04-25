#include "main.h"
#include "bsp_key.h"
#include "bsp_define.h"

#include "bsp_delay.h"

#define LOG_TAG    "KEY"
#include "bsp_log.h"

int key1_state=0, key2_state=0;

void Key_Scan(void)
{
	if(KEY1 == GPIO_PIN_RESET)		
	{
		delay_ms(10);
//		osDelay(10);
		if(KEY1 == GPIO_PIN_RESET)
		{
			key1_state=KEY_DOWN;
			delay_ms(100);
//			osDelay(100);
			if(KEY1 == GPIO_PIN_SET)
			{
				key1_state=KEY_DOWNUP;
			}
		}
		else key1_state=KEY_UP;
	}
	else key1_state=KEY_UP;
	
	if(KEY2 == GPIO_PIN_RESET)		
	{
		delay_ms(10);
//		osDelay(10);
		
		if(KEY2 == GPIO_PIN_RESET)
		{
			key2_state=KEY_DOWN;
			delay_ms(100);
//			osDelay(100);
			if(KEY2 == GPIO_PIN_SET)
			{
				key2_state=KEY_DOWNUP;
			}
		}
		else key2_state=KEY_UP;
	}
	else key2_state=KEY_UP;

}

