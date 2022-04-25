#include "main.h"
#include "bsp_led.h"

#define LOG_TAG    "LED"
#include "bsp_log.h"

void LED_Init(void)
{
	//πÿ±’À˘”–µ∆
	LED_Blue_Off();
	LED_Green_Off();
	
	LOG_I("LED Init Success\r\n");
}
