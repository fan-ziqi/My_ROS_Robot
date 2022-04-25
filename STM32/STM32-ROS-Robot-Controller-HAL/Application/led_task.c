#include "main.h"
#include "bsp_led.h"
#include "bsp_vin.h"

#define LOG_TAG    "LED"
#include "bsp_log.h"

extern int battery_status;

//20ms
void led_task(void const * argument)
{
	//开机提示信息
	LED_Blue_On();
	LED_Green_On();
	osDelay(100);	
	LED_Blue_Off();
	LED_Green_Off();
	osDelay(300);

	//绿灯点亮，提示运行
	LED_Green_On();
	while(1)
	{
		//全部初始化完成且电量充足，绿灯点亮，蓝灯熄灭，提示运行
		if(battery_status == IS_FULL)
		{
			LED_Blue_Off();
			LED_Green_On();
		}
		if(battery_status == NEED_CHARGE)
		{
			//需要充电，绿灯继续亮，蓝灯闪烁
			LED_Green_On();
			LED_Blue_Toggle();
			if(battery_status == NEED_POWEROFF)
			{
				//绿灯熄灭，蓝灯点亮，进入停机状态，电机停止转动
				LED_Blue_On();
				LED_Green_Off();
				while(1) ;
			}
		}

		osDelay(20);
	}
}
