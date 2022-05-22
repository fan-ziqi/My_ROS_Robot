/*
	Copyright 2022 Fan Ziqi

	Licensed under the Apache License, Version 2.0 (the "License");
	you may not use this file except in compliance with the License.
	You may obtain a copy of the License at

		http://www.apache.org/licenses/LICENSE-2.0

	Unless required by applicable law or agreed to in writing, software
	distributed under the License is distributed on an "AS IS" BASIS,
	WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
	See the License for the specific language governing permissions and
	limitations under the License.
*/

#include "main.h"
#include "bsp_led.h"
#include "bsp_vin.h"

#define LOG_TAG    "LED"
#include "bsp_log.h"

extern int battery_status;

//20ms
void led_task(void const * argument)
{
	LED_Init();
	
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
			osDelay(100);
			if(battery_status == NEED_POWEROFF)
			{
				//绿灯熄灭，蓝灯点亮，进入停机状态，电机停止转动
				LED_Blue_On();
				LED_Green_Off();
				while(1)
				{
					if(battery_status != NEED_POWEROFF)
					{
						break;
					}
					osDelay(200);
				}
			}
		}

		osDelay(200);
	}
}
