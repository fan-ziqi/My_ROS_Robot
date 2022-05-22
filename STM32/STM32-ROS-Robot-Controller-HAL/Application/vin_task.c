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

#include "bsp_vin.h"
#include "bsp_led.h"

#define LOG_TAG    "VIN"
#include "bsp_log.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;


int battery_status = IS_FULL;

#define VOLTAGE_DROP            0.13214f
#define VOLTAGE_CHG							11.50f
#define VOLTAGE_OFF							11.00f

float battery_voltage;

//100ms
void vin_task(void const * argument)
{
		osDelay(1000);
	
		VIN_Init();
	
    while(1)
    {
        battery_voltage = get_battery_voltage() + VOLTAGE_DROP;
			
				if(battery_voltage < VOLTAGE_CHG)
				{
					//需要充电
					battery_status = NEED_CHARGE;
				  
					if(battery_voltage < VOLTAGE_OFF)
					{
						osDelay(5000);
						battery_voltage = get_battery_voltage() + VOLTAGE_DROP;

						//电压持续小于VOLTAGE_OFF 5秒，进入停机保护状态
						if(battery_voltage < VOLTAGE_OFF)
						{
							battery_status = NEED_POWEROFF;
							//停止电机放在电机的任务里
							
							//停止检测
							LOG_I("电压过低，停止工作\r\n");
							while(1)
							{
								battery_voltage = get_battery_voltage() + VOLTAGE_DROP;
								if(battery_voltage >= VOLTAGE_OFF)
								{
									LOG_I("电压达到要求，重新工作\r\n");
									break;
								}
								osDelay(200);
							}								
						}
					}
				}
				else
				{
					//电池正常
					battery_status = IS_FULL;
				}
				
        osDelay(200);
    }
}
