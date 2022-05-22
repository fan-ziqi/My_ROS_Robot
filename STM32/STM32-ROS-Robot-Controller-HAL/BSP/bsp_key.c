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
		osDelay(10);
		if(KEY1 == GPIO_PIN_RESET)
		{
//			LOG_D("KEY1_DOWN\r\n");
			key1_state=KEY_DOWN;
			osDelay(100);
			if(KEY1 == GPIO_PIN_SET)
			{
				LOG_D("KEY1_DOWNUP\r\n");
				key1_state=KEY_DOWNUP;
			}
		}
		else key1_state=KEY_UP;
	}
	else key1_state=KEY_UP;
	
	if(KEY2 == GPIO_PIN_RESET)		
	{
		osDelay(10);
		
		if(KEY2 == GPIO_PIN_RESET)
		{
//			LOG_D("KEY2_DOWN\r\n");
			key2_state=KEY_DOWN;
			osDelay(100);
			if(KEY2 == GPIO_PIN_SET)
			{
				LOG_D("KEY2_DOWNUP\r\n");
				key2_state=KEY_DOWNUP;
			}
		}
		else key2_state=KEY_UP;
	}
	else key2_state=KEY_UP;

}

