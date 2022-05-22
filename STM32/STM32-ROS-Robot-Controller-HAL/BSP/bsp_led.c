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

#define LOG_TAG    "LED"
#include "bsp_log.h"

void LED_Init(void)
{
	//�ر����е�
	LED_Blue_Off();
	LED_Green_Off();
	
	LOG_I("LED Init Success\r\n");
}
