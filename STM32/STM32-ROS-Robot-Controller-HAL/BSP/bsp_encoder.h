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

#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H
#include "bsp_define.h"

#define ENCODER_MID_VALUE 30000

void Encoder_Init(void);
void Encoder_Set_Counter(int8_t Motor_Num, int16_t count);
uint16_t Encoder_Get_Counter(int8_t Motor_Num);
uint16_t Encoder_Get_Dir(int8_t Motor_Num);

#endif
