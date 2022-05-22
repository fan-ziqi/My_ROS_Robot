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

#ifndef BSP_CMD_H
#define BSP_CMD_H
#include "bsp_define.h"

#define cmd_exit() \
	if(CMD_Flag) CMD_Flag = 0;\
	if(!strcmp(CMD_Buffer,"exit"))\
	{\
		printf("exit\r\n");\
		break;\
	}\
	for(int i = CMD_Buffer_Count+2; i >= 0; i--)\
	{\
		CMD_Buffer[i] = 0;\
	}\
	CMD_Buffer_Count = 0;

void reboot(void);
void help(void);
void cmd(void);
void debug_on(void);
void debug_off(void);
void show_imu(void);
void show_battery(void);
void show_battery_once(void);
void show_send_to_pi(void);
void set_pid(void);
void set_speed(void);
#endif
