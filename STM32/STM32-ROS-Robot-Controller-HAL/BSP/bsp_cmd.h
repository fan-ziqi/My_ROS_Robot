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
#endif
