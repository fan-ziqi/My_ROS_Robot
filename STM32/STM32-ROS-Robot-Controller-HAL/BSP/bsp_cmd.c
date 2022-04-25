#include "main.h"
#include "bsp_cmd.h"
#include "bsp_define.h"
#include <cstring>

#define LOG_TAG    "CMD"
#include "bsp_log.h"

#define SCB_AIRCR       (*(volatile unsigned long *)0xE000ED0C)  /* Reset control Address Register */
#define SCB_RESET_VALUE 0x05FA0004                               /* Reset value, write to SCB_AIRCR can reset cpu */

char CMD_Buffer[50] = {0};
int CMD_Buffer_Count = 0;
int CMD_Flag = 0;

void cmd(void)
{
	if(CMD_Flag)
	{
		CMD_Flag = 0;
	}
	else
	{
		return;
	}

	printf("cmd> ");
	if     (!strcmp(CMD_Buffer,"hi"))			 {printf("fuck you\r\n");}
	else if(!strcmp(CMD_Buffer,"reboot"))	 {reboot();}
	else if(!strcmp(CMD_Buffer,"battery")) {battery();}
	else if(!strcmp(CMD_Buffer,"help"))	   {help();}
	else
	{
		printf("unknown cmd '%s'\r\n",CMD_Buffer);
		help();
	}
	
	//缓存区清零
	for(;CMD_Buffer_Count>0;CMD_Buffer_Count--)
	{
		CMD_Buffer[CMD_Buffer_Count] = 0;
	}
}

void reboot(void)
{
		printf("\r\n^^^^^^^^^^^^^^^^^^^^^System Rebooting^^^^^^^^^^^^^^^^^^^^^\r\n\r\n\r\n\r\n");
    SCB_AIRCR = SCB_RESET_VALUE;
}

extern float battery_voltage;
void battery(void)
{
		printf("当前电压: %f\r\n", battery_voltage);
}

void help(void)
{
	printf("Available commands: \r\n");
	printf("reboot     ----------     system reboot\r\n");
	printf("battery    ----------     check battery\r\n");
	printf("help       ----------     help list\r\n");
}
