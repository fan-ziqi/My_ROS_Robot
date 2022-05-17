#include "main.h"
#include "bsp_cmd.h"
#include "bsp_define.h"
#include <cstring>

#include "bsp_imu.h"

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
	if     (!strcmp(CMD_Buffer,"hi"))			           {printf("hi there~\r\n");}
	else if(!strcmp(CMD_Buffer,"reboot"))	           {reboot();}
	else if(!strcmp(CMD_Buffer,"help"))	             {help();}
	else if(!strcmp(CMD_Buffer,"debug_on"))	         {debug_on();}
	else if(!strcmp(CMD_Buffer,"debug_off"))         {debug_off();}
	else if(!strcmp(CMD_Buffer,"show_imu"))          {show_imu();}
	else if(!strcmp(CMD_Buffer,"show_battery"))      {show_battery();}
	else if(!strcmp(CMD_Buffer,"show_battery_once")) {show_battery_once();}
	else if(!strcmp(CMD_Buffer,"show_send_to_pi"))   {show_send_to_pi();}
	else
	{
		printf("unknown cmd '%s'\r\n",CMD_Buffer);
		help();
	}
	
	//缓存区清零
	for(int i = CMD_Buffer_Count; i >= 0; i--)
	{
		CMD_Buffer[i] = 0;
	}
	CMD_Buffer_Count = 0;
		
}

void reboot(void)
{
		printf("\r\n^^^^^^^^^^^^^^^^^^^^^System Rebooting^^^^^^^^^^^^^^^^^^^^^\r\n\r\n\r\n\r\n");
    SCB_AIRCR = SCB_RESET_VALUE;
}

void help(void)
{
	printf("Available commands: \r\n");
	printf("hi                   ----------     Say hi to me\r\n");
	printf("reboot               ----------     System reboot\r\n");
	printf("help                 ----------     Help list\r\n");
	printf("debug_on             ----------     Show debug messages\r\n");
	printf("debug_off            ----------     Don't show debug messages\r\n");
	printf("show_imu             ----------     Show mpu6050 Pitch Roll Yaw\r\n");
	printf("show_battery         ----------     Show battery\r\n");
	printf("show_battery_once    ----------     Show battery once\r\n");
	printf("show_send_to_pi      ----------     Show data send to pi\r\n");

}



void debug_on(void)
{
	elog_set_filter_lvl(ELOG_LVL_DEBUG);
	printf("debug_on, elog_set_filter_lvl: ELOG_LVL_DEBUG\r\n");
}

void debug_off(void)
{
	elog_set_filter_lvl(ELOG_LVL_INFO);
	printf("debug_off, elog_set_filter_lvl: ELOG_LVL_INFO\r\n");
}

extern struct imu_data robot_imu_dmp_data;
void show_imu(void)
{
	while(1)
	{
		printf("Pitch:%d Roll:%d Yaw:%d\r\n", 
					robot_imu_dmp_data.pitch,
					robot_imu_dmp_data.roll,
					robot_imu_dmp_data.yaw);
		
		cmd_exit();
		
		osDelay(20);
	}
}

extern float battery_voltage;
void show_battery(void)
{
	while(1)
	{
		printf("当前电压 = %.1f\r\n", battery_voltage);
		
		cmd_exit();
		
		osDelay(20);
	}
}
void show_battery_once(void)
{
		printf("当前电压: %f\r\n", battery_voltage);
}

extern int16_t SendData[];
void show_send_to_pi(void)
{
	while(1)
	{
		for(int i = 0; i < 24; i++)
		{
			printf("%d\t", SendData[i]);
		}
		printf("\r\n");
		
		cmd_exit();
		
		osDelay(10);
	}
}



