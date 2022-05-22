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
#include "bsp_cmd.h"
#include "bsp_define.h"
#include <cstring>
#include <stdlib.h>

#include "bsp_imu.h"

#define LOG_TAG    "CMD"
#include "bsp_log.h"

#define SCB_AIRCR       (*(volatile unsigned long *)0xE000ED0C)  /* Reset control Address Register */
#define SCB_RESET_VALUE 0x05FA0004                               /* Reset value, write to SCB_AIRCR can reset cpu */

void split(char *src,const char *separator,char **dest,int *num) {
     char *pNext;
     //��¼�ָ������� 
     int count = 0;
     //ԭ�ַ���Ϊ�� 
     if (src == NULL || strlen(src) == 0)
        return;
    //δ����ָ���
     if (separator == NULL || strlen(separator) == 0)
        return;   
	/*
		c����string���к�����
		������ 
		char *strtok(char *str, const char *delim)
		������ 
	    str -- Ҫ���ֽ��һ��С�ַ������ַ�����
    	delim -- �����ָ����� C �ַ�����
    	����ֵ��
		�ú������ر��ֽ�ĵ�һ�����ַ��������û�пɼ������ַ������򷵻�һ����ָ�롣 

	*/
	char *strtok(char *str, const char *delim); 
	 //��õ�һ���ɷָ����ָ���ַ��� 
    pNext = strtok(src,separator);
    while(pNext != NULL) {
     	//���뵽Ŀ���ַ��������� 
        *dest++ = pNext; 
        ++count;
        /*
			strtok()�������ַ����ָ��һ����Ƭ�Ρ�����sָ�����ָ���ַ���������delim��Ϊ�ָ��ַ����а����������ַ���
			��strtok()�ڲ���s���ַ����з��ֲ���delim�а����ķָ��ַ�ʱ,��Ὣ���ַ���Ϊ\0 �ַ���
			�ڵ�һ�ε���ʱ��strtok()����������s�ַ���������ĵ����򽫲���s���ó�NULL��
			ÿ�ε��óɹ��򷵻�ָ�򱻷ָ��Ƭ�ε�ָ�롣

		*/  
        pNext = strtok(NULL,separator);  
    }  
    *num = count;
}    


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
	else if(strstr(CMD_Buffer,"set_pid"))            {set_pid();}
	else if(strstr(CMD_Buffer,"set_speed"))          {set_speed();}
	else
	{
		printf("unknown cmd '%s'\r\n",CMD_Buffer);
		help();
	}
	
	//����������
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
	printf("set_pid              ----------     Set motor pid, format: [set_pid P I D] \r\n");
	printf("set_speed            ----------     Set robot speed, format: [set_speed X(m/s) Y(m/s) Yaw(deg/s)] \r\n");

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

void show_imu(void)
{
	extern struct imu_data robot_imu_dmp_data;
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


void show_battery(void)
{
	extern float battery_voltage;
	while(1)
	{
		printf("��ǰ��ѹ = %.1f\r\n", battery_voltage);
		
		cmd_exit();
		
		osDelay(20);
	}
}
void show_battery_once(void)
{
	extern float battery_voltage;
	printf("��ǰ��ѹ: %f\r\n", battery_voltage);
}


void show_send_to_pi(void)
{
	extern int16_t SendData[];
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

void set_pid(void)
{
	extern int16_t motor_kp;  //���ת��PID-P
	extern int16_t motor_ki;    //���ת��PID-I
	extern int16_t motor_kd;  //���ת��PID-D
	
	char *buf[4]={0};
	int num=0;
	split(CMD_Buffer," ",buf,&num);
	
	motor_kp = atoi(buf[1]);
	motor_ki = atoi(buf[2]);
	motor_kd = atoi(buf[3]);
	
	printf("\tSet motor pid to: \tP[%d]\tI[%d]\tD[%d]\t\r\n",
					motor_kp, 
					motor_ki, 
					motor_kd);
}

void set_speed(void)
{
	extern int16_t robot_target_speed[];  // X Y Yaw
	
	char *buf[4]={0};
	int num=0;
	split(CMD_Buffer," ",buf,&num);
	
	robot_target_speed[0] = (int16_t)(atof(buf[1])*1000);
	robot_target_speed[1] = (int16_t)(atof(buf[2])*1000);
	robot_target_speed[2] = (int16_t)((atof(buf[3])*3.1415926/180)*1000);
	
	printf("\tSet robot speed to: \tX[%.2fm/s]\tY[%.2fm/s]\tYaw[%.1fdeg/s]\t\r\n",
					atof(buf[1]), 
					atof(buf[2]), 
					atof(buf[3])*3.1415926/180);
}
