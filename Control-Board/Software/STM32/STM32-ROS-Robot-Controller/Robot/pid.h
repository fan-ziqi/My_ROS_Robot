/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PID_H
#define __PID_H

/* Includes ------------------------------------------------------------------*/	 
#include "stm32f10x.h"

//OpenCRP���ܺ���
int16_t PID_MotorVelocityCtlA(int16_t spd_target, int16_t spd_current);   //PID���ƺ��������A
int16_t PID_MotorVelocityCtlB(int16_t spd_target, int16_t spd_current);   //PID���ƺ��������B
int16_t PID_MotorVelocityCtlC(int16_t spd_target, int16_t spd_current);   //PID���ƺ��������C
int16_t PID_MotorVelocityCtlD(int16_t spd_target, int16_t spd_current);   //PID���ƺ��������D

#endif
