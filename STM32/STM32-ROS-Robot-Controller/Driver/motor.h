/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOTOR_H
#define __MOTOR_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

//OpenCRP驱动接口函数
void MOTOR_Init(uint8_t freq_khz); //电机PWM控制初始化
void MOTOR1_SetSpeed(int16_t speed);   //电机1控制
void MOTOR2_SetSpeed(int16_t speed);   //电机2控制
void MOTOR3_SetSpeed(int16_t speed);   //电机3控制
void MOTOR4_SetSpeed(int16_t speed);   //电机4控制

#endif
