/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOTOR_H
#define __MOTOR_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

//OpenCRP�����ӿں���
void MOTOR_Init(uint8_t freq_khz); //���PWM���Ƴ�ʼ��
void MOTOR1_SetSpeed(int16_t speed);   //���1����
void MOTOR2_SetSpeed(int16_t speed);   //���2����
void MOTOR3_SetSpeed(int16_t speed);   //���3����
void MOTOR4_SetSpeed(int16_t speed);   //���4����

#endif
