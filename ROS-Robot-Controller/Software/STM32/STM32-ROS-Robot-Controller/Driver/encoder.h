/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ENCODER_H
#define __ENCODER_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

//OpenCRP�����ӿں���
void ENCODER_Motor1_Init(uint16_t cycle);          //��������ʼ��
uint16_t ENCODER_Motor1_GetCounter(void);          //��������ȡ��������ֵ
void ENCODER_Motor1_SetCounter(uint16_t count);    //���������ü�������ֵ

void ENCODER_Motor2_Init(uint16_t cycle);          //��������ʼ��
uint16_t ENCODER_Motor2_GetCounter(void);          //��������ȡ��������ֵ
void ENCODER_Motor2_SetCounter(uint16_t count);    //���������ü�������ֵ

void ENCODER_Motor3_Init(uint16_t cycle);          //��������ʼ��
uint16_t ENCODER_Motor3_GetCounter(void);          //��������ȡ��������ֵ
void ENCODER_Motor3_SetCounter(uint16_t count);    //���������ü�������ֵ

void ENCODER_Motor4_Init(uint16_t cycle);          //��������ʼ��
uint16_t ENCODER_Motor4_GetCounter(void);          //��������ȡ��������ֵ
void ENCODER_Motor4_SetCounter(uint16_t count);    //���������ü�������ֵ

#endif
