/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ENCODER_H
#define __ENCODER_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

//OpenCRP驱动接口函数
void ENCODER_Motor1_Init(uint16_t cycle);          //编码器初始化
uint16_t ENCODER_Motor1_GetCounter(void);          //编码器获取计数器数值
void ENCODER_Motor1_SetCounter(uint16_t count);    //编码器设置计数器数值

void ENCODER_Motor2_Init(uint16_t cycle);          //编码器初始化
uint16_t ENCODER_Motor2_GetCounter(void);          //编码器获取计数器数值
void ENCODER_Motor2_SetCounter(uint16_t count);    //编码器设置计数器数值

void ENCODER_Motor3_Init(uint16_t cycle);          //编码器初始化
uint16_t ENCODER_Motor3_GetCounter(void);          //编码器获取计数器数值
void ENCODER_Motor3_SetCounter(uint16_t count);    //编码器设置计数器数值

void ENCODER_Motor4_Init(uint16_t cycle);          //编码器初始化
uint16_t ENCODER_Motor4_GetCounter(void);          //编码器获取计数器数值
void ENCODER_Motor4_SetCounter(uint16_t count);    //编码器设置计数器数值

#endif
