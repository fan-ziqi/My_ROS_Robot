/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SERVO_H
#define __SERVO_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

//X-SOFT接口函数
void SERVO_AB_Init(void);                 //舵机AB接口初始化
void SERVO_A_SetAngle(uint16_t angle);    //舵机A控制   
void SERVO_B_SetAngle(uint16_t angle);    //舵机B控制

void SERVO_CD_Init(void);                 //舵机CD接口初始化
void SERVO_C_SetAngle(uint16_t angle);    //舵机C控制 
void SERVO_D_SetAngle(uint16_t angle);    //舵机D控制 

void SERVO_EF_Init(void);               //舵机EFGH接口初始化
void SERVO_E_SetAngle(uint16_t angle);    //舵机E控制
void SERVO_F_SetAngle(uint16_t angle);    //舵机F控制 

void SERVO_GH_Init(void);               //舵机EFGH接口初始化
void SERVO_G_SetAngle(uint16_t angle);    //舵机G控制
void SERVO_H_SetAngle(uint16_t angle);    //舵机H控制 

#endif

/******************* (C) 版权 2018 XTARK **************************************/
