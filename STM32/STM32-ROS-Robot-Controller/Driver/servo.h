/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SERVO_H
#define __SERVO_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

//X-SOFT�ӿں���
void SERVO_AB_Init(void);                 //���AB�ӿڳ�ʼ��
void SERVO_A_SetAngle(uint16_t angle);    //���A����   
void SERVO_B_SetAngle(uint16_t angle);    //���B����

void SERVO_CD_Init(void);                 //���CD�ӿڳ�ʼ��
void SERVO_C_SetAngle(uint16_t angle);    //���C���� 
void SERVO_D_SetAngle(uint16_t angle);    //���D���� 

void SERVO_EF_Init(void);               //���EFGH�ӿڳ�ʼ��
void SERVO_E_SetAngle(uint16_t angle);    //���E����
void SERVO_F_SetAngle(uint16_t angle);    //���F���� 

void SERVO_GH_Init(void);               //���EFGH�ӿڳ�ʼ��
void SERVO_G_SetAngle(uint16_t angle);    //���G����
void SERVO_H_SetAngle(uint16_t angle);    //���H���� 

#endif

/******************* (C) ��Ȩ 2018 XTARK **************************************/
