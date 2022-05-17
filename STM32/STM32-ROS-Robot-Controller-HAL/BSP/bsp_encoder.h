#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H
#include "bsp_define.h"

#define ENCODER_MID_VALUE 30000

void Encoder_Init(void);
void Encoder_Set_Counter(int8_t Motor_Num, int16_t count);
uint16_t Encoder_Get_Counter(int8_t Motor_Num);
uint16_t Encoder_Get_Dir(int8_t Motor_Num);

#endif
