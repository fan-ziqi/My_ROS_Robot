#ifndef BSP_DELAY_H
#define BSP_DELAY_H
#include "bsp_define.h"

void Delay_Init(void);
void delay_ms(u32 nms);
void delay_us(u32 nus);
void delay_xms(u32 nms);
#endif

