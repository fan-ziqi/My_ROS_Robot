#ifndef BSP_USART_H
#define BSP_USART_H
#include "main.h"
#include "bsp_define.h"

void USART_Init(void);

bool USART_Send_Pack(UART_HandleTypeDef *huart, short * WriteData, unsigned char MessageLenth, unsigned int MessageCode);

unsigned char CalCrc(unsigned char * VectorData, unsigned short len);


#endif

