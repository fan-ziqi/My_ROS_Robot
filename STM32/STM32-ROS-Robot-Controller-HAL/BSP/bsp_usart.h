#ifndef BSP_USART_H
#define BSP_USART_H
#include "bsp_define.h"

void USART_Init(void);
void usart1_send_char(u8 c);

bool USART_Debug_Send(short * WriteData, unsigned char MessageLenth, unsigned int MessageCode);

unsigned char CalCrc(unsigned char * VectorData, unsigned short len);


#endif

