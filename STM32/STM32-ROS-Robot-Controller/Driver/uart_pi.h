/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UART_PI_H
#define __UART_PI_H

/* Includes ------------------------------------------------------------------*/	 
#include "stm32f10x.h"

//OpenCRP驱动接口函数
void UART_PI_Init(uint32_t baud);  //扩展串口初始化
uint8_t UART_PI_GetData(uint8_t *pbuf);
void UART_PI_SendPacket(uint8_t *pbuf, uint8_t len, uint8_t num);  //发送数据（X-Protocol协议）

#endif 
