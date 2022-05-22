/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UART_DB_H
#define __UART_DB_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

//OpenCRP驱动接口函数
void UART_DB_Init(uint32_t baud);  //UART 调试串口初始化
uint8_t UART_DB_GetData(uint8_t *pbuf);  //UART 获取接收的数据
void UART_DB_SendPacket(uint8_t *pbuf, uint8_t len, uint8_t num);  //UART 发送数据（X-Protocol协议）

void usart1_send_char(u8 c);

#endif 
