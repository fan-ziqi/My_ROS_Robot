/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UART_DB_H
#define __UART_DB_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

//OpenCRP�����ӿں���
void UART_DB_Init(uint32_t baud);  //UART ���Դ��ڳ�ʼ��
uint8_t UART_DB_GetData(uint8_t *pbuf);  //UART ��ȡ���յ�����
void UART_DB_SendPacket(uint8_t *pbuf, uint8_t len, uint8_t num);  //UART �������ݣ�X-ProtocolЭ�飩

void usart1_send_char(u8 c);

#endif 
