/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UART_PI_H
#define __UART_PI_H

/* Includes ------------------------------------------------------------------*/	 
#include "stm32f10x.h"

//OpenCRP�����ӿں���
void UART_PI_Init(uint32_t baud);  //��չ���ڳ�ʼ��
uint8_t UART_PI_GetData(uint8_t *pbuf);
void UART_PI_SendPacket(uint8_t *pbuf, uint8_t len, uint8_t num);  //�������ݣ�X-ProtocolЭ�飩

#endif 
