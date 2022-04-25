#include "main.h"
#include "bsp_usart.h"
#include "bsp_define.h"

#include <stdlib.h>

#define LOG_TAG    "USART"
#include "bsp_log.h"

// ��Ϣͷ/��Ϣβ����
const unsigned char HEAD[2] = {0x55, 0xaa};
const unsigned char END[2] = {0x0d, 0x0a};

// ���ڷ���/����������, data_transmit��С��data�������͵��ֽ�������
union SerialMessageUnion
{
	short data;
	unsigned char data_transmit[2];
}SendUnion, ReceiveUnion;

extern UART_HandleTypeDef huart1;

static uint8_t uart_db_flag_rx_ok = 0; //���ճɹ���־
static uint8_t USART_Debug_RX_Count=0;       //���ռ�����
static uint8_t USART_Debug_RX_Length=0;       //���ռ�����
static uint8_t uart_db_rx_checksum;    //֡ͷ����У���
static uint8_t USART_Debug_RX_Package[255];     //���ջ��壬��������С�ڵ���32Byte
static uint8_t uart_db_tx_buf[40];     //���ͻ���

u8 USART_Debug_RX_Buffer[1];//HAL��ʹ�õĴ��ڽ��ջ���

int fputc(int c, FILE *stream)
{
    HAL_UART_Transmit(&huart1, (unsigned char *)&c, 1, 1000);   
    return 1;
}

uint8_t ch_r;
int fgetc(FILE * F)    
{
    HAL_UART_Receive (&huart1,&ch_r,1,0xffff);//����
    return ch_r;
}

void usart1_send_char(u8 c)
{       
	HAL_UART_Transmit(&huart1, (unsigned char *)&c, 1, 1000);   
}

extern char CMD_Buffer[];
extern int CMD_Buffer_Count;
extern int CMD_Flag;

// ��Ϣ��ʽ: 
// ��Ϣͷ1 ��Ϣͷ2 ��Ϣ���� ��Ϣ�� [��Ϣ����] У���� ��Ϣβ1 ��Ϣβ2
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance==USART1)
	{
		//���з�Ϊ0d0a
		if(USART_Debug_RX_Buffer[0] == 0x0d)
		{
			;
		}
		else if(USART_Debug_RX_Buffer[0] == 0x0a)
		{
			CMD_Flag = 1;
		}
		else
		{
			CMD_Buffer[CMD_Buffer_Count] = USART_Debug_RX_Buffer[0];		
			CMD_Buffer_Count++;
		}
	}
	HAL_UART_Receive_IT(&huart1, (uint8_t *)USART_Debug_RX_Buffer, 1);  // ���¿��������ж�
//	if (huart->Instance==USART1)
//	{
//		if(USART_Debug_RX_Count == 0) //==����֡ͷ + ����
//		{
//			
//		}
//		if(USART_Debug_RX_Count < 2) //==����֡ͷ + ����
//		{
//			if(USART_Debug_RX_Count == 0) //����֡ͷ1 0xAA
//			{
//				if(USART_Debug_RX_Buffer[0] == 0xAA)
//				{
//					USART_Debug_RX_Package[0] = USART_Debug_RX_Buffer[0];
//					USART_Debug_RX_Count = 1;
//				}
//				else
//				{
//					LOG_E("RECEIVE ERROR: HEAD1 is: 0x%02x, but it should be: 0x%02x\r\n", (int)USART_Debug_RX_Buffer[0], (int)HEAD[0]);
//				}
//			}
//			else if(USART_Debug_RX_Count == 1) //����֡ͷ2 0x55
//			{
//				if(USART_Debug_RX_Buffer[0] == 0x55)
//				{
//					USART_Debug_RX_Package[1] = USART_Debug_RX_Buffer[0];
//					USART_Debug_RX_Count = 2;
//				}
//				else
//				{
//					LOG_E("RECEIVE ERROR: HEAD2 is: 0x%02x, but it should be: 0x%02x\r\n", (int)USART_Debug_RX_Buffer[0], (int)HEAD[1]);
//					USART_Debug_RX_Count = 0;
//				}				
//			}
//		}
//		else if(USART_Debug_RX_Count == 2) //�������ݳ���
//		{
//			USART_Debug_RX_Length = USART_Debug_RX_Buffer[0];
//			USART_Debug_RX_Package[2] = USART_Debug_RX_Length;
//			USART_Debug_RX_Count = 3;
//		}
//		else //��������
//		{//3 < length
//			if(USART_Debug_RX_Count < (USART_Debug_RX_Package[2]-1) )
//			{
//				USART_Debug_RX_Package[USART_Debug_RX_Count] = USART_Debug_RX_Buffer[0];
//				USART_Debug_RX_Count++;
//				uart_db_rx_checksum = uart_db_rx_checksum + USART_Debug_RX_Buffer[0];					
//			}
//			else    //�ж����1λ
//			{
//				//����У��
//				if( USART_Debug_RX_Buffer[0] == uart_db_rx_checksum )  //У����ȷ
//				{	
//					//�˴��������ݽ���
//					uart_db_flag_rx_ok = 1;
//					
//					//������ɣ��ָ���ʼ״̬
//					USART_Debug_RX_Count = 0;					
//				}
//			}
//		}
//	} 
//  HAL_UART_Receive_IT(&huart1, (uint8_t *)USART_Debug_RX_Buffer, 1);  // ���¿��������ж�
}

/**
  * @��  ��  ��ȡ���յ�����
  * @��  ��  *pbuf����������ָ��,��1���ֽ�Ϊ֡���룬����Ϊ����
  * @����ֵ	 0-�����ݽ��գ�other-��Ҫ��ȡ�������ֽڸ���
  */
uint8_t USART_DB_GetData(uint8_t *pbuf)
{
	uint8_t cnt,i;
	
	if(uart_db_flag_rx_ok != 0)
	{
		cnt = USART_Debug_RX_Package[2]-4;
		
		for(i=0; i<cnt; i++)
		{
			*(pbuf+i) = USART_Debug_RX_Package[3+i];
		}
		
		uart_db_flag_rx_ok = 0;
		return cnt;
	}
	else
	{
		return 0;
	}	
}

// ���ڷ��ͺ���
bool USART_Debug_Send(short * WriteData, unsigned char MessageLenth, unsigned int MessageCode)
{
	// ���㷢����Ϣ����
  unsigned char SendLength = (unsigned int)(2 * MessageLenth) + 7;

	// ������Ϣ������(�䳤����)
  unsigned char * SendBuf = (unsigned char *)malloc(SendLength * sizeof(unsigned char));

	// ��Ϣͷ SendBuf[0] SendBuf[1]
	for(int i = 0; i < 2; i++)
	{
			SendBuf[i] = HEAD[i];
	}

	// ��Ϣ���� SendBuf[2]
	SendBuf[2] =  SendLength;

	// ��Ϣ�� SendBuf[3]
	SendBuf[3] = (unsigned char)MessageCode;
	
	// ��Ϣ���� SendBuf[4] ~ SendBuf[4 + 2*MessageLenth - 1]
	for(unsigned int i = 0; i < (unsigned int)MessageLenth; i++)
	{
			SendUnion.data = WriteData[i]; //������ϢUnion����
			for(int j = 0; j < 2; j++)
			{
					//���뻺����
					SendBuf[i * 2 + 4 + j] = SendUnion.data_transmit[j];
			}
	}
	
	// У������� SendBuf[2*MessageLenth + 4]
	SendBuf[2*MessageLenth + 4] = CalCrc(SendBuf, 2*MessageLenth + 4); //У��MessageLenth��Ϣ+2��Ϣͷ+1��Ϣ����+1��Ϣ��=MessageLenth + 4

	// ��Ϣβ SendBuf[MessageLenth + 5] SendBuf[MessageLenth + 6]
	SendBuf[2*MessageLenth + 5] = END[0];
	SendBuf[2*MessageLenth + 6] = END[1];
	
	// DEBUG BEGIN
	LOG_D("Write:(DEC -> HEX) ");
	for(unsigned int i = 0; i < (unsigned int)MessageLenth; i++)
	{
			SendUnion.data = WriteData[i]; //������ϢUnion����
			LOG_D("%d -> 0x%02x 0x%02x",SendUnion.data, (unsigned int)SendUnion.data_transmit[0], (unsigned int)SendUnion.data_transmit[1]);
	}
	LOG_D("\r\nWrite:(ALL HEX) ");
	for(int i = 0; i < SendLength; i++)
	{
			LOG_D("0x%02x", (unsigned int)SendBuf[i]);
	}
	LOG_D("\r\nWrite: SendLength: %d", (unsigned int)SendLength);
	// DEBUG END

	// ���ڷ�����Ϣ
	HAL_UART_Transmit(&huart1, (uint8_t *)SendBuf, sizeof(SendBuf), 100);
	
	//��ϢUnion����
	SendUnion.data = 0; 

	return true;

}

// 8λѭ������У����㺯��
unsigned char CalCrc(unsigned char * VectorData, unsigned short len)
{
    unsigned char crc;
    crc = 0;
    while(len--)
    {
        crc ^= (*VectorData++);
        for (int i = 0; i < 8; i++)
        {
            if(crc&0x01)
                crc = (crc >> 1) ^ 0x8C;
            else
                crc >>= 1;
        }
    }
    return crc;
}

void USART_Init(void)
{
	HAL_UART_Receive_IT(&huart1, (uint8_t *)USART_Debug_RX_Buffer, 1);  // ���������ж�

	LOG_I("USART Init Success\r\n");
}
